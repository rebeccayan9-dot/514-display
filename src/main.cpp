/*
 * ================================================================
 *  514 – Display Node  (Gauge Node)
 *  Board : Seeed Studio XIAO ESP32-C3
 * ================================================================
 *  Wakes from deep-sleep on button press (D4).
 *  Connects to the Vision Node over BLE as a central/client.
 *  Drives a 28BYJ-48 stepper pointer + WS2812B NeoPixel.
 *
 *  BLE "FOCUS" → stepper → POS_FOCUS,  LED green.
 *  BLE "REST"  → stepper → POS_REST,   LED blue.
 *  Button (awake) → stepper home, LED blue.
 *  No activity for 3 min → LED off, stepper home, deep-sleep.
 * ================================================================
 */

#include <Arduino.h>
#include <Stepper.h>
#include <Adafruit_NeoPixel.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEClient.h>
#include "esp_sleep.h"
#include "driver/gpio.h"

// ── Pin definitions ──────────────────────────────────────────────────────────
//
//  28BYJ-48 coil winding order required by Stepper.h for smooth rotation:
//  Stepper(steps, IN1, IN3, IN2, IN4)  →  passes them in 1-3-2-4 sequence.
#define STEP_P1   D0   // ULN2003 IN1
#define STEP_P2   D2   // ULN2003 IN3  (listed 2nd → Stepper uses 1-3-2-4)
#define STEP_P3   D1   // ULN2003 IN2
#define STEP_P4   D3   // ULN2003 IN4

#define BTN_PIN   D4   // Active-LOW (INPUT_PULLUP), to GND
#define LED_PIN   D5   // WS2812B DIN
#define NPIX      1

// Raw GPIO number for D4 on XIAO ESP32-C3 (D4 = GPIO6).
// Used in the deep-sleep wakeup bit-mask.
// ⚠ If your board variant defines D4 differently, update this number.
#define BTN_GPIO  6

// ── Stepper ──────────────────────────────────────────────────────────────────
#define STEPS_REV   2048    // 28BYJ-48 full-step with 64:1 gearbox
#define MOTOR_RPM   10

// ▼ Tune these step counts to match your physical dial face ▼
#define POS_HOME    0
#define POS_REST    0          //   0° – idle / rest
#define POS_FOCUS   512        //  90° – person detected  (= STEPS_REV / 4)

// ── BLE ──────────────────────────────────────────────────────────────────────
// ▼ These MUST match exactly what your Vision Node advertises ▼
#define VISION_NAME   "514-VisionNode"
#define SVC_UUID      "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_UUID     "beb5483e-36e1-4688-b7f5-ea07361b26a8"

#define SIG_FOCUS     "FOCUS"
#define SIG_REST      "REST"
#define SCAN_DUR_S    5      // seconds per BLE scan window

// ── Auto-sleep ───────────────────────────────────────────────────────────────
#define SLEEP_MS   (3UL * 60UL * 1000UL)   // 3 minutes of inactivity

// ── RTC memory – persists through deep sleep ─────────────────────────────────
RTC_DATA_ATTR int rtcPos = 0;   // last known stepper position in steps

// ── Objects & state ──────────────────────────────────────────────────────────
Stepper           motor(STEPS_REV, STEP_P1, STEP_P2, STEP_P3, STEP_P4);
Adafruit_NeoPixel strip(NPIX, LED_PIN, NEO_GRB + NEO_KHZ800);

int           curPos    = 0;
unsigned long lastActMs = 0;

static bool                     bleFound = false;
static bool                     bleConn  = false;
static BLEAdvertisedDevice*     pTarget  = nullptr;
static BLEClient*               pClient  = nullptr;
static BLERemoteCharacteristic* pChar    = nullptr;

// ── Forward declarations ──────────────────────────────────────────────────────
void doRest();
void doFocus();
void goSleep();
bool connectBLE();
void startScan();

// ─────────────────────────────────────────────────────────────────────────────
//  Stepper helpers
// ─────────────────────────────────────────────────────────────────────────────

// Power off coils when stationary – prevents overheating.
void stepperOff() {
    digitalWrite(STEP_P1, LOW);
    digitalWrite(STEP_P2, LOW);
    digitalWrite(STEP_P3, LOW);
    digitalWrite(STEP_P4, LOW);
}

// Absolute move (blocking).  Updates curPos and rtcPos.
void moveTo(int target) {
    int delta = target - curPos;
    if (delta != 0) motor.step(delta);
    curPos = target;
    rtcPos = curPos;
    stepperOff();
}

// ─────────────────────────────────────────────────────────────────────────────
//  NeoPixel helpers
// ─────────────────────────────────────────────────────────────────────────────
void setLED(uint8_t r, uint8_t g, uint8_t b) {
    strip.setPixelColor(0, strip.Color(r, g, b));
    strip.show();
}

void ledOff() {
    strip.clear();
    strip.show();
}

// ─────────────────────────────────────────────────────────────────────────────
//  Gauge actions
// ─────────────────────────────────────────────────────────────────────────────
void doFocus() {
    Serial.println("[GAUGE] FOCUS");
    moveTo(POS_FOCUS);
    setLED(0, 200, 0);    // Green
    lastActMs = millis();
}

void doRest() {
    Serial.println("[GAUGE] REST");
    moveTo(POS_REST);
    setLED(0, 0, 200);    // Blue
    lastActMs = millis();
}

// ─────────────────────────────────────────────────────────────────────────────
//  Deep sleep
// ─────────────────────────────────────────────────────────────────────────────
void goSleep() {
    Serial.println("[SLEEP] Going to deep sleep – press button to wake");
    Serial.flush();

    moveTo(POS_HOME);
    ledOff();

    if (pClient && pClient->isConnected()) pClient->disconnect();
    BLEDevice::deinit(false);
    delay(100);

    // Wake on button press (GPIO LOW).
    // A 10 kΩ external pull-up on the button line is recommended to keep the
    // pin reliably HIGH during deep sleep if the internal pull-up is released.
    esp_deep_sleep_enable_gpio_wakeup(1ULL << BTN_GPIO,
                                      ESP_GPIO_WAKEUP_GPIO_LOW);
    esp_deep_sleep_start();
    // never returns
}

// ─────────────────────────────────────────────────────────────────────────────
//  BLE – notification from Vision Node
// ─────────────────────────────────────────────────────────────────────────────
void notifyCB(BLERemoteCharacteristic*, uint8_t* data, size_t len, bool) {
    String val;
    val.reserve(len);
    for (size_t i = 0; i < len; i++) val += (char)data[i];
    Serial.print("[BLE] Notify: "); Serial.println(val);
    lastActMs = millis();
    if      (val == SIG_FOCUS) doFocus();
    else if (val == SIG_REST)  doRest();
}

// ─────────────────────────────────────────────────────────────────────────────
//  BLE – client connection callbacks
// ─────────────────────────────────────────────────────────────────────────────
class ClientCB : public BLEClientCallbacks {
    void onConnect(BLEClient*) override {
        bleConn   = true;
        lastActMs = millis();
        Serial.println("[BLE] Connected to Vision Node");
    }
    void onDisconnect(BLEClient*) override {
        bleConn = false;
        Serial.println("[BLE] Disconnected");
        setLED(200, 0, 0);    // Red – link lost
    }
};

// ─────────────────────────────────────────────────────────────────────────────
//  BLE – scan callbacks
// ─────────────────────────────────────────────────────────────────────────────
class ScanCB : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice dev) override {
        Serial.printf("[BLE] Found: %s\n", dev.getName().c_str());
        if (dev.getName() == VISION_NAME) {
            BLEDevice::getScan()->stop();
            pTarget  = new BLEAdvertisedDevice(dev);
            bleFound = true;
        }
    }
};

// ─────────────────────────────────────────────────────────────────────────────
//  BLE – connect, discover service & characteristic, subscribe
// ─────────────────────────────────────────────────────────────────────────────
bool connectBLE() {
    Serial.printf("[BLE] Connecting to %s\n",
                  pTarget->getAddress().toString().c_str());

    pClient = BLEDevice::createClient();
    pClient->setClientCallbacks(new ClientCB());

    if (!pClient->connect(pTarget)) {
        Serial.println("[BLE] Connect failed");
        return false;
    }

    BLERemoteService* svc = pClient->getService(SVC_UUID);
    if (!svc) {
        Serial.println("[BLE] Service not found");
        pClient->disconnect();
        return false;
    }

    pChar = svc->getCharacteristic(CHAR_UUID);
    if (!pChar) {
        Serial.println("[BLE] Characteristic not found");
        pClient->disconnect();
        return false;
    }

    if (pChar->canNotify()) {
        pChar->registerForNotify(notifyCB);
        Serial.println("[BLE] Subscribed to notifications");
    }

    // Read current state immediately so the gauge is correct on first connect.
    if (pChar->canRead()) {
        std::string raw = pChar->readValue();
        String val(raw.c_str());
        Serial.print("[BLE] Initial value: "); Serial.println(val);
        if (val == SIG_FOCUS) doFocus();
        else                  doRest();
    }

    lastActMs = millis();
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  BLE – (re)start scan helper
// ─────────────────────────────────────────────────────────────────────────────
void startScan() {
    Serial.println("[BLE] Scanning for Vision Node...");
    setLED(200, 100, 0);                              // Orange = searching
    BLEDevice::getScan()->start(SCAN_DUR_S, false);   // non-blocking
}

// ─────────────────────────────────────────────────────────────────────────────
//  setup()
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(300);

    // Log wakeup reason
    auto cause = esp_sleep_get_wakeup_cause();
    if (cause == ESP_SLEEP_WAKEUP_GPIO) {
        Serial.println("[BOOT] Woke from deep sleep (button press)");
    } else {
        Serial.println("[BOOT] Cold start");
        rtcPos = 0;   // physical position unknown – assume home
    }
    curPos = rtcPos;

    // NeoPixel
    strip.begin();
    strip.setBrightness(80);
    ledOff();

    // Button
    pinMode(BTN_PIN, INPUT_PULLUP);

    // Stepper
    motor.setSpeed(MOTOR_RPM);
    stepperOff();

    lastActMs = millis();

    // BLE – init as central (client)
    BLEDevice::init("514-GaugeNode");
    BLEScan* scan = BLEDevice::getScan();
    scan->setAdvertisedDeviceCallbacks(new ScanCB());
    scan->setActiveScan(true);
    scan->setInterval(100);
    scan->setWindow(99);

    startScan();
}

// ─────────────────────────────────────────────────────────────────────────────
//  loop()
// ─────────────────────────────────────────────────────────────────────────────
void loop() {

    // ── Button: manual home reset ──────────────────────────────────────────
    if (digitalRead(BTN_PIN) == LOW) {
        delay(50);                               // debounce
        if (digitalRead(BTN_PIN) == LOW) {
            Serial.println("[BTN] Manual home reset");
            moveTo(POS_HOME);
            setLED(255, 255, 255); delay(200);   // brief white flash
            setLED(0, 0, 200);                   // back to blue
            lastActMs = millis();
            while (digitalRead(BTN_PIN) == LOW) delay(10);  // wait for release
        }
    }

    // ── BLE state machine ──────────────────────────────────────────────────
    if (bleFound && !bleConn) {
        // Vision Node was found during scan – attempt connection
        if (!connectBLE()) {
            // Failed – clean up and restart scan
            bleFound = false;
            delete pTarget;
            pTarget = nullptr;
            startScan();
        }
    } else if (!bleConn && !bleFound && !BLEDevice::getScan()->isScanning()) {
        // Scan ended without finding Vision Node – retry
        startScan();
    }

    // ── Auto-sleep on inactivity ───────────────────────────────────────────
    if (millis() - lastActMs >= SLEEP_MS) {
        goSleep();
    }

    delay(100);
}
