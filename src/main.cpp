/*
 * ================================================================
 *  514 – Display Node  (Gauge Node)
 *  Board : Seeed Studio XIAO ESP32-C3
 * ================================================================
 *  Motor only rotates FORWARD (one direction).
 *  12 o'clock = home (position 0).
 *  FOCUS: pointer gradually moves 12→9 over 3 minutes.
 *  Alarm at 9 o'clock: LED cycles red/yellow/blue/green.
 *  Button: fast-forward spin back to 12 o'clock + reset timer.
 *  REST: pause timer, hold current position.
 * ================================================================
 */

#include <Arduino.h>
#include <AccelStepper.h>
#include <Adafruit_NeoPixel.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEClient.h>

// ── Pin definitions ───────────────────────────────────────────────────────────
#define BTN_PIN   D6
#define LED_PIN   D10
#define NPIX      1

// ── Stepper ───────────────────────────────────────────────────────────────────
#define STEPS_REV        4096   // half-step, 64:1 gearbox
#define MOTOR_MAX_SPEED  50
#define MOTOR_ACCEL      20
#define MOTOR_FAST_SPEED 100
#define MOTOR_FAST_ACCEL 50

// 12 o'clock = 0,  9 o'clock = STEPS_REV/4 (90° forward)
#define POS_ALARM  (STEPS_REV / 4)   // 1024 steps

// ── Timing ───────────────────────────────────────────────────────────────────
#define ALARM_DELAY_MS  (3UL * 60UL * 1000UL)
#define ALARM_COLOR_MS  500

// ── BLE ───────────────────────────────────────────────────────────────────────
#define SVC_UUID   "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_UUID  "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define SIG_FOCUS  "FOCUS"
#define SIG_REST   "REST"
#define SCAN_DUR_S 5

// ── Alarm colours ─────────────────────────────────────────────────────────────
static const uint8_t ALARM_COLORS[4][3] = {
    {200,   0,   0},
    {200, 150,   0},
    {  0,   0, 200},
    {  0, 200,   0},
};

// ── Objects ───────────────────────────────────────────────────────────────────
AccelStepper      motor(AccelStepper::HALF4WIRE, D0, D1, D2, D3);
Adafruit_NeoPixel strip(NPIX, LED_PIN, NEO_GRB + NEO_KHZ800);

// ── State ─────────────────────────────────────────────────────────────────────
enum GaugeState { STATE_REST, STATE_FOCUS, STATE_ALARM };
GaugeState    gaugeState    = STATE_REST;
unsigned long focusStartMs  = 0;
unsigned long focusAccumMs  = 0;
unsigned long lastAlarmMs   = 0;
int           alarmColorIdx = 0;
int           curPos        = 0;

// ── BLE ───────────────────────────────────────────────────────────────────────
static bool                     bleFound    = false;
static bool                     bleConn     = false;
static bool                     bleScanning = false;
static unsigned long            scanStartMs = 0;
static BLEAdvertisedDevice*     pTarget     = nullptr;
static BLEClient*               pClient     = nullptr;
static BLERemoteCharacteristic* pChar       = nullptr;

void startScan();

// ── Stepper helpers ───────────────────────────────────────────────────────────
void moveTo(int target) {
    if (target <= curPos) return;   // forward only
    motor.enableOutputs();
    motor.runToNewPosition(target);
    curPos = target;
    motor.disableOutputs();
}

void returnHome() {
    Serial.println("[GAUGE] Return home");
    motor.enableOutputs();
    motor.setMaxSpeed(MOTOR_FAST_SPEED);
    motor.setAcceleration(MOTOR_FAST_ACCEL);
    motor.runToNewPosition(STEPS_REV);
    motor.setCurrentPosition(0);
    curPos = 0;
    motor.disableOutputs();
    motor.setMaxSpeed(MOTOR_MAX_SPEED);
    motor.setAcceleration(MOTOR_ACCEL);
    gaugeState   = STATE_REST;
    focusAccumMs = 0;
    strip.setPixelColor(0, strip.Color(0, 200, 0));
    strip.show();
}

// ── LED helpers ───────────────────────────────────────────────────────────────
void setLED(uint8_t r, uint8_t g, uint8_t b) {
    strip.setPixelColor(0, strip.Color(r, g, b));
    strip.show();
}

// ── Gauge actions ─────────────────────────────────────────────────────────────
void doFocus() {
    Serial.println("[GAUGE] FOCUS");
    if (gaugeState == STATE_ALARM) return;
    if (gaugeState == STATE_REST) {
        focusStartMs = millis() - focusAccumMs;
        gaugeState   = STATE_FOCUS;
        setLED(200, 0, 0);
    }
}

void doRest() {
    Serial.println("[GAUGE] REST");
    if (gaugeState == STATE_FOCUS) {
        focusAccumMs = millis() - focusStartMs;
    }                                                                  
    if (gaugeState != STATE_ALARM) {                                          
        gaugeState = STATE_REST; 
        setLED(0, 200, 0);
    }
}

// ── BLE callbacks ─────────────────────────────────────────────────────────────
void notifyCB(BLERemoteCharacteristic*, uint8_t* data, size_t len, bool) {
    String val;
    val.reserve(len);
    for (size_t i = 0; i < len; i++) val += (char)data[i];
    Serial.print("[BLE] "); Serial.println(val);
    if      (val == SIG_FOCUS) doFocus();
    else if (val == SIG_REST)  doRest();
}

class ClientCB : public BLEClientCallbacks {
    void onConnect(BLEClient*) override {
        bleConn = true;
        Serial.println("[BLE] Connected");
        setLED(0, 0, 200);
    }
    void onDisconnect(BLEClient*) override {
        bleConn  = false;
        bleFound = false;
        Serial.println("[BLE] Disconnected");
        setLED(200, 200, 0);
        startScan();
    }
};

class ScanCB : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice dev) override {
        Serial.printf("[BLE] Found: %s\n", dev.getName().c_str());
        if (dev.haveServiceUUID() &&
            dev.isAdvertisingService(BLEUUID(SVC_UUID))) {
            BLEDevice::getScan()->stop();
            pTarget     = new BLEAdvertisedDevice(dev);
            bleFound    = true;
            bleScanning = false;
        }
    }
};

bool connectBLE() {
    Serial.printf("[BLE] Connecting to %s\n",
                  pTarget->getAddress().toString().c_str());
    pClient = BLEDevice::createClient();
    pClient->setClientCallbacks(new ClientCB());
    if (!pClient->connect(pTarget)) { Serial.println("[BLE] Connect failed"); return false; }

    BLERemoteService* svc = pClient->getService(SVC_UUID);
    if (!svc) { pClient->disconnect(); return false; }

    pChar = svc->getCharacteristic(CHAR_UUID);
    if (!pChar) { pClient->disconnect(); return false; }

    if (pChar->canNotify()) pChar->registerForNotify(notifyCB);

    delay(500);   // wait for subscription to be acknowledged before reading

    if (pChar->canRead()) {
        std::string raw = pChar->readValue();
        String val(raw.c_str());
        if (val == SIG_FOCUS) doFocus();
        else                  doRest();
    }
    return true;
}

void startScan() {
    Serial.println("[BLE] Scanning...");
    setLED(255, 255, 255);
    bleScanning = true;
    scanStartMs = millis();
    BLEDevice::getScan()->start(SCAN_DUR_S, false);
}

// ── setup ─────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.println("[BOOT] Start");

    strip.begin();
    strip.setBrightness(80);
    setLED(0, 0, 0);

    pinMode(BTN_PIN, INPUT_PULLUP);

    motor.setMaxSpeed(MOTOR_MAX_SPEED);
    motor.setAcceleration(MOTOR_ACCEL);
    motor.setCurrentPosition(0);

    // Boot sweep: 12→9→12 (forward all the way around)
    motor.enableOutputs();
    motor.setMaxSpeed(MOTOR_FAST_SPEED);
    motor.setAcceleration(MOTOR_FAST_ACCEL);
    motor.runToNewPosition(POS_ALARM);   // go to 9 o'clock
    motor.runToNewPosition(STEPS_REV);   // continue forward back to 12
    motor.setCurrentPosition(0);
    curPos = 0;
    motor.disableOutputs();
    motor.setMaxSpeed(MOTOR_MAX_SPEED);
    motor.setAcceleration(MOTOR_ACCEL);

    BLEDevice::init("514-GaugeNode");
    BLEScan* scan = BLEDevice::getScan();
    scan->setAdvertisedDeviceCallbacks(new ScanCB());
    scan->setActiveScan(true);
    scan->setInterval(100);
    scan->setWindow(99);
    startScan();
}

// ── loop ──────────────────────────────────────────────────────────────────────
void loop() {

    // ── Button: return to 12 o'clock ──────────────────────────────────────
    if (digitalRead(BTN_PIN) == LOW) {
        delay(50);
        if (digitalRead(BTN_PIN) == LOW) {
            Serial.println("[BTN] Reset");
            returnHome();
            while (digitalRead(BTN_PIN) == LOW) delay(10);
        }
    }

    // ── Progressive pointer during FOCUS ──────────────────────────────────
    if (gaugeState == STATE_FOCUS) {
        unsigned long elapsed = millis() - focusStartMs;

        if (elapsed >= ALARM_DELAY_MS) {
            Serial.println("[GAUGE] ALARM");
            gaugeState    = STATE_ALARM;
            lastAlarmMs   = millis();
            alarmColorIdx = 0;
            moveTo(POS_ALARM);
        } else {
            int target = (int)((float)elapsed / ALARM_DELAY_MS * POS_ALARM);
            if (target > curPos + 10) moveTo(target);
        }
    }

    // ── Alarm LED cycle ────────────────────────────────────────────────────
    if (gaugeState == STATE_ALARM &&
        millis() - lastAlarmMs >= ALARM_COLOR_MS) {
        lastAlarmMs   = millis();
        alarmColorIdx = (alarmColorIdx + 1) % 4;
        setLED(ALARM_COLORS[alarmColorIdx][0],
               ALARM_COLORS[alarmColorIdx][1],
               ALARM_COLORS[alarmColorIdx][2]);
    }

    // ── BLE state machine ──────────────────────────────────────────────────
    if (bleFound && !bleConn) {
        if (!connectBLE()) {
            bleFound = false;
            delete pTarget; pTarget = nullptr;
            startScan();
        }
    } else if (!bleConn && !bleFound && bleScanning &&
               millis() - scanStartMs >= (SCAN_DUR_S * 1000UL)) {
        bleScanning = false;
    } else if (!bleConn && !bleFound && !bleScanning) {
        startScan();
    }

    delay(100);
}
