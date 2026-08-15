#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>

// ============================================================================
//  ADDED (does NOT change any docking / charging logic):
//   1. WiFi is now STATION mode — joins the shared "LUMA" network so the
//      base-station laptop and your phone can reach this ESP by IP.
//   2. A transparent Serial "tee": every Serial.print(...) the docking /
//      charging code already does is ALSO shipped, line-by-line, over WiFi
//      to the base-station dashboard (/api/dock_log) — shown in a separate
//      "Docking & Charging" panel. Nothing in the docking code was edited.
//   3. On connect the ESP registers its current IP with the base station
//      (/api/dock_register) so the laptop always knows this ESP's address,
//      even with DHCP — no fragile static IP needed.
//  The base station calls this ESP's existing /landed route (5 s after the
//  drone touches down) to start docking. That route is UNCHANGED.
// ============================================================================

// ---- network config — EDIT ALL FOUR TO MATCH YOUR SETUP ----
// ⚠️ Never commit real credentials to a public repository.
static const char* WIFI_SSID   = "YOUR_WIFI_SSID";
static const char* WIFI_PASS   = "YOUR_WIFI_PASSWORD";
// Ground PC's IP on that network. Find it on the PC with:  hostname -I
// (DHCP can change it between sessions — give the PC a static lease to keep it fixed.)
static const char* BASE_URL    = "http://192.168.1.100:8000";
// Shared token so machine-to-machine calls bypass the dashboard password.
// Must match IROC_TOKEN on the base station. CHANGE THIS to your own value.
static const char* DOCK_TOKEN  = "CHANGE_ME";

// ---- transparent Serial → WiFi log mirror ----
// Capture the real UART BEFORE we redirect the Serial name below.
HardwareSerial& RealSerial = Serial;

struct LogLine { char text[160]; };
static QueueHandle_t g_logQueue = nullptr;

class NetTee : public Print {
  char   lineBuf[160];
  size_t lineLen = 0;
 public:
  void begin(unsigned long baud) { RealSerial.begin(baud); }
  size_t write(uint8_t c) override {
    RealSerial.write(c);                 // still prints to USB serial as before
    if (c == '\r') return 1;
    if (c == '\n' || lineLen >= sizeof(lineBuf) - 1) {
      lineBuf[lineLen] = 0;
      if (lineLen > 0 && g_logQueue) {
        LogLine ll;
        strncpy(ll.text, lineBuf, sizeof(ll.text));
        ll.text[sizeof(ll.text) - 1] = 0;
        xQueueSend(g_logQueue, &ll, 0);  // non-blocking; drop if queue full
      }
      lineLen = 0;
    } else {
      lineBuf[lineLen++] = (char)c;
    }
    return 1;
  }
  size_t write(const uint8_t* b, size_t n) override {
    for (size_t i = 0; i < n; i++) write(b[i]);
    return n;
  }
};
NetTee NetSerial;
// From here on, every "Serial.xxx" in the (unchanged) docking code is teed.
#define Serial NetSerial

// ---- register this ESP's IP with the base station ----
void registerWithBase() {
  if (WiFi.status() != WL_CONNECTED) return;
  HTTPClient http;
  // Token goes in a header, not the query string — query strings are recorded
  // in proxy and web-server access logs in plain text.
  String url = String(BASE_URL) + "/api/dock_register?ip=" + WiFi.localIP().toString();
  http.begin(url);
  http.addHeader("X-Auth-Token", DOCK_TOKEN);
  http.setConnectTimeout(1500);
  http.POST("");
  http.end();
}

// ---- background task (core 0): drains the log queue + periodic re-register.
//      Runs on core 0 so its blocking HTTP never disturbs the motor step
//      timing in loop() (which runs on core 1). ----
void logTask(void* arg) {
  LogLine ll;
  unsigned long lastReg = 0;
  for (;;) {
    if (xQueueReceive(g_logQueue, &ll, pdMS_TO_TICKS(1000)) == pdTRUE) {
      String batch = ll.text;
      while (xQueueReceive(g_logQueue, &ll, 0) == pdTRUE) {   // batch bursts
        batch += "\n";
        batch += ll.text;
        if (batch.length() > 1200) break;
      }
      if (WiFi.status() == WL_CONNECTED) {
        HTTPClient http;
        http.begin(String(BASE_URL) + "/api/dock_log");
        http.addHeader("X-Auth-Token", DOCK_TOKEN);
        http.addHeader("Content-Type", "text/plain");
        http.setConnectTimeout(1500);
        http.POST(batch);
        http.end();
      }
    }
    if (millis() - lastReg > 30000UL) { lastReg = millis(); registerWithBase(); }
  }
}

// =====================
// Motor 1
// =====================
const int stepPin1 = 16;
const int dirPin1  = 17;

// =====================
// Motor 2
// =====================
const int stepPin2 = 18;
const int dirPin2  = 19;

WebServer server(80);

// Motor states
volatile bool motor1Moving = false;
volatile bool motor2Moving = false;

// =====================
// Voltage divider / contact sense
// =====================
#define PAD_A_PIN 32
#define PAD_B_PIN 33

// Divider ratio = (R1 + R2) / R2.  Set this to YOUR actual resistors.
// ⚠️ The ESP32 ADC is only linear up to ~2.4 V. Your MAX pack voltage must
//    land BELOW that on the ADC pin, or it clips and reads LOW:
//      ratio 5.7 (47k+10k) -> 17 V = 2.98 V at pin  ❌ CLIPS  (your bug)
//      ratio 7.9 (47k+6.8k)-> 17 V = 2.15 V at pin  ✅ safe
//      ratio 7.7 (100k+15k)-> 17 V = 2.21 V at pin  ✅ safe
//    If you change the resistors, update this number to match.
#define DIVIDER_RATIO       5.7f
#define ADC_REF             3.3f      // (unused now — kept for reference)
#define ADC_MAX             4095.0f   // (unused now — kept for reference)

// ── Fine trim applied AFTER the divider maths: actual = (v * SLOPE) + OFFSET
//    Start at 1.0 / 0.0, then calibrate with two multimeter points
//    (procedure in the notes). Do NOT reuse old numbers — the reading
//    method changed to the factory-calibrated analogReadMilliVolts().
#define CAL_SLOPE   1.0f
#define CAL_OFFSET  0.0f

#define HIGH_THRESHOLD  3.0f
#define LOW_THRESHOLD   0.5f

// =====================
// BTS7960 charging control
// =====================
#define RPWM_PIN  26
#define LPWM_PIN  27
#define EN_PIN    25

// 4S Li-ion pack: most common Li-ion cells (e.g. 18650/21700) are rated
// to 4.2V/cell full charge, same ceiling as LiPo — so 4 x 4.2V = 16.8V.
// Verify this matches YOUR specific cell's datasheet before relying on it.
// NOTE: All automatic charge-stopping logic has been removed per request.
// Charging now starts after the countdown and runs CONTINUOUSLY with no
// pause, no voltage check, and no automatic stop. The battery must be
// physically removed from the pads to end the charge cycle (removing it
// breaks pad contact, which triggers the abort-to-IDLE path below).

#define STABILIZE_MS     3000UL
#define COUNTDOWN_MS     10000UL

// ── Final seat push: after contact is made, wait 2 s, then drive the motors
//    FORWARD a little more and STOP there permanently.
//    The rods NEVER reverse / never return. ──
#define PUSH_DELAY_MS   2000UL   // wait 2 s after contact, then push forward
#define PUSH_TIME_MS    1000UL   // how long to drive forward (TUNE THIS)

// Docking state
volatile bool dockingActive = false;
volatile bool contactMade   = false;

// Docking safety timeout — if motors run this long without contact, stop anyway
unsigned long dockingStartTime = 0;
#define DOCKING_TIMEOUT_MS  30000UL

// Contact settle delay — wait this long after first detecting contact
// before actually stopping the docking motors, so the drone fully seats.
bool contactPending = false;
unsigned long contactPendingStart = 0;
#define CONTACT_SETTLE_MS  1000UL

// Cached pad voltages — updated periodically, NOT blocking the step loop
float cachedVA = 0.0f;
float cachedVB = 0.0f;
unsigned long lastSenseCheck = 0;
const unsigned long SENSE_INTERVAL_MS = 20;

// ── Charging sequence state machine ──
enum ChargeState {
  CS_IDLE,            // waiting for contactMade to become true
  CS_STABILIZING,
  CS_POLARITY_CONFIRMED,
  CS_COUNTDOWN,
  CS_CHARGING,
  CS_COMPLETE,
  CS_ERROR
};

ChargeState chargeState = CS_IDLE;
unsigned long chargeStateTimer = 0;
unsigned long lastChargeResumeTime = 0;
unsigned long totalChargeElapsed = 0;

bool  padA_positive = false;
float batteryVoltageAtStart = 0.0f;
float batteryVoltageFinal   = 0.0f;

// ── Final forward-push state (no reverse anywhere) ──
enum PushState { PS_NONE, PS_WAIT, PS_PUSHING, PS_DONE };
volatile PushState pushState = PS_NONE;
unsigned long pushTimer = 0;

// ===== HTML UI =====
const char MAIN_page[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Dual Motor Control</title>
<style>
body {
    font-family: Arial;
    text-align: center;
    margin-top: 40px;
    background-color: #222;
    color: white;
}
.btn {
    padding: 25px 40px;
    font-size: 22px;
    margin: 10px;
    border-radius: 12px;
    background-color: #007BFF;
    color: white;
    border: none;
}
.btn-land {
    padding: 25px 40px;
    font-size: 22px;
    margin: 10px;
    border-radius: 12px;
    background-color: #28a745;
    color: white;
    border: none;
}
.btn-stop {
    padding: 25px 40px;
    font-size: 22px;
    margin: 10px;
    border-radius: 12px;
    background-color: #dc3545;
    color: white;
    border: none;
}
#status {
    margin-top: 20px;
    font-size: 16px;
    background:#333;
    padding: 15px;
    border-radius: 10px;
    white-space: pre-line;
}
</style>
</head>
<body>

<h2>Motor 1</h2>

<button class="btn"
ontouchstart="startMotor('/start1?dir=fwd', event)"
ontouchend="stopMotor('/stop1', event)"
onmousedown="startMotor('/start1?dir=fwd', event)"
onmouseup="stopMotor('/stop1', event)">
Forward
</button>

<button class="btn"
ontouchstart="startMotor('/start1?dir=rev', event)"
ontouchend="stopMotor('/stop1', event)"
onmousedown="startMotor('/start1?dir=rev', event)"
onmouseup="stopMotor('/stop1', event)">
Reverse
</button>

<br><br>

<h2>Motor 2</h2>

<button class="btn"
ontouchstart="startMotor('/start2?dir=fwd', event)"
ontouchend="stopMotor('/stop2', event)"
onmousedown="startMotor('/start2?dir=fwd', event)"
onmouseup="stopMotor('/stop2', event)">
Forward
</button>

<button class="btn"
ontouchstart="startMotor('/start2?dir=rev', event)"
ontouchend="stopMotor('/stop2', event)"
onmousedown="startMotor('/start2?dir=rev', event)"
onmouseup="stopMotor('/stop2', event)">
Reverse
</button>

<br><br>

<h2>Docking</h2>
<button class="btn-land" onclick="fetch('/landed')">LANDED</button>
<button class="btn-stop" onclick="fetch('/dockstop')">STOP</button>

<div id="status">Loading status...</div>

<script>
function startMotor(url, e){
    e.preventDefault();
    fetch(url);
}

function stopMotor(url, e){
    e.preventDefault();
    fetch(url);
}

function refreshStatus(){
  fetch('/status').then(r => r.text()).then(t => {
    document.getElementById('status').innerText = t;
  });
}
setInterval(refreshStatus, 1000);
refreshStatus();
</script>

</body>
</html>
)=====";

// ===== ROUTES =====

void handleRoot() {
  server.send(200, "text/html", MAIN_page);
}

// ===== Motor 1 ===== (unchanged)

void handleStart1() {
  if (server.hasArg("dir")) {
    motor1Moving = false;
    delay(5);
    String dir = server.arg("dir");
    if (dir == "fwd") digitalWrite(dirPin1, HIGH);
    else digitalWrite(dirPin1, LOW);
    motor1Moving = true;
  }
  server.send(200, "text/plain", "OK");
}

void handleStop1() {
  motor1Moving = false;
  server.send(200, "text/plain", "OK");
}

// ===== Motor 2 ===== (unchanged)

void handleStart2() {
  if (server.hasArg("dir")) {
    motor2Moving = false;
    delay(5);
    String dir = server.arg("dir");
    if (dir == "fwd") digitalWrite(dirPin2, HIGH);
    else digitalWrite(dirPin2, LOW);
    motor2Moving = true;
  }
  server.send(200, "text/plain", "OK");
}

void handleStop2() {
  motor2Moving = false;
  server.send(200, "text/plain", "OK");
}

// ===== Docking control =====

void handleLanded() {
  if (!dockingActive && chargeState == CS_IDLE) {
    contactMade = false;
    digitalWrite(dirPin1, HIGH);
    digitalWrite(dirPin2, HIGH);
    motor1Moving = true;
    motor2Moving = true;
    dockingActive = true;
    dockingStartTime = millis();
    pushState = PS_NONE;                // cancel any pending push from a prior cycle
    Serial.println(">>> LANDED received. Docking started. <<<");
  }
  server.send(200, "text/plain", "OK");
}

void handleDockStop() {
  motor1Moving = false;
  motor2Moving = false;
  dockingActive = false;
  contactPending = false;
  // Also fully reset charging sequence and force charging off
  digitalWrite(EN_PIN, LOW);
  digitalWrite(RPWM_PIN, LOW);
  digitalWrite(LPWM_PIN, LOW);
  contactMade = false;
  chargeState = CS_IDLE;
  pushState = PS_NONE;                 // also cancel any push in progress
  Serial.println(">>> Docking / charging manually stopped and reset. <<<");
  server.send(200, "text/plain", "OK");
}

// FAST, single-sample read — no delay, no blocking.
// Two-point calibration applied, but clamped so that near-zero raw
// readings (no contact) don't get pushed up by the offset term.
float readPadVoltageFast(int pin) {
  // analogReadMilliVolts() uses the ESP32's FACTORY eFuse ADC calibration,
  // which corrects most of the chip's non-linearity — far more accurate than
  // raw analogRead() * 3.3/4095. Average 4 samples to steady the reading.
  uint32_t mv = 0;
  for (int i = 0; i < 4; i++) mv += analogReadMilliVolts(pin);
  float v_pin = (mv / 4.0f) / 1000.0f;          // volts AT THE ADC PIN
  float v = v_pin * DIVIDER_RATIO;              // volts at the pad

  if (v < 0.3f) {
    // genuinely disconnected — don't apply the offset, or "no contact"
    // would falsely read as a real voltage
    return 0.0f;
  }

  return (v * CAL_SLOPE) + CAL_OFFSET;
}

void setChargeDirection(bool padA_is_positive) {
  if (padA_is_positive) {
    digitalWrite(RPWM_PIN, HIGH);
    digitalWrite(LPWM_PIN, LOW);
  } else {
    digitalWrite(RPWM_PIN, LOW);
    digitalWrite(LPWM_PIN, HIGH);
  }
}

void chargingDisable() {
  digitalWrite(EN_PIN, LOW);
  digitalWrite(RPWM_PIN, LOW);
  digitalWrite(LPWM_PIN, LOW);
}

void chargingEnable() {
  // CRITICAL FIX: must re-apply direction before enabling.
  // Previously this only set EN_PIN HIGH, leaving RPWM/LPWM both LOW
  // after a pause/resume cycle (since chargingDisable() zeroes them).
  // Both-LOW with EN-HIGH is an undefined/fault state on the H-bridge —
  // this was the cause of wire heating after the first check window.
  setChargeDirection(padA_positive);
  digitalWrite(EN_PIN, HIGH);
}

void handleStatus() {
  String s = "";
  s += "Docking active: " + String(dockingActive ? "YES" : "NO") + "\n";
  s += "Contact made: " + String(contactMade ? "YES" : "NO") + "\n";
  s += "Pad A: " + String(cachedVA, 2) + " V\n";
  s += "Pad B: " + String(cachedVB, 2) + " V\n";
  s += "Charge state: ";
  switch (chargeState) {
    case CS_IDLE:               s += "IDLE\n"; break;
    case CS_STABILIZING:        s += "STABILIZING\n"; break;
    case CS_POLARITY_CONFIRMED: s += "POLARITY_CONFIRMED\n"; break;
    case CS_COUNTDOWN:          s += "COUNTDOWN\n"; break;
    case CS_CHARGING:           s += "CHARGING\n"; break;
    case CS_COMPLETE:           s += "COMPLETE\n"; break;
    case CS_ERROR:              s += "ERROR\n"; break;
  }
  s += "Battery at start: " + String(batteryVoltageAtStart, 2) + " V\n";
  if (chargeState == CS_COMPLETE) {
    s += "Battery final: " + String(batteryVoltageFinal, 2) + " V\n";
  }
  server.send(200, "text/plain", s);
}

// ============================================================================
//  SMOOTH STEP GENERATION (A4988) — hardware LEDC square wave
//  The step pulses were being made in loop() with delayMicroseconds(), which
//  WiFi/the web server kept interrupting → uneven pulses, one motor fast/one
//  slow, and extra noise. Now the ESP32 LEDC peripheral outputs a rock-steady
//  square wave on each STEP pin at the SAME frequency, so both motors run at
//  identical speed with zero CPU jitter and much less noise. Full torque is
//  preserved (set current via the A4988 VREF pot — see notes).
//  We only start/stop the wave to match the existing motorXMoving flags;
//  none of the docking/charging logic changed.
// ============================================================================
#define STEP_HZ    400       // steps/sec for BOTH motors. LOWER = MORE TORQUE (slower).
                             //   weak/too fast? lower to 250-300.  too slow? raise it.
#define STEP_CH1   0
#define STEP_CH2   1
#define STEP_DUTY  128       // 50% duty on 8-bit (0-255)

void stepperSetup() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(stepPin1, STEP_HZ, 8);
  ledcAttach(stepPin2, STEP_HZ, 8);
  ledcWrite(stepPin1, 0);
  ledcWrite(stepPin2, 0);
#else
  ledcSetup(STEP_CH1, STEP_HZ, 8);
  ledcAttachPin(stepPin1, STEP_CH1);
  ledcSetup(STEP_CH2, STEP_HZ, 8);
  ledcAttachPin(stepPin2, STEP_CH2);
  ledcWrite(STEP_CH1, 0);
  ledcWrite(STEP_CH2, 0);
#endif
}

void stepperRun(int motor, bool on) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(motor == 1 ? stepPin1 : stepPin2, on ? STEP_DUTY : 0);
#else
  ledcWrite(motor == 1 ? STEP_CH1 : STEP_CH2, on ? STEP_DUTY : 0);
#endif
}

// ===== SETUP =====

void setup() {
  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  digitalWrite(stepPin1, LOW);
  digitalWrite(stepPin2, LOW);
  digitalWrite(dirPin1, LOW);
  digitalWrite(dirPin2, LOW);

  stepperSetup();   // hardware LEDC step generation (smooth, equal speed)

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  pinMode(RPWM_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  chargingDisable();

  Serial.begin(115200);

  // ── log mirror: queue + background sender on core 0 (added) ──
  g_logQueue = xQueueCreate(60, sizeof(LogLine));
  xTaskCreatePinnedToCore(logTask, "dockLogTask", 6144, nullptr, 1, nullptr, 0);

  // ===== Join the shared "LUMA" WiFi (STATION mode) =====
  // (was WiFi.softAP("Luma Base", ...) — now joins the common network so the
  //  base station + your phone reach this ESP by IP.)
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  // CRITICAL for stepper timing: disable WiFi modem-sleep. In STA mode the
  // ESP32 otherwise parks the radio between router beacons and the wake-ups
  // stall the CPU for a few ms — which jitters the delayMicroseconds() step
  // pulses and makes the steppers buzz/stall. AP mode never did this.
  WiFi.setSleep(false);

  Serial.println();
  Serial.print("Connecting to WiFi: ");
  Serial.println(WIFI_SSID);

  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 20000UL) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.println("WiFi Connected");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    registerWithBase();     // tell the base station where to find this ESP
  } else {
    Serial.println();
    Serial.println("WiFi FAILED to connect within 20s — check SSID/password.");
  }

  server.on("/", handleRoot);
  server.on("/start1", handleStart1);
  server.on("/stop1", handleStop1);
  server.on("/start2", handleStart2);
  server.on("/stop2", handleStop2);
  server.on("/landed", handleLanded);
  server.on("/dockstop", handleDockStop);
  server.on("/status", handleStatus);

  server.begin();

  Serial.println("Web Server Started");
}

// ===== LOOP =====

void loop() {

  server.handleClient();

  // ── Pad sensing — every SENSE_INTERVAL_MS, fast, non-blocking ──
  unsigned long now = millis();
  bool freshReading = false;

  if (now - lastSenseCheck >= SENSE_INTERVAL_MS) {
    lastSenseCheck = now;
    cachedVA = readPadVoltageFast(PAD_A_PIN);
    cachedVB = readPadVoltageFast(PAD_B_PIN);
    freshReading = true;

    bool a_high = (cachedVA > HIGH_THRESHOLD);
    bool b_high = (cachedVB > HIGH_THRESHOLD);
    bool a_low  = (cachedVA < LOW_THRESHOLD);
    bool b_low  = (cachedVB < LOW_THRESHOLD);
    bool contactOK = (a_high && b_low) || (b_high && a_low);

    // ── Docking stop — wait 1 second after first contact before actually
    //    stopping, so the drone settles fully into the pads.
    //    ALSO: 30 second maximum timeout fallback if contact never happens ──
    if (dockingActive) {
      if (millis() - dockingStartTime >= DOCKING_TIMEOUT_MS) {
        motor1Moving = false;
        motor2Moving = false;
        dockingActive = false;
        contactPending = false;
        Serial.println(">>> DOCKING TIMEOUT (30s) — motors stopped, no contact made. <<<");
      }
      else if (contactOK) {
        if (!contactPending) {
          contactPending = true;
          contactPendingStart = millis();
          Serial.println(">>> Contact detected. Settling for 1 second... <<<");
        } else if (millis() - contactPendingStart >= CONTACT_SETTLE_MS) {
          motor1Moving = false;
          motor2Moving = false;
          dockingActive = false;
          contactMade = true;
          contactPending = false;
          pushState = PS_WAIT;              // schedule the final forward push
          pushTimer = millis();
          Serial.println(">>> Settled. Docking motors stopped. <<<");
        }
      } else {
        // contact was lost before settle time completed — reset pending state
        if (contactPending) {
          Serial.println(">>> Contact lost during settle — resuming docking <<<");
        }
        contactPending = false;
      }
    }

    // ── Charging sequence state machine (only runs after contactMade) ──
    if (contactMade) {
      switch (chargeState) {

        case CS_IDLE: {
          Serial.println(">>> Contact confirmed. Stabilizing... <<<");
          chargeStateTimer = millis();
          chargeState = CS_STABILIZING;
          break;
        }

        case CS_STABILIZING: {
          if (!contactOK) {
            Serial.println(">>> ABORT: contact lost during stabilization <<<");
            chargingDisable();
            contactMade = false;
            chargeState = CS_IDLE;
            break;
          }
          if (millis() - chargeStateTimer >= STABILIZE_MS) {
            chargeState = CS_POLARITY_CONFIRMED;
          }
          break;
        }

        case CS_POLARITY_CONFIRMED: {
          if (!contactOK) {
            Serial.println(">>> ABORT: contact lost before polarity confirm <<<");
            chargingDisable();
            contactMade = false;
            chargeState = CS_IDLE;
            break;
          }

          if (a_high && b_low) {
            padA_positive = true;
            batteryVoltageAtStart = cachedVA;
          } else {
            padA_positive = false;
            batteryVoltageAtStart = cachedVB;
          }

          setChargeDirection(padA_positive);

          Serial.println("-------------------------------------------");
          Serial.print("POLARITY : Pad ");
          Serial.print(padA_positive ? "A" : "B");
          Serial.println(" = POSITIVE");
          Serial.print("BATTERY VOLTAGE: ");
          Serial.print(batteryVoltageAtStart, 2);
          Serial.println(" V");
          Serial.println("-------------------------------------------");

          Serial.println("Beginning charging process in 10 seconds...");
          chargeStateTimer = millis();
          chargeState = CS_COUNTDOWN;
          break;
        }

        case CS_COUNTDOWN: {
          if (!contactOK) {
            Serial.println(">>> ABORT: contact lost during countdown <<<");
            chargingDisable();
            contactMade = false;
            chargeState = CS_IDLE;
            break;
          }

          static unsigned long lastCountdownPrint = 0;
          unsigned long elapsed = millis() - chargeStateTimer;
          unsigned long remaining = (COUNTDOWN_MS - elapsed) / 1000;

          if (millis() - lastCountdownPrint >= 1000) {
            lastCountdownPrint = millis();
            Serial.print(">>> Charging starts in: ");
            Serial.print(remaining + 1);
            Serial.println(" s");
          }

          if (elapsed >= COUNTDOWN_MS) {
            chargingEnable();
            lastChargeResumeTime = millis();
            totalChargeElapsed = 0;
            Serial.println(">>> CHARGING STARTED <<<");
            chargeState = CS_CHARGING;
          }
          break;
        }

        case CS_CHARGING: {
          unsigned long chargingNow = millis() - lastChargeResumeTime;

          static unsigned long lastStatusPrint = 0;
          if (millis() - lastStatusPrint >= 5000) {
            lastStatusPrint = millis();
            Serial.print("[CHARGING] total time: ");
            Serial.print((totalChargeElapsed + chargingNow) / 1000);
            Serial.println(" s");
          }

          // Charging runs continuously — no automatic stop. The motors already
          // did their final forward push right after contact and are parked;
          // nothing reverses.
          break;
        }

        case CS_COMPLETE: {
          static bool printedOnce = false;
          if (!printedOnce) {
            chargingDisable();
            Serial.println("===========================================");
            Serial.println(" CHARGE CYCLE COMPLETE. ");
            Serial.println("===========================================");
            printedOnce = true;
          }
          break;
        }

        case CS_ERROR: {
          static bool printedOnce = false;
          if (!printedOnce) {
            chargingDisable();
            Serial.println("Charging halted due to error.");
            printedOnce = true;
          }
          break;
        }
      }
    }
  }

  // ── Final seat push: 2 s after contact, drive BOTH motors FORWARD a little
  //    more, then STOP permanently. Nothing ever reverses — the rods stay put. ──
  switch (pushState) {
    case PS_WAIT:
      if (millis() - pushTimer >= PUSH_DELAY_MS) {
        digitalWrite(dirPin1, HIGH);           // FORWARD (same direction as docking)
        digitalWrite(dirPin2, HIGH);
        motor1Moving = true;
        motor2Moving = true;
        pushTimer = millis();
        pushState = PS_PUSHING;
        Serial.println(">>> Final seat push (forward)... <<<");
      }
      break;
    case PS_PUSHING:
      if (millis() - pushTimer >= PUSH_TIME_MS) {
        motor1Moving = false;
        motor2Moving = false;
        pushState = PS_DONE;
        Serial.println(">>> Push done. Rods STOPPED here — no reverse. <<<");
      }
      break;
    default:
      break;   // PS_NONE, PS_DONE
  }

  // ── Live pad voltage printout, throttled to once per 500ms ──
  static unsigned long lastVoltagePrint = 0;
  if (freshReading && millis() - lastVoltagePrint >= 500) {
    lastVoltagePrint = millis();
    Serial.print("Pad A: "); Serial.print(cachedVA, 2);
    Serial.print(" V | Pad B: "); Serial.print(cachedVB, 2);
    // raw ADC pin voltage — for calibration / spotting ADC clipping.
    // If this stops rising above ~2400 mV while the real pack voltage keeps
    // going up, the ADC is CLIPPING and you must change the divider resistor.
    Serial.print(" V  [pinA="); Serial.print(analogReadMilliVolts(PAD_A_PIN));
    Serial.print("mV pinB="); Serial.print(analogReadMilliVolts(PAD_B_PIN));
    Serial.println("mV]");
  }

  // ── Step generation is now HARDWARE (LEDC). We only switch the constant
  //    square wave on/off to follow the (unchanged) motorXMoving flags, so
  //    both motors run at exactly STEP_HZ — same speed, no jitter, quieter. ──
  static bool m1prev = false, m2prev = false;
  if (motor1Moving != m1prev) { stepperRun(1, motor1Moving); m1prev = motor1Moving; }
  if (motor2Moving != m2prev) { stepperRun(2, motor2Moving); m2prev = motor2Moving; }

  delay(1);   // yield to WiFi / other tasks — stepping no longer needs the CPU
}
