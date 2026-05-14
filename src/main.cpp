#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>

// =======================================================
// USER SETTINGS
// =======================================================

const char* AP_SSID = "Stinger-RPM";
const char* AP_PASSWORD = "12345678";

// Pins
const int HALL_PIN = 27;
const int SERVO_PIN = 25;
const int KILL_RELAY_PIN = 26;

// Relay logic
// This code assumes:
// relay OFF = engine killed
// relay ON  = engine allowed to run
//
// If your relay module is active LOW, set this to LOW.
// If your relay module is active HIGH, set this to HIGH.
const int RELAY_RUN_STATE = HIGH;
const int RELAY_KILL_STATE = LOW;

// RPM settings
const float PULSES_PER_REV = 1.0;
const uint32_t RPM_SAMPLE_MS = 100;
const uint32_t RPM_TIMEOUT_MS = 1000;

// Hall sensor noise filter
const uint32_t MIN_PULSE_SPACING_US = 1000;

// Servo calibration
const int SERVO_MIN_US = 1000;
const int SERVO_MAX_US = 2000;

// Default throttle limits
int idleThrottlePercent = 0;
int maxThrottlePercent = 30;

// =======================================================
// GLOBALS
// =======================================================

WebServer server(80);
Servo throttleServo;

volatile uint32_t pulseCount = 0;
volatile uint32_t lastPulseMicros = 0;
portMUX_TYPE pulseMux = portMUX_INITIALIZER_UNLOCKED;

float currentRPM = 0.0;

int throttleInputPercent = 0;
int throttleOutputPercent = 0;

bool engineRunEnabled = false;

uint32_t lastRPMCalcMs = 0;
uint32_t lastSamplePulseCount = 0;

// =======================================================
// WEBPAGE
// =======================================================

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>Stinger RPM Control</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">

  <style>
    body {
      font-family: Arial, sans-serif;
      background: #111;
      color: #eee;
      text-align: center;
      margin: 0;
      padding: 20px;
    }

    .card {
      background: #1c1c1c;
      border-radius: 14px;
      padding: 20px;
      margin: 15px auto;
      max-width: 780px;
      box-shadow: 0 0 20px rgba(0,0,0,0.45);
    }

    .rpm-value {
      font-size: 3.4rem;
      font-weight: bold;
      color: #00ff99;
    }

    .throttle-yellow {
      color: #ffd400;
      font-weight: bold;
    }

    .run-green {
      color: #00ff99;
      font-weight: bold;
    }

    .kill-red {
      color: #ff6666;
      font-weight: bold;
    }

    .data-row {
      font-size: 1.2rem;
      margin: 8px 0;
    }

    input {
      font-size: 1.2rem;
      padding: 8px;
      width: 100px;
      text-align: center;
      border-radius: 8px;
      border: none;
      margin: 5px;
    }

    button {
      font-size: 1.05rem;
      padding: 10px 18px;
      border: none;
      border-radius: 8px;
      margin: 8px;
      cursor: pointer;
      background: #00aaff;
      color: white;
    }

    button:hover {
      background: #0088cc;
    }

    .danger {
      background: #cc3333;
    }

    .danger:hover {
      background: #aa2222;
    }

    .safe {
      background: #118844;
    }

    .safe:hover {
      background: #0d6633;
    }

    .big-kill {
      font-size: 1.35rem;
      padding: 16px 28px;
      background: #ff2222;
      font-weight: bold;
    }

    .big-kill:hover {
      background: #cc0000;
    }

    .small {
      color: #aaa;
      font-size: 0.9rem;
    }

    canvas {
      width: 100%;
      height: 300px;
      background: #080808;
      border-radius: 10px;
    }

    hr {
      border: none;
      border-top: 1px solid #333;
      margin: 20px 0;
    }
  </style>
</head>

<body>
  <h1>Stinger 125cc RPM + Throttle</h1>

  <div class="card">
    <h2>Engine Kill Switch</h2>

    <div class="data-row">
      Engine status: <span id="engineStatus" class="kill-red">KILLED</span>
    </div>

    <button class="safe" onclick="enableRun()">Enable Run</button>
    <button class="big-kill" onclick="killEngine()">KILL ENGINE</button>

    <p class="small">
      Kill command also sends throttle input to 0%.
    </p>
  </div>

  <div class="card">
    <h2>Current RPM</h2>
    <div class="rpm-value" id="rpm">0</div>
    <p class="small">Live Hall sensor reading</p>
  </div>

  <div class="card">
    <h2>Throttle</h2>

    <div class="data-row">
      Input command: <span class="throttle-yellow" id="throttleInputDisplay">0</span>%
    </div>

    <div class="data-row">
      Actual servo output: <span class="throttle-yellow" id="throttleOutputDisplay">0</span>%
    </div>

    <div class="data-row">
      Idle setting: <span id="idleDisplay">0</span>%
    </div>

    <div class="data-row">
      Max setting: <span id="maxDisplay">30</span>%
    </div>

    <hr>

    <h3>Manual Throttle Command</h3>
    <input id="throttleInput" type="number" min="0" max="100" value="0">
    <button onclick="setThrottle()">Set Throttle</button>
    <button class="danger" onclick="idleNow()">Idle Now</button>

    <hr>

    <h3>Throttle Limits</h3>

    <p>Idle throttle %</p>
    <input id="idleInput" type="number" min="0" max="100" value="0">

    <p>Max throttle %</p>
    <input id="maxInput" type="number" min="0" max="100" value="30">

    <br>
    <button onclick="setLimits()">Set Limits</button>

    <p class="small">
      The throttle command is 0-100%. Servo output is mapped between idle and max.
    </p>
  </div>

  <div class="card">
    <h2>RPM Graph</h2>
    <canvas id="graph" width="700" height="300"></canvas>

    <br><br>

    <button onclick="startLogging()">Start Logging</button>
    <button onclick="stopLogging()">Stop Logging</button>
    <button onclick="downloadCSV()">Download RPM CSV</button>
    <button class="danger" onclick="clearLog()">Clear RPM Log</button>

    <p class="small">
      Logging status: <span id="loggingStatus">STOPPED</span>
    </p>

    <p class="small">
      Samples logged: <span id="sampleCount">0</span>
    </p>
  </div>

<script>
let rpmData = [];
const maxPoints = 120;

let rpmLog = [];
let loggingEnabled = false;
let logStartTime = null;

let limitsHaveLoaded = false;

const canvas = document.getElementById("graph");
const ctx = canvas.getContext("2d");

function drawGraph() {
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  ctx.fillStyle = "#080808";
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  ctx.strokeStyle = "#333";
  ctx.lineWidth = 1;

  for (let i = 0; i <= 5; i++) {
    const y = i * canvas.height / 5;
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(canvas.width, y);
    ctx.stroke();
  }

  if (rpmData.length < 2) return;

  let maxRPM = Math.max(...rpmData, 1000);
  maxRPM = Math.ceil(maxRPM / 1000) * 1000;

  ctx.strokeStyle = "#00ff99";
  ctx.lineWidth = 3;
  ctx.beginPath();

  for (let i = 0; i < rpmData.length; i++) {
    const x = i * canvas.width / (maxPoints - 1);
    const y = canvas.height - (rpmData[i] / maxRPM) * canvas.height;

    if (i === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  }

  ctx.stroke();

  ctx.fillStyle = "#aaa";
  ctx.font = "14px Arial";
  ctx.fillText(maxRPM + " RPM", 10, 20);
  ctx.fillText("0 RPM", 10, canvas.height - 10);
}

async function updateData() {
  try {
    const response = await fetch("/data");
    const data = await response.json();

    document.getElementById("rpm").innerText = Math.round(data.rpm);

    document.getElementById("throttleInputDisplay").innerText = data.throttleInput;
    document.getElementById("throttleOutputDisplay").innerText = data.throttleOutput;
    document.getElementById("idleDisplay").innerText = data.idleThrottle;
    document.getElementById("maxDisplay").innerText = data.maxThrottle;

    const engineStatus = document.getElementById("engineStatus");

    if (data.engineRunEnabled) {
      engineStatus.innerText = "RUN ENABLED";
      engineStatus.className = "run-green";
    } else {
      engineStatus.innerText = "KILLED";
      engineStatus.className = "kill-red";
    }

    if (!limitsHaveLoaded) {
      document.getElementById("idleInput").value = data.idleThrottle;
      document.getElementById("maxInput").value = data.maxThrottle;
      limitsHaveLoaded = true;
    }

    const now = new Date();

    rpmData.push(data.rpm);

    if (loggingEnabled) {
      rpmLog.push({
        seconds: ((now - logStartTime) / 1000).toFixed(2),
        rpm: Math.round(data.rpm),
        throttleInput: data.throttleInput,
        throttleOutput: data.throttleOutput,
        idleThrottle: data.idleThrottle,
        maxThrottle: data.maxThrottle,
        engineRunEnabled: data.engineRunEnabled ? 1 : 0
      });

      document.getElementById("sampleCount").innerText = rpmLog.length;
    }

    if (rpmData.length > maxPoints) {
      rpmData.shift();
    }

    drawGraph();
  } catch (error) {
    console.log("Failed to update data:", error);
  }
}

async function setThrottle() {
  let value = Number(document.getElementById("throttleInput").value);

  if (value < 0) value = 0;
  if (value > 100) value = 100;

  await fetch("/setThrottle?value=" + value);
  updateData();
}

async function idleNow() {
  document.getElementById("throttleInput").value = 0;
  await fetch("/setThrottle?value=0");
  updateData();
}

async function setLimits() {
  let idle = Number(document.getElementById("idleInput").value);
  let max = Number(document.getElementById("maxInput").value);

  if (idle < 0) idle = 0;
  if (idle > 100) idle = 100;

  if (max < 0) max = 0;
  if (max > 100) max = 100;

  if (max < idle) {
    max = idle;
  }

  document.getElementById("idleInput").value = idle;
  document.getElementById("maxInput").value = max;

  await fetch("/setLimits?idle=" + idle + "&max=" + max);
  updateData();
}

async function killEngine() {
  await fetch("/kill");
  document.getElementById("throttleInput").value = 0;
  updateData();
}

async function enableRun() {
  await fetch("/enableRun");
  updateData();
}

function startLogging() {
  rpmLog = [];
  logStartTime = new Date();
  loggingEnabled = true;

  document.getElementById("sampleCount").innerText = "0";
  document.getElementById("loggingStatus").innerText = "RUNNING";
  document.getElementById("loggingStatus").style.color = "#00ff99";
}

function stopLogging() {
  loggingEnabled = false;

  document.getElementById("loggingStatus").innerText = "STOPPED";
  document.getElementById("loggingStatus").style.color = "#ff6666";
}

function downloadCSV() {
  if (rpmLog.length === 0) {
    alert("No RPM data to export yet. Press Start Logging first.");
    return;
  }

  let csv = "seconds,rpm,throttle_input_percent,throttle_output_percent,idle_throttle_percent,max_throttle_percent,engine_run_enabled\n";

  rpmLog.forEach(row => {
    csv += row.seconds + ",";
    csv += row.rpm + ",";
    csv += row.throttleInput + ",";
    csv += row.throttleOutput + ",";
    csv += row.idleThrottle + ",";
    csv += row.maxThrottle + ",";
    csv += row.engineRunEnabled + "\n";
  });

  const blob = new Blob([csv], { type: "text/csv" });
  const url = URL.createObjectURL(blob);

  const link = document.createElement("a");
  link.href = url;
  link.download = "stinger_rpm_log.csv";

  document.body.appendChild(link);
  link.click();
  document.body.removeChild(link);

  URL.revokeObjectURL(url);
}

function clearLog() {
  rpmLog = [];
  loggingEnabled = false;

  document.getElementById("sampleCount").innerText = "0";
  document.getElementById("loggingStatus").innerText = "STOPPED";
  document.getElementById("loggingStatus").style.color = "#ff6666";

  alert("RPM log cleared.");
}

setInterval(updateData, 100);
updateData();
</script>

</body>
</html>
)rawliteral";

// =======================================================
// KILL SWITCH FUNCTIONS
// =======================================================

void killEngine() {
  engineRunEnabled = false;
  digitalWrite(KILL_RELAY_PIN, RELAY_KILL_STATE);

  // Also command throttle to idle
  throttleInputPercent = 0;
}

void enableRun() {
  engineRunEnabled = true;
  digitalWrite(KILL_RELAY_PIN, RELAY_RUN_STATE);
}

// =======================================================
// THROTTLE FUNCTIONS
// =======================================================

int percentToMicroseconds(int percent) {
  percent = constrain(percent, 0, 100);
  return map(percent, 0, 100, SERVO_MIN_US, SERVO_MAX_US);
}

int mapInputToThrottleOutput(int inputPercent) {
  inputPercent = constrain(inputPercent, 0, 100);

  idleThrottlePercent = constrain(idleThrottlePercent, 0, 100);
  maxThrottlePercent = constrain(maxThrottlePercent, 0, 100);

  if (maxThrottlePercent < idleThrottlePercent) {
    maxThrottlePercent = idleThrottlePercent;
  }

  return map(inputPercent, 0, 100, idleThrottlePercent, maxThrottlePercent);
}

void applyThrottle() {
  throttleOutputPercent = mapInputToThrottleOutput(throttleInputPercent);

  int servoMicros = percentToMicroseconds(throttleOutputPercent);
  throttleServo.writeMicroseconds(servoMicros);
}

void setThrottleInput(int inputPercent) {
  throttleInputPercent = constrain(inputPercent, 0, 100);
  applyThrottle();
}

void setThrottleLimits(int idlePercent, int maxPercent) {
  idleThrottlePercent = constrain(idlePercent, 0, 100);
  maxThrottlePercent = constrain(maxPercent, 0, 100);

  if (maxThrottlePercent < idleThrottlePercent) {
    maxThrottlePercent = idleThrottlePercent;
  }

  applyThrottle();
}

// =======================================================
// RPM FUNCTIONS
// =======================================================

void IRAM_ATTR hallISR() {
  uint32_t now = micros();

  if ((uint32_t)(now - lastPulseMicros) > MIN_PULSE_SPACING_US) {
    portENTER_CRITICAL_ISR(&pulseMux);
    pulseCount++;
    lastPulseMicros = now;
    portEXIT_CRITICAL_ISR(&pulseMux);
  }
}

void calculateRPM() {
  uint32_t nowMs = millis();

  if (nowMs - lastRPMCalcMs < RPM_SAMPLE_MS) {
    return;
  }

  uint32_t totalPulses;
  uint32_t lastPulseCopy;

  portENTER_CRITICAL(&pulseMux);
  totalPulses = pulseCount;
  lastPulseCopy = lastPulseMicros;
  portEXIT_CRITICAL(&pulseMux);

  uint32_t pulseDelta = totalPulses - lastSamplePulseCount;
  float sampleMinutes = (nowMs - lastRPMCalcMs) / 60000.0;

  if (sampleMinutes > 0) {
    currentRPM = (pulseDelta / PULSES_PER_REV) / sampleMinutes;
  }

  if ((uint32_t)(micros() - lastPulseCopy) > RPM_TIMEOUT_MS * 1000UL) {
    currentRPM = 0.0;
  }

  lastSamplePulseCount = totalPulses;
  lastRPMCalcMs = nowMs;
}

// =======================================================
// WEB HANDLERS
// =======================================================

void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleData() {
  char json[280];

  snprintf(
    json,
    sizeof(json),
    "{\"rpm\":%.1f,\"throttleInput\":%d,\"throttleOutput\":%d,\"idleThrottle\":%d,\"maxThrottle\":%d,\"engineRunEnabled\":%s}",
    currentRPM,
    throttleInputPercent,
    throttleOutputPercent,
    idleThrottlePercent,
    maxThrottlePercent,
    engineRunEnabled ? "true" : "false"
  );

  server.send(200, "application/json", json);
}

void handleSetThrottle() {
  if (server.hasArg("value")) {
    int value = server.arg("value").toInt();
    setThrottleInput(value);
  }

  server.send(200, "text/plain", "OK");
}

void handleSetLimits() {
  if (server.hasArg("idle") && server.hasArg("max")) {
    int idle = server.arg("idle").toInt();
    int max = server.arg("max").toInt();

    setThrottleLimits(idle, max);
  }

  server.send(200, "text/plain", "OK");
}

void handleKill() {
  killEngine();
  applyThrottle();
  server.send(200, "text/plain", "KILLED");
}

void handleEnableRun() {
  enableRun();
  server.send(200, "text/plain", "RUN ENABLED");
}

// =======================================================
// SETUP
// =======================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(KILL_RELAY_PIN, OUTPUT);

  // Failsafe: start killed
  digitalWrite(KILL_RELAY_PIN, RELAY_KILL_STATE);
  engineRunEnabled = false;

  pinMode(HALL_PIN, INPUT_PULLUP);

  attachInterrupt(
    digitalPinToInterrupt(HALL_PIN),
    hallISR,
    FALLING
  );

  throttleServo.setPeriodHertz(50);
  throttleServo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);

  setThrottleInput(0);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);

  IPAddress ip = WiFi.softAPIP();

  Serial.println();
  Serial.println("======================================");
  Serial.println("ESP32 Stinger RPM Controller Started");
  Serial.println("======================================");
  Serial.print("WiFi Name: ");
  Serial.println(AP_SSID);
  Serial.print("Password: ");
  Serial.println(AP_PASSWORD);
  Serial.print("Open: http://");
  Serial.println(ip);
  Serial.println("======================================");

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/setThrottle", handleSetThrottle);
  server.on("/setLimits", handleSetLimits);
  server.on("/kill", handleKill);
  server.on("/enableRun", handleEnableRun);

  server.begin();

  lastRPMCalcMs = millis();
}

// =======================================================
// LOOP
// =======================================================

void loop() {
  server.handleClient();
  calculateRPM();

  static uint32_t lastSerialPrintMs = 0;

  if (millis() - lastSerialPrintMs >= 1000) {
    lastSerialPrintMs = millis();

    Serial.print("RPM: ");
    Serial.print(currentRPM);

    Serial.print(" | Input: ");
    Serial.print(throttleInputPercent);
    Serial.print("%");

    Serial.print(" | Servo Output: ");
    Serial.print(throttleOutputPercent);
    Serial.print("%");

    Serial.print(" | Idle: ");
    Serial.print(idleThrottlePercent);
    Serial.print("%");

    Serial.print(" | Max: ");
    Serial.print(maxThrottlePercent);
    Serial.print("%");

    Serial.print(" | Engine: ");
    Serial.println(engineRunEnabled ? "RUN ENABLED" : "KILLED");
  }
}