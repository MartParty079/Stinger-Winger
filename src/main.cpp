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
const int KILL_RELAY_PIN = 26;

// Four servo signal pins
const int SERVO1_PIN = 25;
const int SERVO2_PIN = 14;
const int SERVO3_PIN = 12;
const int SERVO4_PIN = 13;

// Relay logic
// Assumption:
// relay OFF = engine killed
// relay ON  = engine allowed to run
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

// Servo trim limits
// Individual trims are added after the main throttle output.
// Example: base output 20%, trim +3% => servo output 23%.
const int MIN_SERVO_TRIM = -30;
const int MAX_SERVO_TRIM = 30;

// =======================================================
// GLOBALS
// =======================================================

WebServer server(80);

Servo servos[4];
const int servoPins[4] = {
  SERVO1_PIN,
  SERVO2_PIN,
  SERVO3_PIN,
  SERVO4_PIN
};

volatile uint32_t pulseCount = 0;
volatile uint32_t lastPulseMicros = 0;
portMUX_TYPE pulseMux = portMUX_INITIALIZER_UNLOCKED;

float currentRPM = 0.0;

int throttleInputPercent = 0;       // 0-100 command from web
int throttleBaseOutputPercent = 0;  // mapped output before individual trims

int servoTrimPercent[4] = {0, 0, 0, 0};
int servoOutputPercent[4] = {0, 0, 0, 0};

int throttleArray[21] = {
  0, 10, 20, 
  30, 40, 50, 
  60, 70, 80, 
  90, 100, 90, 
  80, 70, 60, 
  50, 40, 30, 
  20, 10, 0}; //Array of servo steps

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
      max-width: 820px;
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

    table {
      margin: 10px auto;
      border-collapse: collapse;
      width: 90%;
      max-width: 650px;
    }

    th, td {
      border-bottom: 1px solid #333;
      padding: 10px;
    }

    th {
      color: #aaa;
    }
  </style>
</head>

<body>
  <h1>Stinger 125cc RPM + 4-Servo Throttle</h1>

  <div class="card">
    <h2>Engine Kill Switch</h2>

    <div class="data-row">
      Engine status: <span id="engineStatus" class="kill-red">KILLED</span>
    </div>

    <button class="safe" onclick="enableRun()">Enable Run</button>
    <button class="big-kill" onclick="killEngine()">KILL ENGINE</button>

    <p class="small">
      Kill command also sends all throttle servos to idle.
    </p>
  </div>

  <div class="card">
    <h2>Current RPM</h2>
    <div class="rpm-value" id="rpm">0</div>
    <p class="small">Live Hall sensor reading</p>
  </div>

  <div class="card">
    <h2>Main Throttle Control</h2>

    <div class="data-row">
      Input command: <span class="throttle-yellow" id="throttleInputDisplay">0</span>%
    </div>

    <div class="data-row">
      Base throttle output: <span class="throttle-yellow" id="throttleBaseOutputDisplay">0</span>%
    </div>

    <div class="data-row">
      Idle setting: <span id="idleDisplay">0</span>%
    </div>

    <div class="data-row">
      Max setting: <span id="maxDisplay">30</span>%
    </div>

    <hr>

    <h3>Control All 4 Servos Together</h3>
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
      Main throttle command is 0-100%. Base output is mapped between idle and max.
    </p>
  </div>

  <div class="card">
    <h2>Individual Servo Adjustments</h2>

    <p class="small">
      Trim is added to the base throttle output. Example: base 20%, trim +3%, servo output 23%.
    </p>

    <table>
      <tr>
        <th>Servo</th>
        <th>Trim %</th>
        <th>Actual Output %</th>
      </tr>

      <tr>
        <td>Servo 1</td>
        <td><input id="trim1Input" type="number" min="-30" max="30" value="0"></td>
        <td><span class="throttle-yellow" id="servo1Output">0</span>%</td>
      </tr>

      <tr>
        <td>Servo 2</td>
        <td><input id="trim2Input" type="number" min="-30" max="30" value="0"></td>
        <td><span class="throttle-yellow" id="servo2Output">0</span>%</td>
      </tr>

      <tr>
        <td>Servo 3</td>
        <td><input id="trim3Input" type="number" min="-30" max="30" value="0"></td>
        <td><span class="throttle-yellow" id="servo3Output">0</span>%</td>
      </tr>

      <tr>
        <td>Servo 4</td>
        <td><input id="trim4Input" type="number" min="-30" max="30" value="0"></td>
        <td><span class="throttle-yellow" id="servo4Output">0</span>%</td>
      </tr>
    </table>

    <button onclick="setTrims()">Set Servo Trims</button>
    <button onclick="zeroTrims()">Zero Trims</button>
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
let trimsHaveLoaded = false;

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
    document.getElementById("throttleBaseOutputDisplay").innerText = data.throttleBaseOutput;

    document.getElementById("idleDisplay").innerText = data.idleThrottle;
    document.getElementById("maxDisplay").innerText = data.maxThrottle;

    document.getElementById("servo1Output").innerText = data.servo1Output;
    document.getElementById("servo2Output").innerText = data.servo2Output;
    document.getElementById("servo3Output").innerText = data.servo3Output;
    document.getElementById("servo4Output").innerText = data.servo4Output;

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

    if (!trimsHaveLoaded) {
      document.getElementById("trim1Input").value = data.servo1Trim;
      document.getElementById("trim2Input").value = data.servo2Trim;
      document.getElementById("trim3Input").value = data.servo3Trim;
      document.getElementById("trim4Input").value = data.servo4Trim;
      trimsHaveLoaded = true;
    }

    const now = new Date();

    rpmData.push(data.rpm);

    if (loggingEnabled) {
      rpmLog.push({
        seconds: ((now - logStartTime) / 1000).toFixed(2),
        rpm: Math.round(data.rpm),
        throttleInput: data.throttleInput,
        throttleBaseOutput: data.throttleBaseOutput,
        servo1Output: data.servo1Output,
        servo2Output: data.servo2Output,
        servo3Output: data.servo3Output,
        servo4Output: data.servo4Output,
        servo1Trim: data.servo1Trim,
        servo2Trim: data.servo2Trim,
        servo3Trim: data.servo3Trim,
        servo4Trim: data.servo4Trim,
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

async function setTrims() {
  let t1 = Number(document.getElementById("trim1Input").value);
  let t2 = Number(document.getElementById("trim2Input").value);
  let t3 = Number(document.getElementById("trim3Input").value);
  let t4 = Number(document.getElementById("trim4Input").value);

  t1 = Math.max(-30, Math.min(30, t1));
  t2 = Math.max(-30, Math.min(30, t2));
  t3 = Math.max(-30, Math.min(30, t3));
  t4 = Math.max(-30, Math.min(30, t4));

  document.getElementById("trim1Input").value = t1;
  document.getElementById("trim2Input").value = t2;
  document.getElementById("trim3Input").value = t3;
  document.getElementById("trim4Input").value = t4;

  await fetch("/setTrims?t1=" + t1 + "&t2=" + t2 + "&t3=" + t3 + "&t4=" + t4);
  updateData();
}

async function zeroTrims() {
  document.getElementById("trim1Input").value = 0;
  document.getElementById("trim2Input").value = 0;
  document.getElementById("trim3Input").value = 0;
  document.getElementById("trim4Input").value = 0;

  await fetch("/setTrims?t1=0&t2=0&t3=0&t4=0");
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

  let csv = "seconds,rpm,throttle_input_percent,throttle_base_output_percent,servo1_output_percent,servo2_output_percent,servo3_output_percent,servo4_output_percent,servo1_trim_percent,servo2_trim_percent,servo3_trim_percent,servo4_trim_percent,idle_throttle_percent,max_throttle_percent,engine_run_enabled\n";

  rpmLog.forEach(row => {
    csv += row.seconds + ",";
    csv += row.rpm + ",";
    csv += row.throttleInput + ",";
    csv += row.throttleBaseOutput + ",";
    csv += row.servo1Output + ",";
    csv += row.servo2Output + ",";
    csv += row.servo3Output + ",";
    csv += row.servo4Output + ",";
    csv += row.servo1Trim + ",";
    csv += row.servo2Trim + ",";
    csv += row.servo3Trim + ",";
    csv += row.servo4Trim + ",";
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

  // Also command all throttle servos to idle
  throttleInputPercent = 0;
}

void enableRun() {
  engineRunEnabled = true;
  digitalWrite(KILL_RELAY_PIN, RELAY_RUN_STATE);
}

// =======================================================
// THROTTLE / SERVO FUNCTIONS
// =======================================================

int percentToMicroseconds(int percent) {
  percent = constrain(percent, 0, 100);
  return map(percent, 0, 100, SERVO_MIN_US, SERVO_MAX_US);
}

int mapInputToBaseThrottleOutput(int inputPercent) {
  inputPercent = constrain(inputPercent, 0, 100);

  idleThrottlePercent = constrain(idleThrottlePercent, 0, 100);
  maxThrottlePercent = constrain(maxThrottlePercent, 0, 100);

  if (maxThrottlePercent < idleThrottlePercent) {
    maxThrottlePercent = idleThrottlePercent;
  }

  return map(inputPercent, 0, 100, idleThrottlePercent, maxThrottlePercent);
}

void applyThrottle() {
  throttleBaseOutputPercent = mapInputToBaseThrottleOutput(throttleInputPercent);

  for (int i = 0; i < 4; i++) {
    servoTrimPercent[i] = constrain(servoTrimPercent[i], MIN_SERVO_TRIM, MAX_SERVO_TRIM);

    servoOutputPercent[i] = throttleBaseOutputPercent + servoTrimPercent[i];
    servoOutputPercent[i] = constrain(servoOutputPercent[i], 0, 100);

    int servoMicros = percentToMicroseconds(servoOutputPercent[i]);
    servos[i].writeMicroseconds(servoMicros);
  }
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

void setServoTrims(int t1, int t2, int t3, int t4) {
  servoTrimPercent[0] = constrain(t1, MIN_SERVO_TRIM, MAX_SERVO_TRIM);
  servoTrimPercent[1] = constrain(t2, MIN_SERVO_TRIM, MAX_SERVO_TRIM);
  servoTrimPercent[2] = constrain(t3, MIN_SERVO_TRIM, MAX_SERVO_TRIM);
  servoTrimPercent[3] = constrain(t4, MIN_SERVO_TRIM, MAX_SERVO_TRIM);

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
  char json[600];

  snprintf(
    json,
    sizeof(json),
    "{"
    "\"rpm\":%.1f,"
    "\"throttleInput\":%d,"
    "\"throttleBaseOutput\":%d,"
    "\"idleThrottle\":%d,"
    "\"maxThrottle\":%d,"
    "\"servo1Trim\":%d,"
    "\"servo2Trim\":%d,"
    "\"servo3Trim\":%d,"
    "\"servo4Trim\":%d,"
    "\"servo1Output\":%d,"
    "\"servo2Output\":%d,"
    "\"servo3Output\":%d,"
    "\"servo4Output\":%d,"
    "\"engineRunEnabled\":%s"
    "}",
    currentRPM,
    throttleInputPercent,
    throttleBaseOutputPercent,
    idleThrottlePercent,
    maxThrottlePercent,
    servoTrimPercent[0],
    servoTrimPercent[1],
    servoTrimPercent[2],
    servoTrimPercent[3],
    servoOutputPercent[0],
    servoOutputPercent[1],
    servoOutputPercent[2],
    servoOutputPercent[3],
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

void handleSetTrims() {
  if (
    server.hasArg("t1") &&
    server.hasArg("t2") &&
    server.hasArg("t3") &&
    server.hasArg("t4")
  ) {
    int t1 = server.arg("t1").toInt();
    int t2 = server.arg("t2").toInt();
    int t3 = server.arg("t3").toInt();
    int t4 = server.arg("t4").toInt();

    setServoTrims(t1, t2, t3, t4);
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
// Throttle Sweep
// =======================================================

void throttleSweep() {
  // Set throttle level to array defined values
  //--Todo - Make read array from JSON
  //--Todo - Make for loop to make arrays
  //int throttleSize = 10; //Step size in percent
  //int throttleSteps = 100/10; //Amount of steps
  
  //for loop to fill in the array

  // WARNING
  // This code only uses servo 1

  


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

  for (int i = 0; i < 4; i++) {
    servos[i].setPeriodHertz(50);
    servos[i].attach(servoPins[i], SERVO_MIN_US, SERVO_MAX_US);
  }

  setThrottleInput(0);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);

  IPAddress ip = WiFi.softAPIP();

  Serial.println();
  Serial.println("======================================");
  Serial.println("ESP32 Stinger 4-Servo RPM Controller Started");
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
  server.on("/setTrims", handleSetTrims);
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

    Serial.print(" | Base: ");
    Serial.print(throttleBaseOutputPercent);
    Serial.print("%");

    Serial.print(" | S1: ");
    Serial.print(servoOutputPercent[0]);
    Serial.print("%");

    Serial.print(" | S2: ");
    Serial.print(servoOutputPercent[1]);
    Serial.print("%");

    Serial.print(" | S3: ");
    Serial.print(servoOutputPercent[2]);
    Serial.print("%");

    Serial.print(" | S4: ");
    Serial.print(servoOutputPercent[3]);
    Serial.print("%");

    Serial.print(" | Engine: ");
    Serial.println(engineRunEnabled ? "RUN ENABLED" : "KILLED");
  }
}