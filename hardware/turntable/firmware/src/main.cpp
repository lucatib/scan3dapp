// Scan3D turntable: a 28BYJ-48 on a ULN2003 board turns the platter through a 16:64 gear pair.
//
// Button: a short press starts or stops, a double press reverses, holding it speeds up to full speed (and the speed
// reached stays when it is released).
// The same text commands, one per line, over USB serial (115200) and over Bluetooth LE: Nordic UART Service, device
// "Scan3D Turntable"; write commands to RX 6E400002-..., replies are notified on TX 6E400003-... in chunks of up to
// 20 bytes, each line ending in \n.
//   run                  turn continuously
//   stop                 ramp down and stop
//   speed <s>            seconds per platter turn (20..600), also while turning
//   accel <deg/s2>       ramp acceleration of the platter (1..100), for starting, stopping and speed changes
//   dir <1|-1>           direction for run (turn and goto set their own)
//   reverse              the other direction
//   turn <deg>           turn by this angle and stop (negative: the other way); prints "done ..." when there
//   goto <deg> [1|-1]    turn to this angle and stop, the shorter way or in the given direction; "done ..." when there
//   zero                 make the current angle 0
//   status
// Every reply is one line, starting with "ok", "err" or "done"; "event" lines report changes made with the button.
// Lines carry state=, dir=, speed= (s per turn), accel= (deg/s2) and angle= (degrees, 0..360).

#include <Arduino.h>
#include <BLEDevice.h>
#include <math.h>
#include <stdarg.h>

namespace {

constexpr uint8_t COIL_PINS[4] = {4, 5, 6, 7};  // ULN2003 IN1..IN4
constexpr uint8_t BUTTON_PIN = 15;              // to GND, internal pull-up

// The 28BYJ-48 gearbox is 63.684:1, not 64: 4075.77 half steps per motor turn. The gears add 64/16.
constexpr double HALF_STEPS_PER_TURN = 4075.7728 * 64.0 / 16.0;
constexpr double DEG_PER_HALF_STEP = 360.0 / HALF_STEPS_PER_TURN;
constexpr float MIN_SECONDS_PER_TURN = 20;   // ~815 half steps/s, about what the motor manages at 5 V
constexpr float MAX_SECONDS_PER_TURN = 600;
constexpr float START_RATE = 150;            // half steps/s, from standstill
constexpr float MIN_ACCEL_DEG = 1, MAX_ACCEL_DEG = 100;  // platter deg/s^2

constexpr uint32_t DEBOUNCE_MS = 25;
constexpr uint32_t DOUBLE_PRESS_MS = 350;  // a second press this soon after the first: reverse
constexpr uint32_t HOLD_MS = 600;          // held this long: speed up
constexpr float HOLD_ACCEL_DEG = 3;        // platter deg/s^2 while held: standstill to full speed in about 6 s

constexpr uint8_t HALF_STEP[8] = {0b0001, 0b0011, 0b0010, 0b0110, 0b0100, 0b1100, 0b1000, 0b1001};

// Bluetooth LE: the Nordic UART Service, so any BLE terminal app can drive it too.
constexpr char DEVICE_NAME[] = "Scan3D Turntable";
constexpr char NUS_SERVICE[] = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
constexpr char NUS_RX[] = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";  // the phone writes commands
constexpr char NUS_TX[] = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";  // the turntable notifies replies
constexpr size_t LINE_LEN = 64;
constexpr int BLE_CHUNK = 20;  // fits the default ATT MTU, so the phone need not negotiate a bigger one

enum class Mode { Stopped, Running, Moving };

Mode mode = Mode::Stopped;
int direction = 1;
float secondsPerTurn = 40;
float acceleration = 600; // half steps/s^2: 13 deg/s^2 on the platter, full speed at 40 s/turn in about 0.7 s
float rate = 0;          // half steps/s now, 0 when stopped
int64_t position = 0;    // half steps from zero, signed
int64_t remaining = 0;   // Moving: half steps still to go
bool reportDone = false; // Moving because of "turn" or "goto", not a ramp-down
uint8_t phase = 0;
uint32_t lastStepUs = 0;

struct Line {
  char text[LINE_LEN];
};
QueueHandle_t bleLines;  // BLE task -> loop(): the motor state is only touched in loop()
BLECharacteristic* txChar = nullptr;
volatile bool bleConnected = false;

// One line out to the serial port and, when a phone is connected, to BLE.
void emit(const char* format, ...) {
  char text[160];
  va_list args;
  va_start(args, format);
  int n = vsnprintf(text, sizeof(text) - 1, format, args);
  va_end(args);
  if (n < 0) return;
  n = min(n, (int)sizeof(text) - 2);
  text[n++] = '\n';
  text[n] = '\0';
  Serial.print(text);
  if (!bleConnected || !txChar) return;
  for (int i = 0; i < n; i += BLE_CHUNK) {
    txChar->setValue((uint8_t*)text + i, min(BLE_CHUNK, n - i));
    txChar->notify();
  }
}

double angle() {
  double a = fmod(position * DEG_PER_HALF_STEP, 360.0);
  return a < 0 ? a + 360.0 : a;
}

void writeCoils(uint8_t bits) {
  for (int i = 0; i < 4; i++) digitalWrite(COIL_PINS[i], (bits >> i) & 1);
}

void showState() {
#ifdef RGB_BUILTIN
  if (mode == Mode::Stopped) rgbLedWrite(RGB_BUILTIN, 0, 0, 0);
  else rgbLedWrite(RGB_BUILTIN, 0, direction > 0 ? 24 : 0, direction > 0 ? 0 : 24);
#endif
}

void halt() {
  mode = Mode::Stopped;
  rate = 0;
  writeCoils(0);  // the gearbox holds the platter; the coils would only get warm
  showState();
}

float targetRate() { return HALF_STEPS_PER_TURN / secondsPerTurn; }

float stopDistance() { return (rate * rate - START_RATE * START_RATE) / (2 * acceleration); }

void start(Mode m) {
  if (mode == Mode::Stopped) {
    rate = START_RATE;
    lastStepUs = micros();
    writeCoils(HALF_STEP[phase]);
  }
  mode = m;
  showState();
}

void requestStop() {
  if (mode == Mode::Stopped) return;
  if (rate <= START_RATE) return halt();
  mode = Mode::Moving;
  remaining = (int64_t)ceilf(stopDistance()) + 1;
  reportDone = false;
}

void printStatus(const char* prefix) {
  const char* m = mode == Mode::Stopped ? "stopped" : mode == Mode::Running ? "running" : "moving";
  emit("%s state=%s dir=%d speed=%.1f accel=%.1f angle=%.2f", prefix, m, direction, secondsPerTurn,
       acceleration * DEG_PER_HALF_STEP, angle());
}

void serviceMotor() {
  if (mode == Mode::Stopped) return;
  uint32_t now = micros();
  uint32_t period = (uint32_t)(1e6f / rate);
  if (now - lastStepUs < period) return;
  lastStepUs = (now - lastStepUs > 4 * period) ? now : lastStepUs + period;

  phase = (phase + direction) & 7;
  writeCoils(HALF_STEP[phase]);
  position += direction;

  float target = targetRate();
  if (mode == Mode::Moving) {
    if (--remaining <= 0) {
      halt();
      if (reportDone) printStatus("done");
      return;
    }
    if (remaining <= stopDistance()) target = START_RATE;
  }
  float dv = acceleration / rate;  // per step: dv/dt = a, dt = 1/rate
  rate = rate < target ? fminf(target, rate + dv) : fmaxf(target, rate - dv);
}

void reverse() {
  direction = -direction;
  if (mode != Mode::Stopped) rate = START_RATE;  // the gearbox makes the platter's inertia negligible
  showState();
}

// Short press: start or stop (acted on once no second press follows). Double press: reverse.
// Hold: turn, speeding up at HOLD_ACCEL_DEG up to full speed; on release it keeps the speed it reached.
void serviceButton() {
  static bool stable = true, last = true;  // released = HIGH
  static uint32_t changedMs = 0, pressedMs = 0, releasedMs = 0, holdMs = 0;
  static bool holding = false, pendingPress = false, secondPress = false;
  static float holdRate = 0;
  bool level = digitalRead(BUTTON_PIN);
  uint32_t now = millis();
  if (level != last) {
    last = level;
    changedMs = now;
  }
  if (level != stable && now - changedMs >= DEBOUNCE_MS) {
    stable = level;
    if (!stable) {  // pressed
      pressedMs = now;
      secondPress = pendingPress && now - releasedMs < DOUBLE_PRESS_MS;
      pendingPress = false;
    } else if (holding) {  // released after a hold
      holding = false;
      printStatus("event");
    } else if (secondPress) {
      secondPress = false;
      reverse();
      printStatus("event");
    } else {
      pendingPress = true;
      releasedMs = now;
    }
  }
  if (pendingPress && stable && now - releasedMs >= DOUBLE_PRESS_MS) {
    pendingPress = false;
    if (mode == Mode::Stopped) start(Mode::Running);
    else requestStop();
    printStatus("event");
  }
  if (!stable && now - pressedMs >= HOLD_MS) {
    if (!holding) {
      holding = true;
      secondPress = false;
      holdMs = now;
      holdRate = mode == Mode::Stopped ? START_RATE : fmaxf(rate, START_RATE);
      start(Mode::Running);
      printStatus("event");
    }
    float maxRate = HALF_STEPS_PER_TURN / MIN_SECONDS_PER_TURN;
    holdRate = fminf(maxRate, holdRate + HOLD_ACCEL_DEG / DEG_PER_HALF_STEP * (now - holdMs) / 1000.0f);
    holdMs = now;
    secondsPerTurn = HALF_STEPS_PER_TURN / holdRate;
  }
}

// Turn by this many degrees (negative: the other way) and stop there.
void turnBy(double degrees) {
  int64_t steps = llround(fabs(degrees) / DEG_PER_HALF_STEP);
  if (steps == 0) return printStatus("done");
  if ((degrees < 0) != (direction < 0)) reverse();
  remaining = steps;
  reportDone = true;
  start(Mode::Moving);
  printStatus("ok");
}

void handleCommand(char* line) {
  float value, way;
  if (strcmp(line, "run") == 0) {
    start(Mode::Running);
    printStatus("ok");
  } else if (strcmp(line, "stop") == 0) {
    requestStop();
    printStatus("ok");
  } else if (sscanf(line, "speed %f", &value) == 1) {
    secondsPerTurn = constrain(value, MIN_SECONDS_PER_TURN, MAX_SECONDS_PER_TURN);
    printStatus("ok");
  } else if (sscanf(line, "accel %f", &value) == 1) {
    acceleration = constrain(value, MIN_ACCEL_DEG, MAX_ACCEL_DEG) / DEG_PER_HALF_STEP;
    printStatus("ok");
  } else if (sscanf(line, "dir %f", &value) == 1) {
    if ((value < 0) != (direction < 0)) reverse();
    printStatus("ok");
  } else if (strcmp(line, "reverse") == 0) {
    reverse();
    printStatus("ok");
  } else if (sscanf(line, "turn %f", &value) == 1) {
    turnBy(value);
  } else if (int n = sscanf(line, "goto %f %f", &value, &way); n >= 1) {
    double delta = fmod(value - angle(), 360.0);  // -360..360
    if (n == 2 && way > 0) delta = delta < 0 ? delta + 360 : delta;
    else if (n == 2 && way < 0) delta = delta > 0 ? delta - 360 : delta;
    else delta = delta > 180 ? delta - 360 : delta <= -180 ? delta + 360 : delta;
    turnBy(delta);
  } else if (strcmp(line, "zero") == 0) {
    position = 0;
    printStatus("ok");
  } else if (strcmp(line, "status") == 0) {
    printStatus("ok");
  } else if (line[0] != '\0') {
    emit("err unknown command: %s", line);
  }
}

void serviceSerial() {
  static char line[LINE_LEN];
  static size_t length = 0;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      line[length] = '\0';
      length = 0;
      handleCommand(line);
    } else if (length < sizeof(line) - 1) {
      line[length++] = c;
    }
  }
}

void serviceBle() {
  Line line;
  while (xQueueReceive(bleLines, &line, 0) == pdTRUE) handleCommand(line.text);
}

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*) override { bleConnected = true; }
  void onDisconnect(BLEServer* server) override {
    bleConnected = false;
    server->startAdvertising();  // let the phone, or another one, connect again
  }
};

// Runs in the BLE task: split what was written into lines and hand them to loop().
class RxCallbacks : public BLECharacteristicCallbacks {
  Line pending{};
  size_t length = 0;

  void flush() {
    pending.text[length] = '\0';
    length = 0;
    if (pending.text[0] != '\0') xQueueSend(bleLines, &pending, 0);
  }

  void onWrite(BLECharacteristic* characteristic) override {
    String value = characteristic->getValue();
    for (size_t i = 0; i < value.length(); i++) {
      char c = value[i];
      if (c == '\n' || c == '\r') flush();
      else if (length < LINE_LEN - 1) pending.text[length++] = c;
    }
    if (length > 0) flush();  // a write without a newline is a whole command too
  }
};

void setupBle() {
  bleLines = xQueueCreate(8, sizeof(Line));
  BLEDevice::init(DEVICE_NAME);
  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());
  BLEService* service = server->createService(NUS_SERVICE);
  txChar = service->createCharacteristic(NUS_TX, BLECharacteristic::PROPERTY_NOTIFY);  // NimBLE adds the 2902
  BLECharacteristic* rx =
      service->createCharacteristic(NUS_RX, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rx->setCallbacks(new RxCallbacks());
  service->start();
  BLEAdvertising* advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(NUS_SERVICE);
  advertising->setScanResponse(true);  // the 128-bit UUID fills the packet; the name goes in the scan response
  BLEDevice::startAdvertising();
}

}  // namespace

void setup() {
  for (uint8_t pin : COIL_PINS) pinMode(pin, OUTPUT);
  writeCoils(0);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Serial.begin(115200);
  setupBle();
  showState();
  printStatus("ok ready");
}

void loop() {
  serviceMotor();
  serviceButton();
  serviceSerial();
  serviceBle();
}
