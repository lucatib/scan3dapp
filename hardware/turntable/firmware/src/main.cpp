// Scan3D turntable: a 28BYJ-48 on a ULN2003 board turns the platter through a 16:64 gear pair.
//
// Button: a short press starts or stops, a long press reverses.
// Serial, 115200, one command per line:
//   run              turn continuously
//   stop             ramp down and stop
//   speed <s>        seconds per platter turn (20..600)
//   dir <1|-1>       direction for run
//   turn <deg>       turn by this angle and stop (negative: the other way); prints "done ..." when there
//   zero             make the current angle 0
//   status
// Every reply is one line, starting with "ok", "err" or "done".

#include <Arduino.h>
#include <math.h>

namespace {

constexpr uint8_t COIL_PINS[4] = {4, 5, 6, 7};  // ULN2003 IN1..IN4
constexpr uint8_t BUTTON_PIN = 15;              // to GND, internal pull-up

// The 28BYJ-48 gearbox is 63.684:1, not 64: 4075.77 half steps per motor turn. The gears add 64/16.
constexpr double HALF_STEPS_PER_TURN = 4075.7728 * 64.0 / 16.0;
constexpr float MIN_SECONDS_PER_TURN = 20;   // ~815 half steps/s, about what the motor manages at 5 V
constexpr float MAX_SECONDS_PER_TURN = 600;
constexpr float START_RATE = 150;            // half steps/s, from standstill
constexpr float ACCELERATION = 600;          // half steps/s^2

constexpr uint32_t DEBOUNCE_MS = 25;
constexpr uint32_t LONG_PRESS_MS = 800;

constexpr uint8_t HALF_STEP[8] = {0b0001, 0b0011, 0b0010, 0b0110, 0b0100, 0b1100, 0b1000, 0b1001};

enum class Mode { Stopped, Running, Moving };

Mode mode = Mode::Stopped;
int direction = 1;
float secondsPerTurn = 40;
float rate = 0;          // half steps/s now, 0 when stopped
int64_t position = 0;    // half steps from zero, signed
int64_t remaining = 0;   // Moving: half steps still to go
bool reportDone = false; // Moving because of "turn", not a ramp-down
uint8_t phase = 0;
uint32_t lastStepUs = 0;

double angle() {
  double a = fmod(position / HALF_STEPS_PER_TURN * 360.0, 360.0);
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

float stopDistance() { return (rate * rate - START_RATE * START_RATE) / (2 * ACCELERATION); }

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
  Serial.printf("%s state=%s dir=%d speed=%.1f angle=%.2f\n", prefix, m, direction, secondsPerTurn, angle());
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
  float dv = ACCELERATION / rate;  // per step: dv/dt = a, dt = 1/rate
  rate = rate < target ? fminf(target, rate + dv) : fmaxf(target, rate - dv);
}

void reverse() {
  direction = -direction;
  if (mode != Mode::Stopped) rate = START_RATE;  // the gearbox makes the platter's inertia negligible
  showState();
}

void serviceButton() {
  static bool stable = true, last = true;  // released = HIGH
  static uint32_t changedMs = 0, pressedMs = 0;
  static bool longDone = false;
  bool level = digitalRead(BUTTON_PIN);
  uint32_t now = millis();
  if (level != last) {
    last = level;
    changedMs = now;
  }
  if (level != stable && now - changedMs >= DEBOUNCE_MS) {
    stable = level;
    if (!stable) {
      pressedMs = now;
      longDone = false;
    } else if (!longDone) {
      if (mode == Mode::Stopped) start(Mode::Running);
      else requestStop();
    }
  }
  if (!stable && !longDone && now - pressedMs >= LONG_PRESS_MS) {
    longDone = true;
    reverse();
  }
}

void handleCommand(char* line) {
  float value;
  if (strcmp(line, "run") == 0) {
    start(Mode::Running);
    printStatus("ok");
  } else if (strcmp(line, "stop") == 0) {
    requestStop();
    printStatus("ok");
  } else if (sscanf(line, "speed %f", &value) == 1) {
    secondsPerTurn = constrain(value, MIN_SECONDS_PER_TURN, MAX_SECONDS_PER_TURN);
    printStatus("ok");
  } else if (sscanf(line, "dir %f", &value) == 1) {
    if ((value < 0) != (direction < 0)) reverse();
    printStatus("ok");
  } else if (sscanf(line, "turn %f", &value) == 1) {
    int64_t steps = llround(fabs(value) / 360.0 * HALF_STEPS_PER_TURN);
    if (steps == 0) return printStatus("done");
    if ((value < 0) != (direction < 0)) reverse();
    remaining = steps;
    reportDone = true;
    start(Mode::Moving);
    printStatus("ok");
  } else if (strcmp(line, "zero") == 0) {
    position = 0;
    printStatus("ok");
  } else if (strcmp(line, "status") == 0) {
    printStatus("ok");
  } else if (line[0] != '\0') {
    Serial.printf("err unknown command: %s\n", line);
  }
}

void serviceSerial() {
  static char line[64];
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

}  // namespace

void setup() {
  for (uint8_t pin : COIL_PINS) pinMode(pin, OUTPUT);
  writeCoils(0);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Serial.begin(115200);
  showState();
  printStatus("ok ready");
}

void loop() {
  serviceMotor();
  serviceButton();
  serviceSerial();
}
