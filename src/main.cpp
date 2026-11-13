#include <Arduino.h>
#include <HX711.h>
#include <Preferences.h>

#include "pins.h"

// ---- Tuning -----------------------------------------------------------------

// Steps per 180 degrees of the gearbox output = (full steps per rev * microstepping * gear ratio) / 2.
// 1.8 deg motors have 200 full steps/rev; set MICROSTEPS to match MS1-MS3 jumpers.
constexpr int FULL_STEPS_PER_REV = 200;
constexpr int MICROSTEPS = 1;
constexpr int GEAR_RATIO = 29; // both motors drive through a 29:1 reduction
constexpr long STEPS_180 = (long)FULL_STEPS_PER_REV * MICROSTEPS * GEAR_RATIO / 2;
constexpr long STEPS_360 = STEPS_180 * 2;

// How far the lid opens (spin_lid and the automatic sequence).
constexpr int LID_OPEN_DEGREES = 25;
constexpr long LID_OPEN_STEPS = STEPS_180 * LID_OPEN_DEGREES / 180;

// Position adjust buttons nudge the output shaft by this much per press.
constexpr int JOG_DEGREES = 5;
constexpr long JOG_STEPS = STEPS_180 * JOG_DEGREES / 180;

// The lid opens in JOG_DEGREES increments with a pause between each (timing
// "lid_step_pause" below); one long move doesn't have the torque to lift it.

// Step pulse period in microseconds. Starts slow and ramps to cruise to avoid stalls.
constexpr unsigned STEP_US_START = 4000;
constexpr unsigned STEP_US_CRUISE = 750;      // pinion
constexpr unsigned LID_STEP_US_CRUISE = 1500;
constexpr unsigned LIFT_LID_STEP_US_CRUISE = LID_STEP_US_CRUISE * 2; // Lift Lid button: half speed
constexpr long RAMP_STEPS = 60;

// Flip these if a motor turns the wrong way for lid "open" / pinion "forward".
constexpr bool LID_OPEN_DIR = HIGH;
constexpr bool ARM_FORWARD_DIR = LOW;

// HX711 calibration factor in raw counts per gram (what HX711::set_scale takes
// when calibrating against a known mass in grams).
constexpr float CALIBRATION_FACTOR = 635.0f;

// Candy detection. A grab shows up as a spike (hand in the bowl), then the
// reading rings and settles. Once settled, the drop from the pre-grab weight
// is how much was taken; more than CANDY_G means more than one candy.
constexpr float CANDY_G = 10.0f;
constexpr float SPIKE_G = 7.0f;             // deviation from rest that starts a grab
constexpr int SPIKE_CONFIRM_READS = 2;      // consecutive readings beyond SPIKE_G
constexpr float STABLE_BAND_G = 3.0f;       // max - min within the window to count as settled
constexpr int STABLE_WINDOW = 5;            // readings (~0.5 s at the HX711's 10 Hz)
constexpr unsigned long REFILL_ACCEPT_MS = 5000; // settled heavier than before for this long = refill
constexpr float REST_TRACK_RATE = 0.02f;    // slow drift tracking while idle

// The pinion's full turn stops partway (pinion_pause timing), then finishes the turn.
// The first leg launches the arm; the rest of the turn resets the pinion.
constexpr int PINION_PAUSE_DEGREES = 90;

// Spool: reels the launched arm back in by its tether, then lets the same length
// back out once the lid is shut, so the tether is slack for the next launch.
// PLACEHOLDERS until the spool is measured: set the gear ratio to the spool
// motor's gearbox (1 if direct drive) and REEL_DEGREES to however far the spool
// must turn to pull the arm fully home.
constexpr int SPOOL_GEAR_RATIO = 1;
constexpr long SPOOL_STEPS_360 = (long)FULL_STEPS_PER_REV * MICROSTEPS * SPOOL_GEAR_RATIO;
constexpr long SPOOL_REEL_DEGREES = 720;
constexpr long SPOOL_REEL_STEPS = SPOOL_STEPS_360 * SPOOL_REEL_DEGREES / 360;
// The spool turns many times per launch, so its adjust buttons move further
// than the lid/pinion's 5 degrees. Keep the web page's button labels in step.
constexpr int SPOOL_JOG_DEGREES = 10;
constexpr long SPOOL_JOG_STEPS = SPOOL_STEPS_360 * SPOOL_JOG_DEGREES / 360;
constexpr unsigned SPOOL_STEP_US_CRUISE = 1000;
constexpr bool SPOOL_IN_DIR = HIGH; // flip if "Spool In" lets the tether out

// ---- Sequence timings (ms) ----------------------------------------------------
// Adjustable from the web app and saved to flash (NVS), so they survive power
// cycles. The values here are only the defaults used before anything is saved.
struct Timing {
  const char *key;          // name used on the serial protocol and as the NVS key
  unsigned long value;
  unsigned long defaultValue;
};
Timing timings[] = {
  {"lid_step_pause", 25, 25},     // pause between lid-opening increments
  {"lid_to_arm", 100, 100},       // lid open -> pinion starts
  {"pinion_pause", 1000, 1000},   // pause partway through the pinion turn
  {"arm_hold", 5000, 5000},       // pinion reset -> spool reels the arm in
  {"spool_to_lid", 250, 250},     // arm reeled in -> lid closes
  {"lid_to_unspool", 250, 250},   // lid closed -> spool lets the tether back out
  {"cooldown", 1500, 1500},       // let things settle before re-reading the rest weight
};
constexpr int NUM_TIMINGS = sizeof(timings) / sizeof(timings[0]);
constexpr unsigned long TIMING_MAX_MS = 60000;
enum { T_LID_STEP_PAUSE, T_LID_TO_ARM, T_PINION_PAUSE, T_ARM_HOLD, T_SPOOL_TO_LID, T_LID_TO_UNSPOOL, T_COOLDOWN };
#define TIMING(i) (timings[i].value)

// -----------------------------------------------------------------------------

HX711 scale;
long baseline = 0;
int spikeCount = 0;

float restGrams = 0;      // settled weight before the current grab
bool settling = false;    // a grab is in progress, waiting for the reading to settle
bool restPending = false; // take the next reading as the rest weight
unsigned long lastUnsettledMs = 0;
float window[STABLE_WINDOW];
int windowLen = 0;
int windowIdx = 0;

// Streaming to the web app. Serial protocol:
//   in:  "start" / "stop"
//        "spin_lid"                (+LID_OPEN_DEGREES)
//        "lift_lid"                (to the open position, LID_OPEN_DEGREES from start)
//        "reset_lid"               (back to start position)
//        "run_sequence"            (run the full candy-taken sequence now)
//        "spin_arm"                (one full 360 degree turn forward, ends where it started)
//        "spool_in"                (reel the arm in, to SPOOL_REEL_DEGREES from start)
//        "spool_out"               (back to the spool's start position)
//        "jog_lid_fwd" / "jog_lid_back" / "jog_arm_fwd" / "jog_arm_back"
//        "jog_spool_fwd" / "jog_spool_back"   (SPOOL_JOG_DEGREES, fwd = reel in)
//                                  (nudge JOG_DEGREES; shifts the start position with it)
//        "step_fire" / "step_reel" / "step_close" / "step_unspool"
//                                  (one sequence phase each: the web page's Machine State steps)
//        "get_state"               (replies with an "F,state" line)
//        "get_timings"             (replies with a "C" line)
//        "set_timings <key>=<ms>,<key>=<ms>,..."
//                                  (saves to flash, replies with a "C" line)
//   out: "D,<seconds>,<grams>" per sample, "S,<seconds>,<grams>" on spike,
//        "T,<seconds>,<grams taken>" when more than one candy was taken
//        "C,<key>=<ms>,<key>=<ms>,..." current sequence timings
//        "M,..." motor move durations in us, sent after every "C" (see reportMotion)
//        "F,<phase>" as a sequence phase starts: trigger, fire, reel, close, unspool;
//        "F,done" when the sequence or step ends; "F,refused,<why>";
//        "F,state,lid=<0|1>,spool=<0|1>" (see reportState)
bool sampling = false;
unsigned long samplingStartMs = 0;

// Lid steps away from its start position, positive = open. The pinion only
// ever makes full turns, so it always ends back at its start position.
long lidPos = 0;
// Spool steps away from its start (slack tether) position, positive = reeled in.
long spoolPos = 0;

void driversEnabled(bool on) {
  digitalWrite(PIN_DRV_EN, on ? LOW : HIGH);
}

// Blocking move with a simple linear accel/decel ramp.
void moveSteps(int stepPin, int dirPin, bool dir, long steps, unsigned cruiseUs) {
  digitalWrite(dirPin, dir);
  delayMicroseconds(5); // DIR setup time

  for (long i = 0; i < steps; i++) {
    long fromEdge = min(i, steps - 1 - i);
    unsigned period = cruiseUs;
    if (fromEdge < RAMP_STEPS) {
      period = STEP_US_START - (STEP_US_START - cruiseUs) * fromEdge / RAMP_STEPS;
    }
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(period - 5);
  }
}

// Signed move that keeps track of position. Positive = forwardDir.
void moveBy(int stepPin, int dirPin, bool forwardDir, long &pos, long steps, unsigned cruiseUs) {
  if (steps == 0) return;
  moveSteps(stepPin, dirPin, steps > 0 ? forwardDir : !forwardDir, labs(steps), cruiseUs);
  pos += steps;
}

void moveLid(long steps, unsigned cruiseUs = LID_STEP_US_CRUISE) { moveBy(PIN_LID_STEP, PIN_LID_DIR, LID_OPEN_DIR, lidPos, steps, cruiseUs); }

void moveLidInIncrements(long steps, unsigned cruiseUs = LID_STEP_US_CRUISE) {
  long remaining = labs(steps);
  while (remaining > 0) {
    long chunk = min(remaining, JOG_STEPS);
    moveLid(steps > 0 ? chunk : -chunk, cruiseUs);
    remaining -= chunk;
    if (remaining > 0) delay(TIMING(T_LID_STEP_PAUSE));
  }
}
constexpr long PINION_FIRST_LEG = STEPS_180 * PINION_PAUSE_DEGREES / 180;
void launchArm() { moveSteps(PIN_ARM_STEP, PIN_ARM_DIR, ARM_FORWARD_DIR, PINION_FIRST_LEG, STEP_US_CRUISE); }
void resetPinion() { moveSteps(PIN_ARM_STEP, PIN_ARM_DIR, ARM_FORWARD_DIR, STEPS_360 - PINION_FIRST_LEG, STEP_US_CRUISE); }

void spinArm() {
  launchArm();
  delay(TIMING(T_PINION_PAUSE));
  resetPinion();
}

void moveSpool(long steps) { moveBy(PIN_SPOOL_STEP, PIN_SPOOL_DIR, SPOOL_IN_DIR, spoolPos, steps, SPOOL_STEP_US_CRUISE); }
void spoolIn() { moveSpool(SPOOL_REEL_STEPS - spoolPos); }
void spoolOut() { moveSpool(-spoolPos); }

// Anything away from its start position needs the drivers to keep holding it.
bool atHome() { return lidPos == 0 && spoolPos == 0; }

void rebaseline() {
  baseline = scale.read_average(10);
  spikeCount = 0;
  Serial.printf("baseline = %ld\n", baseline);
}

float readGrams() {
  return (scale.read() - baseline) / CALIBRATION_FACTOR;
}

void clearWindow() {
  windowLen = 0;
  windowIdx = 0;
}

void pushWindow(float grams) {
  window[windowIdx] = grams;
  windowIdx = (windowIdx + 1) % STABLE_WINDOW;
  if (windowLen < STABLE_WINDOW) windowLen++;
}

// True once the window is full and its spread is within STABLE_BAND_G.
bool windowSettled(float &mean) {
  if (windowLen < STABLE_WINDOW) return false;
  float lo = window[0], hi = window[0], sum = 0;
  for (int i = 0; i < STABLE_WINDOW; i++) {
    lo = min(lo, window[i]);
    hi = max(hi, window[i]);
    sum += window[i];
  }
  mean = sum / STABLE_WINDOW;
  return hi - lo <= STABLE_BAND_G;
}

// Re-read the resting weight (e.g. after the motors have shaken things up).
void resetRest() {
  restGrams = (scale.read_average(10) - baseline) / CALIBRATION_FACTOR;
  settling = false;
  spikeCount = 0;
  clearWindow();
}

// ---- Sequence phases ----------------------------------------------------------
// The sequence is four motor phases, which are also the web page's Machine State
// transitions. Each phase announces itself with "F,<phase>" as it starts, so the
// page animates in step with the motors whether the phase came from a grab, from
// run_sequence, or from the page's own step_* command. The pauses each phase
// starts with are the page's too; arm_hold belongs to run_sequence alone (the
// page's step_reel reels at once).
void announce(const char *phase) { Serial.printf("F,%s\n", phase); }

// "F,state,lid=<0|1>,spool=<0|1>": lid open?, tether reeled in? Lets the page
// pick its state on connect and after the manual buttons.
void reportState() { Serial.printf("F,state,lid=%d,spool=%d\n", lidPos > 0 ? 1 : 0, spoolPos > 0 ? 1 : 0); }

// Absolute moves throughout, so each phase still lands right if the test
// buttons left the lid open or the spool reeled in.
void phaseFire() {    // lid opens, pinion shoots the arm out and re-cocks
  announce("fire");
  moveLidInIncrements(LID_OPEN_STEPS - lidPos);
  delay(TIMING(T_LID_TO_ARM));
  launchArm();
  delay(TIMING(T_PINION_PAUSE));
  resetPinion();
}
void phaseReel() {    // reel the arm back in through the open lid
  announce("reel");
  spoolIn();
}
void phaseClose() {   // close the lid over the reeled-in arm
  announce("close");
  delay(TIMING(T_SPOOL_TO_LID));
  moveLid(-lidPos);
}
void phaseUnspool() { // let the tether out so the next launch is free, then settle
  announce("unspool");
  delay(TIMING(T_LID_TO_UNSPOOL));
  spoolOut();
  driversEnabled(!atHome());
  delay(TIMING(T_COOLDOWN));
  resetRest();
}

void runSequence() {
  Serial.println("Load spike -> running sequence");
  digitalWrite(PIN_STATUS_LED, HIGH);
  driversEnabled(true);
  phaseFire();
  delay(TIMING(T_ARM_HOLD));
  phaseReel();
  phaseClose();
  phaseUnspool();
  driversEnabled(false);
  digitalWrite(PIN_STATUS_LED, LOW);
  announce("done");
}

// One phase on its own, for the page's Next button. Firing with the tether
// reeled in would launch the arm against its own string, so that is refused.
void runPhase(const String &cmd) {
  if (cmd == "step_fire" && spoolPos != 0) {
    Serial.println("F,refused,the tether is reeled in: let it out first");
    return;
  }
  digitalWrite(PIN_STATUS_LED, HIGH);
  driversEnabled(true);
  if (cmd == "step_fire") phaseFire();
  else if (cmd == "step_reel") phaseReel();
  else if (cmd == "step_close") phaseClose();
  else phaseUnspool();
  driversEnabled(!atHome());
  digitalWrite(PIN_STATUS_LED, LOW);
  announce("done");
}

Preferences prefs;

void loadTimings() {
  prefs.begin("timings", true);
  for (auto &t : timings) t.value = min((unsigned long)prefs.getULong(t.key, t.defaultValue), TIMING_MAX_MS);
  prefs.end();
}

// How long moveSteps() takes for a move, in microseconds: the same per-step
// periods it waits, plus its DIR setup time. Kept next to the "M" report so the
// web page's sequence timeline shows this board's real motor times.
unsigned long moveTimeUs(long steps, unsigned cruiseUs) {
  unsigned long us = 5;
  for (long i = 0; i < steps; i++) {
    long fromEdge = min(i, steps - 1 - i);
    unsigned period = cruiseUs;
    if (fromEdge < RAMP_STEPS) period = STEP_US_START - (STEP_US_START - cruiseUs) * fromEdge / RAMP_STEPS;
    us += period;
  }
  return us;
}

// "M,lid_open=<us>;<us>;...,launch=<us>,reset=<us>,spool_in=<us>,lid_close=<us>,spool_out=<us>"
// Motor move durations for the sequence, starting from home. lid_open lists each
// JOG_STEPS increment (lid_step_pause sits between them).
void reportMotion() {
  Serial.print("M,lid_open=");
  for (long remaining = LID_OPEN_STEPS; remaining > 0;) {
    long chunk = min(remaining, JOG_STEPS);
    remaining -= chunk;
    Serial.printf("%lu%s", moveTimeUs(chunk, LID_STEP_US_CRUISE), remaining > 0 ? ";" : "");
  }
  Serial.printf(",launch=%lu", moveTimeUs(PINION_FIRST_LEG, STEP_US_CRUISE));
  Serial.printf(",reset=%lu", moveTimeUs(STEPS_360 - PINION_FIRST_LEG, STEP_US_CRUISE));
  Serial.printf(",spool_in=%lu", moveTimeUs(SPOOL_REEL_STEPS, SPOOL_STEP_US_CRUISE));
  Serial.printf(",lid_close=%lu", moveTimeUs(LID_OPEN_STEPS, LID_STEP_US_CRUISE));
  Serial.printf(",spool_out=%lu\n", moveTimeUs(SPOOL_REEL_STEPS, SPOOL_STEP_US_CRUISE));
}

void reportTimings() {
  Serial.print("C");
  for (auto &t : timings) Serial.printf(",%s=%lu", t.key, t.value);
  Serial.println();
  reportMotion();
}

// args: "key=ms,key=ms,...". Unknown keys and bad numbers are ignored; values
// are clamped to TIMING_MAX_MS. Only changed values are written to flash.
void setTimings(String args) {
  prefs.begin("timings", false);
  int start = 0;
  while (start < (int)args.length()) {
    int end = args.indexOf(',', start);
    if (end < 0) end = args.length();
    String pair = args.substring(start, end);
    start = end + 1;
    int eq = pair.indexOf('=');
    if (eq < 0) continue;
    String key = pair.substring(0, eq);
    String num = pair.substring(eq + 1);
    key.trim();
    num.trim();
    if (num.length() == 0) continue;
    bool digits = true;
    for (unsigned i = 0; i < num.length(); i++) digits &= isDigit(num[i]) != 0;
    if (!digits) continue;
    unsigned long v = min((unsigned long)num.toInt(), TIMING_MAX_MS);
    for (auto &t : timings) {
      if (key == t.key && t.value != v) {
        t.value = v;
        prefs.putULong(t.key, v);
      }
    }
  }
  prefs.end();
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_LID_STEP, OUTPUT);
  pinMode(PIN_LID_DIR, OUTPUT);
  pinMode(PIN_ARM_STEP, OUTPUT);
  pinMode(PIN_ARM_DIR, OUTPUT);
  pinMode(PIN_SPOOL_STEP, OUTPUT);
  pinMode(PIN_SPOOL_DIR, OUTPUT);
  pinMode(PIN_DRV_EN, OUTPUT);
  pinMode(PIN_STATUS_LED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
  driversEnabled(false);

  loadTimings();

  scale.begin(PIN_HX711_DOUT, PIN_HX711_SCK);
  delay(500);
  rebaseline();
  resetRest();
  reportTimings();
}

void handleCommands() {
  while (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "start") {
      sampling = true;
      samplingStartMs = millis();
    } else if (cmd == "stop") {
      sampling = false;
    } else if (cmd == "spin_lid" || cmd == "lift_lid" || cmd == "spin_arm" || cmd == "reset_lid" ||
               cmd == "spool_in" || cmd == "spool_out") {
      driversEnabled(true);
      if (cmd == "spin_lid") moveLidInIncrements(LID_OPEN_STEPS);
      else if (cmd == "lift_lid") moveLidInIncrements(LID_OPEN_STEPS - lidPos, LIFT_LID_STEP_US_CRUISE);
      else if (cmd == "spin_arm") spinArm();
      else if (cmd == "spool_in") spoolIn();
      else if (cmd == "spool_out") spoolOut();
      else moveLid(-lidPos);
      // Keep holding torque while anything is away from its start position.
      driversEnabled(!atHome());
      resetRest();
      reportState();
    } else if (cmd == "step_fire" || cmd == "step_reel" || cmd == "step_close" || cmd == "step_unspool") {
      runPhase(cmd);
    } else if (cmd == "get_state") {
      reportState();
    } else if (cmd == "get_timings") {
      reportTimings();
    } else if (cmd.startsWith("set_timings")) {
      setTimings(cmd.substring(strlen("set_timings")));
      reportTimings();
    } else if (cmd == "run_sequence") {
      runSequence();
    } else if (cmd.startsWith("jog_")) {
      // Move without updating lidPos/spoolPos, so the start position moves
      // with the motor and later moves use the adjusted position.
      bool fwd = cmd.endsWith("_fwd");
      driversEnabled(true);
      if (cmd.startsWith("jog_lid")) {
        moveSteps(PIN_LID_STEP, PIN_LID_DIR, fwd ? LID_OPEN_DIR : !LID_OPEN_DIR, JOG_STEPS, LID_STEP_US_CRUISE);
      } else if (cmd.startsWith("jog_spool")) {
        moveSteps(PIN_SPOOL_STEP, PIN_SPOOL_DIR, fwd ? SPOOL_IN_DIR : !SPOOL_IN_DIR, SPOOL_JOG_STEPS, SPOOL_STEP_US_CRUISE);
      } else {
        moveSteps(PIN_ARM_STEP, PIN_ARM_DIR, fwd ? ARM_FORWARD_DIR : !ARM_FORWARD_DIR, JOG_STEPS, STEP_US_CRUISE);
      }
      driversEnabled(!atHome());
      // Re-take the rest weight from the next reading rather than blocking for
      // a full average, so held-down adjust buttons stay responsive.
      restPending = true;
    }
  }
}

void loop() {
  handleCommands();
  if (!scale.is_ready()) return;

  float grams = readGrams();
  float t = (millis() - samplingStartMs) / 1000.0f;
  if (sampling) Serial.printf("D,%.3f,%.3f\n", t, grams);

  if (restPending) {
    restPending = false;
    restGrams = grams;
    settling = false;
    spikeCount = 0;
    clearWindow();
    return;
  }

  if (!settling) {
    spikeCount = (fabs(grams - restGrams) > SPIKE_G) ? spikeCount + 1 : 0;
    if (spikeCount < SPIKE_CONFIRM_READS) {
      restGrams += (grams - restGrams) * REST_TRACK_RATE;
      return;
    }
    if (sampling) Serial.printf("S,%.3f,%.3f\n", t, grams);
    settling = true;
    spikeCount = 0;
    clearWindow();
    lastUnsettledMs = millis();
  }

  pushWindow(grams);
  float settled;
  if (!windowSettled(settled)) {
    lastUnsettledMs = millis();
    return;
  }

  // Settled well above the old rest is usually a hand resting in the bowl, so
  // keep waiting; if it stays there long enough, treat it as a refill.
  if (settled > restGrams + SPIKE_G && millis() - lastUnsettledMs < REFILL_ACCEPT_MS) return;

  float taken = restGrams - settled;
  restGrams = settled;
  settling = false;

  if (taken > CANDY_G) {
    if (sampling) Serial.printf("T,%.3f,%.3f\n", t, taken);
    announce("trigger");
    runSequence();
  }
}
