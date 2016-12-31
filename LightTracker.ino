#define BAUD_RATE         (250000)
#define FLIP_DELAY_MS     (550)
#define LOG_BASE          (1.03f)
#define LOOP_DELAY_MS     (14)
#define MAX_TRAV          (3.0f)    // degrees
#define NUM_READS         (2)       // how many times to sample light sensors
#define NUM_SENSORS       (3)
#define OCCL_THRESH_ABS   (200.0f)
#define OCCL_THRESH_REL   (0.02f)
#define PRINT_FREQ        (6)
#define READ_DELAY_MS     (3)

#define HORZ_INITIAL      (40)
#define VERT_INITIAL      (60)

// degree values by which servos are offset from
// the world frame
// i.e. the horz servo, at a commanded 0 degrees,
// is actually HORZ_OFFSET degrees off its world axis
#define HORZ_OFFSET       (-40.0f)
#define VERT_OFFSET       (-18.0f)

// pins of servo control lines
#define HORZ_PIN          (9)
#define VERT_PIN          (8)

#define SERVO_MIN         (10)
#define SERVO_MAX         (180)

// see computeFlipAndMagicGimbalSolution() function
#define MAGIC_ANGLE_2D    (45)
#define MAGIC_ANGLE_3D    (54.7356103172f)

// move gimbals?
#define MOVE

// travel directly to target instead of interpolating?
//#define ALWAYS_DIRECT

#define rad2deg(x)        57.295779513082f*(x)
#define deg2rad(x)        0.0174532925199f*(x)

#include <Servo.h>

Servo horz_servo, vert_servo;

const int sensor_pins[NUM_SENSORS] = {0, 2, 1};
const char deg      = static_cast<char>(176);      // degree symbol
const char percent  = static_cast<char>(37);       // percent symbol

bool flipped, go_direct, is_occl[NUM_SENSORS];

float norm, d_horz, d_vert, factor;
float angle[NUM_SENSORS], l_lum[NUM_SENSORS], p_vec[NUM_SENSORS], w_vec[NUM_SENSORS];

int steps, i, j, num_occl, r_lum[NUM_SENSORS];

int horz,             vert;
int req_horz,         req_vert;
int decided_horz,     decided_vert;
int corrected_horz,   corrected_vert;
int travel_horz,      travel_vert;

String out;

void setup() {
  Serial.begin(BAUD_RATE);

  horz_servo.attach(HORZ_PIN);
  vert_servo.attach(VERT_PIN);

  horz_servo.write(horz = HORZ_INITIAL);
  vert_servo.write(vert = VERT_INITIAL);

  steps = 0;
  flipped = false;

  // give the gimbals time to reach their default positions
  delay(FLIP_DELAY_MS);
}

// Read luminosity values from all sensors
// multiple times, waiting in between
// and averaging the result.
// Also compute magnitude for normalization.
void readLuminosities() {
  norm = 0.0f;
  for (i = 0; i < NUM_SENSORS; ++i) {
    // read immediately so no unnecessary delay call
    r_lum[i] = analogRead(sensor_pins[i]);
    for (j = 1; j < NUM_READS; ++j) {
      delay(READ_DELAY_MS);
      r_lum[i] += analogRead(sensor_pins[i]);
    }

    // average and linearize incoming raw lums, which are log scale
    l_lum[i] = pow(LOG_BASE, static_cast<float>(r_lum[i]) / NUM_READS);
    r_lum[i] /= NUM_READS;

    // keep track of sum of squares for normalization
    norm += l_lum[i] * l_lum[i];
  }

  // inverse sqrt for normalization
  norm = norm ? pow(norm, -0.5f) : 0.0f;
}

// Normalize the linearlized lums,
// compute Euler angles,
// and examine the light for possible
// signs of occluded sensors.
// Two criteria are used for occlusion
// analysis: an absolute threshold below
// which occlusion is probable, and
// a relative threshold - if a sensor
// represents a very small fraction of
// total light intensity, it is likely
// occluded.
void normalizeAndAnalyzeOcclusion() {
  num_occl = 0;
  for (i = 0; i < NUM_SENSORS; ++i) {
    angle[i] = rad2deg(acos(p_vec[i] = l_lum[i] * norm));
    num_occl += (is_occl[i] = l_lum[i] < OCCL_THRESH_ABS || p_vec[i] < OCCL_THRESH_REL);
  }
}

// Perform coordinate frame transformation
// on luminosity vector from Platform frame
// to World frame. The gimbals have offsets,
// so correct for those; then the conversion
// is simply   a rotation of 'v' about -z,
// followed by a rotation of 'h' about the *new* -y.
// The sequential nature suggests two chained
// rotation matrices, which is the method used here.
// The result was evaluated symbolically and simplified
// to produce an output vector directly, for speed.
void transformPlatformFrameToWorldFrame() {
  float h = deg2rad(horz + HORZ_OFFSET);
  float v = deg2rad(vert + VERT_OFFSET);
  w_vec[0] =  cos(h) * cos(v) * p_vec[0] - cos(h) * sin(v) * p_vec[1] + sin(h) * p_vec[2];
  w_vec[1] =           sin(v) * p_vec[0] +          cos(v) * p_vec[1]                    ;
  w_vec[2] = -cos(v) * sin(h) * p_vec[0] + sin(h) * sin(v) * p_vec[1] + cos(h) * p_vec[2];
}

// The naive solution cannot account for flips.
// It simply determines the horz and vert
// pointing that would point at the lum vector
// with the +x axis of the platform IF horz
// had full range. Both of these assumptions
// are wrong; we do not want the +x axis to be
// the pointing solution, as then it would be
// too easy to occlude the light and lose it.
// We want the magic angle (opposite corners of cube)
// pointing vector so changes in light position are
// most likely to be tracked successfully. Furthermore,
// horz only has range SERVO_MIN to SERVO_MAX.
// These are both addressed in the next stage of
// processing.
void computeNaiveGimbalSolution() {
  req_horz = static_cast<int>(-rad2deg(atan2(w_vec[2], w_vec[0])) - HORZ_OFFSET + 0.5f);
  req_vert = static_cast<int>(rad2deg(asin(w_vec[1])) - VERT_OFFSET + 0.5f);
}

// Flipping allows the system to recognize
// when a requested horz is unattainable
// and compensate by pointing in the
// opposite direction, but flipping the
// vert gimbal over on its back so it's
// upside-down and pointing across the
// platform at the target.
// If a transition occurs between normal
// and flipped modes or vice versa,
// the go_direct flag is set, prompting
// the system to travel directly to the target
// instead of the usual interpolation,
// because the flip action cannot
// propagate through interpolated steps.
// During this analysis, offsets for
// magic angle are also applied automatically.
// See https://en.wikipedia.org/wiki/Magic_angle
void computeFlipAndMagicGimbalSolution() {
  go_direct = false;
  // don't try to change flip modes if
  // everything is occluded
  if (num_occl != NUM_SENSORS) {
    // if we're in flip mode...
    if (flipped) {
      // ...and we are currently still flipped...
      if (req_horz < 60) {
        // ...apply the flip to the req values
        // by pointing the opposite way...
        req_horz += 165 - MAGIC_ANGLE_2D;
        // ...and flipping vert on its back.
        req_vert = 180 - req_vert;
      }
      else if (req_horz > 208) {
        // ...apply the flip to the req values
        // by pointing the opposite way...
        req_horz -= 205;
        // ...and flipping vert on its back.
        req_vert = 180 - req_vert;
      }
      // if we are no longer flipped...
      else {
        // ...clear the flip flag...
        flipped = false;
        // ...force a direct travel...
        go_direct = true;
        // ...and still apply the magic angle correction.
        // Note the small empirical correction factor
        // on both horz and vert which arises from
        // nonlinearity in the sensors' target estimation
        // when far off-axis, like when flipping.
        req_horz += MAGIC_ANGLE_2D - 5;
        req_vert -= 2;
      }
    }
    else {
      // if we aren't in flip mode...
      // ...but we should be...
      // (this would be req_horz < 0
      // ideally but some hysteresis
      // guarding helps prevent constant
      // flipping back and forth when at
      // a flip boundary)
      if (req_horz < 7) {
        // ...set the flip flag...
        flipped = true;
        // ...force a direct travel...
        go_direct = true;
        // ...and flip the gimbals as before.
        req_horz += 180 - 2 * MAGIC_ANGLE_2D;
        req_vert = 180 - req_vert;
      }
      // if we should be in flip mode
      // off the other direction (also
      // with hysteresis guarding, instead
      // of req_horz > 180)...
      if (req_horz > 160) {
        // ...set the flip flag...
        flipped = true;
        // ...force a direct travel...
        go_direct = true;
        // ... and flip the gimbals.
        // Empirical correction terms.
        req_horz -= (180 + MAGIC_ANGLE_2D - 15);
        req_vert = 180 - req_vert - 5;
      }
      // if we aren't in flip mode
      // and aren't entering it,
      // just do magic angle correction.
      req_horz += MAGIC_ANGLE_2D;
    }
  }
  // We don't really want vert in danger of occlusion from the wiring
  // on the back of the sensors perpendicular to the platform,
  // so only do 35, not 45, magic angle
  // correction on vert.
  req_vert -= 35;
}

// Now we have a good gimbal solution,
// but we may not want to go directly there
// (unless we're flipping). Instead,
// we interpolate, going a few degrees,
// then recomputing so changes in light can
// be more easily tracked. Nicer on the gimbals
// as well.
void computeGimbalTargets() {
#ifdef ALWAYS_DIRECT
  go_direct = true;
#endif

  // Travel deltas all the way to the solution
  d_horz = static_cast<float>(req_horz - horz);
  d_vert = static_cast<float>(req_vert - vert);

  // if we ARE going direct, or the requested travel is small...
  if (go_direct || (abs(d_horz) <= MAX_TRAV && abs(d_vert) <= MAX_TRAV)) {
    // ...easy, just send the gimbals there.
    decided_horz = req_horz;
    decided_vert = req_vert;
    travel_horz = req_horz - horz;
    travel_vert = req_vert - vert;
  }
  // if not...
  else {
    // ...scale the deltas proportionally so the larger one is exactly the max travel.
    factor = abs(d_horz) > abs(d_vert) ? MAX_TRAV / abs(d_horz) : MAX_TRAV / abs(d_vert);

    decided_horz = horz + (travel_horz = static_cast<int>(d_horz * factor + 0.5f));
    decided_vert = vert + (travel_vert = static_cast<int>(d_vert * factor + 0.5f));
  }
}

// Constrain the decided-on gimbal travel targets
// to SERVO_MIN and SERVO_MAX, the physical limitations
// of the servo motors
void constrainGimbalTargets() {
  corrected_horz = decided_horz > SERVO_MAX ? SERVO_MAX : (decided_horz < SERVO_MIN ? SERVO_MIN : decided_horz);
  corrected_vert = decided_vert > SERVO_MAX ? SERVO_MAX : (decided_vert < SERVO_MIN ? SERVO_MIN : decided_vert);
}

// Move the gimbals and update
// the current position
void moveToTargets() {
  horz_servo.write(horz = corrected_horz);
  vert_servo.write(vert = corrected_vert);
}

void printDebugInfo() {
  // only print every PRINT_FREQ
  if (++steps >= PRINT_FREQ) {
    steps = 0;
    out =  String("Luminosity Vector - RAW             (") + r_lum[0] + String(", ") + r_lum[1] + String(", ") + r_lum[2] + String(")\r\n");
    out += String("Luminosity Vector - LINEARIZED      (") + l_lum[0] + String(", ") + l_lum[1] + String(", ") + l_lum[2] + String(")\r\n");
    out += String("Luminosity Vector - MAGNITUDE        ") + 1.0f / norm + String("\r\n");
    out += String("Luminosity Vector - NORMALIZED      (") + p_vec[0] + String(", ") + p_vec[1] + String(", ") + p_vec[2] + String(")\r\n");
    out += String("Computed Euler Angles               (") + angle[0] + deg + String(", ") + angle[1] + deg + String(", ") + angle[2] + deg + String(")\r\n");
    out += String("Predictive Occlusion Analysis       (") + (is_occl[0] ? "OCCLUDED!" : "OK") + String(", ") + (is_occl[1] ? "OCCLUDED!" : "OK") + String(", ") + (is_occl[2] ? "OCCLUDED!" : "OK") + String(")\r\n");
    if (num_occl) out += String(">>> WARN: at least one sensor occluded. Will attempt to recover.\r\n");
    if (num_occl == NUM_SENSORS) out += ">>> WARN: all sensors occluded! Not moving.\r\n";
    out += String("Occlusion-Corrected Euler Angles    (") + (is_occl[0] ? String(">90") : String(angle[0])) + deg + String(", ") + (is_occl[1] ? String(">90") : String(angle[1])) + deg + String(", ") + (is_occl[2] ? String(">90") : String(angle[2])) + deg + String(")\r\n");
    out += String("Current Platform Attitude           (") + horz + deg + String(", ") + vert + deg + String(")\r\n");
    out += String("Luminosity Vector - WORLD FRAME     (") + w_vec[0] + String(", ") + w_vec[1] + String(", ") + w_vec[2] + String(")\r\n");
    out += String("Computed Gimbal Solution            (") + req_horz + deg + String(", ") + req_vert + deg + String(")\r\n");
    out += String("Computed Gimbal Travel Deltas       (") + (travel_horz < 0 ? String(travel_horz) : String('+') + String(travel_horz)) + deg + String(", ") + (travel_vert < 0 ? String(travel_vert) : String('+') + String(travel_vert)) + deg + String(")\r\n");
    out += String("Computed Gimbal Travel Targets      (") + decided_horz + deg + String(", ") + decided_vert + deg + String(")\r\n");
    if (decided_horz != corrected_horz) out += ">>> WARN: computed HORZ gimbal travel target out of range. Constraining.\r\n";
    if (decided_vert != corrected_vert) out += ">>> WARN: computed VERT gimbal travel target out of range. Constraining.\r\n";
    if (decided_horz != corrected_horz || decided_vert != corrected_vert) {
      out += String("Constrained Gimbal Travel Targets   (") + corrected_horz + deg + String(", ") + corrected_vert + deg + String(")\r\n");
    }
    if (go_direct) out += "FLIP DETECTED: traveling directly to gimbal solution NOW.\r\n";
    out += flipped ? "Gimbal Mode: FLIPPED\r\n" : "Gimbal Mode: Normal\r\n";

    Serial.println(out);
  }
}

void loop() {
  readLuminosities();
  normalizeAndAnalyzeOcclusion();
  transformPlatformFrameToWorldFrame();
  computeNaiveGimbalSolution();
  computeFlipAndMagicGimbalSolution();
  computeGimbalTargets();
  constrainGimbalTargets();

#ifdef PRINT_FREQ
  printDebugInfo();
#endif

#ifdef MOVE
  if (num_occl != NUM_SENSORS) moveToTargets();
#endif

  delay(go_direct ? FLIP_DELAY_MS : LOOP_DELAY_MS);
}
