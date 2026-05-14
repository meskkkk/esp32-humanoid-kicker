#include <Wire.h>
#include <math.h>
#include <Adafruit_PWMServoDriver.h>

#define I2C_SDA 32
#define I2C_SCL 33

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN 150
#define SERVOMAX 600
#define SERVO_FREQ 50

// ═════════════════════════════════════════════════════════════════════
// SERVO CHANNEL MAPPING
// Defines the PCA9685 output channels connected to each servo
// ═════════════════════════════════════════════════════════════════════
const int R_HIP = 0;
const int R_KNEE = 1;
const int R_FOOT = 2;

const int L_HIP = 3;
const int L_KNEE = 4;
const int L_FOOT = 5;

// ═════════════════════════════════════════════════════════════════════
// SERVO CALIBRATION OFFSETS
// Used to align each servo to its mechanical center position
// ═════════════════════════════════════════════════════════════════════
#define HIP_L_OFFSET 90
#define KNEE_L_OFFSET 90
#define ANKLE_L_OFFSET 90

#define HIP_R_OFFSET 90
#define KNEE_R_OFFSET 90
#define ANKLE_R_OFFSET 90

// ═════════════════════════════════════════════════════════════════════
// ROBOT LEG DIMENSIONS (CENTIMETERS)
// L1 = upper leg length
// L2 = lower leg length
// ═════════════════════════════════════════════════════════════════════
#define L1 5.0f
#define L2 5.7f

// ═════════════════════════════════════════════════════════════════════
// WALKING AND BALANCE PARAMETERS
// ═════════════════════════════════════════════════════════════════════
#define STEP_HEIGHT 10.0f
#define STEP_CLEARANCE 0.6f
#define X_BALANCE_OFFSET 0.5f

// ═════════════════════════════════════════════════════════════════════
// BALL TRACKING STATE VARIABLES
// These values are updated from the Python vision system
// ═════════════════════════════════════════════════════════════════════
float targetX = 0.0;
float targetY = 0.0;

unsigned long lastSeenTime = 0;
unsigned long lastKickTime = 0;

// Blind charge state tracking
bool inBlindCharge = false;
unsigned long blindChargeStartTime = 0;
int blindChargeSteps = 0;

// ═════════════════════════════════════════════════════════════════════
// MAIN CONTROL PARAMETERS
// ═════════════════════════════════════════════════════════════════════

// Maximum time allowed without seeing the ball
const unsigned long TIMEOUT_MS = 1500;

// Delay after kicking before tracking resumes
const unsigned long COOLDOWN_MS = 6000;

// Distance threshold where the ball disappears beneath the camera view
const float BLIND_SPOT_THRESHOLD = 9.5;

// Maximum horizontal offset still considered centered
const float X_ALIGNMENT_TOLERANCE = 10.0;

// Number of walking steps performed during blind charge
const int BLIND_CHARGE_STEPS = 2;

// ═════════════════════════════════════════════════════════════════════
// SERVO CONTROL FUNCTIONS
// ═════════════════════════════════════════════════════════════════════

void writeServo(int channel, int angle) {

  // Limit servo angle to valid range
  angle = constrain(angle, 0, 180);

  // Convert angle to PCA9685 PWM pulse range
  pwm.setPWM(
    channel,
    0,
    map(angle, 0, 180, SERVOMIN, SERVOMAX));
}

void updateServoPos(int hipDeg, int kneeDeg, int ankleDeg, char leg) {

  // Left leg servo mapping
  if (leg == 'l') {

    writeServo(L_HIP, HIP_L_OFFSET + hipDeg);
    writeServo(L_KNEE, KNEE_L_OFFSET + kneeDeg);
    writeServo(L_FOOT, ankleDeg);

  }
  // Right leg servo mapping
  else {

    writeServo(R_HIP, HIP_R_OFFSET - hipDeg);
    writeServo(R_KNEE, KNEE_R_OFFSET - kneeDeg);
    writeServo(R_FOOT, 2 * ANKLE_R_OFFSET - ankleDeg);
  }
}

// ═════════════════════════════════════════════════════════════════════
// INVERSE KINEMATICS
// Converts target foot position into servo joint angles
// ═════════════════════════════════════════════════════════════════════

void pos(float x, float z, char leg) {

  float hipRad2 = atan(x / z);
  float z2 = z / cos(hipRad2);

  float hipRad1 = acos(
    constrain(
      (sq(L1) + sq(z2) - sq(L2)) / (2.0f * L1 * z2),
      -1.0f,
      1.0f));

  float kneeRad = PI - acos(constrain((sq(L1) + sq(L2) - sq(z2)) / (2.0f * L1 * L2), -1.0f, 1.0f));

  float ankleRad =
    PI / 2.0f + hipRad2 - acos(constrain((sq(L2) + sq(z2) - sq(L1)) / (2.0f * L2 * z2), -1.0f, 1.0f));

  int hipDeg = (int)((hipRad1 + hipRad2) * (180.0f / PI));
  int kneeDeg = (int)(kneeRad * (180.0f / PI));
  int ankleDeg = (int)(ankleRad * (180.0f / PI));

  updateServoPos(hipDeg, kneeDeg, ankleDeg, leg);
}

// ═════════════════════════════════════════════════════════════════════
// WALKING GAIT
// Generates alternating left/right walking motion
// ═════════════════════════════════════════════════════════════════════

void takeStep(float stepLength, int stepVelocity) {

  // Right leg swing phase
  for (float i = stepLength; i >= -stepLength; i -= 0.5f) {

    pos(
      i + X_BALANCE_OFFSET,
      STEP_HEIGHT + STEP_CLEARANCE,
      'r');

    pos(
      (-i * 0.3f) + X_BALANCE_OFFSET,
      STEP_HEIGHT,
      'l');

    delay(stepVelocity);
  }

  // Left leg swing phase
  for (float i = stepLength; i >= -stepLength; i -= 0.5f) {

    pos(
      i + X_BALANCE_OFFSET,
      STEP_HEIGHT + STEP_CLEARANCE,
      'l');

    pos(
      (-i * 0.3f) + X_BALANCE_OFFSET,
      STEP_HEIGHT,
      'r');

    delay(stepVelocity);
  }
}

// ═════════════════════════════════════════════════════════════════════
// STARTUP POSITIONING
// Smoothly lowers the robot into walking stance
// ═════════════════════════════════════════════════════════════════════

void initialize() {

  for (float i = 10.7f; i >= STEP_HEIGHT; i -= 0.1f) {

    pos(X_BALANCE_OFFSET, i, 'l');
    pos(X_BALANCE_OFFSET, i, 'r');

    delay(10);
  }
}

// ═════════════════════════════════════════════════════════════════════
// BALL KICKING SEQUENCE
// Multi-phase kicking animation for stable striking
// ═════════════════════════════════════════════════════════════════════

void shootBall() {

  Serial.println("KICK SEQUENCE INITIATED");

  // Phase 1: Stabilize robot before kick
  Serial.println("Planting feet");

  pos(X_BALANCE_OFFSET, STEP_HEIGHT, 'l');
  pos(X_BALANCE_OFFSET, STEP_HEIGHT, 'r');

  delay(400);

  // Phase 2: Pull kicking leg backward
  Serial.println("Winding up");

  pos(1.5, STEP_HEIGHT, 'l');
  pos(-4.0, STEP_HEIGHT, 'r');

  delay(350);

  // Phase 3: Execute kick motion
  Serial.println("STRIKE");

  pos(-2.5, STEP_HEIGHT, 'l');
  pos(4.5, 9.5f, 'r');

  delay(300);

  // Phase 4: Continue forward momentum
  Serial.println("Follow through");

  pos(-1.5, STEP_HEIGHT, 'l');
  pos(3.0, 10.0f, 'r');

  delay(250);

  // Phase 5: Return to neutral standing position
  Serial.println("Recovering to stance");

  pos(X_BALANCE_OFFSET, STEP_HEIGHT, 'l');
  pos(X_BALANCE_OFFSET, STEP_HEIGHT, 'r');

  delay(800);

  Serial.println("KICK COMPLETE");
}

// ═════════════════════════════════════════════════════════════════════
// SYSTEM INITIALIZATION
// ═════════════════════════════════════════════════════════════════════

void setup() {

  Serial.begin(115200);

  // Initialize I2C communication
  Wire.begin(I2C_SDA, I2C_SCL);

  // Initialize PCA9685 servo controller
  pwm.begin();

  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);

  delay(200);

  // Move all servos to neutral calibration positions
  writeServo(L_HIP, HIP_L_OFFSET);
  writeServo(L_KNEE, KNEE_L_OFFSET);
  writeServo(L_FOOT, ANKLE_L_OFFSET);

  writeServo(R_HIP, HIP_R_OFFSET);
  writeServo(R_KNEE, KNEE_R_OFFSET);
  writeServo(R_FOOT, ANKLE_R_OFFSET);

  delay(3000);

  // Move robot into initial standing stance
  initialize();

  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.println("SOCCER ROBOT READY");
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.println();
}

// ═════════════════════════════════════════════════════════════════════
// MAIN CONTROL LOOP
// ═════════════════════════════════════════════════════════════════════

void loop() {

  // ═══════════════════════════════════════════════════════════════════
  // STEP 1: RECEIVE BALL COORDINATES FROM PYTHON VISION SYSTEM
  // ═══════════════════════════════════════════════════════════════════

  while (Serial.available() > 0) {

    String data = Serial.readStringUntil('\n');

    int commaIndex = data.indexOf(',');

    if (commaIndex > 0) {

      float newX =
        data.substring(0, commaIndex).toFloat();

      float newY =
        data.substring(commaIndex + 1).toFloat();

      // "0,0" indicates ball lost
      if (newX == 0.0 && newY == 0.0) {

        // Keep previous tracking state
      } else {

        // Update latest valid ball position
        targetX = newX;
        targetY = newY;

        lastSeenTime = millis();
      }
    }
  }

  // ═══════════════════════════════════════════════════════════════════
  // STEP 2: UPDATE STATE FLAGS
  // ═══════════════════════════════════════════════════════════════════

  unsigned long timeSinceLastSeen =
    millis() - lastSeenTime;

  unsigned long timeSinceKick =
    millis() - lastKickTime;

  bool isCoolingDown =
    (timeSinceKick < COOLDOWN_MS);

  bool isBallVisible =
    (timeSinceLastSeen < TIMEOUT_MS);

  // ═══════════════════════════════════════════════════════════════════
  // STEP 3: MAIN ROBOT STATE MACHINE
  // ═══════════════════════════════════════════════════════════════════

  if (isCoolingDown) {

    // ─────────────────────────────────────────────────────────────────
    // STATE: POST-KICK RECOVERY COOLDOWN
    // Robot remains stationary after kick
    // ─────────────────────────────────────────────────────────────────

    inBlindCharge = false;
    blindChargeSteps = 0;

    pos(X_BALANCE_OFFSET, STEP_HEIGHT, 'l');
    pos(X_BALANCE_OFFSET, STEP_HEIGHT, 'r');

    delay(50);
  }

  else if (inBlindCharge) {

    // ─────────────────────────────────────────────────────────────────
    // STATE: BLIND CHARGE
    // Continue walking after ball disappears beneath camera view
    // ─────────────────────────────────────────────────────────────────

    if (blindChargeSteps < BLIND_CHARGE_STEPS) {

      Serial.print("Blind charge step ");
      Serial.print(blindChargeSteps + 1);
      Serial.print("/");
      Serial.println(BLIND_CHARGE_STEPS);

      // Faster forward walking during final approach
      takeStep(1.5, 55);

      blindChargeSteps++;
    }

    else {

      // Blind charge completed - execute kick
      Serial.println("Blind charge complete - executing kick");

      shootBall();

      // Reset tracking state after kick
      lastKickTime = millis();

      inBlindCharge = false;
      blindChargeSteps = 0;

      targetX = 0.0;
      targetY = 100.0;

      lastSeenTime = 0;
    }
  }

  else if (!isBallVisible) {

    // ─────────────────────────────────────────────────────────────────
    // STATE: BALL NOT VISIBLE
    // Decide whether to start blind charge or remain idle
    // ─────────────────────────────────────────────────────────────────

    // Check whether the ball disappeared in the blind spot
    if (
      targetY > 0.0 && targetY <= BLIND_SPOT_THRESHOLD && abs(targetX) <= X_ALIGNMENT_TOLERANCE) {

      // Ball disappeared close and centered
      // Begin blind forward charge toward expected ball position

      Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
      Serial.println("BALL ENTERED BLIND SPOT");

      Serial.print("Last position: X=");
      Serial.print(targetX);

      Serial.print(" Y=");
      Serial.println(targetY);

      Serial.println("INITIATING BLIND CHARGE");
      Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");

      inBlindCharge = true;

      blindChargeStartTime = millis();

      blindChargeSteps = 0;
    }

    else {

      // Ball lost outside valid kick range
      // Remain stationary

      pos(X_BALANCE_OFFSET, STEP_HEIGHT, 'l');
      pos(X_BALANCE_OFFSET, STEP_HEIGHT, 'r');

      delay(50);
    }
  }

  else {

    // ─────────────────────────────────────────────────────────────────
    // STATE: BALL VISIBLE
    // Continue approaching the ball
    // ─────────────────────────────────────────────────────────────────

    Serial.print("Ball visible: X=");
    Serial.print(targetX);

    Serial.print(" Y=");
    Serial.print(targetY);

    // Check horizontal alignment
    if (abs(targetX) > X_ALIGNMENT_TOLERANCE) {

      Serial.println(" OFF-CENTER - Continue approaching");
    }

    else {

      Serial.println(" ALIGNED");
    }

    // Walking speed logic
    if (targetY < 15.0) {

      // Slow controlled approach when near ball
      takeStep(1.5, 55);
    }

    else {

      // Standard walking approach
      takeStep(1.5, 55);
    }
  }
}