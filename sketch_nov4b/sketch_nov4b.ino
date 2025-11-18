#include <Servo.h>

// ------------------- Servo Pins (PWM pins only) -------------------
#define BASE_PIN     7  // S1
#define ARM_PIN      6  // S2
#define FOREARM_PIN  5  // S3
#define WRIST_PIN    4  // S4
#define CLAW_PIN     3  // S5

// ------------------- Servo Objects -------------------
Servo baseServo, armServo, forearmServo, wristServo, clawServo;

// ------------------- Helper: smooth movement -------------------
void moveServoSmooth(Servo &servo, int target, bool reversed=false, int stepDelay=15) {
  int start = servo.read();  // current position
  if (reversed) target = 180 - target;

  int step = (target > start) ? 1 : -1;
  for (int pos = start; pos != target; pos += step) {
    servo.write(pos);
    delay(stepDelay);
  }
  servo.write(target);
}

// ------------------- Move all joints smoothly -------------------
void moveArm(int base, int arm, int forearm, int wrist, int claw, int delayAfter=500) {
  moveServoSmooth(baseServo, base, true);
  moveServoSmooth(armServo, arm);
  moveServoSmooth(forearmServo, forearm);
  moveServoSmooth(wristServo, wrist);
  moveServoSmooth(clawServo, claw, true);

  delay(delayAfter);
}

// ------------------- Setup -------------------
void setup() {
  Serial.begin(9600);
  Serial.println("MiniArm ready. Type 'go' or 'neutral'.");

  // Attach servos
  baseServo.attach(BASE_PIN);
  armServo.attach(ARM_PIN);
  forearmServo.attach(FOREARM_PIN);
  wristServo.attach(WRIST_PIN);
  clawServo.attach(CLAW_PIN);

  // Move to neutral position
  moveArm(90, 80, 82, 79, 94);
}

// ------------------- Main Loop -------------------
void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "go") {
      Serial.println("Performing pick & drop...");

      // Neutral
      moveArm(90, 80, 82, 79, 94, 800);

      // Move to right side
      moveArm(90, 80, 82, 73, 174, 1000);

      // Lower arm
      moveArm(62, 39, 124, 137, 174, 1000);

      // Close claw (grab)
      moveArm(62, 39, 124, 137, 174, 600);

      // Lift
      moveArm(90, 39, 124, 90, 174, 1000);

      // Move to left
      moveArm(90, 39, 124, 90, 13, 1000);

      // Lower arm
      moveArm(88, 38, 152, 90, 13, 800);

      // Open claw (drop)
      moveArm(50, 38, 152, 90, 13, 600);

      // Return to neutral
      moveArm(90, 80, 82, 79, 94, 1000);

      Serial.println("Done! Waiting for next command.");
    }
    else if (cmd == "neutral") {
      moveArm(90, 80, 82, 79, 94);
      Serial.println("Moved to neutral.");
    }
    else {
      Serial.println("Commands: go | neutral");
    }
  }
}
