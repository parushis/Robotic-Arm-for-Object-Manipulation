#include <Servo.h>

// Create servo objects for each joint
Servo claw;
Servo wrist;
Servo forearm;
Servo arm;
Servo pantilt;

bool done = false; 

void setup() {
  Serial.begin(115200);
  
  // Attach servos to pins (update with your wiring!)
  claw.attach(1);
  wrist.attach(2);
  forearm.attach(3);
  arm.attach(4);
  pantilt.attach(5);

  delay(1000);

  // Move to initial "home" position
  claw.write(90);      // middle position
  wrist.write(0);
  forearm.write(0);
  arm.write(90);
  pantilt.write(180);
  
  delay(2000);  // wait 2s before starting
  Serial.println("Made it hereee");
}

void loop() {
  if(!done) {
      // Repeat a "wave" 3 times
  for (int i = 0; i < 3; i++) {
    // Move wrist left
    wrist.write(60);
    delay(400);

    // Move wrist right
    wrist.write(120);
    delay(400);
  }

  // Small base turn for extra wave motion
  arm.write(70);
  delay(500);
  arm.write(110);
  delay(500);
  arm.write(90); // back to center

  // Pause before next wave
  delay(1500);

  done = true;
  }
  
}

