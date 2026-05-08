#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150 // Minimum pulse length for 0 degrees
#define SERVOMAX  600 // Maximum pulse length for 180 degrees
#define SERVO_FREQ 50 

// CHANGE THIS NUMBER to test different channels (0 through 15)
int testChannel = 0; 

void setup() {
  Serial.begin(9600);
  Serial.println("Starting Single Servo Test...");

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);

  Serial.print("Sending Channel ");
  Serial.print(testChannel);
  Serial.println(" to exactly 0 degrees.");
  
  // Command the specific channel to the 0-degree position
  pwm.setPWM(testChannel, 0, SERVOMIN);
}

void loop() {
  // We leave the loop empty so it holds the 0-degree position steadily
}