/*
* Simple code for the L298N motor controller
* and Metal Gearmotor 37Dx52L mm with 64 CPR Encoder 
*/
#include <L298N.h>
#include <Encoder.h>

// motor pin definition
#define M_EN 9
#define M_IN1 8
#define M_IN2 7
#define M_EN2 6
#define M_EN1 5

// create a motor instance
L298N motor(M_EN, M_IN1, M_IN2);
Encoder enc(M_EN1, M_EN2);
// PWM signal for the motor driver
unsigned short driverPwm = 0;
float kp = 1;
long prevEnc  = -999;
int desiredPos = 10000;

// variables for sending information frequency configuration
long previousMillis = 0; 
long interval = 200;  

void setup() {
  Serial.begin(115200);
  Serial.println("Basic Motor Test:");
}

void loop() {
  unsigned long currentMillis = millis();
  long currEnc = enc.read();
  if(currentMillis - previousMillis > interval){
    Serial.print("PWM value: ");
    Serial.print(driverPwm);
    Serial.print("\tEncoder Value: ");
    Serial.println(currEnc);
    
    previousMillis = currentMillis;   
  }

  // check if the data has been sent from the computer:
  // to make this code work properly, select "No line ending" in monitor
  if (Serial.available()) {
    // Narrowing convertion without check! Don't do this in production:)
    driverPwm = Serial.parseInt();
  }

  motor.setSpeed(driverPwm);
  motor.forward();
}
