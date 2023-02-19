//This code is to use with L298n Dual H-bridge motor driver<br>//It just turns on a DC motor for a certain time in a direction, turn it off, turn in the other direction and turn it off again
//refer to surtrtech.blogspot.com for more information

#include <Encoder.h>
#include <L298N.h>

int in1 = 9; //Declaring the pins where in1 in2 from the driver are wired 
int in2 = 8; //here they are wired with D9 and D8 from Arduino
int in2_1 = 7;
int m_en2 = 6;
int m_en1 = 5;

L298N motor(in1, in2,  in2_1);
Encoder enc(m_en1, m_en2);

long current_val = 0;
long error_val = 0;
int desiredPos = 1500;
long previousMillis = 0; 
long interval = 10;  
unsigned short driverPwm = 0;

int kp = 2;


void setup() {
  Serial.begin(115200);

  pinMode(in1, OUTPUT); //Declaring the pin modes, obviously they're outputs
  pinMode(in2, OUTPUT);
}
//Before starting the loop you should create functions type "void" to control the driver's pins
//Here I created three functions, one to turn the motor in a direction "#1", the other one to the other direction "#3"
//and the second one to stop the motor
//For changing directions you switch the HIGH with LOW and vice-versa

void controlMotor(int speed, int dir){
  motor.setSpeed(speed);

  if(dir == L298N::BACKWARD)
    motor.run(L298N::FORWARD);
  else if (dir == L298N::FORWARD)
    motor.run(L298N::BACKWARD);
  else
    motor.run(L298N::STOP);

}

void loop() {
  unsigned long currentMillis = millis();
  long currEnc = enc.read();
 // current_val = digitalRead(m_en2);
 // Serial.println(currEnc);

  if(currentMillis - previousMillis > interval){
      error_val = currEnc - desiredPos;
      int effort = kp*error_val;
      int direction = (abs(error_val > 0 ) ? L298N::FORWARD : L298N::BACKWARD);
      previousMillis = currentMillis;  
      controlMotor(effort, direction);  


      Serial.print("PWM value: ");
      Serial.print(driverPwm);
      Serial.print("\tEncoder Value: ");
      Serial.print(currEnc);
      Serial.print("\tError value: ");
      Serial.print(error_val);
      Serial.print("\tDirection: ");
      Serial.println(direction);

    }




}