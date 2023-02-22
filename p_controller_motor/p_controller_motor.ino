#include <Encoder.h>
#include <L298N.h>

// Insert kp value here
double kp = 2;
// Do not edit beyond this

int in1 = 9; 
int in2 = 8; 
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

void setup() {
  Serial.begin(115200);

  pinMode(in1, OUTPUT); 
  pinMode(in2, OUTPUT);
}

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
