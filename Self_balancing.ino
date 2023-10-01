#include <MPU6050_tockn.h>;
#include <Wire.h>;
MPU6050 mpu6050(Wire);

const int ENA = 5, ENB = 6, IN1 = 7, IN2 = 8, IN3 = 9, IN4 = 10;
float x, y, z;
float kp =2.000 ;
float ki = 0.0001;
float error = 0;
float desired = 90;
float errorsum = 0;
unsigned long lasttime;
unsigned long currenttime;
float delta_T;
float getspeedPwm;
float  getspeedPwm, speedPwmA, speedPwmB;
int directionA,directionB;

float gyroscope(){
  mpu6050.update();
  x = mpu6050.getAngleX();
  y = mpu6050.getAngleY();
  z = mpu6050.getAngleZ();
  Serial.print("\tangleZ : ");
  Serial.println(z);
  return x;
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  lasttime = millis();
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4 , OUTPUT);

}
void carmove(int dirA, int dirB, int PwmA, int PwmB) {
  analogWrite(enA, PwmA);
  analogWrite(enB, PwmB);
  if (dirA == 1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } 
  else {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  if (dirB == 1) {
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  } 
  else {
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  }
}
void loop() {
  actual = gyroscope();
  current_time = millis();
  delta_T = float(current_time - lasttime) / 1000.0000;
  lasttime = current_time;
  error = desired - actual;
  errorsum = errorsum + (error * delta_T);
  getspeedPwm = error * kp + ki * errorsum;

  if (getspeedPwm < 0) {
    directionA = 1;
    directionB = 1;
  } 
  else {
    directionA = 0;
    directionB = 0;
  }

  speedconPwmA = constrain(abs(getspeedPwm), 0, 255);
  speedPwmA = speedconPwmA;
  speedPwmA = speedPwmB
  carmove(directionA, directionB,speedPwmA,speedPwmB);
}
