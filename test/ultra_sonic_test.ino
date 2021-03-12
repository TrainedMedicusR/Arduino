#include <Servo.h>

Servo servoLeft;
Servo servoRight;
int trigPin=7;   // 定义超声波信号接收接口
int echoPin=8;  // 定义超声波信号发出接口

void backward(){
  servoLeft.attach(13);  // Attach left signal to pin 13
  servoRight.attach(12); // Attach right signal to pin 12
  servoLeft.writeMicroseconds(1700);  // Left wheel counterclockwise
  servoRight.writeMicroseconds(1300); // Right wheel clockwise
  delay(500);                          // Maneuver for time ms
}

void forward(){
  servoLeft.attach(13);  // Attach left signal to pin 13
  servoRight.attach(12); // Attach right signal to pin 12
  servoLeft.writeMicroseconds(1300);  // Left wheel counterclockwise
  servoRight.writeMicroseconds(1700); // Right wheel clockwise
  delay(500);                          // Maneuver for time ms
}

void left(){
  servoLeft.attach(13);  // Attach left signal to pin 13
  servoRight.attach(12); // Attach right signal to pin 12
  servoLeft.writeMicroseconds(1300);  // Left wheel counterclockwise
  servoRight.writeMicroseconds(1300); // Right wheel clockwise
  delay(550);                          // Maneuver for time ms
}

void right(){
  servoLeft.attach(13);  // Attach left signal to pin 13
  servoRight.attach(12); // Attach right signal to pin 12
  servoLeft.writeMicroseconds(1700);  // Left wheel counterclockwise
  servoRight.writeMicroseconds(1700); // Right wheel clockwise
  delay(550);                          // Maneuver for time ms
}

void stop1(){
  servoLeft.detach();
  servoRight.detach();
  delay(4000);
}

void setup() {
  servoLeft.attach(13);  // Attach left signal to pin 13
  servoRight.attach(12); // Attach right signal to pin 12
  //Motor Initialisation
  Serial.begin(9600); 
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

}

int getDistance()
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
 
  long duration = pulseIn(echoPin, HIGH);
 
  long distance = (duration/2) / 29.1;     // Divide by 29.1 or multiply by 0.034
  if (distance >=50)
  {
    //如果距离小于50厘米返回数据
    return 50;
  }//如果距离小于50厘米小灯熄灭
  else
    return distance;
}



void loop() {
  Serial.println(getDistance());
  delay(1000);
}
