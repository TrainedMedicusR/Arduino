#include <Servo.h>

#define STOP      0
#define FORWARD   1
#define BACKWARD  2
#define TURNLEFT  3
#define TURNRIGHT 4

Servo servoLeft;
Servo servoRight;

void backward(){
  servoLeft.attach(13);  // Attach left signal to pin 13
  servoRight.attach(12); // Attach right signal to pin 12
  servoLeft.writeMicroseconds(1700);  // Left wheel counterclockwise
  servoRight.writeMicroseconds(1300); // Right wheel clockwise
}

void forward(){
  servoLeft.attach(13);  // Attach left signal to pin 13
  servoRight.attach(12); // Attach right signal to pin 12
  servoLeft.writeMicroseconds(1300);  // Left wheel counterclockwise
  servoRight.writeMicroseconds(1700); // Right wheel clockwise
}

void left(){
  servoLeft.attach(13);  // Attach left signal to pin 13
  servoRight.attach(12); // Attach right signal to pin 12
  servoLeft.writeMicroseconds(1300);  // Left wheel counterclockwise
  servoRight.writeMicroseconds(1300); // Right wheel clockwise
}

void right(){
  servoLeft.attach(13);  // Attach left signal to pin 13
  servoRight.attach(12); // Attach right signal to pin 12
  servoLeft.writeMicroseconds(1700);  // Left wheel counterclockwise
  servoRight.writeMicroseconds(1700); // Right wheel clockwise
}

void stop1(){
  servoLeft.detach();
  servoRight.detach();
}




void setup() {
 pinMode(A0, INPUT);
 pinMode(A2, INPUT);
 pinMode(7, INPUT);
 Serial.begin(9600);
}

void loop() {
  int x = analogRead(A0)/100;
  int y = analogRead(A2)/100;
  int button = digitalRead(7);
  if(x<5){
    backward();
  }else if(x>5){
    forward();
  }else{
    if(y<5){
      left();
    }else if(y>5){
      right();
    }else{
      stop1();
    }
  }
}
