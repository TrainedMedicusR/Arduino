#include <Servo.h>

#define STOP      0
#define FORWARD   1
#define BACKWARD  2
#define TURNLEFT  3
#define TURNRIGHT 4

Servo servoLeft;
Servo servoRight;
Servo myServo;  //舵机

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
}


void setup() {
  // put your setup code here, to run once:
  //串口初始化
  servoLeft.attach(13);  // Attach left signal to pin 13
  servoRight.attach(12); // Attach right signal to pin 12
  //Motor Initialisation

  Serial.begin(9600); 
  //舵机引脚初始化
  myServo.attach(9);
  //测速引脚初始化
  // pinMode(leftPWM, OUTPUT);
  // pinMode(rightPWM, OUTPUT);
  //超声波控制引脚初始化
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  avoidance_test();
}

void motorRun(int cmd)
{
  // analogWrite(leftPWM, value);  //设置PWM输出，即设置速度
  // analogWrite(rightPWM, value);
  switch(cmd){
    case FORWARD:
      Serial.println("FORWARD"); //输出状态
      forward();
      break;
     case BACKWARD:
      Serial.println("BACKWARD"); //输出状态
      backward();
      break;
     case TURNLEFT:
      Serial.println("TURN  LEFT"); //输出状态
      left();
      break;
     case TURNRIGHT:
      Serial.println("TURN  RIGHT"); //输出状态
      right();
      break;
     default:
      Serial.println("STOP"); //输出状态
      stop1();
  }
}
void avoidance()
{
  int pos;
  int dis[3];//距离
  motorRun(FORWARD);
  myServo.write(90);
  dis[1]=getDistance(); //中间
  
  if(dis[1]<30)
  {
    motorRun(STOP);
    for (pos = 90; pos <= 150; pos += 1) 
    {
      myServo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15ms for the servo to reach the position
    }
    dis[2]=getDistance(); //左边
    for (pos = 150; pos >= 30; pos -= 1) 
    {
      myServo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15ms for the servo to reach the position
      if(pos==90)
        dis[1]=getDistance(); //中间
    }
    dis[0]=getDistance();  //右边
    for (pos = 30; pos <= 90; pos += 1) 
    {
      myServo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15ms for the servo to reach the position
    }
    if(dis[0]<dis[2]) //右边距离障碍的距离比左边近
    {
      //左转
      motorRun(TURNLEFT);
      delay(500);
    }
    else  //右边距离障碍的距离比左边远
    {
      //右转
      motorRun(TURNRIGHT);
      delay(500);
    } 
  }
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


void avoidance_test()
{
  int pos;
  int dis[3];//距离
  motorRun(FORWARD);
  myServo.write(90);
  dis[1]=getDistance(); //中间
  
  if(dis[1]<15)
  {
    motorRun(STOP);
    delay(2000);
    dis[2]=getDistance(); //左边
    delay(2000);
    dis[0]=getDistance();  //右边
    
    if(dis[0]<dis[2]) //右边距离障碍的距离比左边近
    {
      //左转
      motorRun(TURNLEFT);
      delay(500);
    }
    else  //右边距离障碍的距离比左边远
    {
      //右转
      motorRun(TURNRIGHT);
      delay(500);
    } 
  }
}
