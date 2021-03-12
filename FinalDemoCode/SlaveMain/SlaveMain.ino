/*
Group  W08-16
Final Demonstration Sketch
Slave Board Code
  The bluetooth slave board and Boe-Bot code for the system designed

  The slave board mainly has three functions:
  1. Bluetooth Setup and read the command sent from master board
  2. Boe-bot move. servos output 
  3. Automatic obstacle avoidance function
  4. Read data from sensor


This sketch was written by ELEC1601 group W08-16,
with lots of help from Arduino Community.

Version: 1.1.99
Date: November 5th,2019
Author: Wenbo Li, Tianshu Shen, Yichen Zhang
*/

//import necessary class
#include <Servo.h>
#include <SoftwareSerial.h>

// Constants for Pin Setup and command
#define RxD 7
#define TxD 6
#define DEBUG_ENABLED  1
#define STOP      0
#define FORWARD   1
#define BACKWARD  2
#define TURNLEFT  3
#define TURNRIGHT 4
#define trigPin   9   // ultra-sonic trig pin
#define echoPin   8   // ultra-sonic echo pin 
#define sensorPin 5   // temperature sensor pin
#define buzzerPin 10


// Customize the name of the slave bluetooth and set the avoidance function control boolean

String slaveNameCmd = "\r\n+STNA=Slave16\r\n";
boolean avoid = false;

// Set the bluetooth connection identifier, data buffer

SoftwareSerial blueToothSerial(RxD,TxD);
int nameIndex = 0;
int addrIndex = 0;
String recvBuf;
String slaveAddr;
String connectCmd = "\r\n+CONN=";

// Declare the ServoMotors and the servo to control the orientation of ultrasonic sensor
Servo servoLeft;
Servo servoRight;
Servo myServo;


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Servo Motor Run Code~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/* 
 * backward() function  
 * attach the servo
 * set specific servo output values to make the bot move backward
 *
 */

void backward(){
  servoLeft.attach(13);  // Attach left signal to pin 13
  servoRight.attach(12); // Attach right signal to pin 12
  servoLeft.writeMicroseconds(1700);  // Left wheel counterclockwise
  servoRight.writeMicroseconds(1300); // Right wheel clockwise
}

/* 
 * forward() function  
 * attach the servo
 * set specific servo output values to make the bot move forward
 *
 */
void forward(){
  servoLeft.attach(13);  // Attach left signal to pin 13
  servoRight.attach(12); // Attach right signal to pin 12
  servoLeft.writeMicroseconds(1300);  // Left wheel counterclockwise
  servoRight.writeMicroseconds(1700); // Right wheel clockwise
}

/* 
 * left() function  
 * attach the servo
 * set specific servo output values to make the bot turn left
 */
void left(){
  servoLeft.attach(13);  // Attach left signal to pin 13
  servoRight.attach(12); // Attach right signal to pin 12
  servoLeft.writeMicroseconds(1300);  // Left wheel counterclockwise
  servoRight.writeMicroseconds(1300); // Right wheel clockwise
}

/* 
 * backward() function  
 * attach the servo
 * set specific servo output values to make the bot turn right
 */
void right(){
  servoLeft.attach(13);  // Attach left signal to pin 13
  servoRight.attach(12); // Attach right signal to pin 12
  servoLeft.writeMicroseconds(1700);  // Left wheel counterclockwise
  servoRight.writeMicroseconds(1700); // Right wheel clockwise
}

/* 
 * stop() function  
 * detach the servo
 * make the bot stop
 */
void stop1(){
  servoLeft.detach();
  servoRight.detach();
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Bluetooth Code~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

/* 
 * blueToothSetup() function  
 * setup the input and output pin for bluetooth
 * call the bluetoothConnection function
 */
void blueToothSetup(){
    pinMode(RxD, INPUT);
    pinMode(TxD, OUTPUT);
    setupBlueToothConnection();
}



/* 
 * setup() function  
 * Begin computer serial for testing
 * call all the other setup function to complete the setup
 * setup the buzzerPin
 * setup the sensorPin
 */
void setup() {
    Serial.begin(9600);
    //joyStickSetup();
    blueToothSetup();
    ultraSonicSetup();
    pinMode(buzzerPin,OUTPUT);
    pinMode(sensorPin,INPUT);
}

/* 
 * loop() function
 * call the blueToothLoop function
 * 
 */
void loop() {
    bluetoothLoop();
}

/* 
 * ultraSonicSetup() function
 * setup the ultrasonic sensor
 * attach the servo and wait for ultrasonic control
 *
 */
void ultraSonicSetup(){
  //Motor Initialisation
  servoLeft.attach(13);  // Attach left signal to pin 13
  servoRight.attach(12); // Attach right signal to pin 12

  //initiate the servo to control the ultrasonic sensor orientation
  myServo.attach(4);
  //initiate the control pins of ultrasonic sensor
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}


/* 
 * blueToothLoop() function  
 * Continuously listen to the command from the masterboard 
 * Simultaneously sent back the data collected
 * Interpret the command and make the motor run
 * Note: auto-avoidance system is embeded in the bluetoothLoop
 *
 */
void bluetoothLoop()
{
    char recvChar;
    while(1)
    {
      //call the avoidance function
      avoidance();
      if(blueToothSerial.available())

        {//Listen to the data sent from the master board
            recvChar = blueToothSerial.read();
            //interpret different command sent from the master board and respond
            if (!avoid){
              if(recvChar == ']'){
                motorRun(TURNLEFT);
              }else if(recvChar == '['){
                motorRun(TURNRIGHT);
              }else if(recvChar == ','){
                motorRun(FORWARD);
              }else if(recvChar == '.'){
                motorRun(BACKWARD);
              }else {
                motorRun(999);
              }
              Serial.print(recvChar);  
            }
        }

      if(Serial.available())

        {//check if there's any data sent from the local serial terminal, you can add the other applications here

            recvChar  = Serial.read();

            blueToothSerial.print(recvChar); 

        }
    }
}


/* 
 * setupBlueToothConnection() function  
 * Begin the bluetooth inquiry mode and pair it with the master board
 * After initial establishment of the connection, flush the buffer
 * Listen to the command sent from the master board
 *
 */
void setupBlueToothConnection()
{
    blueToothSerial.begin(38400);                           // Set the bluetooth communication serial to 38400
    blueToothSerial.print("\r\n+STWMOD=0\r\n");             // set the bluetooth working mode: slave
    blueToothSerial.print(slaveNameCmd);                    // set the customized bluetooth name 
    blueToothSerial.print("\r\n+STOAUT=1\r\n");             // set the connection authentication to True
    blueToothSerial.print("\r\n+STAUTO=0\r\n");             // Forbid the autoconnection
    delay(2000);                                            

    blueToothSerial.print("\r\n+INQ=1\r\n");                // begin inquiring
    Serial.println("The slave bluetooth is inquirable!");
    delay(2000);                                           

    blueToothSerial.flush();
}


/* 
 * avoidance() function  
 * Get the distance from the ultrasonic sensor, to check if there is a front obstacle
 * If there is, override the bluetooth control signal
 * Use servo to turn the sensor to check the left side obstacle and the right side obstacle.
 * If there is obstacle on the left, turn right; Otherwise, turn left;
 * Then continue bluetooth signal
 *
 */
void avoidance()
{
  int pos = 0;
  int dis[3] = {0,0,0};//distance array for [left, front, right]
  
  myServo.write(90);
  dis[1]=getDistance();// get the front distance first
  
  Serial.println(dis[1]);

  if(dis[1]<20)
  {
    tone(buzzerPin, 1000, 1000);
    avoid = true;
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
        dis[1]=getDistance(); //front distance
    }
    dis[0]=getDistance();  //right distance
    for (pos = 30; pos <= 90; pos += 1) 
    {
      myServo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15ms for the servo to reach the position
    }
    if(dis[0]<dis[2]) //check the distance of right and left
    {
      //turn left
      motorRun(TURNLEFT);
      delay(500);
    }
    else  
    {
      //turn right
      motorRun(TURNRIGHT);
      delay(500);
    } 
  }else{
    avoid = false;
  }
}

/* 
 * getDistance() function  
 * Use the ultrasonic sensor to send the ultrasonic wave to front 
 * Receive the reflective wave and calculate the voltage by pulseIn() function in Arduino Library
 * convert the voltage to centimeters 
 *
 */
int getDistance()
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
 
  long distance = (duration/2) / 29.1;     // Divide by 29.1 or multiply by 0.034 to get the distance in cm
  if (distance >=50){
    //if the distance is more than 50cm, just return the max value 50
    return 50;
  }
  else
    return distance;
}

/* 
 * motorRun() function  
 * This is a wrapper function for servo control.
 * There are 4 different move mode of out bot(Forward, Backward, Left, Right),
 * The input value represents the specific move mode of the bot
 * This wrapper is used to simplify the control unit
 * inputs:
 *  int cmd - integer value of move command
 *
 */
void motorRun(int cmd)
{
  switch(cmd){
    case FORWARD:
      Serial.println("FORWARD"); 
      forward();
      break;
     case BACKWARD:
      Serial.println("BACKWARD"); 
      backward();
      break;
     case TURNLEFT:
      Serial.println("TURN  LEFT");
      left();
      break;
     case TURNRIGHT:
      Serial.println("TURN  RIGHT"); 
      right();
      break;
     default:
      Serial.println("STOP");
      stop1();
  }
}
