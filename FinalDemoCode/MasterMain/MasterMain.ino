/*
Group  W08-16
Final Demonstration Sketch
Master Board Code
  The bluetooth master board code for the system designed

  The master board mainly has three functions:
  1. Bluetooth Setup and send the command to the slave
  2. JoyStick Control
  3. Receive the data collected from slave board and output


This sketch was written by ELEC1601 group W08-16,
with lots of help from Arduino Community.

Version: 1.1.99
Date: November 5th,2019
Author: Wenbo Li, Tianshu Shen, Yichen Zhang
*/


//import the necessary class
#include <Servo.h>
#include <SoftwareSerial.h>

// Constants for Pin Setup and circuit connections 
#define RxD 7
#define TxD 6
#define DEBUG_ENABLED  1
#define STOP      0
#define FORWARD   1
#define BACKWARD  2
#define TURNLEFT  3
#define TURNRIGHT 4

// Customize the name of the master bluetooth and refer to the slave's bluetooth name

String masterNameCmd = "\r\n+STNA=Master16\r\n";
String slaveName = ";Slave16";

// Set the bluetooth connection identifier, data buffer

SoftwareSerial blueToothSerial(RxD,TxD);
String retSymb = "+RTINQ=";
int nameIndex = 0;
int addrIndex = 0;
String recvBuf;
String slaveAddr;
String connectCmd = "\r\n+CONN=";

// Declare the ServoMotor
Servo servoLeft;
Servo servoRight;


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Setup Sensors and Bluetooth~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

/* 
 * joyStickSetup() function  
 * setup the analog input pins and digital input for joystick
 * 
 */
void joyStickSetup(){
  pinMode(A0, INPUT);
  pinMode(A2, INPUT);
  pinMode(10, INPUT);
}

/* 
 * blueToothSetup() function  
 * setup the input and output pin for bluetooth
 * call the bluetoothConnection function
 * Flush and refresh the bluetooth buffer
 */
void blueToothSetup(){
    pinMode(RxD, INPUT);
    pinMode(TxD, OUTPUT);
    setupBlueToothConnection();
    //reset the buffer zone
    delay(1000);
    Serial.flush();
    blueToothSerial.flush();
}

/* 
 * setupBlueToothConnection() function  
 * Begin the bluetooth inquiry mode and pair it with the slave board
 * After establishment of the connection, send the command to the slave board
 *
 */
void setupBlueToothConnection()
{
    blueToothSerial.begin(38400);                 // Set BluetoothBee BaudRate to default baud rate 38400
    blueToothSerial.print("\r\n+STWMOD=1\r\n");   // set the bluetooth work in master mode
    blueToothSerial.print(masterNameCmd);         // set the bluetooth name as
    blueToothSerial.print("\r\n+STAUTO=0\r\n");   // Auto-connection is forbidden here
    delay(2000);                                  // This delay is required.

    blueToothSerial.flush();
    blueToothSerial.print("\r\n+INQ=1\r\n");      //make the master inquire
    Serial.println("Master is inquiring!");
    delay(2000); // Wait for the inquiring process

    /* 
    * this loop mainly functions as the paring process
    * Begin searching for the slave board
    * pair the slave board by verifying the name and address
    *
    */
    char recvChar;
    while(1)
    {
        if(blueToothSerial.available())
        {
            recvChar = blueToothSerial.read();
            recvBuf += recvChar;
            nameIndex = recvBuf.indexOf(slaveName);             //get the position of slave name
                                                                //nameIndex -= 1;
                                                                //get the position of the end of the slave address
            if ( nameIndex != -1 )
            {
                //Serial.print(recvBuf);
                addrIndex = (recvBuf.indexOf(retSymb,(nameIndex - retSymb.length()- 18) ) + retSymb.length());//get the start position of slave address
                slaveAddr = recvBuf.substring(addrIndex, nameIndex);//get the string of slave address
                break;
            }
        }
    }

    //test the stability of bluetooth connection and print to the console 
    connectCmd += slaveAddr;
    connectCmd += "\r\n";
    int connectOK = 0;
    Serial.print("Connecting to slave:");
    Serial.print(slaveAddr);
    Serial.println(slaveName);

    // The last step of the bluetooth connection
    // channel created
    // communication data verification
    do
    {
        blueToothSerial.print(connectCmd);//send connection command
        recvBuf = "";
        while(1)
        {
            if(blueToothSerial.available()){
                recvChar = blueToothSerial.read();
                recvBuf += recvChar;
                if(recvBuf.indexOf("CONNECT:OK") != -1)
                {
                    connectOK = 1;
                    Serial.println("Connected!");
                    blueToothSerial.print("Connected!");
                    break;
                }
                else if(recvBuf.indexOf("CONNECT:FAIL") != -1)
                {
                    Serial.println("Connect again!");
                    break;
                }
            }
        }
    }while(0 == connectOK);
}


/* 
 * setup() function  
 * Begin computer serial for testing
 * call all the other setup function to complete the setup
 *
 */
void setup() {
    Serial.begin(9600);
    joyStickSetup();
    blueToothSetup();
}




/* 
 * loop() function
 * Continuously read 2 voltage values from A0 and A2 
 * Convert these values to the x offsets and y offsets of controller
 * Interpret it to the move command of the slave board
 * Send the command via the bluetooth Connection
 *
 */
void loop() {
  int x = analogRead(A0)/100;
    int y = analogRead(A2)/100;
    int button = digitalRead(10);
    if(x<5){
        Serial.println("backward");
        blueToothSerial.print(',');
        blueToothSerial.flush();
    }else if(x>5){
      Serial.println("forward");
        blueToothSerial.print('.');
        blueToothSerial.flush();
    }else{
        if(y<5){
          Serial.println("left");
          blueToothSerial.print('[');
          blueToothSerial.flush();
        }else if(y>5){
          Serial.println("right");
          blueToothSerial.print(']');
          blueToothSerial.flush();
        }else{
          Serial.println("stop");
          blueToothSerial.print('=');
          blueToothSerial.flush();
        }
    }
}
