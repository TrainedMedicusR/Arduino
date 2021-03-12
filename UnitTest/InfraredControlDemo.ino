#include <Servo.h> 

Servo servoLeft;
Servo servoRight;

void setup()                                 // Built-in initialization block
{
  servoLeft.attach(13);  // Attach left signal to pin 13
  servoRight.attach(12); // Attach right signal to pin 12
  tone(4, 3000, 1000);                       // Play tone for 1 second
  delay(1000);                               // Delay to finish tone

  pinMode(10, INPUT);  pinMode(9, OUTPUT);   // Left IR LED & Receiver

  Serial.begin(9600);                        // Set data rate to 9600 bps
}  
 
void loop()                                  // Main loop auto-repeats
{
  int irLeft = irDetect(9, 10, 38000);       // Check for object
  int irRight = irDetect(2, 3, 38000);
  Serial.println(irLeft);// Display 1/0 no detect/detect
  forward();
  if (irLeft==0 & irRight == 0){
    backward();
  }else if(irLeft == 0){
    right();
  }else if(irRight == 0){
    left();
  }
  //delay(100);                                // 0.1 second delay
}

// IR Object Detection Function

void backward(){
  servoLeft.writeMicroseconds(1700);  // Left wheel counterclockwise
  servoRight.writeMicroseconds(1300); // Right wheel clockwise
  delay(500);                          // Maneuver for time ms
}

void forward(){
  servoLeft.writeMicroseconds(1300);  // Left wheel counterclockwise
  servoRight.writeMicroseconds(1700); // Right wheel clockwise
  delay(50);                          // Maneuver for time ms
}

void left(){
  servoLeft.writeMicroseconds(1300);  // Left wheel counterclockwise
  servoRight.writeMicroseconds(1300); // Right wheel clockwise
  delay(550);                          // Maneuver for time ms
}

void right(){
  servoLeft.writeMicroseconds(1700);  // Left wheel counterclockwise
  servoRight.writeMicroseconds(1700); // Right wheel clockwise
  delay(550);                          // Maneuver for time ms
}

int irDetect(int irLedPin, int irReceiverPin, long frequency)
{
  tone(irLedPin, frequency, 8);              // IRLED 38 kHz for at least 1 ms
  delay(1);                                  // Wait 1 ms
  int ir = digitalRead(irReceiverPin);// IR receiver -> ir variable
  delay(1);                                  // Down time before recheck
  return ir;                                 // Return 1 no detect, 0 detect
}
