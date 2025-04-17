#include <Wire.h>
#include <PWMServo.h>
#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "TeensyThreads.h"
#include "Qwiic_LED_Stick.h"
#include "Timer.h"

int buttonInput1 = 2;
int buttonInput2 = 1;
int buttonInput3 = 3;
int potInput1 = 14; 
int potInput2 = 15;
int potInput3 = 16;
int touchInput1 = 21;
int touchInput2 = 22;
int servoOutput1 = 10;
int servoOutput2 = 11;
int servoOutput3 = 12;

LED LEDStick1; //Create an object of the LED class
LED LEDStick2; //Create an object of the LED class
LED LEDStick3; //Create an object of the LED class

PWMServo myservo1;  // create servo object to control a servo
PWMServo myservo2;  // create servo object to control a servo
PWMServo myservo3;  // create servo object to control a servo

int pos = 0;    // variable to store the servo position

void setup() {
  Serial.begin(115200);

  pinMode(buttonInput1,INPUT);
  pinMode(buttonInput2,INPUT);
  pinMode(buttonInput3,INPUT);
  myservo1.attach(servoOutput1);  // attaches the servo on pin 9 to the servo object
  myservo2.attach(servoOutput2);  // attaches the servo on pin 9 to the servo object
  myservo3.attach(servoOutput3);  // attaches the servo on pin 9 to the servo object

  Wire.begin();
  
  //Start up communication with the LED Stick
  if (LEDStick1.begin(0x23) == false){
    Serial.println("Qwiic LED Stick1 failed to begin. Please check wiring and try again!");
    while(1);
  }

  Serial.println("Qwiic LED Stick ready!");

    //Start up communication with the LED Stick
  if (LEDStick2.begin(0x24) == false){
    Serial.println("Qwiic LED Stick2 failed to begin. Please check wiring and try again!");
    while(1);
  }

  Serial.println("Qwiic LED Stick ready!");

    //Start up communication with the LED Stick
  if (LEDStick3.begin(0x25) == false){
    Serial.println("Qwiic LED Stick3 failed to begin. Please check wiring and try again!");
    while(1);
  }

  Serial.println("Qwiic LED Stick ready!");

}

void loop() {
  // put your main code here, to run repeatedly:
  LEDStick1.setLEDColor(50, 50, 50);
  LEDStick2.setLEDColor(50, 50, 50);
  LEDStick3.setLEDColor(50, 50, 50);
  LEDStick1.setLEDBrightness(1);
  LEDStick2.setLEDBrightness(1);
  LEDStick3.setLEDBrightness(1);
  for(pos = 0; pos < 180; pos += 1) { // goes from 0 degrees to 180 degrees, 1 degree steps
    myservo1.write(pos);              // tell servo to go to position in variable 'pos'
    myservo2.write(pos);    
    myservo3.write(pos);    
    char buffer[120];
    sprintf(buffer,"buttons: %d, %d, %d, pots: %d, %d, %d, touch: %d, %d",digitalRead(3),digitalRead(2),digitalRead(1),analogRead(potInput1),analogRead(potInput2),analogRead(potInput3),analogRead(touchInput1),analogRead(touchInput2));
    Serial.println(buffer);
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  //Turn off all LEDs
  LEDStick1.LEDOff();
  LEDStick2.LEDOff();
  LEDStick3.LEDOff();
  for(pos = 180; pos>=1; pos-=1) {   // goes from 180 degrees to 0 degrees
    myservo1.write(pos);              // tell servo to go to position in variable 'pos'
    myservo2.write(pos);    
    myservo3.write(pos);             // tell servo to go to position in variable 'pos'
    char buffer[120];
    sprintf(buffer,"buttons: %d, %d, %d, pots: %d, %d, %d, touch: %d, %d",digitalRead(3),digitalRead(2),digitalRead(1),analogRead(potInput1),analogRead(potInput2),analogRead(potInput3),analogRead(touchInput1),analogRead(touchInput2));
    Serial.println(buffer);
    delay(15);                       // waits 15ms for the servo to reach the position
  }
}