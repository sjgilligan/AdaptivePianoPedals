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
int potInput1 = 14; //15
int potInput2 = 15;
int potInput3 = 16;
int touchInput1 = 21;
int touchInput2 = 22;

void setup() {
  Serial.begin(115200);

  pinMode(buttonInput1,INPUT);
  pinMode(buttonInput2,INPUT);
  pinMode(buttonInput3,INPUT);
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:


  char buffer[120];
  sprintf(buffer,"buttons: %d, %d, %d, pots: %d, %d, %d, touch: %d, %d",digitalRead(3),digitalRead(2),digitalRead(1),analogRead(potInput1),analogRead(potInput2),analogRead(potInput3),analogRead(touchInput1),analogRead(touchInput2));
  Serial.println(buffer);
  delay(10);
}
