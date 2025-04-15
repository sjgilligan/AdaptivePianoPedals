/*
  An I2C based LED Stick
  By: Ciara Jekel
  SparkFun Electronics
  Date: June 11th, 2018
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/14783

  This example blinks the entire LED Stick.

*/

#include <Wire.h>
#include "Qwiic_LED_Stick.h" // Click here to get the library: http://librarymanager/All#SparkFun_Qwiic_LED_Stick

LED LEDStick1; //Create an object of the LED class
LED LEDStick2; //Create an object of the LED class
LED LEDStick3; //Create an object of the LED class

void setup() {
  Wire.begin();
  Serial.begin(115200);
  
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
  //Set all LEDs the same color (white)
  LEDStick1.setLEDColor(50, 50, 50);
  LEDStick2.setLEDColor(50, 50, 50);
  LEDStick3.setLEDColor(50, 50, 50);
  delay(1000);
  //Turn off all LEDs
  LEDStick1.LEDOff();
  LEDStick2.LEDOff();
  LEDStick3.LEDOff();
  delay(1000);

}
