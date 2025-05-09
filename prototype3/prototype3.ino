#include <Wire.h>
#include "Servo.h"
#include <Arduino.h>
#include "TeensyThreads.h"
#include "Qwiic_LED_Stick.h"


#define SENS_COEF_1 200

#define INITIAL_THRESH 1 
#define WAITING_THRESH 50
#define TOGGLE_THRESH 600
#define POT_CHANGE 10

typedef enum {
  WAITING, INITIAL_PRESS, TOGGLE,
} sustain_state_t;

typedef enum
{
  INIT,
  MIN,
  MAX,
  SENS,
} control_state_t;

PWMServo servo1;
LED LEDStick1;

int ledStickOutput1 = 10;

int buttonInput1 = 0;
int buttonState1 = LOW;

control_state_t conState1 = MIN;
control_state_t last_conState1 = MIN;
int potInput1 = A1; //15
int sensorInput1 = A6; //20
int motorOutput1 = 14;

int potValue1 = 0;
//int ledValue1 = -1;
int sensorValue1 = 0;
//int last_ledValue1 = 0;

int maxDepres1 = 2000;
int minDepres1 = 1000;
int sens1 = 1; //min 0, max 1024
int pos1 = 0; //lin servo is 1000-2000
sustain_state_t susState1 = WAITING;

void cycle_conState1(){
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  if (interrupt_time - last_interrupt_time > 200) 
  {
  switch(conState1){
    case INIT:
      conState1 = MIN;
      break;
    case MIN:
      conState1 = MAX;
      break;
    case MAX:
       conState1 = SENS;
       break;
    default:
       conState1 = MIN;
       break;
  }
  }
  last_interrupt_time = interrupt_time;
  //led_stick(LEDStick1,conState1,ledValue1);
  //delay(1000);
}





int get_led_value(int input_value, int state){
  int led_value;

  if(state == SENS){
    led_value = map(input_value,0,1023,0,9);
  }else{

  switch (input_value/18) {
    case 0:
        if (input_value < 2) {
            led_value = -1;
        } else {
            led_value = 0;
        }
        break;
    case 1:
        led_value = 1;
        break;
    case 2:
        led_value = 2;
        break;
    case 3:
        led_value = 3;
        break;
    case 4:
        led_value = 4;
        break;
    case 5:
        led_value = 5;
        break;
    case 6:
        led_value = 6;
        break;
    case 7:
        led_value = 7;
        break;
    case 8:
        led_value = 8;
        break;
    default:
        led_value = 9;
        break;
  }
  }
  
  return led_value;
}

void led_stick(LED &led, control_state_t state, int min, int max, int sens){  
  if(min == max){
    if(max != 9){
      max = max + 1;
    }else if(max == 9){
      min = 8;
    }
  }
  
  if(min < 0 && max < 0 && sens < 0){
    led.LEDOff();
  }else{
    switch(state){
      case MIN:
        led.LEDOff();
        
        led.setLEDColor(max, 0, 124, 0);
        led.setLEDColor(min, 124, 0, 0);
        // Serial.print("MIN ");
        // Serial.print(min);
        // Serial.print("MAX ");
        // Serial.println(max);
        break;
      case MAX:
        led.LEDOff();
        
        led.setLEDColor(max, 0, 124, 0);
        led.setLEDColor(min, 124, 0, 0);
        // Serial.print("MIN ");
        // Serial.print(min);
        // Serial.print("MAX ");
        // Serial.println(max);
        break;
      case SENS:
        led.LEDOff();
        led.setLEDColor(sens, 0, 0, 124);
        break;
      default:
        led.LEDOff();
        led.setLEDColor(5, 100, 100, 100);
        break;
    }
  }
}

void sustain_control(){
  if(susState1 == WAITING && sensorValue1 > 10){
    susState1 = INITIAL_PRESS; //wating -> inital_press
  }

  if(susState1 == INITIAL_PRESS && sensorValue1 < 10){
    susState1 = WAITING; //initial_press -> waiting
  }

  if(susState1 == INITIAL_PRESS && sensorValue1 > TOGGLE_THRESH){
    susState1 = TOGGLE; //initial_press -> toggle
  }

  if(susState1 == TOGGLE && sensorValue1 > TOGGLE_THRESH){
    pos1 = map(sensorValue1,0,1023,minDepres1,maxDepres1);
    //Serial.println(pos1);
    servo1.write(pos1);
    delay(1); 
  }else{
    susState1 = WAITING;
    servo1.write(minDepres1);
  }



}

void get_sensor_inputs(){
  sensorValue1 = analogRead(sensorInput1);
  sensorValue1 = sensorValue1 * sens1;
}

void get_pot_inputs(){
  // static int old_sens1 = 1;
  // static int old_maxDepres1 = 179;
  // static int old_minDepres1 = 0;

  static int last_potValue1;
  int loc_potValue1;

  loc_potValue1 = analogRead(potInput1);
  //Serial.println(loc_potValue1);
  if(abs(loc_potValue1 - last_potValue1) > POT_CHANGE){
    potValue1 = loc_potValue1;
    if(conState1 == SENS){
      // minDepres1 = old_minDepres1;
      // maxDepres1 = old_maxDepres1;
      //sens1 = log((potValue1 / SENS_COEF_1) + 1);
      if(potValue1 < 5){
        sens1 = 0;
      }else{
        sens1 = map(potValue1,0,1024,1,5);
      }
      //Serial.println(sens1);
      //old_sens1 = sens1;
    }else if(conState1 == MAX){
      // sens1 = old_sens1;
      // minDepres1 = old_minDepres1;
      maxDepres1 = map(potValue1,0,1023,0,179);
      //old_maxDepres1 = maxDepres1;
    }else if(conState1 == MIN){
      // maxDepres1 = old_maxDepres1;
      // sens1 = old_sens1;
      minDepres1 = map(potValue1,0,1023,0,179);
      //old_minDepres1 = minDepres1;
    }
    last_potValue1 = loc_potValue1;
  }
}

void setup() {
  Wire.begin();
  Serial.begin(115200);

  pinMode(buttonInput1,INPUT);
  pinMode(ledStickOutput1,OUTPUT);
  digitalWrite(ledStickOutput1,HIGH);
  servo1.attach(motorOutput1, 1000, 2000);

  servo1.write(0);
  delay(1000);

  attachInterrupt(digitalPinToInterrupt(buttonInput1), cycle_conState1, RISING);

  if (LEDStick1.begin() == false){
    Serial.println("Qwiic LED Stick failed to begin. Please check wiring and try again!");
    while(1);
  }

  LEDStick1.setLEDBrightness(5);
  //led_stick(LEDStick1,MIN,0,9,0);
  Serial.println("Qwiic LED Stick ready!");
}

void loop() {
  get_sensor_inputs();
  get_pot_inputs();


  char buffer[40];
  sprintf(buffer,"min: %d, max: %d, sens: %d, pos: %d, pot: %d",minDepres1,maxDepres1,sensorValue1,pos1,potValue1);
  //Serial.println(buffer);
  sustain_control();
  
  led_stick(LEDStick1,conState1,get_led_value(minDepres1,conState1),get_led_value(maxDepres1,conState1),get_led_value(sensorValue1,conState1));

  delay(10);
}
