#include <Wire.h>
#include <PWMServo.h>
#include "Qwiic_LED_Stick.h"

#define SENS_COEF_1 30

#define INITIAL_THRESH 1 
#define WAITING_THRESH 50
#define TOGGLE_THRESH 1400

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

int buttonInput1 = 0;
int buttonState1 = LOW;

control_state_t conState1 = MIN;
control_state_t last_conState1 = MIN;
int potInput1 = A1; //15
int sensorInput1 = A6; //20
int motorOutput1 = 14;

int potValue1 = 0;
int ledValue1 = -1;
int sensorValue1 = 0;
int last_ledValue1 = 0;

int maxDepres1 = 0;
int minDepres1 = 0;
int sens1 = 512; //min 0, max 1024
int pos1 = 0;
sustain_state_t susState1 = WAITING;

control_state_t cycle_state(control_state_t state){
  switch(state){
    case INIT:
      return MIN;
    case MIN:
      return MAX;
    case MAX:
      return SENS;
    default:
      return MIN; 
  }
}

void get_sensor_inputs(){
  
}

void get_pot_inputs(){

}

int get_led_value(int pot_value){
  int led_value;

  switch (pot_value / 100) {
    case 0:
        if (pot_value < 2) {
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
  
  return led_value;
}

void led_stick(LED &led, control_state_t state, int value){  
  if(value < 0){
    led.LEDOff();
  }else{
    switch(state){
      case MIN:
        led.LEDOff();
        led.setLEDColor(value, 255, 0, 0);
        break;
      case MAX:
        led.LEDOff();
        led.setLEDColor(value, 0, 255, 0);
        break;
      case SENS:
        led.LEDOff();
        led.setLEDColor(value, 0, 0, 255);
        break;
      default:
        led.LEDOff();
        led.setLEDColor(value, 100, 100, 100);
        break;
    }
  }
}

void setup() {
  Wire.begin();
  Serial.begin(115200);
  servo1.attach(motorOutput1);

  if (LEDStick1.begin() == false){
    Serial.println("Qwiic LED Stick failed to begin. Please check wiring and try again!");
    while(1);
  }

  Serial.println("Qwiic LED Stick ready!");
}

void loop() {
  sensorValue1 = analogRead(sensorInput1);
  sensorValue1 = sensorValue1 * sens1;

  potValue1 = analogRead(potInput1);

  Serial.print(sensorValue1);
  Serial.print("\n");

  //actuate
  if(susState1 == WAITING && sensorValue1 > 10){
    susState1 = INITIAL_PRESS; //wating -> inital_press
  }

  if(susState1 == INITIAL_PRESS && sensorValue1 < 10){
    susState1 = WAITING; //initial_press -> waiting
  }

  if(susState1 == INITIAL_PRESS && sensorValue1 > TOGGLE_THRESH){
    while(1){
      sensorValue1 = analogRead(sensorInput1);
      sensorValue1 = sensorValue1 * sens1;
      susState1 = TOGGLE; //initial_press -> toggle
      pos1 = map(sensorValue1,0,1023,0,179);
      servo1.write(pos1);

      sens1 = log((potValue1 / SENS_COEF_1) + 1);
      ledValue1 = get_led_value(sensorValue1);
      Serial.print(sensorValue1);
      Serial.print("\n");


      delay(1); 



    }
  }

  // if(susState1 == TOGGLE){
  //   //delay(500);
  //   susState1 = WAITING; //toggle -> waiting
  //   //ITimer.stopTimer();
  //   //timer_counter = 0;
  // }

  //button check (could be thread)
  buttonState1 = digitalRead(buttonInput1);
  
  if(buttonState1 == HIGH){
    conState1 = cycle_state(conState1);
    Serial.print(conState1);
    Serial.print("\n");
    led_stick(LEDStick1,conState1,ledValue1);
    delay(1000);
  }

  if(conState1 == SENS){
    sens1 = log((potValue1 / SENS_COEF_1) + 1);
    ledValue1 = get_led_value(sensorValue1);
    Serial.print(sensorValue1);
    Serial.print("\n");
  }else{
    potValue1 = analogRead(potInput1);
    ledValue1 = get_led_value(potValue1);
  }
  
  if(last_ledValue1 != ledValue1){
    Serial.print(ledValue1);
    Serial.print("\n");
    led_stick(LEDStick1,conState1,ledValue1);

    last_ledValue1 = ledValue1;
  }


  //Serial.print(potValue1);


  delay(1);
}
