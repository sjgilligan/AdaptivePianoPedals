#include <Wire.h>
#include <PWMServo.h>
#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "TeensyThreads.h"
#include "Qwiic_LED_Stick.h"
#include "Timer.h"

#define SENS_COEF_1 200

#define INITIAL_THRESH 1 
#define WAITING_THRESH 50
#define TOGGLE_THRESH 600
#define POT_CHANGE 10
#define MAX_DURATION_CUTOFF 10000

typedef enum {
  WAITING, INITIAL_PRESS, TOGGLE, DEPRESS
} sustain_state_t;

typedef enum
{
  INIT,
  MIN,
  MAX,
  MAX_DURATION,
  SENS,
} control_state_t;

PWMServo servo1;
PWMServo servo2;
PWMServo servo3;

LED LEDStick1;
LED LEDStick2;
LED LEDStick3;
Timer sustain_timer;

Adafruit_MPU6050 mpu;

float prevAccelz = 0.0;
float accelZ;
float diff;

// int ledStickOutput1 = 10;
// int ledStickOutput2 = 10;
// int ledStickOutput3 = 10;

int buttonInput1 = 2;
int buttonInput2 = 1;
int buttonInput3 = 0;
int buttonState1 = LOW;
int buttonState2 = LOW;
int buttonState3 = LOW;

int potInput1 = 14; //15
int potInput2 = 15;
int potInput3 = 16;
int potValue1 = 0;
int potValue2 = 0;
int potValue3 = 0;

control_state_t conState1 = MIN;
control_state_t conState2 = MIN;
control_state_t conState3 = MIN;

int sensorInput1 = 21;
int sensorInput2 = 22;
int sensorValue1 = 0;
int sensorValue2 = 0;

int motorOutput1 = 10;
int motorOutput2 = 11;
int motorOutput3 = 12;

int maxDepres1 = 179;
int maxDepres2 = 179;
int maxDepres3 = 179;
int minDepres1 = 0;
int minDepres2 = 0;
int minDepres3 = 0;

int sens1 = 1; //min 0, max 1024
int sens2 = 1;
int sens3 = 1;

int pos1 = 0;
int pos2 = 0;
int pos3 = 0;

int sustain_duration = 1000;
int max_sustain_duration = 4000;
sustain_state_t susState1 = WAITING;

bool sensor_activated = true;

void cycle_conState1(){
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  if (interrupt_time - last_interrupt_time > 200) 
  {
  switch(conState1){
    case INIT:
      while(minDepres1 < map(potValue1,0,1023,0,179)){
        servo1.write(minDepres1);
      } 
      conState1 = MIN;
      break;
    case MIN:
      while(minDepres1 < map(potValue1,0,1023,0,179)){
        servo1.write(minDepres1);
      } 
      conState1 = MAX;
      break;
    case MAX:
      servo1.write(minDepres1);
      conState1 = MAX_DURATION;
      break;
    case MAX_DURATION:
		  servo1.write(minDepres1);
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

void cycle_conState2(){
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  if (interrupt_time - last_interrupt_time > 200) 
  {
  switch(conState2){
    case INIT:
      while(minDepres2 < map(potValue2,0,1023,0,179)){
        servo1.write(minDepres2);
      } 
      conState2 = MIN;
      break;
    case MIN:
      while(minDepres2 < map(potValue2,0,1023,0,179)){
        servo1.write(minDepres2);
      } 
      conState2 = MAX;
      break;
    case MAX:
      servo1.write(minDepres2);
      conState2 = MAX_DURATION;
      break;
    case MAX_DURATION:
		  servo1.write(minDepres2);
		  conState2 = SENS;
      break;
    default:
      conState2 = MIN;
      break;
  }
  }
  last_interrupt_time = interrupt_time;
  //led_stick(LEDStick1,conState1,ledValue1);
  //delay(1000);
}

void cycle_conState3(){
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  if (interrupt_time - last_interrupt_time > 200) 
  {
  switch(conState3){
    case INIT:
      while(minDepres3 < map(potValue3,0,1023,0,179)){
        servo1.write(minDepres3);
      } 
      conState3 = MIN;
      break;
    case MIN:
      while(minDepres3 < map(potValue3,0,1023,0,179)){
        servo1.write(minDepres3);
      } 
      conState3 = MAX;
      break;
    case MAX:
      servo1.write(minDepres3);
      conState3 = MAX_DURATION;
      break;
    case MAX_DURATION:
		  servo1.write(minDepres3);
		  conState3 = SENS;
      break;
    default:
      conState3 = MIN;
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
  }else if(state == MAX_DURATION){
    led_value = map(input_value,0,4000,0,9);
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

void led_stick(LED &led, control_state_t state, int min, int max, int duration, int sens){  
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
	    case MAX_DURATION:
		    led.LEDOff();
        led.setLEDColor(duration, 124, 124, 0);
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
  int sustain_counter = 0;
  if(sensor_activated == true){
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
      //delay(300);
      sustain_duration = map(sensorValue1,TOGGLE_THRESH,1023,0,max_sustain_duration);
      susState1 = DEPRESS;
      pos1 = maxDepres1;
    }if(susState1 == DEPRESS && sustain_counter < sustain_duration){
      do{
        servo1.write(pos1);
        sustain_counter++;
        delay(1);
      }while(sustain_counter < sustain_duration);
      sustain_counter = 0;
      susState1 = WAITING;
      servo1.write(minDepres1);
      pos1 = minDepres1;
    }else if(susState1 == DEPRESS){
	    //sustain_timer.stop();
      
    }
  }
}

void get_touch_sensor_inputs(){
  sensorValue1 = analogRead(sensorInput1);
  sensorValue1 = sensorValue1 * sens1;

  sensorValue2 = analogRead(sensorInput2);
  sensorValue2 = sensorValue2 * sens2;
}

void get_imu_sensor_inputs(){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  accelZ = a.acceleration.z;
}

void get_pot_inputs(){
  // static int old_sens1 = 1;
  // static int old_maxDepres1 = 179;
  // static int old_minDepres1 = 0;

  static int last_potValue1;
  int loc_potValue1;

  static int last_potValue2;
  int loc_potValue2;

  static int last_potValue3;
  int loc_potValue3;

  loc_potValue1 = analogRead(potInput1);
  //Serial.println(loc_potValue1);
  if(abs(loc_potValue1 - last_potValue1) > POT_CHANGE){
    potValue1 = loc_potValue1;
    if(conState1 == SENS){
      sensor_activated = true;
      if(potValue1 < 5){
        sens1 = 0;
      }else{
        sens1 = map(potValue1,0,1024,1,5);
      }
    }else if(conState1 == MAX){
      sensor_activated = false;
      maxDepres1 = map(potValue1,0,1023,0,179);
      if(maxDepres1 < minDepres1){
        maxDepres1 = minDepres1;
      }
      pos1 = maxDepres1;
      servo1.write(pos1);
      //delay(1);
    }else if(conState1 == MIN){
      sensor_activated = true;
      minDepres1 = map(potValue1,0,1023,0,179);
      if(minDepres1 > maxDepres1){
        minDepres1 = maxDepres1;
      }
    }else if(conState1 == MAX_DURATION){
      sensor_activated = true;
      max_sustain_duration = map(potValue1,0,1023,0,MAX_DURATION_CUTOFF);
	}
    last_potValue1 = loc_potValue1;
  }

  loc_potValue2 = analogRead(potInput2);
  //Serial.println(loc_potValue2);
  if(abs(loc_potValue2 - last_potValue2) > POT_CHANGE){
    potValue2 = loc_potValue2;
    if(conState2 == SENS){
      sensor_activated = true;
      if(potValue2 < 5){
        sens2 = 0;
      }else{
        sens2 = map(potValue2,0,1024,1,5);
      }
    }else if(conState2 == MAX){
      sensor_activated = false;
      maxDepres2 = map(potValue2,0,1023,0,179);
      if(maxDepres2 < minDepres2){
        maxDepres2 = minDepres2;
      }
      pos2 = maxDepres2;
      servo2.write(pos2);
      //delay(2);
    }else if(conState2 == MIN){
      sensor_activated = true;
      minDepres2 = map(potValue2,0,1023,0,179);
      if(minDepres2 > maxDepres2){
        minDepres2 = maxDepres2;
      }
    }else if(conState2 == MAX_DURATION){
      sensor_activated = true;
      max_sustain_duration = map(potValue2,0,1023,0,MAX_DURATION_CUTOFF);
	}
    last_potValue2 = loc_potValue2;
  }

  loc_potValue3 = analogRead(potInput3);
  //Serial.println(loc_potValue3);
  if(abs(loc_potValue3 - last_potValue3) > POT_CHANGE){
    potValue3 = loc_potValue3;
    if(conState3 == SENS){
      sensor_activated = true;
      if(potValue3 < 5){
        sens3 = 0;
      }else{
        sens3 = map(potValue3,0,1024,1,5);
      }
    }else if(conState3 == MAX){
      sensor_activated = false;
      maxDepres3 = map(potValue3,0,1023,0,179);
      if(maxDepres3 < minDepres3){
        maxDepres3 = minDepres3;
      }
      pos3 = maxDepres3;
      servo3.write(pos3);
      //delay(3);
    }else if(conState3 == MIN){
      sensor_activated = true;
      minDepres3 = map(potValue3,0,1023,0,179);
      if(minDepres3 > maxDepres3){
        minDepres3 = maxDepres3;
      }
    }else if(conState3 == MAX_DURATION){
      sensor_activated = true;
      max_sustain_duration = map(potValue3,0,1023,0,MAX_DURATION_CUTOFF);
	}
    last_potValue3 = loc_potValue3;
  }
}

void setup() {
  Wire.begin();
  Serial.begin(115200);

  pinMode(buttonInput1,INPUT);
  pinMode(buttonInput2,INPUT);
  pinMode(buttonInput3,INPUT);



  // pinMode(ledStickOutput1,OUTPUT);
  // digitalWrite(ledStickOutput1,HIGH);
  servo1.attach(motorOutput1, 1000, 2000);


  servo1.write(0);
  delay(1000);

  attachInterrupt(digitalPinToInterrupt(buttonInput1), cycle_conState1, RISING);
  attachInterrupt(digitalPinToInterrupt(buttonInput2), cycle_conState2, RISING);
  attachInterrupt(digitalPinToInterrupt(buttonInput3), cycle_conState3, RISING);


  if (LEDStick1.begin(0x25) == false){
    Serial.println("Qwiic LED Stick1 failed to begin. Please check wiring and try again!");
    while(1);
  }

  if (LEDStick2.begin(0x24) == false){
    Serial.println("Qwiic LED Stick2 failed to begin. Please check wiring and try again!");
    while(1);
  }

  if (LEDStick3.begin(0x23) == false){
    Serial.println("Qwiic LED Stick3 failed to begin. Please check wiring and try again!");
    while(1);
  }

  LEDStick1.setLEDBrightness(5);
  LEDStick2.setLEDBrightness(5);
  LEDStick3.setLEDBrightness(5);
  //led_stick(LEDStick1,MIN,0,9,0);
  Serial.println("Qwiic LED Sticks ready!");
}

void loop() {
  //get_touch_sensor_inputs();
  //get_imu_sensor_inputs();

  get_pot_inputs();

  //sustain_control();

  char buffer1[120];
  sprintf(buffer1,"Pedal1: min: %d, max: %d, sens: %d, pos: %d, pot: %d, con_state: %d, state: %d, sustain: %d",minDepres1,maxDepres1,sensorValue1,pos1,potValue1,conState1,susState1,sustain_duration);
  Serial.println(buffer1);

  char buffer2[120];
  sprintf(buffer2,"Pedal2: min: %d, max: %d, sens: %d, pos: %d, pot: %d, con_state: %d",minDepres2,maxDepres2,sensorValue2,pos2,potValue2,conState2);
  Serial.println(buffer2);

  char buffer3[120];
  sprintf(buffer3,"Pedal3: min: %d, max: %d, sens: %f, pos: %d, pot: %d, con_state: %d",minDepres3,maxDepres3,diff,pos3,potValue3,conState3);
  Serial.println(buffer3);

  led_stick(LEDStick1,conState1,get_led_value(minDepres1,conState1),get_led_value(maxDepres1,conState1),get_led_value(max_sustain_duration,conState1),get_led_value(sensorValue1,conState1));
  //led_stick(LEDStick2,conState2,get_led_value(minDepres2,conState2),get_led_value(maxDepres2,conState2),get_led_value(max_sustain_duration,conState2),get_led_value(sensorValue1,conState2));
  //led_stick(LEDStick1,conState3,get_led_value(minDepres3,conState3),get_led_value(maxDepres3,conState3),get_led_value(max_sustain_duration,conState3),get_led_value(sensorValue1,conState3));

  delay(10);
}
