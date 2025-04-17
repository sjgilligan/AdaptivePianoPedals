#include <Wire.h>
#include <PWMServo.h>
#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
//#include "TeensyThreads.h"
#include "Qwiic_LED_Stick.h"
#include "Timer.h"

#define SENS_COEF_1 200

#define INITIAL_THRESH 1
#define WAITING_THRESH 50
#define TOGGLE_THRESH 400
#define POT_CHANGE 10
#define MAX_DURATION_CUTOFF 10000
#define SUSTAIN_TRIGGER_THRESH 10 //should be 10-50

typedef enum {
  WAITING,
  INITIAL_PRESS,
  TOGGLE,
  DEPRESS
} sustain_state_t;

typedef enum {
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

int buttonInput1 = 3;
int buttonInput2 = 2;
int buttonInput3 = 1;
int buttonState1 = LOW;
int buttonState2 = LOW;
int buttonState3 = LOW;

int potInput1 = 14;  //15
int potInput2 = 15;
int potInput3 = 16;
int potValue1 = 0;
int potValue2 = 0;
int potValue3 = 0;

control_state_t conState1 = MIN;
control_state_t conState2 = MIN;
control_state_t conState3 = MIN;

int sensorInput1 = 22;
int sensorInput2 = 21;
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

int sens1 = 1;  //min 0, max 1024
int sens2 = 1;
int sens3 = 1;

int pos1 = 0;
int pos2 = 0;
int pos3 = 0;

int sustain_window;
int max_sustain_duration1 = 4000;
//int max_sustain_duration2 = 4000;
sustain_state_t susState1 = WAITING;
int sustain_sensor_direction;

bool sensor1_activated = true;
bool sensor2_activated = true;
bool sensor3_activated = true;
bool soft_pedal = false;


void cycle_conState1() {
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  if (interrupt_time - last_interrupt_time > 200) {
    switch (conState1) {
      case INIT:
        while (minDepres1 < map(potValue1, 0, 1023, 0, 179)) {
          servo1.write(minDepres1);
        }
        conState1 = MIN;
        break;
      case MIN:
        while (minDepres1 < map(potValue1, 0, 1023, 0, 179)) {
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

  //delay(1000);
}

void cycle_conState2() {
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  if (interrupt_time - last_interrupt_time > 200) {
    switch (conState2) {
      case INIT:
        while (minDepres2 < map(potValue2, 0, 1023, 0, 179)) {
          servo2.write(minDepres2);
        }
        conState2 = MIN;
        break;
      case MIN:
        while (minDepres2 < map(potValue2, 0, 1023, 0, 179)) {
          servo2.write(minDepres2);
        }
        conState2 = MAX;
        break;
      case MAX:
        //servo2.write(minDepres2);
        conState2 = SENS;
        break;
      case MAX_DURATION:
        servo2.write(minDepres2);
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

void cycle_conState3() {
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  if (interrupt_time - last_interrupt_time > 200) {
    switch (conState3) {
      case INIT:
        while (minDepres3 < map(potValue3, 0, 1023, 0, 179)) {
          servo3.write(minDepres3);
        }
        conState3 = MIN;
        break;
      case MIN:
        while (minDepres3 < map(potValue3, 0, 1023, 0, 179)) {
          servo3.write(minDepres3);
        }
        conState3 = MAX;
        break;
      case MAX:
        servo3.write(minDepres3);
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




int get_led_value(int input_value, int state) {
  int led_value;

  if (state == SENS) {
    led_value = map(input_value, 0, 1023, 0, 9);
  } else if (state == MAX_DURATION) {
    led_value = map(input_value, 0, MAX_DURATION_CUTOFF, 0, 9);
  } else {

    switch (input_value / 18) {
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

void led_stick(LED &led, control_state_t state, int min, int max, int duration, int sens) {
  if (min == max) {
    if (max != 9) {
      max = max + 1;
    } else if (max == 9) {
      min = 8;
    }
  }

  if (min < 0 && max < 0 && sens < 0) {
    led.LEDOff();
  } else {
    switch (state) {
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

void sustain_control() {
  static int sustain_counter = 0;
  if (sensor1_activated == true) {
    if (susState1 == WAITING && sensorValue1 > SUSTAIN_TRIGGER_THRESH && sustain_sensor_direction > 0) {
      susState1 = INITIAL_PRESS;  //wating -> inital_press
    }

    if (susState1 == INITIAL_PRESS && sensorValue1 < SUSTAIN_TRIGGER_THRESH) {
      susState1 = WAITING;  //initial_press -> waiting
    }

    if (susState1 == INITIAL_PRESS && sensorValue1 > TOGGLE_THRESH sustain_sensor_direction > 0) {
      
      susState1 = TOGGLE;  //initial_press -> toggle
    }

    if (susState1 == TOGGLE && sensorValue1 > TOGGLE_THRESH) {
      //delay(300);
      sustain_duration = map(sensorValue1, TOGGLE_THRESH, 1023, , 9);
      susState1 = DEPRESS;
      pos1 = maxDepres1;
    }
    if (susState1 == DEPRESS && sustain_counter < sustain_duration) {
      servo1.write(pos1);
      sustain_counter++;
      Serial.print("Sustain counter: ");
      Serial.println(sustain_counter);
    } else if (susState1 == DEPRESS && sustain_counter >= sustain_duration) {
      sustain_counter = 0;
      susState1 = WAITING;
      servo1.write(minDepres1);
      pos1 = minDepres1;
    }
  }
}

void get_touch_sensor_inputs() {
  static int last_sensorValue1 = 0;

  sensorValue1 = analogRead(sensorInput1);
  if((sensorValue1 * sens1) < 1023){ //limit sensorValue1 to 1023
    sensorValue1 = sensorValue1 * sens1;
  }else{
    sensorValue1 = 1023;
  }

  sustian_sensor_direction = sensorValue1 - last_sensorValue1;
  last_sensorValue1 = sensorValue1;

  sensorValue2 = analogRead(sensorInput2);
  if((sensorValue2 * sens2) < 1023){ //limit sensorValue2 to 1023
    sensorValue2 = sensorValue2 * sens2;
  }else{
    sensorValue2 = 1023;
  }
}

void soft_control(int change) {
  if (change > 0.4 && !soft_pedal) {
    Serial.println("Shoulder raised - Pressing pedal.");
    for (int pos = minDepres3; pos <= maxDepres3; pos += 10) {
      servo3.write(pos);
      delay(100);
    }
    soft_pedal = true;
    delay(1000);
  } else if (change > 0.4 && soft_pedal) {
    Serial.println("Shoulder lowered - Releasing pedal.");
    for (int pos = maxDepres3; pos >= minDepres3; pos -= 10) {
      servo3.write(pos);
      delay(100);
    }
    soft_pedal = false;
    delay(1000);
  }
}

void get_pot_inputs() {
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
  if (abs(loc_potValue1 - last_potValue1) > POT_CHANGE) {
    potValue1 = loc_potValue1;
    if (conState1 == SENS) {
      sensor1_activated = true;
      if (potValue1 < 5) {
        sens1 = 0;
      } else {
        sens1 = map(potValue1, 0, 1024, 1, 5);
      }
    } else if (conState1 == MAX) {
      sensor1_activated = false;
      maxDepres1 = map(potValue1, 0, 1023, 0, 179);
      if (maxDepres1 < minDepres1) {
        maxDepres1 = minDepres1;
      }
      pos1 = maxDepres1;
      servo1.write(pos1);
      //delay(1);
    } else if (conState1 == MIN) {
      sensor1_activated = true;
      minDepres1 = map(potValue1, 0, 1023, 0, 179);
      if (minDepres1 > maxDepres1) {
        minDepres1 = maxDepres1;
      }
    } else if (conState1 == MAX_DURATION) {
      sensor1_activated = true;
      sustian_window = map(potValue1, 0, 1023, 0, 9);
    }
    last_potValue1 = loc_potValue1;
  }

  loc_potValue2 = analogRead(potInput2);
  //Serial.println(loc_potValue2);
  if (abs(loc_potValue2 - last_potValue2) > POT_CHANGE) {
    potValue2 = loc_potValue2;
    if (conState2 == SENS) {
      sensor2_activated = true;
      if (potValue2 < 5) {
        sens2 = 0;
      } else {
        sens2 = map(potValue2, 0, 1024, 1, 5);
      }
    } else if (conState2 == MAX) {
      sensor2_activated = false;
      maxDepres2 = map(potValue2, 0, 1023, 0, 179);
      if (maxDepres2 < minDepres2) {
        maxDepres2 = minDepres2;
      }
      pos2 = maxDepres2;
      servo2.write(pos2);
      //delay(2);
    } else if (conState2 == MIN) {
      sensor2_activated = true;
      minDepres2 = map(potValue2, 0, 1023, 0, 179);
      if (minDepres2 > maxDepres2) {
        minDepres2 = maxDepres2;
      }
    } //else if (conState2 == MAX_DURATION) {
      //sensor2_activated = true;
      //max_sustain_duration2 = map(potValue2, 0, 1023, 0, MAX_DURATION_CUTOFF);
    //}
    last_potValue2 = loc_potValue2;
  }

  loc_potValue3 = analogRead(potInput3);
  //Serial.println(loc_potValue3);
  if (abs(loc_potValue3 - last_potValue3) > POT_CHANGE) {
    potValue3 = loc_potValue3;
    if (conState3 == SENS) {
      if (soft_pedal) {
        sens3 = 8;
      } else {
        sens3 = 1;
      }
    } else if (conState3 == MAX) {
      maxDepres3 = map(potValue3, 0, 1023, 0, 179);
      if (maxDepres3 < minDepres3) {
        maxDepres3 = minDepres3;
      }
      pos3 = maxDepres3;
      servo3.write(pos3);
      //delay(3);
    } else if (conState3 == MIN) {
      minDepres3 = map(potValue3, 0, 1023, 0, 179);
      if (minDepres3 > maxDepres3) {
        minDepres3 = maxDepres3;
      }
      last_potValue3 = loc_potValue3;
    }
  }
}

int get_imu() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  return a.acceleration.z;
}

void setup() {
  Wire.begin();
  Serial.begin(115200);

  pinMode(buttonInput1, INPUT);
  pinMode(buttonInput2, INPUT);
  pinMode(buttonInput3, INPUT);



  // pinMode(ledStickOutput1,OUTPUT);
  // digitalWrite(ledStickOutput1,HIGH);
  servo1.attach(motorOutput1, 1000, 2000);
  servo2.attach(motorOutput2, 1000, 2000);
  servo3.attach(motorOutput3, 1000, 2000);


  servo1.write(0);
  delay(1000);
  servo2.write(0);
  delay(1000);
  servo3.write(0);
  delay(1000);

  attachInterrupt(digitalPinToInterrupt(buttonInput1), cycle_conState1, RISING);
  attachInterrupt(digitalPinToInterrupt(buttonInput2), cycle_conState2, RISING);
  attachInterrupt(digitalPinToInterrupt(buttonInput3), cycle_conState3, RISING);


  if (LEDStick1.begin(0x25) == false) {
    Serial.println("Qwiic LED Stick1 failed to begin. Please check wiring and try again!");
    while (1)
      ;
  }

  if (LEDStick2.begin(0x24) == false) {
    Serial.println("Qwiic LED Stick2 failed to begin. Please check wiring and try again!");
    while (1)
      ;
  }

  if (LEDStick3.begin(0x23) == false) {
    Serial.println("Qwiic LED Stick3 failed to begin. Please check wiring and try again!");
    while (1)
      ;
  }

  if (!mpu.begin()) {
    Serial.println("MPU6050 connection failed");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 connected");

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(100);

  prevAccelz = get_imu();

  LEDStick1.setLEDBrightness(5);
  LEDStick2.setLEDBrightness(5);
  LEDStick3.setLEDBrightness(5);
  //led_stick(LEDStick1,MIN,0,9,0);
  Serial.println("Qwiic LED Sticks ready!");
}

void loop() {
  get_touch_sensor_inputs();

  get_pot_inputs();

  sustain_control();

  Serial.println(prevAccelz);

  accelZ = get_imu();
  diff = accelZ - prevAccelz;
  soft_control(diff);

  //Serial.println(servo3.read());
  prevAccelz = accelZ;

  delay(10);

  char buffer1[120];
  sprintf(buffer1, "Pedal1: min: %d, max: %d, sens: %d, pos: %d, pot: %d, con_state: %d, state: %d, sustain: %d", minDepres1, maxDepres1, sensorValue1, pos1, potValue1, conState1, susState1, sustain_window);
  Serial.println(buffer1);

  char buffer2[120];
  sprintf(buffer2, "Pedal2: min: %d, max: %d, sens: %d, pos: %d, pot: %d, con_state: %d", minDepres2, maxDepres2, sensorValue2, sustain_window, potValue2, conState2);
  Serial.println(buffer2);

  char buffer3[120];
  sprintf(buffer3, "Pedal3: min: %d, max: %d, sens: %d, pos: %d, pot: %d, con_state: %d", minDepres3, maxDepres3, soft_pedal, pos3, potValue3, conState3);
  Serial.println(buffer3);

  led_stick(LEDStick1, conState1, get_led_value(minDepres1, conState1), get_led_value(maxDepres1, conState1), get_led_value(max_sustain_duration1, conState1), get_led_value(sensorValue1, conState1));
  led_stick(LEDStick2, conState2, get_led_value(minDepres2, conState2), get_led_value(maxDepres2, conState2), get_led_value(max_sustain_duration2, conState2), get_led_value(sensorValue2, conState2));
  led_stick(LEDStick3, conState3, get_led_value(minDepres3, conState3), get_led_value(maxDepres3, conState3), -1, sens3);

  //delay(100);
}