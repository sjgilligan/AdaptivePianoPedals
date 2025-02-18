#include <Wire.h>
#include "Qwiic_LED_Stick.h"

typedef enum
{
  INIT,
  MIN,
  MAX,
  SENS,
} control_state_t;

LED LEDStick1;

int buttonInput1 = 0;

control_state_t conState1 = INIT;
int potInput1 = A1; //15
int sensorInput1 = A6; //20
int motorOutput1 = 14;

int potValue1 = 0;
int ledValue1 = 0;
int last_ledValue1 = 0;

void led_stick(LED *led, control_state_t state, int value){  
  if(value < 0){
    led.LEDOff();
  }

  switch(state){
    case MIN:
      LEDStick1.LEDOff();
      LEDStick1.setLEDColor(ledValue1, 255, 0, 0);
      break;
    case MAX:
      LEDStick1.LEDOff();
      LEDStick1.setLEDColor(ledValue1, 0, 255, 0);
      break;
    case SENS:
      LEDStick1.LEDOff();
      LEDStick1.setLEDColor(ledValue1, 0, 0, 255);
      break;
    default:
      LEDStick1.LEDOff();
      LEDStick1.setLEDColor(ledValue1, 100, 100, 100);
      break;
  }

}

void setup() {
  Wire.begin();
  Serial.begin(115200);
  if (LEDStick1.begin() == false){
    Serial.println("Qwiic LED Stick failed to begin. Please check wiring and try again!");
    //while(1);
  }

  //Serial.println("Qwiic LED Stick ready!");
}

void loop() {

  potValue1 = analogRead(potInput1);

  switch (potValue1 / 100) {
    case 0:
        if (potValue1 < 2) {
            ledValue1 = -1;
        } else {
            ledValue1 = 0;
        }
        break;
    case 1:
        ledValue1 = 1;
        break;
    case 2:
        ledValue1 = 2;
        break;
    case 3:
        ledValue1 = 3;
        break;
    case 4:
        ledValue1 = 4;
        break;
    case 5:
        ledValue1 = 5;
        break;
    case 6:
        ledValue1 = 6;
        break;
    case 7:
        ledValue1 = 7;
        break;
    case 8:
        ledValue1 = 8;
        break;
    default:
        ledValue1 = 9;
        break;
}


  if(last_ledValue1 != ledValue1){
    Serial.print("change\n");
    if(ledValue1 == -1){
      LEDStick1.LEDOff();
    }else{
      LEDStick1.LEDOff();
      LEDStick1.setLEDColor(ledValue1, 255, 0, 0);
    }
    last_ledValue1 = ledValue1;
  }

  // for (int i = 0; i <= 10; i++){
  //   if(i == ledValue1){
  //     LEDStick1.setLEDBrightness(i, 30);
  //     LEDStick1.setLEDColor(i, 255, 0, 0);
  //   }else{
  //     LEDStick1.setLEDBrightness(i, 0);
  //     //LEDSt1.setLEDColor(i, 0, 0, 0);
  //   }
  // }
  Serial.print(potValue1);
  Serial.print("\n");

  delay(1);
}
