#include <ESP32Servo.h>
#define TIMER_BASE_CLK 80000000
#include <ESP32TimerInterrupt.h>

#define _TIMERINTERRUPT_LOGLEVEL_     3

#define INITIAL_THRESH 1 
#define WAITING_THRESH 50
#define TOGGLE_THRESH 1400

enum state_type{
  WAITING, INITIAL_PRESS, TOGGLE,
};

Servo servo1;  // create servo object to control a servo


state_type current_state1 = WAITING;

int servoPin = 4;
int sensorPin = 34;
int LEDPin = 12;

int pos1 = 0; //posistion of servo1
bool toggle1 = false; //toggle of servo1;

int sensor1Value = 0; //value of sensor1

int timer_counter = 0;

ESP32Timer ITimer(0);

bool IRAM_ATTR TimerHandler0(void * timerNo){
  timer_counter++;
  return true;
}


void setup() {
  Serial.begin(9600);
  servo1.attach(servoPin);

  //sensor1Timer.begin(read_sensor1,1000);
  ITimer.attachInterruptInterval(1000 * 1000, TimerHandler0);

  pinMode(LEDPin, OUTPUT);
  digitalWrite(LEDPin, HIGH);
}

void loop() {
  sensor1Value = analogRead(sensorPin);
  Serial.print(sensor1Value);
  delay(10);
  Serial.print("\n");
  if(current_state1 == WAITING && sensor1Value > 10){
    current_state1 = INITIAL_PRESS; //wating -> inital_press
  }

  if(current_state1 == INITIAL_PRESS && sensor1Value < 10){
    current_state1 = WAITING; //initial_press -> waiting
  }

  if(current_state1 == INITIAL_PRESS && sensor1Value > TOGGLE_THRESH){
    current_state1 = TOGGLE; //initial_press -> toggle
    //ITimer.restartTimer();
    toggle1 = !toggle1;

    if(toggle1){
      digitalWrite(LEDPin, HIGH);
      for(pos1 = 0; pos1 < 100; pos1 += 1) { // goes from 0 degrees to 180 degrees, 1 degree steps
        servo1.write(pos1);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15ms for the servo to reach the position
      } 
    }else{
      digitalWrite(LEDPin, LOW);
      for(pos1 = 100; pos1 > 0; pos1 -= 1) { // goes from 180 degrees to 0 degrees, 1 degree steps
        servo1.write(pos1);           // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15ms for the servo to reach the position
      } 
    }
    //delay(100);
  }

  if(current_state1 == TOGGLE){
    //delay(500);
    current_state1 = WAITING; //toggle -> waiting
    //ITimer.stopTimer();
    timer_counter = 0;
  }


  switch (current_state1) {
    case INITIAL_PRESS:
      Serial.println("State: INITIAL_PRESS");
      break;
    case WAITING:
      Serial.println("State: WAITING");
      break;
    case TOGGLE:
      Serial.println("State: TOGGLE");
      break;
    default:
      Serial.println("State: UNKNOWN");
      break;
  }

  //Serial.println(timer_counter);
  
  delay(1);
  // put your main code here, to run repeatedly:

}
