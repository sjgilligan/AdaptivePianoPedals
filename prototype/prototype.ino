#include <PWMServo.h>

#define INITIAL_THRESH 1 
#define WAITING_THRESH 50
#define TOGGLE_THRESH 300

enum state_type{
  WAITING, INITIAL_PRESS, TOGGLE,
};

PWMServo myservo;  // create servo object to control a servo

IntervalTimer timeoutTimer;
IntervalTimer sensor1Timer; 

state_type current_state1 = WAITING;

int servoPin1 = 19;
int sensorPin = A9;

int pos1 = 0; //posistion of servo1
bool toggle1 = false; //toggle of servo1;

int sensor1Value = 0; //value of sensor1

int timer_counter = 0;

void count_up(){
  timer_counter++;
}

void read_sensor1(){
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

  if(current_state1 == INITIAL_PRESS && sensor1Value > 300){
    current_state1 = TOGGLE; //initial_press -> toggle
    timeoutTimer.begin(count_up, 1000000);
    toggle1 = !toggle1;

    if(toggle1){
      for(pos1 = 0; pos1 < 180; pos1 += 1) { // goes from 0 degrees to 180 degrees, 1 degree steps
        myservo.write(pos1);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15ms for the servo to reach the position
      } 
    }else{
      for(pos1 = 180; pos1 > 0; pos1 -= 1) { // goes from 180 degrees to 0 degrees, 1 degree steps
        myservo.write(pos1);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15ms for the servo to reach the position
      } 
    }
    //delay(100);
  }

  if(current_state1 == TOGGLE && timer_counter == 2 ){
    current_state1 = WAITING; //toggle -> waiting
    timeoutTimer.end();
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
 
}

void setup() {
  Serial.begin(9600);
  myservo.attach(servoPin1); //set servo 1 pin

  sensor1Timer.begin(read_sensor1,1000);
}

void loop() {
  

  

  delay(1);
  // put your main code here, to run repeatedly:

}
