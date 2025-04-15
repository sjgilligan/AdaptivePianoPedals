// Sweep
// by BARRAGAN <http://barraganstudio.com>

// http://arduiniana.org/libraries/pwmservo/

//   Board                     SERVO_PIN_A   SERVO_PIN_B   SERVO_PIN_C
//   -----                     -----------   -----------   -----------
//   Arduino Uno, Duemilanove       9            10          (none)
//   Arduino Mega                  11            12            13
//   Sanguino                      13            12          (none)
//   Teensy 1.0                    17            18            15
//   Teensy 2.0                    14            15             4
//   Teensy++ 1.0 or 2.0           25            26            27
//   Teensy LC & 3.x                 (all PWM pins are usable)

#include <PWMServo.h>

PWMServo myservo1;  // create servo object to control a servo
PWMServo myservo2;  // create servo object to control a servo
PWMServo myservo3;  // create servo object to control a servo

int pos = 0;    // variable to store the servo position

void setup() {
  myservo1.attach(10);  // attaches the servo on pin 9 to the servo object
  myservo2.attach(11);  // attaches the servo on pin 9 to the servo object
  myservo3.attach(12);  // attaches the servo on pin 9 to the servo object
  //myservo.attach(SERVO_PIN_A, 1000, 2000); // some motors need min/max setting
}


void loop() {
  for(pos = 0; pos < 180; pos += 1) { // goes from 0 degrees to 180 degrees, 1 degree steps
    myservo1.write(pos);              // tell servo to go to position in variable 'pos'
    myservo2.write(pos);    
    myservo3.write(pos);    
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  for(pos = 180; pos>=1; pos-=1) {   // goes from 180 degrees to 0 degrees
    myservo1.write(pos);              // tell servo to go to position in variable 'pos'
    myservo2.write(pos);    
    myservo3.write(pos);             // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
}
