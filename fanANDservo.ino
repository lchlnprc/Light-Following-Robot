#include <Servo.h>

Servo myservo;  // create servo object to control a servo
int pos = 0;    // variable to store the servo position
int fanPin = 45;  // the digital output pin connected to the MOSFET's gate

void setup() {
  myservo.attach(21);  // attaches the servo on pin 9 to the servo object
  pinMode(fanPin, OUTPUT);
}

void loop() {
  for (pos = 0; pos <= 180; pos += 1) {  // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);  // tell servo to go to position in variable 'pos'
    delay(15);           // waits 15ms for the servo to reach the position
    digitalWrite(fanPin, HIGH);  // turn on the fan
  }
  for (pos = 180; pos >= 0; pos -= 1) {  // goes from 180 degrees to 0 degrees
    myservo.write(pos);  // tell servo to go to position in variable 'pos'
    delay(15);           // waits 15ms for the servo to reach the position
    digitalWrite(fanPin, HIGH);  // turn on the fan
  }
}
