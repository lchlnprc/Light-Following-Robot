#include <Servo.h>
#include <SoftwareSerial.h>  //Need for Wireless

float read_gyro_current_angle();
Servo myservo;  // create servo object to control a servo
int pos = 0;    // variable to store the servo position
int fanPin = 45;  // the digital output pin connected to the MOSFET's gate

int tranTL = A12;
int tranTR = A13;
int tranBL = A14;
int tranBR = A15;

const byte left_front = 50;
const byte left_rear = 51;
const byte right_rear = 46;
const byte right_front = 47;

int TLRead;
int TRRead;
int BLRead;
int BRRead;

Servo left_front_motor;   // create servo object to control Vex Motor Controller 29
Servo left_rear_motor;    // create servo object to control Vex Motor Controller 29
Servo right_rear_motor;   // create servo object to control Vex Motor Controller 29
Servo right_front_motor;  // create servo object to control Vex Motor Controller 29
Servo turret_motor;

float gyroRate = 0;             // read out value of sensor in voltage
float gyroSupplyVoltage = 5;    // supply voltage for gyro
float gyroZeroVoltage = 0;    // the value of voltage when gyro is zero
float gyroSensitivity = 0.007;  // gyro sensitivity unit is (mv/degree/second) get from datasheet
float rotationThreshold = 2;    // because of gyro drifting, defining rotation angular velocity  less than
int T = 100;                    // T is the time of one loop

int speed_val = 100;

void setup() {
  myservo.attach(21);  // attaches the servo on pin 9 to the servo object
  left_front_motor.attach(left_front);    // attaches the servo on pin left_front to turn Vex Motor Controller 29 On
  left_rear_motor.attach(left_rear);      // attaches the servo on pin left_rear to turn Vex Motor Controller 29 On
  right_rear_motor.attach(right_rear);    // attaches the servo on pin right_rear to turn Vex Motor Controller 29 On
  right_front_motor.attach(right_front);  // attaches the servo on pin right_front to turn Vex Motor Controller 29 On
  pinMode(fanPin, OUTPUT);
  Serial.begin(9600);
}

void loop(){

  int angleDesired = findTheLight();
  delay(1000);
  turnToLight(angleDesired);
}






int findTheLight(){

    myservo.write(0);  // tell servo to go to position in variable 'pos'
    int maximum = 0;
    int angleDesired;
    int angle = 0;

    while (angle < 180){
      myservo.write(angle);

      TLRead = analogRead(tranTL);
      TRRead = analogRead(tranTR);
      BLRead = analogRead(tranBL);
      BRRead = analogRead(tranBR);

      int intaverageRead = TLRead + TRRead + BLRead + BRRead / 4;
      if (intaverageRead > maximum){
        maximum = intaverageRead;
        angleDesired = angle;
      }
      angle++;
      delay(20);
    }
    myservo.write(angleDesired);
    return (angleDesired);
    delay(3000);
}

void turnToLight(int angleDesired){

  // PSUEDOCODE
  /*
  We want to get the GYRO working
  Once the GYRO works we change this function to a PID
  We find the angle desired in the findTheLight function
  We set the angle of the servo back to zero
  We turn till that direction is reached 
  We drive straight toward that light source - while avoiding obstacles 
  We scan again  
  */



  if (angleDesired > 90){
    while (1){
  left_front_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_front_motor.writeMicroseconds(1500 - speed_val);

    }
  }
  if (angleDesired < 90){
    while(1){
  left_front_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_front_motor.writeMicroseconds(1500 + speed_val);
      
    }

  }
}

void avoidObstacle(){
  
}




void strafe_left ()
{
  left_font_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_font_motor.writeMicroseconds(1500 - speed_val);
}

void strafe_right ()
{
  left_font_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_font_motor.writeMicroseconds(1500 + speed_val);
}

