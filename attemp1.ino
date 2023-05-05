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

void setup() {
  myservo.attach(21);  // attaches the servo on pin 9 to the servo object
  left_front_motor.attach(left_front);    // attaches the servo on pin left_front to turn Vex Motor Controller 29 On
  left_rear_motor.attach(left_rear);      // attaches the servo on pin left_rear to turn Vex Motor Controller 29 On
  right_rear_motor.attach(right_rear);    // attaches the servo on pin right_rear to turn Vex Motor Controller 29 On
  right_front_motor.attach(right_front);  // attaches the servo on pin right_front to turn Vex Motor Controller 29 On
  pinMode(fanPin, OUTPUT);
  Serial.begin(9600);
}

void loop() {

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

    delay(3000);
     
    while (1){
     digitalWrite(fanPin, HIGH);  // turn on the fan     
      closed_loop_angleTurn((angleDesired-90), 1);    
    }
}

void closed_loop_angleTurn(float angleDesired, int errorsArray) {


  float angle_error = 0;
  float previous_angle_error;
  float angular_velocity = 0;

  float angle_k_p = 4;   // Proportional gain given in course book
  float angle_k_i = 0.0;  // Integral gain given in course book
  float angle_k_d = 0.0001;   // Derivative gain given in course book

  int count = 0;
  unsigned int tnow = millis();
  // MEASUREMENTS

  float radius = 2.6;    // radius of omnidirectional wheels
  float length = 8.5;  // length of robot
  float width = 9.2;   // width of robot

  // theta_dot_1
  
  float theta_dot_1 = 0;
  float theta_dot_2 = 0;
  float theta_dot_3 = 0;
  float theta_dot_4 = 0;

  int integral_angle_error = 0;
  int derivative_angle_error = 0;

 int counter=0;
int currentAngle = 0;

  // Main closed control loop
  while (true) {
      // Serial.println(counter);
    // Read sensor inputs for x distance, y distance, and angle displacement from IR sensors, ultrasound and gyro to form real trajectories
    currentAngle = read_gyro_current_angle();
    

    angle_error = angleDesired - currentAngle;
        Serial.println(currentAngle);

    if (angle_error > 90){
        angle_error = 90;
      }
      if (angle_error < -90){
        angle_error = -90;
      }
//////////////////////////////////////

    if (abs(integral_angle_error) < 10){
      integral_angle_error += angle_error;
    }

//////////////////////////////////////
    
    // Calculate derivatives

    derivative_angle_error = angle_error - previous_angle_error;

    angular_velocity = angle_k_p * angle_error + angle_k_i * integral_angle_error + angle_k_d * derivative_angle_error;
    if (angular_velocity > 20){ angular_velocity = 20;}
    if (angular_velocity < -20){ angular_velocity = -20;}

    // Calculate control outputs
    //matrix that does some stuff dervied into eqns
     theta_dot_1 = ( 1 / radius ) * (-(angular_velocity*(length+width)));
     theta_dot_2 = ( 1 / radius ) * ((angular_velocity*(length+width)));
     theta_dot_3 = ( 1 / radius ) * (-(angular_velocity*(length+width)));
     theta_dot_4 = ( 1 / radius ) * ((angular_velocity*(length+width)));

     
    // THETA to motor
    // Send angular velocities of wheels to robot servo motor
    left_front_motor.writeMicroseconds(1500 - theta_dot_1);
    right_front_motor.writeMicroseconds(1500 + theta_dot_2);
    left_rear_motor.writeMicroseconds(1500 - theta_dot_3);
    right_rear_motor.writeMicroseconds(1500 + theta_dot_4);

    // Update previous error values
    previous_angle_error = angle_error;
  bool angleExit = false;
  bool xExit = false; 
  bool yExit = false;
  if (abs(angle_error)<2){
     angleExit = true;
  
  }

  if (angleExit && xExit && yExit){
    count++;
  }
       //SerialCom->println(count);
      if (angleExit && count > 10){

        currentAngle = 0; 
        
        return;
      }
    if (!angleExit || !xExit || !yExit){
      count = 0;
    }
    //Serial.println(count);
   //Serial.println(currentAngle);
 //delay(100);
  counter = 0;
}
}









float read_gyro_current_angle() {
  //GYRO STUFF <---------------------------------------------

  // put your main code here, to run repeatedly:

  // convert the 0-1023 signal to 0-5v
  int currentAngle = 0;

  gyroRate = (analogRead(A10)) * gyroSupplyVoltage / 1023;

  // find the voltage offset the value of voltage when gyro is zero (still)
  gyroRate -= (gyroZeroVoltage / 1023 * 5);

  // read out voltage divided the gyro sensitivity to calculate the angular velocity
  float angularVelocity = gyroRate / gyroSensitivity;

if (abs(gyroRate)>1){
gyroRate=0;
}
  // if the angular velocity is less than the threshold, ignore it
  if ((angularVelocity >= rotationThreshold) || (angularVelocity <= -rotationThreshold)) {
    //SerialCom->println(angularVelocity);
    // we are running a loop in T. one second will run (1000/T).
    float angleChange = angularVelocity / (1000 / T);
    if (abs(angleChange)>10){
angleChange=0;
}
    currentAngle += angleChange;
    //Serial.println(angleChange);
  }
  
  // keep the angle between 0-360
  if (currentAngle < 0) {
    currentAngle += 360;
  } else if (currentAngle > 359) {
    currentAngle -= 360;
  }

  // control the time per loop
  //delay(100);

  return (currentAngle);
}

