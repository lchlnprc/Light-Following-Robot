/* Project 2 Group 20 */

/////////////////////////////////////////////////////////////////////////////////////////////////

#include <Wire.h>
#include <Servo.h>           //Need for Servo pulse output
#include <SoftwareSerial.h>  //Need for Wireless
#include <TimerOne.h>

//State machine       <------ will need to add a bunch more states to this I think
enum STATE {
  INITIALISING,
  FIND_CLOSEST_FIRE,
  TRAVEL_TO_FIRE,
  AVOID_OBSTACLE,
  FIGHT_FIRE,
  STOPPED
};

// Defines //
#define INTERNAL_LED 13
#define TIMER_FREQ 10
// Serial Data input pin
#define BLUETOOTH_RX 10
// Serial Data output pin
#define BLUETOOTH_TX 11
#define STARTUP_DELAY 10  // Seconds
#define LOOP_DELAY 10     // miliseconds
#define SAMPLE_DELAY 10   // miliseconds
// USB Serial Port
#define OUTPUTMONITOR 0
#define OUTPUTPLOTTER 0
// Bluetooth Serial Port
#define OUTPUTBLUETOOTHMONITOR 1
#define IR_Left A1    // Sharp IR GP2Y0A41SK0F (4-30cm, analog) Back Left
#define IR_Front_Left A4   // Sharp IR GP2Y0A41SK0F (4-30cm, analog) Front Left
#define IR_Right A2   // Sharp IR GP2Y0A41SK0F (4-30cm, analog) Back Right
#define IR_Front_Right A3  // Sharp IR GP2Y0A41SK0F (4-30cm, analog) Front Right
#define echoPin A7  // attach pin A4 Arduino to pin Echo of HC-SR04
#define trigPin A6  //attach pin A5 Arduino to pin Trig of HC-SR04
#define phototransistor_left_1 A15
#define phototransistor_left_2 A14
#define phototransistor_right_1 A12
#define phototransistor_right_2 A13

SoftwareSerial BluetoothSerial(BLUETOOTH_RX, BLUETOOTH_TX);
Servo myservo;  // create servo object to control a servo
int pos = 0;    // variable to store the servo position
int fanPin = 45;  // the digital output pin connected to the MOSFET's gate
int sensorPin = A10;            //define the pin that gyro is connected
int T = 100;                    // T is the time of one loop
int sensorValue = 0;            // read out value of sensor
int averagePhototransistorRead = 0;            //Defining the Global Ultrasonic Reading
int maxPhototransistorRead = 0;                 //Definining Global Max Ultrasonic
int firesExtinguished = 0;
float gyroSupplyVoltage = 5;    // supply voltage for gyro
float gyroZeroVoltage = 0;    // the value of voltage when gyro is zero
float gyroSensitivity = 0.007;  // gyro sensitivity unit is (mv/degree/second) get from datasheet
float rotationThreshold = 2;    // because of gyro drifting, defining rotation angular velocity  less than this value will not be ignored
float gyroRate = 0;             // read out value of sensor in voltage
volatile float currentAngle = 0;         // current angle calculated by angular velocity integral on
byte serialRead = 0;  // for serial print control
//Default motor control pins
const byte left_front = 50;
const byte left_rear = 51;
const byte right_rear = 46;
const byte right_front = 47;
//Default ultrasonic ranging sensor pins, these pins are defined my the Shield
const int TRIG_PIN = 48;
const int ECHO_PIN = 49;
// Anything over 400 cm (23200 us pulse) is "out of range". Hit:If you decrease to this the ranging sensor but the timeout is short, you may not need to read up to 4meters.
const unsigned int MAX_DIST = 23200;

Servo left_front_motor;   // create servo object to control Vex Motor Controller 29
Servo left_rear_motor;    // create servo object to control Vex Motor Controller 29
Servo right_rear_motor;   // create servo object to control Vex Motor Controller 29
Servo right_front_motor;  // create servo object to control Vex Motor Controller 29
Servo turret_motor;

int speed_val = 150;
int speed_change;

//Serial Pointer
HardwareSerial *SerialCom;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////Setup Function

void setup(void) {
    turret_motor.attach(11);
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(INTERNAL_LED, OUTPUT);
    
    // The Trigger pin will tell the sensor to range find
    pinMode(TRIG_PIN, OUTPUT);
    digitalWrite(TRIG_PIN, LOW);
    
    // Setup the Serial port and pointer, the pointer allows switching the debug info through the USB port(Serial) or Bluetooth port(Serial1) with ease.
    SerialCom = &Serial;
    SerialCom->begin(115200);
    SerialCom->println("MECHENG706 Project 2");
    delay(1000);
    Serial.begin(9600);
    // this section is initialize the sensor, find the the value of voltage when gyro is zero
    int i;
    float sum = 0;
    pinMode(sensorPin, INPUT);
    pinMode(fanPin, OUTPUT);

    myservo.attach(21);  // attaches the servo on pin 21 to the servo object
    
    Serial.println("please keep the sensor still for calibration");
    Serial.println("get the gyro zero voltage");
    for (i = 0; i < 100; i++)  //  read 100 values of voltage when gyro is at still, to calculate the zero-drift
    {
      sensorValue = analogRead(sensorPin);
      sum += sensorValue;
      delay(5);
    }
    gyroZeroVoltage = sum / 100;  // average the sum as the zero drifting
    
    //Ultrasonic Setup
    
    pinMode(trigPin, OUTPUT);                          // Sets the trigPin as an OUTPUT
    pinMode(echoPin, INPUT);                           // Sets the echoPin as an INPUT
    
    Serial.println("Setup Complete");
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////MAIN LOOP

void loop(void)  
{
    static STATE machine_state = INITIALISING;
    //Finite-state machine Code
    switch (machine_state) {
        case INITIALISING:
            machine_state = initialising();
            break;
        case FIND_CLOSEST_FIRE:  //Lipo Battery Volage OK
            machine_state = find_closest_fire();
            break;
        case TRAVEL_TO_FIRE:  //Lipo Battery Volage OK
            machine_state = travel_to_fire();
            break;  
        case AVOID_OBSTACLE:  //Lipo Battery Volage OK
            machine_state = avoid_obstacle();
            break;  
        case FIGHT_FIRE:  //Lipo Battery Volage OK
            machine_state = fight_fire();
            break; 
        case STOPPED:  //Lipo Battery Volage OK
            machine_state = stopped();
            break;  
    };
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////STATES

STATE initialising() {
    SerialCom->println("INITIALISING....");
    delay(1000);
    SerialCom->println("Enabling Motors...");
    enable_motors();
    SerialCom->println("RUNNING STATE...");
    return FIND_CLOSEST_FIRE;
}

STATE find_closest_fire() {

    int angle = 0;
    int desiredAngle = 0;
    maxPhototransistorRead = 0;

    myservo.write(87);  // NEED TO FIGURE OUT WHERE SERVO IS SQUARE WITH ROBOT

    while (angle < 360){
        averagePhototransistorRead = (phototransistor(phototransistor_left_1) + phototransistor(phototransistor_left_2) + phototransistor(phototransistor_right_1) + phototransistor(phototransistor_right_2)) / 4;
        if (averagePhototransistorRead > maxPhototransistorRead){
            maxPhototransistorRead = averagePhototransistorRead;
            desiredAngle = angle;
        }
        turn(10);
        angle+=10;
    }

    turn(desiredAngle); //SHOULD NOW BE FACING CLOSEST FIRE

    averagePhototransistorRead = 0; //Reset Global Phototransistor Values
    maxPhototransistorRead = 0;

    return TRAVEL_TO_FIRE;
}

STATE travel_to_fire() {

    straight(); //FUNCTION WITH PID CONTROL ON GYRO AND X AXIS BASED ON BRIGHTEST FIRE SOURCE

    return FIND_CLOSEST_FIRE;
}

STATE avoid_obstacle() {
    return NULL;
}

STATE fight_fire() {

    averagePhototransistorRead = (phototransistor(phototransistor_left_1) + phototransistor(phototransistor_left_2) + phototransistor(phototransistor_right_1) + phototransistor(phototransistor_right_2)) / 4;
    
    if (averagePhototransistorRead < 900){ //Should implement a count average here of some sort to prevent false readings. 
        return FIND_CLOSEST_FIRE;
    }

    digitalWrite(fanPin, HIGH);  // turn on the fan 

    while(averagePhototransistorRead > 900){
        averagePhototransistorRead = (phototransistor(phototransistor_left_1) + phototransistor(phototransistor_left_2) + phototransistor(phototransistor_right_1) + phototransistor(phototransistor_right_2)) / 4;
    }
    //Light should now be out
    digitalWrite(fanPin, LOW);  // turn on the fan

    firesExtinguished++;

    if (firesExtinguished < 2){
        return FIND_CLOSEST_FIRE;
    }
    else {
        return STOPPED;
    }
}


STATE stopped() {
    //Stop of Lipo Battery voltage is too low, to protect Battery
    static byte counter_lipo_voltage_ok;
    static unsigned long previous_millis;
    int Lipo_level_cal;
    disable_motors();

    if (millis() - previous_millis > 500) {  //print massage every 500ms
         previous_millis = millis();
         SerialCom->println("STOPPED---------");


#ifndef NO_BATTERY_V_OK
    //500ms timed if statement to check lipo and output speed settings
    if (is_battery_voltage_OK()) {
        SerialCom->print("Lipo OK waiting of voltage Counter 10 < ");
        SerialCom->println(counter_lipo_voltage_ok);
        counter_lipo_voltage_ok++;
        if (counter_lipo_voltage_ok > 10) {  //Making sure lipo voltage is stable
            counter_lipo_voltage_ok = 0;
            enable_motors();
            SerialCom->println("Lipo OK returning to RUN STATE");
            return FIND_CLOSEST_FIRE;
        }
    } else {
        counter_lipo_voltage_ok = 0;
    }
#endif
  }
  return STOPPED;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////CLOSED LOOP CONTROLS

void turn(float angleDesired) {
    float angle_k_p = 4, angle_k_i = 0.0, angle_k_d = 0.0001;
    float radius = 2.6, length = 8.5, width = 9.2;
    float angle_error, previous_angle_error = 0, integral_angle_error = 0, derivative_angle_error;
    float angular_velocity;
    int count = 0;
    currentAngle = 0;

    while (true) {
        delay(100);
        currentAngle = read_gyro_current_angle();
        angle_error = constrain(angleDesired - currentAngle, -90, 90);
    
        if (abs(integral_angle_error) < 10) {
            integral_angle_error += angle_error;
        }
    
        derivative_angle_error = angle_error - previous_angle_error;
        angular_velocity = constrain(angle_k_p * angle_error + angle_k_i * integral_angle_error + angle_k_d * derivative_angle_error, -20, 20);
    
        float theta_dot_common = (1 / radius) * (angular_velocity * (length + width));
        float theta_dots[4] = {-theta_dot_common, theta_dot_common, -theta_dot_common, theta_dot_common};
    
        left_front_motor.writeMicroseconds(1500 - theta_dots[0]);
        right_front_motor.writeMicroseconds(1500 + theta_dots[1]);
        left_rear_motor.writeMicroseconds(1500 - theta_dots[2]);
        right_rear_motor.writeMicroseconds(1500 + theta_dots[3]);
    
        previous_angle_error = angle_error;
    
        if (abs(angle_error) < 2) {
            count++;
        } else {
            count = 0;
        }
    
        if (count > 10) {
            return;
        }
    }
}

void straight() {

    currentAngle=0; // Reset the global current angle
    averagePhototransistorRead = 0;  // Reset the global phototransistor reading
    maxPhototransistorRead = 0; // Reset the global phototransistor maximum

    float currentAngleMove=0;     
    float x_error = 0, angle_error = 0;
    float x_velocity = 0, angular_velocity = 0;
    float x_k_p = 40, x_k_i = 0.5, x_k_d = 1.002;
    float angle_k_p = 4, angle_k_i = 0.0, angle_k_d = 0.0001; 
    float previous_x_error = 0, previous_angle_error = 0, integral_x_error = 0;
    float integral_angle_error = 0, derivative_x_error = 0, derivative_angle_error = 0;
    float radius = 2.6, length = 8.5, width = 9.2;    // wheel specs
    float theta_dot_1 = 0, theta_dot_2 = 0, theta_dot_3 = 0, theta_dot_4 = 0;
    float x_distance_input = 100;
 
    int count = 0;
    int xDistanceDesired = 4; //<------we could replace this with looking for brightness instead of distance? IDK
    int servoAngle = 0;
    int direction = 0;
    int angleDesired = findLight();

    myservo.write(87);
    servoAngle = 87;
       
    while (true) {

        currentAngle = read_gyro_current_angle();
        x_distance_input = ultrasonic();

        direction = findLightDirection(servoAngle);

        if (direction != 0) {
            servoAngle += direction;
            myservo.write(servoAngle);
        }

        angleDesired = 87 - servoAngle;
    
        
        if (currentAngle > 180) {
            currentAngleMove = currentAngle - 360;
        } else {
            currentAngleMove = currentAngle;
        } 
    
        // Calculate errors // 
        //////////////////////////////////////
        x_error = xDistanceDesired - x_distance_input;
        if (x_error > 200){
            x_error = 200;
        }
        if (x_error < -200){
            x_error = -200;
        }
    
        angle_error = angleDesired - currentAngleMove;
        if (angle_error > 90){
            angle_error = 90;
        }
        if (angle_error < -90){
            angle_error = -90;
        }
    
        if (abs(integral_x_error) < 50){
            integral_x_error += x_error;
        }
    
        if (abs(integral_angle_error) < 10){
            integral_angle_error += angle_error;
        }
        
        // Calculate derivatives
        derivative_x_error = x_error - previous_x_error;
        derivative_angle_error = angle_error - previous_angle_error;
    
        x_velocity = x_k_p * x_error + x_k_i * integral_x_error + x_k_d * derivative_x_error;
        angular_velocity = angle_k_p * angle_error + angle_k_i * integral_angle_error + angle_k_d * derivative_angle_error;
    
        if (x_velocity > 900){ x_velocity = 900;}
        if (x_velocity < -900){ x_velocity = -900;}
    
        if (angular_velocity > 20){ angular_velocity = 20;}
        if (angular_velocity < -20){ angular_velocity = -20;}
    
        // Calculate control outputs
        theta_dot_1 = ( 1 / radius ) * (x_velocity - (angular_velocity*(length+width)));
        theta_dot_2 = ( 1 / radius ) * (x_velocity + (angular_velocity*(length+width)));
        theta_dot_3 = ( 1 / radius ) * (x_velocity - (angular_velocity*(length+width)));
        theta_dot_4 = ( 1 / radius ) * (x_velocity + (angular_velocity*(length+width)));
    
        // Send angular velocities of wheels to robot servo motor
        left_front_motor.writeMicroseconds(1500 - theta_dot_1);
        right_front_motor.writeMicroseconds(1500 + theta_dot_2);
        left_rear_motor.writeMicroseconds(1500 - theta_dot_3);
        right_rear_motor.writeMicroseconds(1500 + theta_dot_4);
        
        // Update previous error values
        previous_x_error = x_error;
        previous_angle_error = angle_error;

        //Exit conditions
        bool angleExit = false;
        bool xExit = false; 
        if (abs(angle_error)<2){
            angleExit = true;
        }
    
        if (abs(x_error)<2 && averagePhototransistorRead > 990){
            xExit = true;
        }

        if (abs(x_error)<2 && averagePhototransistorRead < 900){
            //AVOID OBSTACLE TO BE IMPLEMENTED HERE <-------------------------------------------------OBSTACLE AVOIDANCE!
        }
    
        if (angleExit && xExit){
          count++;
        }
         
        if (count > 10){
            stop;
            currentAngle = 0; 
            return;
        }
        if (!angleExit || !xExit){
            count = 0;
        }

        delay(80); //80 instead of 100 due to delays in moving servo. Needs to total 100 for gyro accuracy
    }

}

int findLight(){

    myservo.write(0);  // tell servo to go to position in variable 'pos'
    int angleDesired;
    int angle = 0;

    while (angle < 180){
        myservo.write(angle);

        averagePhototransistorRead = (phototransistor(phototransistor_left_1) + phototransistor(phototransistor_left_2) + phototransistor(phototransistor_right_1) + phototransistor(phototransistor_right_2)) / 4;
        if (averagePhototransistorRead > maxPhototransistorRead){
            maxPhototransistorRead = averagePhototransistorRead;
            angleDesired = angle;
        }
        angle++;
        delay(20);
    }
    myservo.write(angleDesired);
    return (87 - angleDesired);
}

int findLightDirection(int servoAngle) {
  int leftAngle = servoAngle - 1;
  int rightAngle = servoAngle + 1;
  int leftBrightness = 0;
  int rightBrightness = 0;

  if (leftAngle >= 0) {
    myservo.write(leftAngle);
    delay(10);
    leftBrightness = (phototransistor(phototransistor_left_1) + phototransistor(phototransistor_left_2) + phototransistor(phototransistor_right_1) + phototransistor(phototransistor_right_2)) / 4;
  }
  
  if (rightAngle <= 180) {
    myservo.write(rightAngle);
    delay(10);
    rightBrightness = (phototransistor(phototransistor_left_1) + phototransistor(phototransistor_left_2) + phototransistor(phototransistor_right_1) + phototransistor(phototransistor_right_2)) / 4;
  }

  if (leftBrightness > rightBrightness) {
    return -1; // Move left
  } else if (rightBrightness > leftBrightness) {
    return 1; // Move right
  } else {
    return 0; // No change
  }
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////SENSOR FUNCTIONS

float ultrasonic() {  //<------------------------------------------------ Ultrasonic
     // Clears the trigPin condition
    
    digitalWrite(trigPin, LOW);
    delayMicroseconds(24);
    
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(120);
    digitalWrite(trigPin, LOW);
    
    float duration = pulseIn(echoPin, HIGH);
    float measurement = duration * 0.034 / 2; // convert duration to distance measurement    
    return measurement;
}

float read_gyro_current_angle() {
   
    if (Serial.available())  // Check for input from terminal
    {
        serialRead = Serial.read();  // Read input
        if (serialRead == 49)        // Check for flag to execute, 49 is asci for 1
        {
            Serial.end();  // end the serial communication to display the sensor data on monitor
        }
    }
    
    // convert the 0-1023 signal to 0-5v
    
    gyroRate = (analogRead(sensorPin)) * gyroSupplyVoltage / 1023;
    
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

    return (currentAngle);
}

float read_IR(uint8_t Sensor) {
    
    float distance_measurement;
    float sensorMeasurement = analogRead(Sensor) * (5.0 / 1023.0); // Reading sensor value and converting it to voltage
    
    if(Sensor == phototransistor_left_1){
        distance_measurement = 20.204 * pow(sensorMeasurement, -1.93);
    }
    if(Sensor == phototransistor_left_2){
        distance_measurement = 9.1618 * pow(sensorMeasurement, -1.132);
    }
    if(Sensor == phototransistor_right_1){
        distance_measurement = 8.05 * pow(sensorMeasurement, -1.072);
    }
    if(Sensor == phototransistor_right_2){
        distance_measurement = 23.596 * pow(sensorMeasurement, -1.818);
    }

    return distance_measurement; 
}

float phototransistor(uint8_t Sensor){
    return analogRead(Sensor); //A value of 0 means no light, 1024 means maximum light
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// BASIC KINEMATICS
void disable_motors() {
    left_front_motor.detach();   // detach the servo on pin left_front to turn Vex Motor Controller 29 Off
    left_rear_motor.detach();    // detach the servo on pin left_rear to turn Vex Motor Controller 29 Off
    right_rear_motor.detach();   // detach the servo on pin right_rear to turn Vex Motor Controller 29 Off
    right_front_motor.detach();  // detach the servo on pin right_front to turn Vex Motor Controller 29 Off
  
    pinMode(left_front, INPUT);
    pinMode(left_rear, INPUT);
    pinMode(right_rear, INPUT);
    pinMode(right_front, INPUT);
}

void enable_motors() {
    left_front_motor.attach(left_front);    // attaches the servo on pin left_front to turn Vex Motor Controller 29 On
    left_rear_motor.attach(left_rear);      // attaches the servo on pin left_rear to turn Vex Motor Controller 29 On
    right_rear_motor.attach(right_rear);    // attaches the servo on pin right_rear to turn Vex Motor Controller 29 On
    right_front_motor.attach(right_front);  // attaches the servo on pin right_front to turn Vex Motor Controller 29 On
}

void strafe_left ()
{
    left_front_motor.writeMicroseconds(1500 - speed_val);
    left_rear_motor.writeMicroseconds(1500 + speed_val);
    right_rear_motor.writeMicroseconds(1500 + speed_val);
    right_front_motor.writeMicroseconds(1500 - speed_val);
}

void strafe_right ()
{
    left_front_motor.writeMicroseconds(1500 + speed_val);
    left_rear_motor.writeMicroseconds(1500 - speed_val);
    right_rear_motor.writeMicroseconds(1500 - speed_val);
    right_front_motor.writeMicroseconds(1500 + speed_val);
}

void ccw ()
{
    left_front_motor.writeMicroseconds(1500 - speed_val);
    left_rear_motor.writeMicroseconds(1500 - speed_val);
    right_rear_motor.writeMicroseconds(1500 - speed_val);
    right_front_motor.writeMicroseconds(1500 - speed_val);
}
void cw ()
{
    left_front_motor.writeMicroseconds(1500 + speed_val);
    left_rear_motor.writeMicroseconds(1500 + speed_val);
    right_rear_motor.writeMicroseconds(1500 + speed_val);
    right_front_motor.writeMicroseconds(1500 + speed_val);
}

void forward()
{
    left_front_motor.writeMicroseconds(1500 + speed_val);
    left_rear_motor.writeMicroseconds(1500 + speed_val);
    right_rear_motor.writeMicroseconds(1500 - speed_val);
    right_front_motor.writeMicroseconds(1500 - speed_val);
}

void reverse ()
{
    left_front_motor.writeMicroseconds(1500 - speed_val);
    left_rear_motor.writeMicroseconds(1500 - speed_val);
    right_rear_motor.writeMicroseconds(1500 + speed_val);
    right_front_motor.writeMicroseconds(1500 + speed_val);
}

void stop()  //Stop
{
    left_front_motor.writeMicroseconds(1500);
    left_rear_motor.writeMicroseconds(1500);
    right_rear_motor.writeMicroseconds(1500);
    right_front_motor.writeMicroseconds(1500);
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// BATTERY CHECK & ULTRASONIC RANGE

#ifndef NO_BATTERY_V_OK
boolean is_battery_voltage_OK() {
  static byte Low_voltage_counter;
  static unsigned long previous_millis;

  int Lipo_level_cal;
  int raw_lipo;
  //the voltage of a LiPo cell depends on its chemistry and varies from about 3.5V (discharged) = 717(3.5V Min) https://oscarliang.com/lipo-battery-guide/
  //to about 4.20-4.25V (fully charged) = 860(4.2V Max)
  //Lipo Cell voltage should never go below 3V, So 3.5V is a safety factor.
  raw_lipo = analogRead(A0);
  Lipo_level_cal = (raw_lipo - 717);
  Lipo_level_cal = Lipo_level_cal * 100;
  Lipo_level_cal = Lipo_level_cal / 143;

  if (Lipo_level_cal > 0 && Lipo_level_cal < 160) {
    previous_millis = millis();
    SerialCom->print("Lipo level:");
    SerialCom->print(Lipo_level_cal);
    SerialCom->print("%");
    // SerialCom->print(" : Raw Lipo:");
    // SerialCom->println(raw_lipo);
    SerialCom->println("");
    Low_voltage_counter = 0;
    return true;
  } else {
    if (Lipo_level_cal < 0)
      SerialCom->println("Lipo is Disconnected or Power Switch is turned OFF!!!");
    else if (Lipo_level_cal > 160)
      SerialCom->println("!Lipo is Overchanged!!!");
    else {
      SerialCom->println("Lipo voltage too LOW, any lower and the lipo with be damaged");
      SerialCom->print("Please Re-charge Lipo:");
      SerialCom->print(Lipo_level_cal);
      SerialCom->println("%");
    }

    Low_voltage_counter++;
    if (Low_voltage_counter > 5)
      return false;
    else
      return true;
  }
}
#endif

#ifndef NO_HC - SR04
void HC_SR04_range() {
  unsigned long t1;
  unsigned long t2;
  unsigned long pulse_width;
  float cm;
  float inches;

  // Hold the trigger pin high for at least 10 us
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Wait for pulse on echo pin
  t1 = micros();
  while (digitalRead(ECHO_PIN) == 0) {
    t2 = micros();
    pulse_width = t2 - t1;
    if (pulse_width > (MAX_DIST + 1000)) {
      SerialCom->println("HC-SR04: NOT found");
      return;
    }
  }

  // Measure how long the echo pin was held high (pulse width)
  // Note: the micros() counter will overflow after ~70 min

  t1 = micros();
  while (digitalRead(ECHO_PIN) == 1) {
    t2 = micros();
    pulse_width = t2 - t1;
    if (pulse_width > (MAX_DIST + 1000)) {
      SerialCom->println("HC-SR04: Out of range");
      return;
    }
  }

  t2 = micros();
  pulse_width = t2 - t1;

  // Calculate distance in centimeters and inches. The constants
  // are found in the datasheet, and calculated from the assumed speed
  //of sound in air at sea level (~340 m/s).
  cm = pulse_width / 58.0;
  inches = pulse_width / 148.0;

  // Print out results
  if (pulse_width > MAX_DIST) {
    SerialCom->println("HC-SR04: Out of range");
  } else {
    SerialCom->print("HC-SR04:");
    SerialCom->print(cm);
    SerialCom->println("cm");
  }
}
#endif
