#include <SoftwareSerial.h>
#include <Stepper.h>
#include "MPU9250.h"

const bool DEBUG_MODE = false;

//motor
const int stepsPerRevolution = 400;
Stepper myStepper(stepsPerRevolution, 4, 5, 6, 7);
const double STEPPER_RATIO = 400.0/360.0;

//For the Gyro
MPU9250 myIMU;
double gyro_reset_vel =0.0;
int send_time = 0;
int num_of_measures =0;

//BT
SoftwareSerial bt (11, 12);
bool updateInitials = false;
const char END_OF_DIST = 'd';
const char END_OF_ANG = 'a';
const char UPDATE_INITIALS = 'u';
const int BT_WAIT_TIME = 3;


//photomafsek
const int digitalPhotoPin = 3;
const double PER = 30.43;
const double RATIO = 0.125;

// measure of location
//angles are in degrees!! distances in centimeters
double dist = 160.0, theta = 90.0;
double gyro_ang_vel = 0.0, sensor_vel = 0.0, prev_ang_vel = 0.0; //get from sensors
double dt = 0.0, actual_theta = theta;
double angle_leftovers = 0.0;
unsigned long last_interrupt_time = millis();
unsigned long prev_time = millis();
double dt_interrupt = 0.0;
bool is_interrupt = false, isFirstInterrupt = true;

void setup() {
  Wire.begin();
  
   // set the motor speed at 60 rpm:
  myStepper.setSpeed(10);

  //BT setup
  Serial.begin(9600); //Open Serial connection for debugging
  bt.begin(9600);

  //Gyro setup
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  if (c == 0x71) // WHO_AM_I should always be 0x68
  {
    // Start by performing self test and reporting values
    myIMU.MPU9250SelfTest(myIMU.SelfTest);
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);
    myIMU.initMPU9250();
    myIMU.initAK8963(myIMU.magCalibration);

  } // if (c == 0x71)
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);
  }

  //photomafsek setup
  pinMode(digitalPhotoPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(digitalPhotoPin), photo_interrupt ,CHANGE);
  last_interrupt_time = millis();
  
}

//Main loop
void loop() {

  // Get data from BT
  if(bt.available() > 0){
    //Serial.println("availanle");
    delay(BT_WAIT_TIME);
    char input_char = bt.read();
    if (input_char == '+' || input_char == '-'){
        int usr_input = (int)bt.read();
        if (input_char == '-'){
          usr_input = -usr_input;
        }
        //Serial.print("angle:");Serial.println(usr_input);
        addAngle(usr_input, true);
        Serial.print("From BT:");Serial.println(usr_input);
     }
    input_char = bt.read();
    if (input_char == 'd'){
        dist = (int)bt.read() * 10.0;
        //Serial.print("dist:");Serial.println(dist);
     }
  }//end of BT
  
    //Gyro reading 
    while (millis() < 2000){
      myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values
      myIMU.getGres();
      // Calculate the gyro value into actual degrees per second
      myIMU.gz = (double)myIMU.gyroCount[2]*myIMU.gRes;
      myIMU.updateTime();
      gyro_ang_vel = myIMU.gz;
      if (num_of_measures !=0){
        gyro_reset_vel *=num_of_measures;
      }
      num_of_measures++;
      gyro_reset_vel += gyro_ang_vel;
      gyro_reset_vel /= num_of_measures;
    }
    //Serial.println(gyro_ang_vel);


    //END GYRO
    long last_while_time = micros();
  while(!is_interrupt && millis() - last_interrupt_time < 300 ){
    long while_dt = micros()-last_while_time;
    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values
    myIMU.getGres();
    // Calculate the gyro value into actual degrees per second
    myIMU.gz = (double)myIMU.gyroCount[2]*myIMU.gRes;
    myIMU.updateTime();
    gyro_ang_vel = myIMU.gz - gyro_reset_vel;
    if (abs(gyro_ang_vel)<0.5){
      gyro_ang_vel = 0.0;
    }
    theta += gyro_ang_vel*while_dt/1000000.0;
    last_while_time = micros();
    delay(30);
  }
  if (!is_interrupt){
    sensor_vel = 0.0;
    addAngle(theta-actual_theta, true);
    //Serial.println("too slow");
  }
  else{
    is_interrupt = false;
    last_interrupt_time = millis();
    double mean_theta = 0.5*(theta+actual_theta)*PI/180.0;
    double dx = PER*RATIO;
    double dtheta = atan(sin(mean_theta)/(dist/dx - cos(mean_theta)))*180.0/PI;
    if (mean_theta != 0.0){
      dist = dist*sin(mean_theta)/sin(mean_theta+dtheta*PI/180);
    }
    addAngle(dtheta, true);
    Serial.println(dtheta);
  }   
}

void photo_interrupt(){
  is_interrupt = true;
  //Serial.println("interrupted!");
}

void addAngle(double addition, bool flag){
  if (abs((int)(STEPPER_RATIO*addition+angle_leftovers)) >= 2){
    if (flag){
      myStepper.step((int)(STEPPER_RATIO*addition+angle_leftovers)); //the sending value (addition to the original angle)
     // Serial.println((int)(STEPPER_RATIO*addition+angle_leftovers));
      }
    else{
      myStepper.step((int)(STEPPER_RATIO*addition));
     // Serial.println((int)(STEPPER_RATIO*addition));
    }
    actual_theta += (double)((int)(STEPPER_RATIO*addition+angle_leftovers))/STEPPER_RATIO; //theta for next loop
    theta = actual_theta;
    angle_leftovers = STEPPER_RATIO*addition+angle_leftovers-(int)(STEPPER_RATIO*addition+angle_leftovers);
    //Serial.println(theta);
  }
  else{
      angle_leftovers += STEPPER_RATIO*addition;
  }
  //Serial.println(angle_leftovers);
}

