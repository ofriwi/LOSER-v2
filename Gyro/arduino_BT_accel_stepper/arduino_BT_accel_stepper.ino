#include "MPU9250.h"
#include <SoftwareSerial.h>
#include <Stepper.h>

bool input = false;
float dist;
Stepper myStepper(stepsPerRevolution, 4, 5, 6, 7);

//motor
const int stepsPerRevolution = 720;

//photomafsek
const int rightPin = A3;
const int leftPin = A2;
const float NO_INPUT = 1111.1111;
float right_vel = 0.0, left_vel = 0.0, mean_vel = 0.0;
const float PER = 0.22;
const float RATIO = 0.125;
bool started[2] = {false, false};
bool isChecked[2] = {false, false}; //is the photomafsek interrupted
int current_time[2] = {0,0}, last_loop[2] = {0,0};

//For the Gyro
MPU9250 myIMU;
float gyro_val = 0.0;
float theta = 0.0;
int send_time = 0;

//sending to pi
/**int btdata; // the data given from the computer
char receivedChar;
bool newData = false;
**/

//BT
SoftwareSerial bt (2, 4);
const char END_OF_DIST = 'd';
const char END_OF_ANG = 'a';
const char UPDATE_INITIALS = 'u';

// measure of location
//angles are in degrees!! distances in meters
float my_x = 0.0, my_y = 0.0, theta = 0.0, cam_angle = 0.0, target_x = 0.0;
float dist = 0.0, angle = 0.0;
float dt = 0.0, gyro_ang_vel = 0.0, sensor_vel = 0.0; //get from sensors

void setup() {
  Wire.begin();
  
   // set the motor speed at 60 rpm:
  myStepper.setSpeed(60);
  
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
  } // if (c == 0x71)
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);
  }

}

//Main loop
void loop() {
  // Get data from BT
  if(bt.available() > 0){
    char recievedDist[7] = {'0'};
    char_index = 0;
    input_char = bt.read();
     if (input_char == UPDATE_INITIALS){
     updateInitials = true;
     }
    input_char = bt.read();
    while(bt.available() > 0 && input_char() != END_OF_DIST){
      recievedDist[char_index] = input_char;
      char_index++;
      input_char = bt.read();
    } 
    dist = atof(recievedDist);
    
    char recievedAng[7] = {'0'};
    char_index = 0;
    input_char = bt.read();    
    while (bt.available() > 0 && input_char() != END_OF_ANG){
      recievedAng[char_index] = bt.read();
      char_index++;
      input_char = bt.read();
    }
    angle = atof(recievedAng);
  }//end of BT

    //Gyro reading 
    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values
    myIMU.getGres();
    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = (float)myIMU.gyroCount[0]*myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1]*myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2]*myIMU.gRes;
    gyro_ang_vel = myIMU.gz;
    //END GYRO
  
    //velocity from photomafsek
    float temp_right_vel = speedFromSensor(rightPin,0);
    float temp_left_vel = speedFromSensor(leftPin, 1);
    if (temp_right_vel != NO_INPUT){
      right_vel = temp_right_vel;
      }
    if (temp_left_vel != NO_INPUT){
      left_vel = temp_left_vel;
    }
  
    if (right_vel != NO_INPUT && left_vel != NO_INPUT){
      sensor_vel = 0.5*(right_vel+left_vel);
    }
    else if (right_vel == NO_INPUT){
      sensor_vel = left_vel;
    }
    else{
      sensor_vel = right_vel;
    } 
    //END VELOCITY
  
     if (updateInitials){ //lost of line of sight
      my_x = 0.0, my_y = 0.0;
      target_x = dist;
      theta = angle;
      cam_angle = angle;
      updateInitials = false;
     }
     else{ //updating the location based on the last info about the location (the last LOS)
      dt = millis() - myIMU.count;
      theta += gyro_ang_vel*dt;
      my_x += sensor_vel*cos(theta*math.PI/180)*dt;
      my_y += sensor_vel*sin(theta*math.PI/180)*dt;
      float new_cam_angle = theta + math.atan2((-1.0*y),(target_x-x))*180/math.PI;
      float send_angle = cam_angle-new_cam_angle; //the sending value (addition to the original angle)
     }
    myIMU.count = millis();
    /**myIMU.delt_t = millis() - myIMU.count;
      gyro_val = myIMU.gz;
      theta += gyro_val*myIMU.delt_t*0.001;
      myIMU.count = millis();
    send_time += myIMU.delt_t;
    if (send_time > 200){
      bt.print('g');
      bt.print(theta);
      Serial.println(theta);
      theta = 0.0;
      send_time = 0;
    }**/

  //get a char from serial
//   if (Serial.available() > 0) { 
//     receivedChar = Serial.read();
//     newData = true;
//   }
//    if (newData) {
//     Serial.println(String(receivedChar));
//     newData = false;
//     bt.write(receivedChar);
//    }

}

/**
gets speed from sensor - 0 = right, left = 1
**/
float speedFromSensor (int pin, int index)
{
  current_time[index] = millis()-last_loop[index];
  float velocity = NO_INPUT;
  int val = digitalRead(pin);
  if (current_time[index] > 1000){ //too much time has passed
    started[index] = false;
  }
  if (!started[index]){ 
    if (val){ //a new cycle started
      started[index] = true;
      isChecked[index] = true;
      last_loop[index] = millis();
    }
    else{
      return 0.0;
    }
  }
  else{
    // a normal cycle
    if (val){
      isChecked[index] = true;
    }
    if(!val && isChecked[index]){
          //Serial.println("T = "); Serial.println((float)current_time[index]*0.001);
          velocity = PER*RATIO*3.6 / ((float)current_time[index]*0.001);
          //Serial.println("v = "); Serial.println(velocity);
          isChecked[index] = false;
          last_loop[index] = millis();
    }
   }
  return velocity;
}
