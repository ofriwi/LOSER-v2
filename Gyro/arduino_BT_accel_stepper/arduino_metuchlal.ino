#include "MPU9250.h"
#include <SoftwareSerial.h>

int step_mode = 2;
int steps_per_revolution = 400;
int offset = 128;
bool input = false;
int angle;

//For the Gyro
MPU9250 myIMU;
float gyro_val = 0.0;
float theta = 0.0;
int send_time = 0;

//sending to pi
int btdata; // the data given from the computer
char receivedChar;
bool newData = false;

//BT
SoftwareSerial bt (2, 4);

void setup() {
  Wire.begin();
  
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
  /** input = false;
  if(bt.available() > 0){
    while(bt.available() > 1){
      bt.read();
    }//     WAIT FOR LAST INPUT 
    angle = (int)bt.read() - offset; //Read user input and trigger appropriate function
    input = true;
  }**/

  //Gyro reading 
    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values
    myIMU.getGres();

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = (float)myIMU.gyroCount[0]*myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1]*myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2]*myIMU.gRes;

    myIMU.delt_t = millis() - myIMU.count;
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
    }

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
    
  /*
  // Get data from serial (for debugging)
  if(Serial.available() > 0){
    while(Serial.available() > 1){
      Serial.read();
    }//     WAIT FOR LAST INPUT 
    angle = (int)Serial.parseInt(); //Read user input and trigger appropriate function
    input = true;
  }*/
  

}
