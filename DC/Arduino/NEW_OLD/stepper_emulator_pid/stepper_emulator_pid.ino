#include <SoftwareSerial.h> //Serial library
#include <Encoder.h>
#include <PID_v1.h>
#include "MPU9250.h"

#define enA 10
#define in1 8
#define in2 9

#define CW true
#define CCW false

#define MAX_SPEED 100
#define MIN_SPEED 70
#define SPEED_TRESH 10

#define ENCODER_STEP 0.042939

int counter = 0;

// *** BT ***
SoftwareSerial bt (2,4);  //RX, TX (Switched on the Bluetooth - RX -> TX | TX -> RX)
bool use_bt = true;
int input = 0;
char sign;

// *** GYRO ****
MPU9250 myIMU;
float gyro_val = 0.0;
float theta = 0.0;
int send_time = 0;

//sending to pi
int btdata; // the data given from the computer
char receivedChar;
bool newData = false;

// *** Motor ***
bool cur_direction = CW;
int target = 0;
bool new_input = false;
bool dir_to_target;
double newPosition;

// *** Encoder ***
Encoder myEnc(5, 6);
double oldPosition  = -999;

// *** PID ***
double Setpoint, In, Output;
double Kp = 3, Ki = 1, Kd = 0.5; 
PID pid(&In, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  if (use_bt){
    bt.begin(9600);
    //bt.listen();
  }
  Serial.begin(9600);
  
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  direction(true);
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-(MAX_SPEED-MIN_SPEED), MAX_SPEED-MIN_SPEED);

  gyroSetup();
}

void loop() {
  counter++;
  
  // *** Read Encoder ***
  newPosition = myEnc.read() * ENCODER_STEP;
  In = newPosition;
  if (abs(newPosition - oldPosition) > ENCODER_STEP){
    oldPosition = newPosition;
    Serial.print("Angle: ");Serial.println(newPosition);
    //Serial.print("Distance from target: ");Serial.println(target - newPosition);
  }

  gyroSend();

  
  // *** Get input ***
  if (use_bt){ 
    if (bt.available()){
      delay(3);
      sign = bt.read();
      input = (int)bt.read();

      Serial.print("input: ");
      Serial.print(sign);
      Serial.println(input);
    
      if (sign == '-'){
        input = -input;
      }
      new_input = true;
    }
  }else{
    if (Serial.available() > 0) { 
      input = (int)Serial.parseInt();
      delay(1);
      while(Serial.available()) Serial.read();
      Serial.print("input: ");
      Serial.println(String(input));
      new_input = true;
    }
  }

  
  // *** Move motor ***
  if (new_input){
    new_input = false;
    input = input % 360;
    if (input > 180){
      input -= 360;
    }
    rotate(0);
    target = newPosition + input;
    target = target % 360;
    if (target > 180){
      target -= 360;
    }
    dir_to_target = target>newPosition;

    //Serial.print("SP: "); Serial.println(target);
  }

  move_to_target();
}

void move_to_target(){
  Setpoint = target;
  pid.Compute();
  int speedd = (int)Output;
  speedd = normalize(speedd, MIN_SPEED, MAX_SPEED, SPEED_TRESH);
  if (counter % 15000 == 0){
    Serial.print("Angle: "); Serial.print(In);
    Serial.print("->"); Serial.println(Setpoint);
    Serial.print("Speed: "); Serial.print(Output);
    Serial.print("->"); Serial.println(speedd);
  }
  rotate(speedd);
}

void rotate(int rot_speed){
   bool new_direction = rot_speed>0;
   if (cur_direction != new_direction){
    analogWrite(enA, 0);
    //Serial.println("Change dir");
   }
   direction(rot_speed>0);
   analogWrite(enA, abs(rot_speed));
   cur_direction = new_direction;
}

void direction(bool cw){
  if (cw){
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }else{
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
}

void gyroSetup(){
  Wire.begin();
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

void gyroSend(){
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
      //Serial.print("IMU");Serial.println(theta);
      theta = 0.0;
      send_time = 0;
    }
}

int normalize(int input, int min_value, int max_value, int cut_off){
    int norm = abs(input);
    if (norm < cut_off)
        return 0;
    norm = norm + min_value - cut_off;
    norm = min(norm, max_value);
    if (input < 0)
        norm = -norm;
    return norm;
}
