#include <Stepper.h>
#include <SoftwareSerial.h>

// *** General ***
const bool DEBUG_MODE = true;
bool new_input = false;
int motorSpeed = 0;

// *** Stepper ***
const int stepsPerRevolution = 400;
Stepper myStepper(stepsPerRevolution, 4, 5, 6, 7);
float stepsCount = 0;

// *** communication ***
SoftwareSerial bt (2,4);  //RX, TX (Switched on the Bluetooth - RX -> TX | TX -> RX)
bool use_bt = false;
int usr_input = 0;
char sign;
const int BT_WAIT_TIME = 3, SER_WAIT_TIME = 1; // TODO : optimize

void setup() {
  if (use_bt){
    bt.begin(9600);
    //bt.listen();
  }
  Serial.begin(9600);
  myStepper.setSpeed(motorSpeed);
}

void loop() {

  // *** Get usr_input from BT or serial ***
  // TODO : optimize to prevet time leak
  if (use_bt){ 
    if (bt.available()){
      delay(BT_WAIT_TIME);
      sign = bt.read();
      usr_input = (int)bt.read();
      if (sign == '-'){
        usr_input = -usr_input;
      }
      new_input = true;
    }
  }else{
    if (Serial.available() > 0) { 
      usr_input = (int)Serial.parseInt();
      delay(SER_WAIT_TIME);
      while(Serial.available()) Serial.read();
      new_input = true;
    }
  }

  if (new_input){
      new_input = false;
      if (DEBUG_MODE)
        Serial.print("User input: ");Serial.println(usr_input);
      rotate(new_input);
  }
}

// Rotate motor approximatly degrees and return how much degrees it really did.
float rotate(int degrees){
  int steps = (int)(degrees * 360.0/stepsPerRevolution);
  myStepper.step(deg_to_degrees);
  float real_degrees = steps * stepsPerRevolution / 360.0;
  stepsCount += real_degrees;
  return real_degrees;
}