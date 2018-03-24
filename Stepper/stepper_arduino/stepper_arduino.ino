#include <Stepper.h>
#include <SoftwareSerial.h>

// *** General ***
const bool DEBUG_MODE = true;
bool new_input = false;
bool use_bt = true;
int motorSpeed = 20; // TODO: optimize

// *** Stepper ***
const int stepsPerRevolution = 400; // TODO: choose
Stepper myStepper(stepsPerRevolution, 4, 5, 6, 7);
float degCount = 0;

// *** communication ***
SoftwareSerial bt (11,12);  //RX, TX (Switched on the Bluetooth - RX -> TX | TX -> RX)
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
      Serial.println("BT");
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
      rotate(usr_input);
  }
}

// Rotate motor approximatly degrees and return how much degrees it really did.
float rotate(int degrees){
  int steps = round(degrees / 360.0 * stepsPerRevolution);
  if (DEBUG_MODE)
    Serial.print("Steps: ");Serial.println(steps);
  myStepper.step(steps);
  float real_degrees = steps * 360.0 / stepsPerRevolution;
  if (DEBUG_MODE)
    Serial.print("Real deg: ");Serial.println(real_degrees);
  degCount += real_degrees;
  if (DEBUG_MODE)
    Serial.print("Deg count: ");Serial.println(degCount);
  return real_degrees;
}
