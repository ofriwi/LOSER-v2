#include <Stepper.h>
#include <SoftwareSerial.h>

// *** General ***
const bool DEBUG_MODE = true;
bool new_input = false;
bool use_bt = true;
int motorSpeed = 20; // TODO: optimize

// *** Stepper ***
const int stepsPerRevolution = 400; // TODO: choose
int STEPS_A_TIME = 1;
Stepper myStepper(stepsPerRevolution, 4, 5, 6, 7);
float degCount = 0;

// *** communication ***
SoftwareSerial bt (11,12);  //RX, TX (Switched on the Bluetooth - RX -> TX | TX -> RX)
int usr_input = 0;
char sign;
const int BT_WAIT_TIME = 3, SER_WAIT_TIME = 1; // TODO : optimize

// *** Position ***
int current = 0;
int target = 0;
bool cur_direction = true;

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
      set_target(usr_input);
  }
  move();
}

// set_target motor approximatly degrees and return how much degrees it really did.
void set_target(int degrees){
  int steps = round(degrees / 360.0 * stepsPerRevolution);
  if (DEBUG_MODE)
    Serial.print("Steps: ");Serial.println(steps);
  target = current + steps;
}

void move(){
    int steps = 0;
    if (cur_direction != (target > current)){
      delay(100);
      cur_direction = target>current;
    }
    if (target > current){
        steps = min(target - current, STEPS_A_TIME);
    } else if (target < current){
        steps = -min(current - target, STEPS_A_TIME);
    }
    myStepper.step(steps);
    degCount += 360.0 / stepsPerRevolution * steps;
    current += steps;
}
