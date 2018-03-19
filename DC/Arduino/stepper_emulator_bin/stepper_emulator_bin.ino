#include <SoftwareSerial.h> //Serial library
#include <Encoder.h>


#define DEBUG_MODE true


#define enA 10
#define in1 8
#define in2 9

#define CW true
#define CCW false

#define ENCODER_STEP 0.042939

// *** BT ***
SoftwareSerial bt (2,4);  //RX, TX (Switched on the Bluetooth - RX -> TX | TX -> RX)
bool use_bt = true;
int input = 0;
char sign;

// *** Motor ***
bool cur_direction = CW;
bool to_move = false;
int target = 0;
bool new_input = false;
bool dir_to_target;
double newPosition;
int speedd = 90;

// *** Encoder ***
Encoder myEnc(5, 6);
double oldPosition  = -999;

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
}

void loop() {
  
  // *** Read Encoder ***
  newPosition = myEnc.read() * ENCODER_STEP;
  if (abs(newPosition - oldPosition) > ENCODER_STEP){
    oldPosition = newPosition;
    Serial.println(newPosition);
  }

  // *** Get input ***
  if (use_bt){ 
    if (bt.available()){
      delay(100);
      sign = bt.read();
      input = (int)bt.read();

      Serial.print(sign);
      Serial.println(input);
    
      if (sign == '-'){
        input = -input;
      }
      new_input = true;
    }
  }else{
    if (Serial.available() > 0) { 
      input = Serial.parseInt();
      Serial.println(String(input));
      new_input = true;
    }
  }

  
  // *** Move motor ***
  if (new_input){
    new_input = false;
    Serial.println("new in");
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
    to_move = true;
  }
  if (to_move)
    move_to_target();
}

void move_to_target(){
  if (abs(int(target) % 360 - int(newPosition) % 360) > ENCODER_STEP && dir_to_target == CW){
    rotate(speedd);
  }else if (abs(int(target) % 360 - int(newPosition) % 360) > ENCODER_STEP && dir_to_target == CCW){
    rotate(-speedd);
  }else{
    rotate(0);
    to_move = false;
    target = newPosition;
  }
}

void rotate(int rot_speed){
   bool new_direction = rot_speed>0;
   if (cur_direction != new_direction){
    analogWrite(enA, 0);
    //Serial.println("Change dir");
    delay(100);
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
