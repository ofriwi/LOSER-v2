#include <SoftwareSerial.h> //Serial library
#include <Encoder.h>

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
  double newPosition = myEnc.read() * ENCODER_STEP;
  if (abs(newPosition - oldPosition) > ENCODER_STEP){
    oldPosition = newPosition;
    Serial.println(newPosition);
  }

  // *** Get input ***
  if (use_bt){ 
    if (bt.available()){
      delay(3);
      sign = bt.read();
      input = (int)bt.read();

      Serial.print(sign);
      Serial.println(input);
    
      if (sign == '-'){
        input = -input;
      }
    }
  }else{
    if (Serial.available() > 0) { 
      input = (int)Serial.parseInt();
      delay(1);
      while(Serial.available()) Serial.read();
      Serial.println(String(input));
    }
  }

  
  // *** Move motor ***
  rotate(input);
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


// OLD CODE
/*
  if (input > cur_speed){
    cur_speed++;
    rotate(cur_speed);
    delay(10);
  }else if (input < cur_speed){
    cur_speed--;
    rotate(cur_speed);
    delay(10);
  }
  */
  //cur_speed = input;
  //Serial.println(String(cur_speed));
  //rotate(cur_speed);
