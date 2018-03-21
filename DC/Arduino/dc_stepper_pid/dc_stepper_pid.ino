#include <SoftwareSerial.h>
#include <Encoder.h>
#include <PID_v1.h>

#define DEBUG_MODE true

#define enA 10
#define in1 8
#define in2 9

#define encoderPinA 2
#define encoderPinB 3

#define MAX_SPEED 100
#define MIN_SPEED 0
#define ACCEL_SPEED 50

#define CW true
#define CCW false

double ENCODER_STEP = 0.171756;
float Kp = 8, Ki = 0.5, Kd = 1.4;
//float max_Kp = 4, max_Ki = 2;
double Output;
double newPosition;
double target = 0;
PID pid(&newPosition, &Output, &target, Kp, Ki, Kd, DIRECT);

volatile int encoder0Pos = 0;
volatile int encoder0PinALast = LOW;
volatile int n = LOW;
volatile int m = LOW;

int steps = 0, steps_to_print = 1000;

// *** BT ***
SoftwareSerial bt (11,12);  //RX, TX (Switched on the Bluetooth - RX -> TX | TX -> RX)
bool use_bt = false;
int input = 0;
char sign;

// *** Motor ***
bool cur_direction = CW;
bool to_move = false;
bool new_input = false;
bool dir_to_target;
int rot_speed = 50;
double close_enough = ENCODER_STEP;
bool accel = false;

// *** Encoder ***
Encoder myEnc(5, 6);
double oldPosition  = -999;

void CountA()
{
  n = digitalRead(encoderPinA); 
  if ((encoder0PinALast == LOW) && (n == HIGH)) { 
    if (m == LOW) {                                 // m == direction
      encoder0Pos--; 
    } 
    else { 
      encoder0Pos++; 
    } 
  }
}
void StateB()
{
  m = digitalRead(encoderPinB);
}

void setup() {
  if (use_bt){
    bt.begin(9600);
    //bt.listen();
  }
  Serial.begin(9600);
  Serial.println(ENCODER_STEP);
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  // Encoder
  pinMode (encoderPinA,INPUT); 
  pinMode (encoderPinB,INPUT);
  attachInterrupt(1, CountA, CHANGE);
  attachInterrupt(0, StateB, FALLING);

  // PID
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-(MAX_SPEED-MIN_SPEED), MAX_SPEED-MIN_SPEED);
}

void loop() {
  
  // *** Read Encoder ***
  
  newPosition = -encoder0Pos * ENCODER_STEP;
  if (abs(newPosition - oldPosition) > ENCODER_STEP){
    oldPosition = newPosition;
    accel = false;
  }

  // *** Get input ***
  if (use_bt){ 
    if (bt.available()){
      delay(3);
      sign = bt.read();
      input = (int)bt.read();

      if (sign == '-'){
        input = -input;
      }
      new_input = true;
    }
  }else{
    if (Serial.available() > 0) { 
      input = Serial.parseInt();
      new_input = true;
    }
  }

  
  // *** Move motor ***
  if (new_input){
    new_input = false;
    if (DEBUG_MODE){
      Serial.println(to_deg(input));
      encoder0Pos = 0;
      newPosition = 0;
    }
    //rotate(0);
    input = to_deg(input);
    target = newPosition + input;
    dir_to_target = target>newPosition;
    Serial.print("New position now: "); Serial.println(newPosition);
    Serial.print("new target: "); Serial.println(target);
    to_move = true;
    // TEST
    //if (Output < ACCEL_SPEED){
    //  accel = true;
    //  Serial.print("Acceling: ");Serial.println();
    //}
    // END
  }

  if (to_move)
    move_to_target();
}

void move_to_target(){
  /*if (dir_to_target == CW && target - newPosition > close_enough){
    rotate(rot_speed);
  }else if (dir_to_target == CCW && newPosition - target > close_enough){
    rotate(-rot_speed);*/
  if (abs(to_deg(target - newPosition)) > close_enough){
    pid.Compute();
    // TEST
    //if (accel){
    //  Output = ACCEL_SPEED;
    //}
    // END
    rot_speed = (int)Output;
    /*if (DEBUG_MODE){
      Serial.print("Speed: "); Serial.println(Output);
    }*/
    rotate(rot_speed);
  }else{
    rotate(0);
    //to_move = false;
    /*if (DEBUG_MODE){
      delay(1000);
      Serial.print("Angle from target: "); Serial.println(newPosition-target);
      Serial.print("Target: "); Serial.println(target);
      Serial.print("Position: "); Serial.println(newPosition);
    }*/
    //target = newPosition;
  }
}

void rotate(int rot_speed){
   direction(rot_speed>0);
   analogWrite(enA, abs(rot_speed));
   steps++;
    if (steps % steps_to_print == 0 && DEBUG_MODE){
      Serial.print("Deg: ");Serial.println(newPosition);
      Serial.print("Rot speed: ");Serial.println(rot_speed);
      steps = 0;
    }
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

float to_deg(float deg){
    deg -= floor(deg/360) * 360;
    if (deg > 180){
      deg -= 360;
    }
    return deg;
}

