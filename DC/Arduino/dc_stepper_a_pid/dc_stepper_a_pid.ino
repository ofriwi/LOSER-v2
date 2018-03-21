#include <PID_AutoTune_v0.h>

#include <SoftwareSerial.h>
#include <Encoder.h>
#include <PID_v1.h>

#define DEBUG_MODE true

#define enA 10
#define in1 8
#define in2 9

#define encoderPinA 2
#define encoderPinB 3

#define MAX_SPEED 200
#define MIN_SPEED 0

#define CW true
#define CCW false

double ENCODER_STEP = 0.171756;
float Kp = 0.6, Ki = 0.5, Kd = 0;
double Output;
double newPosition;
double target = 0;
PID pid(&newPosition, &Output, &target, Kp, Ki, Kd, P_ON_M, DIRECT);
PID_ATune aTune(&newPosition, &Output);
boolean tuning = true;
unsigned long  modelTime, serialTime;
double outputStart=50;
double aTuneStep=50, aTuneNoise=1, aTuneStartValue=100;
unsigned int aTuneLookBack=20;
byte ATuneModeRemember=2;


volatile int encoder0Pos = 0;
volatile int encoder0PinALast = LOW;
volatile int n = LOW;
volatile int m = LOW;

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

// *** Encoder ***
Encoder myEnc(5, 6);
double oldPosition  = -999;

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

  // Auto PID
  if(tuning)
  {
    tuning=false;
    changeAutoTune();
    tuning=true;
  }
  
  serialTime = 0;

}

void loop() {
  
  unsigned long now = millis();
  // *** Read Encoder ***
  
  newPosition = -encoder0Pos * ENCODER_STEP;
  if (abs(newPosition - oldPosition) > ENCODER_STEP){
    oldPosition = newPosition;
    if (DEBUG_MODE){
      Serial.print("Deg: ");Serial.println(newPosition);
      Serial.print("Rot speed: ");Serial.println(rot_speed);
    }
  }

  if(tuning)
  {
    byte val = (aTune.Runtime());
    if (val!=0)
    {
      tuning = false;
    }
    if(!tuning)
    { //we're done, set the tuning parameters
      Kp = aTune.GetKp();
      Ki = aTune.GetKi();
      Kd = aTune.GetKd();
      pid.SetTunings(Kp,Ki,Kd);
      AutoTuneHelper(false);
    }
  }
  else{
    pid.Compute();
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
  }

  if (to_move)
    move_to_target();


  // Auto Tune
  if(millis()>serialTime)
  {
    SerialReceive();
    SerialSend();
    serialTime+=500;
  }

}
// ***********************************
void changeAutoTune()
{
 if(!tuning)
  {
    //Set the output to the desired starting frequency.
    Output=aTuneStartValue;
    aTune.SetNoiseBand(aTuneNoise);
    aTune.SetOutputStep(aTuneStep);
    aTune.SetLookbackSec((int)aTuneLookBack);
    AutoTuneHelper(true);
    tuning = true;
  }
  else
  { //cancel autotune
    aTune.Cancel();
    tuning = false;
    AutoTuneHelper(false);
  }
}

void AutoTuneHelper(boolean start)
{
  if(start)
    ATuneModeRemember = pid.GetMode();
  else
    pid.SetMode(ATuneModeRemember);
}


void SerialSend()
{
  Serial.print("target: ");Serial.print(target); Serial.print(" ");
  Serial.print("newPosition: ");Serial.print(newPosition); Serial.print(" ");
  Serial.print("output: ");Serial.print(Output); Serial.print(" ");
  if(tuning){
    Serial.println("tuning mode");
  } else {
    Serial.print("kp: ");Serial.print(pid.GetKp());Serial.print(" ");
    Serial.print("ki: ");Serial.print(pid.GetKi());Serial.print(" ");
    Serial.print("kd: ");Serial.print(pid.GetKd());Serial.println();
  }
}

void SerialReceive()
{
  if(Serial.available())
  {
   char b = Serial.read(); 
   Serial.flush(); 
   if((b=='1' && !tuning) || (b!='1' && tuning))changeAutoTune();
  }
}
// *****************************************
void move_to_target(){
  /*if (dir_to_target == CW && target - newPosition > close_enough){
    rotate(rot_speed);
  }else if (dir_to_target == CCW && newPosition - target > close_enough){
    rotate(-rot_speed);*/
  if (abs(to_deg(target - newPosition)) > close_enough){
    pid.Compute();
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
