#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

#include <SoftwareSerial.h>
#include <Encoder.h>


#define DEBUG_MODE true

#define enA 10
#define in1 8
#define in2 9

#define encoderPinA 2
#define encoderPinB 3


#define CW true
#define CCW false

double ENCODER_STEP = 0.171756;

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
double target = 0;
bool new_input = false;
bool dir_to_target;
double newPosition;
int rot_speed = 50;
double close_enough = ENCODER_STEP;

// *** Encoder ***
Encoder myEnc(5, 6);
double oldPosition  = -999;


byte ATuneModeRemember=2;
double Input=0, output=0, setpoint=180;
double kp=0.6,ki=0.1,kd=0.1;

double kpmodel=1.5, taup=100, theta[50];
double outputStart=5;
double aTuneStep=50, aTuneNoise=1, aTuneStartValue=100;
unsigned int aTuneLookBack=20;

boolean tuning = false;
unsigned long  modelTime, serialTime;

PID myPID(&Input, &output, &setpoint,kp,ki,kd, DIRECT);
PID_ATune aTune(&Input, &output);

//set to false to connect to the real world
boolean useSimulation = false;

void setup()
{
  if (use_bt){
    bt.begin(9600);
    //bt.listen();
  }
  Serial.begin(9600);
  Serial.println(ENCODER_STEP);
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  
  pinMode (encoderPinA,INPUT); 
  pinMode (encoderPinB,INPUT);
  attachInterrupt(1, CountA, CHANGE);
  attachInterrupt(0, StateB, FALLING);

  if(useSimulation)
  {
    for(byte i=0;i<50;i++)
    {
      theta[i]=outputStart;
    }
    modelTime = 0;
  }
  //Setup the pid 
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255);
  if(tuning)
  {
    tuning=false;
    changeAutoTune();
    tuning=true;
  }
  
  serialTime = 0;
  Serial.begin(9600);

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

float to_deg(float deg){
    deg -= floor(deg/360) * 360;
    //if (deg > 180){
      //deg -= 360;
    //}
    return deg;
}

void loop()
{

  unsigned long now = millis();

  newPosition = to_deg(-encoder0Pos * ENCODER_STEP);
  
  if(!useSimulation)
  { //pull the input in from the real world
    Input = newPosition;
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
      kp = aTune.GetKp();
      ki = aTune.GetKi();
      kd = aTune.GetKd();
      myPID.SetTunings(kp,ki,kd);
      AutoTuneHelper(false);
    }
  }
  else myPID.Compute();
  
  if(useSimulation)
  {
    theta[30]=output;
    if(now>=modelTime)
    {
      modelTime +=100; 
      DoModel();
    }
  }
  else
  {
     rotate(output); 
  }
  
  //send-receive with processing if it's time
  if(millis()>serialTime)
  {
    SerialReceive();
    SerialSend();
    serialTime+=500;
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

void changeAutoTune()
{
 if(!tuning)
  {
    //Set the output to the desired starting frequency.
    output=aTuneStartValue;
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
    ATuneModeRemember = myPID.GetMode();
  else
    myPID.SetMode(ATuneModeRemember);
}


void SerialSend()
{
  Serial.print("setpoint: ");Serial.print(setpoint); Serial.print(" ");
  Serial.print("input: ");Serial.print(Input); Serial.print(" ");
  Serial.print("output: ");Serial.print(output); Serial.print(" ");
  if(tuning){
    Serial.println("tuning mode");
  } else {
    Serial.print("kp: ");Serial.print(myPID.GetKp());Serial.print(" ");
    Serial.print("ki: ");Serial.print(myPID.GetKi());Serial.print(" ");
    Serial.print("kd: ");Serial.print(myPID.GetKd());Serial.println();
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

void DoModel()
{
  //cycle the dead time
  for(byte i=0;i<49;i++)
  {
    theta[i] = theta[i+1];
  }
  //compute the input
  Input = (kpmodel / taup) *(theta[0]-outputStart) + Input*(1-1/taup) + ((float)random(-10,10))/100;

}
