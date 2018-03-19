/* Encoder Library - Basic Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
 */

#include <Encoder.h>

#define enA 10
#define in1 8
#define in2 9

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder myEnc(5, 6);
//   avoid using pins with LEDs attached


int rotDirection = 0;
int input = 0;
bool cur_direction = true;


void setup() {
  Serial.begin(9600);
  Serial.println("Basic Encoder Test:");

  //mot
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  direction(true);
}

long oldPosition  = -999;

void loop() {
  long newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    //Serial.println(newPosition);
  }

  //mot

  if (Serial.available() > 0) { 
    input = Serial.parseInt();
    Serial.println(String(input));
  }
    
  rotate(input);
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

