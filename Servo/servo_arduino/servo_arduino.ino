#include <Servo.h>
#include <SoftwareSerial.h>
#define enb 2
#define servo_pin 9
#define reset_to_mid 255
#define offset 100

// *** General ***
const bool DEBUG_MODE = true;
bool new_input = false;
int motor_delay = 2; // TODO: optimize

// *** Servo ***
Servo myservo;
const int min_deg = 20, mid_deg = 50, max_deg = 80;
int current = mid_deg;
const int SER_WAIT_TIME = 1;

void setup() {
  Serial.begin(9600);
  myservo.attach(servo_pin);
  myservo.write(current);
}

void loop() {
  if(Serial.available() > 0){
    Serial.println("Got data");
    delay(SER_WAIT_TIME);
    while(Serial.available() > 1){
      Serial.read();
    }   //  WAIT FOR LAST INPUT*/
    
    int input = (int)Serial.read();
    if (input == reset_to_mid){
      current = mid_deg;
      if (DEBUG_MODE)
        Serial.println("Starting/Stopping.");
    }else{
      current -= input - offset;
      current = max(current, min_deg);
      current = min(current, max_deg);
      if (DEBUG_MODE){
        Serial.print("New input: ");Serial.println(input);
        Serial.print("New position: ");Serial.println(current);
      }
    }
    myservo.write(current);
    delay(motor_delay);
  }
}
