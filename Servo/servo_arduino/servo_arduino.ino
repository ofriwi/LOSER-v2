#include <Servo.h>
#include <SoftwareSerial.h>
#define enb 2
#define servo_pin 9
#define reset_to_mid 255
#define lost 254
#define offset 100


// *** General ***
const bool DEBUG_MODE = true;
bool new_input = false;
int motor_delay = 2; // TODO: optimize

// *** Servo ***
Servo myservo;
int min_deg = 30, mid_deg = 50, max_deg = 90;
int current = mid_deg;
const int SER_WAIT_TIME = 1;

void setup() {
  Serial.begin(9600);
  myservo.attach(servo_pin);
  myservo.write(current);
  pinMode(4, OUTPUT);
}

void loop() {
  if(Serial.available() > 0){
    delay(SER_WAIT_TIME);
    while(Serial.available() > 1){
      Serial.read();
    }   //  WAIT FOR LAST INPUT*/
    int input = (int)Serial.parseInt() - offset;
    if (input == reset_to_mid - offset){
      input = mid_deg;
      current = 0;
      if (DEBUG_MODE)
        Serial.println("Starting/Stopping.");
    }else if (input == lost - offset){
      digitalWrite(4, HIGH);
    }
    digitalWrite(4, LOW);
    current += input;
    current = max(current, min_deg);
    current = min(current, max_deg);
    if (DEBUG_MODE){
      Serial.print("New input: ");Serial.println(input);
      Serial.print("New position: ");Serial.println(current);
    }
    myservo.write(current);
    delay(motor_delay);
  }
}
