
#include <Servo.h>

Servo myservo;  // create servo object to control a servo

int val;    // variable to read the value from the analog pin



void setup() {
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  Serial.begin(9600);

      }

void loop() {
  int sensorValue = analogRead(A0);
  int val = map(sensorValue, 0, 1023, 1200, 1800);     // scale it for use with the servo (value between 0 and 180)
  myservo.write(val);                  // sets the servo position according to the scaled value
  Serial.println(val);
  delay(15); 
}
