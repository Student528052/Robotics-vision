#include <Servo.h> 
 
#define servo 10 //pin location servo motor

Servo myservo; //create servo object named myservo

int pos = 0; //store servo postion

void setup() {
  myservo.attach(10);                    //attach pin location to myservo object
}

void loop() { //Main loop
  for (pos = 0; pos <= 180; pos++ ){     // from 0 to 180 degrees
    myservo.write(pos);                    // writes pos to the servo 
    delay(15);}                            // wait 15ms

  for (pos = 180; pos>= 0; pos--){       // from 180 to 0 degrees
    myservo.write(pos);                  // write pos to servo
    delay(15);                           // wait 15ms
  }
}
