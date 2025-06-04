#define trig_pin 13
#define echo_pin 12


float duration, distance;

void setup() {
  pinMode(trig_pin, OUTPUT) ;
  pinMode(echo_pin, INPUT);
  Serial.begin(9600);
}

void loop() {

  //clear triggerpin
  digitalWrite(trig_pin, LOW);
  delayMicroseconds(2);

  // send a pulse from the trigger pin
  digitalWrite(trig_pin,  HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_pin, LOW);

  duration = pulseIn(echo_pin, HIGH);

  // calculate distance 
  distance = duration * ( 0.0343 / 2);

  
  Serial.print("distance to objecxt: ");
  Serial.println(distance);
  Serial.println(" cm");
  

  delay(100); // wait 100 ms
  
}
