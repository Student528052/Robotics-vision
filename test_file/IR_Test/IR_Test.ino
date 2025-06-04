#define IR_R A2
#define IR_M A1
#define IR_L A0
#define Baud_rate 9600

void setup() {
  Serial.begin(Baud_rate);
  pinMode(IR_R, INPUT);
  pinMode(IR_M, INPUT);
  pinMode(IR_L, INPUT);
  
}

void loop() {
  Serial.print("IR_R: ");
  Serial.print(analogRead(IR_R));
  Serial.print("  ");

  Serial.print("IR_M: ");
  Serial.print(analogRead(IR_M));
  Serial.print("  ");

  Serial.print("IR_L: ");
  Serial.print(analogRead(IR_L));
  Serial.print("  ");

  Serial.println();
  delay(200);

  //White:  50-100
  //black:  600-700
  //red:    150; 50; 50
  //blue:   500; 400; 460
  //green:  400; 240; 300
}
