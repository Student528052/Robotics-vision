#define PWMA 5      //controls power to right motor
#define PWMB 6      //controls power to left motor
#define BIN_1 8     //controls direction of left motor (High = forward, low = backward)
#define AIN_1 7     //controls direction of right motor (High = forward, low = backward)
#define stby 3      // puts H-bridge in standby (low = standby, high = run)



void setup() {
  pinMode(PWMA, OUTPUT); //set IO pin to output
  pinMode(PWMB, OUTPUT);
  pinMode(AIN_1, OUTPUT); 
  pinMode(BIN_1, OUTPUT);
  pinMode(stby, OUTPUT);
  digitalWrite(stby, HIGH); //enables motors to drive
  digitalWrite(PWMA, LOW);
}

void loop() {
  //while (digitalRead(modeswitch) == 1) //wait for modeswitch to be pressed
  //;

  delay(2000);

  //test right wheels for 2 seconds
  digitalWrite(AIN_1, HIGH);
  digitalWrite(PWMA, HIGH);
  delay(2000);

  //stop for 2 seconds
  stoptime(2000);

  //drive backwards for 2 seconds
  digitalWrite(AIN_1, LOW);
  digitalWrite(PWMA, HIGH);
  delay(2000);

  stoptime(2000);
  digitalWrite(PWMA, LOW);

  //test left wheels for 2 seconds
  digitalWrite(BIN_1, HIGH);
  digitalWrite(PWMB, HIGH);
  delay(2000);

  //stop for 2 seconds
  stoptime(2000);

  //drive backwards for 2 seconds
  digitalWrite(BIN_1, LOW);
  digitalWrite(PWMB, HIGH);
  delay(2000);

  stoptime(2000);
  digitalWrite(PWMB, LOW);

}

void stoptime(int mS){
  digitalWrite(stby, LOW);
  delay(mS);
  digitalWrite(stby, HIGH);
}

