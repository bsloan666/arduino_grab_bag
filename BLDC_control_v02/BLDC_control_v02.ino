const unsigned int UA = 3;
const unsigned int UB = 5;
const unsigned int VA = 6;



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(UA, OUTPUT);
  pinMode(UB, OUTPUT);

  delay(5);

  digitalWrite(UA, LOW);
  digitalWrite(UB, LOW);
 
  delay(5);

}
int timing = 10;
int speed = 64;
int current_high = UA;


void loop() {
  int sensor_a = analogRead(A0);
  int sensor_b = analogRead(A1);
  int sensor_c = analogRead(A2);

if(0) {
  Serial.print(sensor_a);
  Serial.print("   ");
   Serial.print(sensor_b);
  Serial.print("   ");
   Serial.print(sensor_c);
  Serial.println();
  //delay(50);
}
int code = int(sensor_a > 5) * 4 + int(sensor_b > 5) * 2 + int(sensor_c > 5);

  int pot = analogRead(A4);
  int direction = pot - 512;
  speed = abs(direction/2);
if(0) {
  Serial.print(pot);
  Serial.print(" ");
  Serial.print(speed);
  Serial.print(" ");
  Serial.println(direction);
}
  if(direction > 0){
    constrain(speed, 64, 255);
    if(code == 4 ) {
      //Serial.println(code);
      digitalWrite(current_high, LOW);
      digitalWrite(UA, LOW);
      analogWrite(UB, speed);
      current_high = UB;
    } else if(code == 2){
      //Serial.println(code);
      digitalWrite(current_high, LOW);
      analogWrite(UA, speed);
      digitalWrite(UB, LOW);
      current_high = UA;
    }  
  } else {
    if( code == 1) {
       //Serial.println(code);
        digitalWrite(current_high, LOW);
        digitalWrite(UB, LOW);
      analogWrite(UA, speed);
      current_high = UA;
    } else if(code == 2){
      //Serial.println(code);
      digitalWrite(current_high, LOW);
      analogWrite(UB, speed);
      digitalWrite(UA, LOW);
      current_high = UB;
    } 
  }
}
