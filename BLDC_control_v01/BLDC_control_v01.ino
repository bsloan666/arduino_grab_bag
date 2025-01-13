const unsigned int UA = 3;
const unsigned int UB = 5;
const unsigned int VA = 6;
const unsigned int VB = 9;
const unsigned int WA = 10;
const unsigned int WB = 11;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(UA, OUTPUT);
  pinMode(UB, OUTPUT);
  pinMode(VA, OUTPUT);
  pinMode(VB, OUTPUT);
  pinMode(WA, OUTPUT);
  pinMode(WB, OUTPUT);
  delay(5);

  digitalWrite(UA, LOW);
  digitalWrite(UB, LOW);
  digitalWrite(VA, LOW);
  digitalWrite(VB, LOW);
  digitalWrite(WA, LOW);
  digitalWrite(WB, LOW); 
  delay(5);

}
int timing = 10;
int speed = 64;
int current_high = UA;


void loop() {
  int sensor_a = analogRead(A0);
  int sensor_b = analogRead(A1);
  int sensor_c = analogRead(A2);

if(1) {
  Serial.print(sensor_a);
  Serial.print("   ");
   Serial.print(sensor_b);
  Serial.print("   ");
   Serial.print(sensor_c);
  Serial.println();
  //delay(50);
}
int code = int(sensor_a > 5) * 4 + int(sensor_b > 5) * 2 + int(sensor_c > 5);

// delay(500);

  if(code == 6) {
    Serial.println(code);
    digitalWrite(current_high, LOW);
    digitalWrite(UA, LOW);
    analogWrite(UB, speed);
    current_high = UB;
  } else if(code == 4){
    Serial.println(code);
    digitalWrite(current_high, LOW);
    analogWrite(UA, speed);
    digitalWrite(UB, LOW);
    current_high = UA;
  } 
/*  
  else
  
   if(code == 5){
    Serial.println(code);
    digitalWrite(current_high, LOW);
    digitalWrite(VA, LOW);
    analogWrite(VB, speed);
    current_high = VB;
  } else if(code == 1) {
    Serial.println(code);
    digitalWrite(current_high, LOW);
    analogWrite(VA, speed);
    digitalWrite(VB, LOW);
    current_high = VA;
  } 
  else 
  if(code == 3){
    Serial.println(code);
    digitalWrite(current_high, LOW);
    digitalWrite(WA, LOW);
    analogWrite(WB, speed);
    current_high = WB;
  } else if(code == 2) {
    Serial.println(code);
    digitalWrite(current_high, LOW);
    analogWrite(WA, speed);
    digitalWrite(WB, LOW);
    current_high = WA;
  }
*/
}
