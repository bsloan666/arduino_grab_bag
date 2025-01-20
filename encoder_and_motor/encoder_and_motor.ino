

#include <Encoder.h>
#include <PID_v1.h>

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder myEnc(20, 21);
//   avoid using pins with LEDs attached

int leftPin = 8;
int rightPin = 9;
int tempVar = 0;

unsigned long previousMillis = 0;
const long interval = 10;

double Pk1 = 0.65; 
double Ik1 = 0.0;
double Dk1 = Pk1 * 1;
double Setpoint1, Input1, Output1, Output1a;    // PID variables position

double Pk2 = 6;  
double Ik2 = 0;
double Dk2 = 0;
double Setpoint2, Input2, Output2, Output2a;    // PID variables velocity

long encValue;
long lastEncValue;
unsigned long diff = 0; 
unsigned long speed = 0;


PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1 , Dk1, DIRECT);
PID PID2(&Input2, &Output2, &Setpoint2, Pk2, Ik2 , Dk2, DIRECT);

void setup() {
  Serial.begin(9600);

  int result2  = digitalPinToInterrupt(2);
  int result3  = digitalPinToInterrupt(3);

  PID1.SetMode(AUTOMATIC);              
  PID1.SetOutputLimits(-255, 255);
  PID1.SetSampleTime(10);

  PID2.SetMode(AUTOMATIC);              
  PID2.SetOutputLimits(-255, 255);
  PID2.SetSampleTime(10);

  pinMode(leftPin, OUTPUT);
  pinMode(rightPin, OUTPUT);
  Setpoint1 = 0;
  Setpoint2 = 64;
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    encValue = myEnc.read();
    diff = abs(encValue - lastEncValue);
   
    Input1 = encValue;
    PID1.Compute();

    Input2 = diff;
    PID2.Compute();

    if(encValue != lastEncValue) {
    // if(0){      
      Serial.print("Encoder: ");
      Serial.print(encValue);
      Serial.print(" Power: ");
      Serial.print(Output1);
      Serial.print(" Speed: ");
      Serial.print(Output2);
      Serial.print(" Setpoint: ");
      Serial.print(Setpoint1);
      Serial.println("");
    }
    lastEncValue = encValue;  
     
    if(Output1 > 0){
      Output1a = abs(Output1);
      Output1a = constrain(Output1a,0,abs(Output2));
      analogWrite(leftPin, Output1a);
      digitalWrite(rightPin, LOW);
    } else if( Output1 < 0){ 
      Output1a = abs(Output1);
      Output1a = constrain(Output1a, 0, abs(Output2));
      digitalWrite(leftPin, LOW);
      analogWrite(rightPin, Output1a);
    } 
    if (Output1 == 0){
      digitalWrite(leftPin, LOW);
      digitalWrite(rightPin, LOW);
    }
   if(Serial.available()){
      int test = Serial.parseInt();

      if(test) {
        Setpoint1 = test;
      }
      Serial.print("Setting Point: ");  
      Serial.println(Setpoint1);         
      Setpoint2 = 128;
    }
  }
}
