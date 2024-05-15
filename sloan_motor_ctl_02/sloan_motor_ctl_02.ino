void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}



int sensor_value;
int control_value;
int diff;
int absdiff;
int speed;

int volts[2] = [HIGH, LOW];


void loop() {
  
  sensor_value = analogRead(A0);
  control_value = analogRead(A1);
  diff = control_value - sensor_value;
  absdiff = abs(diff);
  speed = int(absdiff/204);

  

  if(diff < 0) {
    digitalWrite(3, HIGH);
    digitalWrite(4, LOW);
  } else if(diff > 0) {
    digitalWrite(3, LOW);
    digitalWrite(4, HIGH);
  } 
  if(absdiff < 20){
    digitalWrite(3, LOW);
    digitalWrite(4, LOW);
  }  
}
