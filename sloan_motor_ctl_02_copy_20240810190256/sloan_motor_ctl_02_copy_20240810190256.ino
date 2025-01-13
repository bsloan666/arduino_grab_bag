



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

int sensor_value;
int control_value;
int diff;
int absdiff;
int speed;
int cycle_ticks = 64;
int speed_factor = int(1024/cycle_ticks);

// This value will increment every iteration 
// and reset to 0 when it reaches cycle_ticks 


void loop() {

  // The potentiometer attached to the rotor
  sensor_value = analogRead(A0);

  // The potentiomenter acontrolled by user
  control_value = analogRead(A1);

  diff = control_value - sensor_value;  
  speed = constrain(abs(diff), 0, 255);
  if(diff < -1) {
    // This call sends a PWM signal
    analogWrite(3, speed);
    digitalWrite(4, LOW);
  } else if(diff > 1) {
    digitalWrite(3, LOW);
    analogWrite(4, speed);
  } else {
    digitalWrite(3, LOW);
    digitalWrite(4, LOW);
  }
  //// high attack of the pulse
  //  // set the direction of rotation     
  //if(diff < 0) {
  //  digitalWrite(3, HIGH);
  //  digitalWrite(4, LOW);
  //} else if(diff > 0) {
  //  digitalWrite(3, LOW);
  //  digitalWrite(4, HIGH);
  //// low decay of the pulse  
  //} else {
  //  digitalWrite(3, LOW);
  //  digitalWrite(4, LOW);
  //} 
}
