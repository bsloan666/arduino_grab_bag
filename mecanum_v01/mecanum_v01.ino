#include <Encoder.h>
#include <PID_v1.h>


class MechanumServo {
  private:
    unsigned int motor_a_pin;
    unsigned int motor_b_pin;
    unsigned int encoder_a_pin;
    unsigned int encoder_b_pin;
    double encoder_value;
    double previous_encoder_value;
    double target_position;
    double velocity;
    double target_velocity;
    double output_position_delta, output_position_magnitude;  
    double output_velocity_delta;
    bool encoder_changed; 
    PID position_pid;
    PID velocity_pid;
    Encoder encoder;
    
  public:
    MechanumServo(unsigned int _motor_a_pin, unsigned int _motor_b_pin,
                  unsigned int _encoder_a_pin, unsigned int _encoder_b_pin):
      position_pid(&encoder_value, &output_position_delta, &target_position, 0.65, 0.0, 0.05, DIRECT),
      velocity_pid(&velocity, &output_velocity_delta, &target_velocity, 4, 0.0, 0.0, DIRECT),
      encoder(_encoder_a_pin, _encoder_b_pin)
    {
      motor_a_pin = _motor_a_pin;
      motor_b_pin = _motor_b_pin;
      encoder_a_pin = _encoder_a_pin;
      encoder_b_pin = _encoder_b_pin;
    }
    int get_pos(){
      return encoder_value;
    }
    bool is_moving(){
      return encoder_changed;
    }
    
    void init_pids(int _interval)
    {
      int result2  = digitalPinToInterrupt(encoder_a_pin);
      int result3  = digitalPinToInterrupt(encoder_b_pin);
      position_pid.SetMode(AUTOMATIC);            
      position_pid.SetOutputLimits(-255, 255);
      position_pid.SetSampleTime(_interval);

      velocity_pid.SetMode(AUTOMATIC);              
      velocity_pid.SetOutputLimits(-255, 255);
      velocity_pid.SetSampleTime(_interval);

      pinMode(motor_a_pin, OUTPUT);
      pinMode(motor_b_pin, OUTPUT);
      digitalWrite(motor_a_pin, LOW);
      digitalWrite(motor_b_pin, LOW);
    }
    void init_targets(double init_targ_pos, double init_targ_vel)
    {
       target_position = init_targ_pos;
       target_velocity = init_targ_vel;
    }
    void dump()
    {
      Serial.print("Encoder:");
      Serial.print(encoder_value);
      Serial.print(",");
      Serial.print("Target:");
      Serial.print(target_position);
      Serial.print(",");
      Serial.print("Delta:");
      Serial.print(output_position_delta);
    }
    void set_to_off(){
      digitalWrite(motor_a_pin, LOW);
      digitalWrite(motor_b_pin, LOW);
    }

    void cycle()
    {
      encoder_changed = false;
      encoder_value = encoder.read();
      velocity = abs(encoder_value - previous_encoder_value);
      position_pid.Compute();
      velocity_pid.Compute();
      output_position_magnitude = abs(output_position_delta);
      output_position_magnitude = constrain(output_position_magnitude,0,abs(output_velocity_delta));
      if(output_position_delta > 0){
          digitalWrite(motor_a_pin, LOW);
          analogWrite(motor_b_pin, output_position_magnitude);
      } else if(output_position_delta < 0){ 
          analogWrite(motor_a_pin, output_position_magnitude);
          digitalWrite(motor_b_pin, LOW);
      } else {
          digitalWrite(motor_a_pin, LOW);
          digitalWrite(motor_b_pin, LOW);
      }
      if(previous_encoder_value != encoder_value){
        encoder_changed = true;
      }
      previous_encoder_value = encoder_value;
    }
};

MechanumServo left_front(8, 9, 2, 3);
MechanumServo right_front(4, 5, 18, 19);
MechanumServo left_rear(10, 11, 21, 24);
MechanumServo right_rear(6, 7, 20, 22);

int interval = 20;
int timing = 10;
int speed = 255;
long position;
int prev_code;
int test;
unsigned long previousMillis;

void setup() {
  Serial.begin(9600);

  left_front.init_targets(360, 64);
  left_front.init_pids(interval);
  right_front.init_targets(360, 64);
  right_front.init_pids(interval);
  left_rear.init_targets(360, 64);
  left_rear.init_pids(interval);
  right_rear.init_targets(360, 64);
  right_rear.init_pids(interval);

  delay(100);
}


void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    
    if(Serial.available()){
       left_front.set_to_off();
       right_front.set_to_off();
       left_rear.set_to_off();
       right_rear.set_to_off();
       test = Serial.parseInt();
       if(test){
         left_front.init_targets(test, 240);
         right_front.init_targets(test, 240);
         left_rear.init_targets(test, 240);
         right_rear.init_targets(test, 240);
       }
      
    }
    /*
    if(left_front.is_moving()) {
      left_front.dump();
      Serial.println(" ");
    }
    */
    left_front.cycle(); 
    right_front.cycle(); 
    left_rear.cycle(); 
    right_rear.cycle(); 
  }
}
