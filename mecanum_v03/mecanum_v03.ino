#include <Encoder.h>
#include <PID_v1.h>


class MechanumController {
  private:
    int motor_pin_base;
    int encoder_pin_base;
    double encoder_value;
    double previous_encoder_value;
    double target_position;
    double slave_target_direction;
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
                  unsigned int _slave_a_pin, unsigned int _slave_b_pin,
                  unsigned int _encoder_a_pin, unsigned int _encoder_b_pin):
      position_pid(&encoder_value, &output_position_delta, &target_position, 0.65, 0.0, 0.05, DIRECT),
      velocity_pid(&velocity, &output_velocity_delta, &target_velocity, 4, 0.0, 0.0, DIRECT),
      encoder(_encoder_a_pin, _encoder_b_pin)
    {
      motor_a_pin = _motor_a_pin;
      motor_b_pin = _motor_b_pin;
      slave_a_pin = _slave_a_pin;
      slave_b_pin = _slave_b_pin;
      encoder_a_pin = _encoder_a_pin;
      encoder_b_pin = _encoder_b_pin;
      slave_position = 0;
    }
    int get_pos(){
      return encoder_value;
    }
    bool is_moving(){
      return previous_encoder_value != encoder_value;
    }
    
    void init_pids(int _interval)
    {
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

      pinMode(slave_a_pin, OUTPUT);
      pinMode(slave_b_pin, OUTPUT);
      digitalWrite(slave_a_pin, LOW);
      digitalWrite(slave_b_pin, LOW);
    }
    void init_targets(double init_targ_pos, double slave_targ_dir, double init_targ_vel)
    {
       target_position = init_targ_pos;
       slave_target_direction = slave_targ_dir;
       target_velocity = init_targ_vel;
    }
    void dump()
    {
      Serial.print("'left_pos':");
      Serial.print(encoder_value);
      Serial.print(",");
      Serial.print("'right_pos':");
      Serial.print(slave_position);
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
      output_position_magnitude = constrain(output_position_magnitude, 0, abs(output_velocity_delta));
      if(output_position_delta > 0){
          digitalWrite(motor_a_pin, LOW);
          analogWrite(motor_b_pin, output_position_magnitude);
      } else if(output_position_delta < 0){ 
          analogWrite(motor_a_pin, output_position_magnitude);
          digitalWrite(motor_b_pin, LOW);
      }
      if(slave_target_direction > 0){
          slave_position += velocity;
          digitalWrite(slave_a_pin, LOW);
          analogWrite(slave_b_pin, output_position_magnitude);
      } else if(slave_target_direction < 0){ 
          slave_position += velocity * -1;
          analogWrite(slave_a_pin, output_position_magnitude);
          digitalWrite(slave_b_pin, LOW);
      }
      if(previous_encoder_value != encoder_value){
        encoder_changed = true;
      }
      previous_encoder_value = encoder_value;
    }
};


MechanumServo front(5, 4, 8, 9, 18, 2);
MechanumServo rear(6, 7, 10, 11, 20, 21);

const int SET = 10;
const int GET = 20;

int interval = 20;
int timing = 10;
int speed = 255;
long position;
int prev_code;
int test;
unsigned long previousMillis;
unsigned long longMillis;

void setup() {
  Serial.begin(9600);
  delay(100);
  front.init_pids(interval); 
  rear.init_pids(interval); 
  
  front.init_targets(0.01, 0.01, 16);
  rear.init_targets(0.01, 0.01, 16);
 
  delay(100);
}
int cmd;
double lf_pos;
double f_spd;
double rf_dir;
double lr_pos;
double r_spd;
double rr_dir;

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    
    front.cycle(); 
    rear.cycle();
    //Serial.println();
    if (!(front.is_moving()) && !(rear.is_moving())) {
      if(Serial.available()){
        cmd = Serial.parseInt();
        if(cmd == SET){
          lf_pos = Serial.parseFloat();
          f_spd = Serial.parseFloat();
          rf_dir = Serial.parseFloat();
          lr_pos = Serial.parseFloat();
          r_spd = Serial.parseFloat();
          rr_dir = Serial.parseFloat();
          front.init_targets(lf_pos, rf_dir, f_spd);
          rear.init_targets(lr_pos, rr_dir, r_spd);
          Serial.print("'front': { ");
          front.dump();
          Serial.print("}, 'rear': { ");
          rear.dump();
          Serial.println("}");
        } else if(cmd == GET){
          Serial.print("'front': { ");
          front.dump();
          Serial.print("}, 'rear': { ");
          rear.dump();
          Serial.println("}");
        }
      }
    }
  }
}
