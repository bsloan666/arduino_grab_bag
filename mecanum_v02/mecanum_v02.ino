#include <Encoder.h>
#include <PID_v1.h>


class MechanumServo {
  private:
    int motor_a_pin;
    int motor_b_pin;
    int slave_a_pin;
    int slave_b_pin;
    int encoder_a_pin;
    int encoder_b_pin;
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
    }
    int get_pos(){
      return encoder_value;
    }
    bool is_moving(){
      return encoder_changed;
    }
    
    void init_pids(int _interval)
    {
      if(0){
        int result_a  = digitalPinToInterrupt(encoder_a_pin);
        int result_b  = digitalPinToInterrupt(encoder_b_pin);

        if(result_a != encoder_a_pin){
            Serial.print(encoder_a_pin);
            Serial.print(" cannot be used for interrupt! (");
            Serial.print(result_a);
            Serial.println(")");
        }
        if(result_b != encoder_b_pin){
            Serial.print(encoder_b_pin);
            Serial.print(" cannot be used for interrupt! (");
            Serial.print(result_b);
            Serial.println(")");
        }
      }
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
      }
      if(slave_target_direction > 0){
          digitalWrite(slave_a_pin, LOW);
          analogWrite(slave_b_pin, output_position_magnitude);
      } else if(slave_target_direction < 0){ 
          analogWrite(slave_a_pin, output_position_magnitude);
          digitalWrite(slave_b_pin, LOW);
      }
      if(previous_encoder_value != encoder_value){
        encoder_changed = true;
      }
      previous_encoder_value = encoder_value;
    }
};
class HBridge {
  public:
    HBridge(unsigned int _pin_a, unsigned int _pin_b){
      pin_a = _pin_a;
      pin_b = pin_b;
    }

    void init(){
      pinMode(pin_a, OUTPUT);
      pinMode(pin_b, OUTPUT);
      digitalWrite(pin_a, LOW);
      digitalWrite(pin_b, LOW);
    }  

    void set(int velocity) {
        if(velocity > 0){
          analogWrite(pin_a, constrain(abs(velocity), 0, 255));
          digitalWrite(pin_b, LOW);
        } else if(velocity < 0){
          digitalWrite(pin_a, LOW);
          analogWrite(pin_b, constrain(abs(velocity), 0, 255));
        } else{
          digitalWrite(pin_a, LOW);
          digitalWrite(pin_b, LOW);
        }
    }
  private:
    unsigned int pin_a;
    unsigned int pin_b;
};


MechanumServo front(5, 4, 8, 9, 18, 2);
MechanumServo rear(6, 7, 10, 11, 20, 21);


double lf[4] = { 1, -1,  -3,  -1}; 
double lr[4] = {-1,  -3,  -1,  1}; 
double rf[4] = { 1, -1,  1,  1}; 
double rr[4] = {-1, -1, -1,  1}; 
int index = 0;
int travel_distance = 300;
int travel_time = 3;

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
  

  front.init_targets(0.01, 0.01, 64);
  rear.init_targets(0.01, 0.01, 64);
 
  delay(100);
}



void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    
    front.cycle(); 
    rear.cycle();

    if (currentMillis - longMillis >= (travel_time * 1000)) {
      if(1){
        
      }
      front.init_targets(lf[index] * travel_distance, rf[index], 64);
      rear.init_targets(lr[index] * travel_distance, rr[index], 64);
      longMillis = currentMillis;
      index++;
      if(index >= 4){
        index = 0;
      }
    }
  }
}
