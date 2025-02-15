#include <Encoder.h>
#include <PID_v1.h>


class MechanumController {
  private:
    // default to 4
    unsigned int motor_pin_base;
    // default to 2
    unsigned int encoder_pin_base;
    int slave_polarity[3]; 
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
    MechanumController(unsigned int _motor_base, unsigned int _encoder_base):
      motor_pin_base(_motor_base), encoder_pin_base(_encoder_base),
      position_pid(&encoder_value, &output_position_delta, &target_position, 0.65, 0.0, 0.05, DIRECT),
      velocity_pid(&velocity, &output_velocity_delta, &target_velocity, 4, 0.0, 0.0, DIRECT),
      encoder(_encoder_base, _encoder_base + 1)
    {}
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

      for(unsigned int pin = motor_pin_base; pin < motor_pin_base + 8; pin++){
         pinMode(pin, OUTPUT);
      }
      set_to_off();
      
    }

    void init_targets(double init_targ_pos, double init_targ_vel, int rf, int lr, int rr)
    {
       target_position = init_targ_pos;
       target_velocity = init_targ_vel;
       slave_polarity[0] = rf;
       slave_polarity[1] = lr;
       slave_polarity[2] = rr;

    }

    void dump()
    {
      Serial.print("'position':");
      Serial.print(encoder_value);
      Serial.print("'velocity':");
      Serial.print(velocity);
      Serial.print("'rf':");
      Serial.print(slave_polarity[0]);    
      Serial.print("'lr':");
      Serial.print(slave_polarity[1]);    
      Serial.print("'rr':");
      Serial.print(slave_polarity[2]);      
    }

    void set_to_off(){
      for(unsigned int pin = motor_pin_base; pin < motor_pin_base + 8; pin++){
        digitalWrite(pin, LOW);
      }
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
          digitalWrite(motor_pin_base, LOW);
          analogWrite(motor_pin_base + 1, output_position_magnitude);
      } else if(output_position_delta < 0){ 
          analogWrite(motor_pin_base, output_position_magnitude);
          digitalWrite(motor_pin_base + 1, LOW);
      }
      for(unsigned int index = 0; index < 3; index++){
        unsigned int base = 2 + index * 2;
        if(slave_polarity[index] * output_position_delta > 0){
            digitalWrite(base, LOW);
            analogWrite(base + 1, output_position_magnitude);
        } else if(slave_polarity[index] * output_position_delta < 0){ 
            analogWrite(base, output_position_magnitude);
            digitalWrite(base + 1, LOW);
        }
      }
      if(previous_encoder_value != encoder_value){
        encoder_changed = true;
      }
      previous_encoder_value = encoder_value;
    }
};


MechanumController wheels(4, 2);

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
  wheels.init_pids(interval); 
  
  wheels.init_targets(0.0, 0.0, 1, 1, 1);
  delay(100);
}

int cmd;
double pos;
double spd;
int rf_dir;
int lr_dir;
int rr_dir;

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    
    wheels.cycle(); 
    if (!(wheels.is_moving())) {
      if(Serial.available()){
        cmd = Serial.parseInt();
        if(cmd == SET){
          pos = Serial.parseFloat();
          spd = Serial.parseFloat();
          rf_dir = Serial.parseFloat();
          lr_dir = Serial.parseFloat();
          rr_dir = Serial.parseFloat();
          wheels.init_targets(pos, spd, rf_dir, lr_dir, rr_dir);
          Serial.print("'wheels': { ");
          wheels.dump();
          Serial.println("}");
        } else if(cmd == GET){
          Serial.print("'wheels': { ");
          wheels.dump();
          Serial.println("}");
        }
      }
    }
  }
}
