#include <PID_v1.h>

class EncodedJointActuator
{
  private:
    int sensor_pin;
    int actuator_a_pin;
    int actuator_b_pin;
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

  public:
    EncodedJointActuator(int s_pin, int pin_a, int pin_b,
                        double pkp, double ikp, double dkp,
                        double pkv, double ikv, double dkv):
      position_pid(&encoder_value, &output_position_delta, &target_position, pkp,ikp, dkp, DIRECT),
      velocity_pid(&velocity, &output_velocity_delta, &target_velocity, pkv, ikv, dkv, DIRECT) 
    {
      sensor_pin = s_pin;
      actuator_a_pin = pin_a;
      actuator_b_pin = pin_b;
    }
    int get_pos(){
      return encoder_value;
    }
    bool is_moving(){
      return encoder_changed;
    }
    
    void init_pids()
    {
      position_pid.SetMode(AUTOMATIC);            
      position_pid.SetOutputLimits(-255, 255);
      position_pid.SetSampleTime(20);

      velocity_pid.SetMode(AUTOMATIC);              
      velocity_pid.SetOutputLimits(-255, 255);
      velocity_pid.SetSampleTime(20);

      pinMode(actuator_a_pin, OUTPUT);
      pinMode(actuator_b_pin, OUTPUT);
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
      digitalWrite(actuator_a_pin, LOW);
      digitalWrite(actuator_b_pin, LOW);
    }

    void cycle()
    {
      encoder_changed = false;
      encoder_value = analogRead(sensor_pin);
      velocity = abs(encoder_value - previous_encoder_value);
      position_pid.Compute();
      velocity_pid.Compute();
      output_position_magnitude = abs(output_position_delta);
      output_position_magnitude = constrain(output_position_magnitude,0,abs(output_velocity_delta));
      // use output_position_magnitude as PWM control value
      if(output_position_delta > 0){
          digitalWrite(actuator_a_pin, LOW);
          digitalWrite(actuator_b_pin, HIGH);
      } else if(output_position_delta < 0){ 
          digitalWrite(actuator_a_pin, HIGH);
          digitalWrite(actuator_b_pin, LOW);
      } else {
          digitalWrite(actuator_a_pin, LOW);
          digitalWrite(actuator_b_pin, LOW);
      }
      if(previous_encoder_value != encoder_value){
        encoder_changed = true;
      }
      previous_encoder_value = encoder_value;
    }
};


EncodedJointActuator shoulder_pitch(
  A0, 2, 3, 
  2.80, 0.5, 0.00, 
  1.0, 0.0, 0.0);



int test;
const long interval  = 20;
unsigned long previousMillis = 0;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  shoulder_pitch.init_targets(600, 240);
  shoulder_pitch.init_pids();
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    if(Serial.available()){
       shoulder_pitch.set_to_off();
       test = Serial.parseInt();
       if(test){
         shoulder_pitch.init_targets(test, 240);
       }
      
    }
    
    if(shoulder_pitch.is_moving()) {
      shoulder_pitch.dump();
      Serial.println(" ");
    }
    
    shoulder_pitch.cycle(); 
      
  }

}
