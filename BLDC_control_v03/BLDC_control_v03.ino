

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

HBridge bridge_k(6, 9);
HBridge bridge_r(3, 5);
HBridge bridge_b(10, 11);

unsigned long previousMillis = 0;
unsigned long currentMillis;
int req_speed;
int act_speed = 0;
int req_dir;
int act_dir;
long position;
int prev_code;
int code;
int index;
int vindex = 3;

int k_vals[12] = {-1,  0,  1,  1,  0, -1, -1,  0,  1,  1,  0, -1};
int r_vals[12] = { 0, -1, -1,  0,  1,  1,  0, -1, -1,  0,  1,  1};
int b_vals[12] = { 1,  1,  0, -1, -1,  0,  1,  1,  0, -1, -1,  0};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  bridge_k.init();
  bridge_r.init();
  bridge_b.init();
  delay(5);

}

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

  code = int(sensor_a > 5) * 1 + int(sensor_b > 5) * 2 + int(sensor_c > 5) * 4;
  //Serial.println(code);
  
 //Serial.println(code);
  int pot = analogRead(A4);
  req_dir = pot - 128;
  int abs_dir = abs(req_dir);

  if(0) {
    Serial.print(pot);
    Serial.print(" ");
    Serial.print(req_speed);
    Serial.print(" ");
    Serial.println(req_dir);
  }
  if(req_dir < -5){
    req_speed = map(abs_dir, 5, 128, 0, 300);
    vindex = 3;
    if(code == 1) {
      bridge_k.set(req_speed * -k_vals[vindex + 5]);
      bridge_r.set(req_speed * -r_vals[vindex + 5]);
      bridge_b.set(req_speed * -b_vals[vindex + 5]);
    } else if(code == 3 ) {
      bridge_k.set(req_speed * -k_vals[vindex + 4]);
      bridge_r.set(req_speed * -r_vals[vindex + 4]);
      bridge_b.set(req_speed * -b_vals[vindex + 4]);
    } else if(code == 2 ) {
      bridge_k.set(req_speed * -k_vals[vindex + 3]);
      bridge_r.set(req_speed * -r_vals[vindex + 3]);
      bridge_b.set(req_speed * -b_vals[vindex + 3]);
    } else if(code == 6){
      bridge_k.set(req_speed * -k_vals[vindex + 2]);
      bridge_r.set(req_speed * -r_vals[vindex + 2]);
      bridge_b.set(req_speed * -b_vals[vindex + 2]);
    } else if(code == 4){
      bridge_k.set(req_speed * -k_vals[vindex + 1]);
      bridge_r.set(req_speed * -r_vals[vindex + 1]);
      bridge_b.set(req_speed * -b_vals[vindex + 1]);
    } else if(code == 5){
      bridge_k.set(req_speed * -k_vals[vindex + 0]);
      bridge_r.set(req_speed * -r_vals[vindex + 0]);
      bridge_b.set(req_speed * -b_vals[vindex + 0]);
    }

  } else if (req_dir > 5){
    req_speed = map(abs_dir, 5, 1023, 0, 300);
    vindex = 3;
    if(code == 1) {
      bridge_k.set(req_speed * k_vals[vindex + 5]);
      bridge_r.set(req_speed * r_vals[vindex + 5]);
      bridge_b.set(req_speed * b_vals[vindex + 5]);
    } else if(code == 3 ) {
      bridge_k.set(req_speed * k_vals[vindex + 4]);
      bridge_r.set(req_speed * r_vals[vindex + 4]);
      bridge_b.set(req_speed * b_vals[vindex + 4]);  
    } else if(code == 2 ) {
      bridge_k.set(req_speed * k_vals[vindex + 3]);
      bridge_r.set(req_speed * r_vals[vindex + 3]);
      bridge_b.set(req_speed * b_vals[vindex + 3]);
    } else if(code == 6){
      bridge_k.set(req_speed * k_vals[vindex + 2]);
      bridge_r.set(req_speed * r_vals[vindex + 2]);
      bridge_b.set(req_speed * b_vals[vindex + 2]);
    } else if(code == 4){
      bridge_k.set(req_speed * k_vals[vindex + 1]);
      bridge_r.set(req_speed * r_vals[vindex + 1]);
      bridge_b.set(req_speed * b_vals[vindex + 1]);
    } else if(code == 5){
      bridge_k.set(req_speed * k_vals[vindex + 0]);
      bridge_r.set(req_speed * r_vals[vindex + 0]);
      bridge_b.set(req_speed * b_vals[vindex + 0]);
    }
  } 

  if(0){
    Serial.print("code:");
    Serial.println(code);
    Serial.print(", req_speed:");
    Serial.print(req_speed);
    Serial.print(", position:");
    Serial.print(position);
    Serial.print(", act_speed:");
    Serial.print(act_speed);
    Serial.print(" RPM, req_dir:");
    Serial.println(req_dir);
  }
  index++;
}
