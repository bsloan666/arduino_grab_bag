
#include <Stepper.h>
#include <SPI.h>
#include <SD.h>

const int chipSelect = 53;

const int stepsPerRevolution = 64;
const int maxSpeed = 512;

const int numSteppers = 6;  // change to 7 when head and tail are added

// Indices into the arrays that contain the target and current positions and directions
// for each motor
const int LINE = 0;
const int SPIN = 1;
const int TILT = 2;
const int ROLL = 3;
const int PRES = 4;
const int HEAD = 5;
const int TAIL = 6;
const int increments[numSteppers] = { 175, 85, 100, 333, 1024, -40 };  
//const int dn_tilts[4] = {-185, -98, 3, 108};
//const int up_tilts[4] = {-200, -95, -1, 100};
// the various tasks in which we might currently be engaged.

const int SET = 0;
const int STRIKE = 1;
const int RETURN = 2;
const int FEED = 3;
const int REST = 4;
const int REGISTER = 5;


char text[1280];


int text_pos = 0; 

Stepper steppers[] = {
  {stepsPerRevolution, 22, 24, 23, 25}, // LINE
  {stepsPerRevolution, 26, 28, 27, 29}, // SPIN
  {stepsPerRevolution, 30, 32, 31, 33},   // TILT
  {stepsPerRevolution, 34, 36, 35, 37},     // ROLL
  {stepsPerRevolution, 38, 40, 39, 41}, // PRES
  {stepsPerRevolution, 42, 44, 43, 45}, // HEAD
//  {stepsPerRevolution, 42, 44, 43, 45}  // TAIL
};

char raster[] = "fypqj.1-,|GLFYPQJ.!_,|gl"
                "anrodwmv  HXANRODWMV\n hx"
                "bcetsi';+ UKBCETSI\":= uk"
                "460257 z?#(*$ )@%& Z/398";

unsigned int sectors[128]; 
unsigned int tropics[128];


int targets[numSteppers] = { 
   /*increments[LINE]*/ 0, 
   /*increments[SPIN]*/ 0, 
   /*increments[TILT]*/ 0, 
   /*increments[ROLL]*/ 0, 
   /*increments[PRES]*/ 0,
   /*increments[HEAD]*/ 0
};

int positions[numSteppers] = {0, 0, 0, 0, 0, 0};
int directions[numSteppers];
int phase = SET;

int dir_from_diff(int pos, int tgt) {
  int res = tgt - pos;
  if(res < 0){
    return -1;
  }
  return 1;
}

int end_of_next_word(){
  int curr_pos = text_pos + 1;
  int x_pos = positions[LINE] + increments[LINE];
  while(text[curr_pos] != ' ' && text[curr_pos] != '\n' && text[curr_pos] != '\t' && curr_pos < sizeof(text)){
    x_pos += increments[LINE];
    curr_pos += 1;
  }
  return x_pos;
}

void compute_dirs(){
  for (int x = 0; x < numSteppers; x++) {
     directions[x] = dir_from_diff(positions[x], targets[x]);
  }  
}

void set_targets(){
    if(phase == SET) {
        char current = text[text_pos];
        for (int x = 0; x < 96; x++) {
            if(current == raster[x]) {
                int tilt_dir = dir_from_diff(positions[TILT], targets[TILT]);
                if((current != ' ') && (current != '\n') && current != '\t') {
                    targets[SPIN] = (x % 24) * -increments[SPIN];
                    int index = (1 - (x / 24)) + 2;
                    targets[TILT] = (index - 2) * increments[TILT];
                    targets[LINE] += increments[LINE];
                    targets[HEAD] -= increments[HEAD];             
                    phase = STRIKE;
                } else if(current == ' ') {
                    // A space. the normal case is we just insert a space
                    targets[LINE] += increments[LINE];
                    // if our next word will go off the page, go to the next line. 
                    if( end_of_next_word() > increments[LINE] * 40){ 
                        targets[LINE] = 0;
                        targets[ROLL] += increments[ROLL];
                        phase = RETURN;
                    }
                } else if(current == '\n'){
                    // editorial carriage return (paragraph break) 
                    targets[LINE] = 0;
                    targets[ROLL] += increments[ROLL];
                    phase = RETURN;
                } else if(current == '\t'){
                    // editorial indent or tab
                     targets[LINE] += increments[LINE] * 4;
                }
                break;
            } 
        }
        text_pos += 1;
        // check for EOF
        if(text[text_pos] == 0){
            phase = REST;
            targets[LINE] = 0;
            targets[ROLL] += increments[ROLL];
            targets[SPIN] = 0;
            targets[TILT] = 0;
        }
    } else if(phase == STRIKE) {
        targets[PRES] = increments[PRES];
        positions[PRES] = 0;
        phase = SET;
    } else if(phase == RETURN) {
        phase = SET;   
    } else if(phase == REGISTER) {
        targets[TILT] = -2 * increments[TILT];
        //targets[SPIN] = 0;
        phase = SET;   
    }
}

void setup() {
   
  
  File textFile;
  Serial.begin(9600);
  Serial.flush();
  Serial.println("Initializing SD card...");
  Serial.flush();
  if (!SD.begin()) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");

  textFile = SD.open("text.txt");
  int index = 0;
  if (textFile) {
    while (textFile.available()) {
      text[index] = (textFile.read());
      index += 1;
    }
    text[index] = 0;
    textFile.close();
    Serial.println(text);
    Serial.flush();
  } else {
    Serial.println("error opening test.txt");
  }
  Serial.end();
  delay(1000);
  set_targets();
  compute_dirs();
}

void loop() {
  bool done = true;
  for (int x = 0; x < numSteppers; x++) {
      steppers[x].setSpeed(maxSpeed);
  }    
  for (int x = 0; x < numSteppers; x++) {
    if(positions[x] != targets[x]) {
      steppers[x].step(directions[x]);
      positions[x] += directions[x];
      //Serial.println(positions[LINE]);
      done = false; 
      break;
    } 
  }
  if(done){
     set_targets();
     compute_dirs();     
  }
}
