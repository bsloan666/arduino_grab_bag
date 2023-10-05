/*
    Cribbed from stepper_speedcontrol
    A simple-sh 4-story elevator controller. 

    C.2023 B. Sloan

    We connects 8 pushbuttons (one at each floor and four in the elevator panel)
    to analog pin A0 according to the method described here: 
    https://www.youtube.com/watch?v=Y23vMfynUJ0&ab_channel=MichaelKlements

    The stepper controller is connected to the digital pins as follows

    in1 -> D08
    in2 -> D10
    in3 -> D09
    in4 -> D11
*/                                                                                                                                                                                                      
#include <Stepper.h>

const int stepsPerRevolution = 64;  // change this to fit the number of steps per revolution
// for your motor

int motorSpeed = 0;
int polarity = 2;
int accel_decel = 1;
int maxSpeed = 512;
int target_floor = 0;
int current_floor = 0;
int step_count = 0;  // number of steps the motor has taken
int floor_steps[4] = {0, 10000, 15000, 21000};
int moving = 0;
int total_steps = 0;

// initialize the stepper library on pins 8 through 11:
Stepper myStepper(stepsPerRevolution, 8, 9, 10, 11);


void setup() {
    // for logging  
    Serial.begin(9600);
}

void loop() {
    int button0 = analogRead(A0);
    //Serial.println(button0);
  
    if(moving == 0){
    
        if(button0 < 100){
            // do nothing
        }

        else if(button0 < 160){
            target_floor = 3;
            Serial.println("Third floor selected!");
        }
    
        else if(button0 <  260){
            target_floor = 2;
            Serial.println("Second floor selected!");
        }

        else if(button0< 460){
            target_floor = 1;
            Serial.println("First floor selected!");
        }
        else if(button0 < 1200){
            target_floor = 0;
            Serial.println("Ground floor selected!");
        }
  
     
        if(target_floor != current_floor){
            moving = 1;
            total_steps = floor_steps[target_floor] - floor_steps[current_floor];
            step_count = 0;
            if(total_steps < 0) {
                polarity = -2;
            }
            else {
                polarity = 2;
            }
            Serial.print("Stepping by ");
            Serial.print(polarity);
            Serial.print(" to ");
            Serial.println(total_steps);
            delay(100);
        }
    }
    if(moving != 0){
        myStepper.setSpeed(maxSpeed);
        myStepper.step(polarity);
        step_count = step_count + polarity;
        if (step_count == total_steps) {
            step_count = 0;
            moving = 0;
            current_floor = target_floor;        
            Serial.println("Arrived!");
        }
    }
}
