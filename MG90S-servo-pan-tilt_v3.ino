//servo MG90S
#include <stdio.h>
#include <stdlib.h>
#include <Servo.h>
//Servo myservo;
Servo tilt_servo, pan_servo;

int pos_tilt[] = {0, 10, 20, 30, 40, 50};//rotation axe horizontal
int pos_pan[] = {0, 45, 90, 0, -45, -90};//roation axe vertical

const int POS_INIT_PAN = 85;
const int POS_INIT_TILT = 60;

//global variables to store the current position actualised after each write(angle)
int current_pos_pan_servo = 0;
int current_pos_tilt_servo = 0;

enum axe_servo{
  tilt,
  pan,
};

void servo_go_to_angle(axe_servo servo, int angle_deg){
  int actual_pos = 0;
  
  if(servo == tilt){
    angle_deg += POS_INIT_TILT;
    actual_pos = current_pos_tilt_servo;

    if(actual_pos <= angle_deg){//incrementation to reduce the speed rotation
      for(int i = actual_pos+1; i <= angle_deg; i++){
          tilt_servo.write(i);
          current_pos_tilt_servo = i;
          delay(4);
      }
    }
    else{
      for(int i = actual_pos-1; i >= angle_deg; i--){
          tilt_servo.write(i);
          current_pos_tilt_servo = i;
          delay(4);
      }
    }
  }

  else{
    angle_deg += POS_INIT_PAN;
    actual_pos = current_pos_pan_servo;

    if(actual_pos <= angle_deg){
      for(int i = actual_pos+1; i <= angle_deg; i++){
        pan_servo.write(i);
        current_pos_pan_servo = i;
        delay(4);
      }
    }
    else{
      for(int i = actual_pos-1; i >= angle_deg; i--){
        pan_servo.write(i);
        current_pos_pan_servo = i;
        delay(4);
      }
    }
  }
}

void setup() {
    Serial.begin(9600);
    tilt_servo.attach(9);//can use any PWM pin
    pan_servo.attach(10);
    tilt_servo.write(POS_INIT_TILT);
    pan_servo.write(POS_INIT_PAN);
    delay(2000);
}

void loop() {

  for(int i=0; i<(sizeof(pos_tilt)/sizeof(int)); i++){
    servo_go_to_angle(tilt, pos_tilt[i]);
    
    for(int j=0; j<(sizeof(pos_pan)/sizeof(int)); j++){
      servo_go_to_angle(pan, pos_pan[j]);
      delay(500);
    }
  }
}