#include "encoders.h"
#include "pid.h"
#include "line_sensors.h"
#include <stdlib.h>
#include "kinematics.h"

// You may need to change these depending on how you wire
// in your line sensor.u
#define LINE_LEFT_PIN   A4 //Pin for the left line sensor
#define LINE_CENTRE_PIN A3 //Pin for the centre line sensor
#define LINE_RIGHT_PIN  A2 //Pin for the right line sensor

//MOTOR PINS 
#define R_PWM_PIN 10
#define R_DIR_PIN 16
#define L_PWM_PIN 9
#define L_DIR_PIN 15

#define FWD HIGH
#define BACK LOW

/* STATES */
#define FIND_LINE 0
#define ON_LINE 1
#define OFF_LINE 2
#define GO_HOME 3
#define HALT -1

float CONFIDENCE_THRESHOLD =  100.0f;
float confidence = 0;

int state;


float left;
float centre;
float right;

/* LINE PID */
float P= 0.004;
float I ;
float D;
PID line_pid(P,I,D);


/*MOTOR PID's */
float kp=50;
float kd= 0;
float ki =0;
PID left_motor( kp, kd, ki);
PID right_motor( kp, kd, ki);




/* LINE SENSORS*/
LineSensor line_left(LINE_LEFT_PIN); //Create a line sensor object for the left sensor
LineSensor line_centre(LINE_CENTRE_PIN); //Create a line sensor object for the centre sensor
LineSensor line_right(LINE_RIGHT_PIN); //Create a line sensor object for the right sensor


Kinematics kine;
float angle_from_horizontal;

void setSpeed(float demand, int motor){
  float absolute = abs(demand);

  switch(motor){
    case 0:
      
      if (demand < 0){digitalWrite(R_DIR_PIN, BACK);} else{digitalWrite(R_DIR_PIN,FWD);}  
      analogWrite(R_PWM_PIN, absolute);
      break;

    case 1:
      
      if (demand < 0){digitalWrite(L_DIR_PIN, BACK);} else{digitalWrite(L_DIR_PIN,FWD);}
      analogWrite(L_PWM_PIN, absolute);
      break;
  }
  
}


float centre_of_line(float left, float centre, float right){
   float lineCentre;

   float total = left+right+centre + 0.001;
   float probLeft = left/total;
   float probRight = right/total;
   float probCentre = centre/total;  
   lineCentre = (probLeft * 1000 + probCentre*2000 + probRight*3000 ) -2000;

   lineCentre = constrain(lineCentre,-2000,2000);

   return lineCentre;  

}


// Remember, setup only runs once.
void setup() {
  setupEncoder0();
  e0_last_time = micros();
  e0_old = count_e0;
  e0_new = count_e0;
  

  setupEncoder1();
  e1_last_time = micros();
  e1_old = count_e1;
  e1_new = count_e1;

  // Initialise your other globals variables
  // and devices.
  
  line_left.calibrate();
  line_centre.calibrate();
  line_right.calibrate();
  
  // Initialise the Serial communication
  Serial.begin( 9600 );
  analogWrite(6, 5);
  
  delay(1000);
  analogWrite(6,0);
  digitalWrite(13,LOW);
  Serial.println("***RESET***");
  digitalWrite(R_DIR_PIN, FWD);
  digitalWrite(L_DIR_PIN, FWD);

  /*Initial State*/
  state = FIND_LINE;
  confidence =0;
 
  
}


void find_line(){

  /*
 * Values near -2000 mean that the left sensor is over the centre of the line
 * Values near 0 means that the centre os over the centre of the line
 * Values near 2000 means that the right sensor is over the line 
 * 
 */

  float confidence_new;
 setSpeed(-30,1);
 setSpeed(-30,0);
  float l= line_left.readCalibrated();
  float r= line_right.readCalibrated();
  float c= line_centre.readCalibrated();


  if ( l > 100 || c > 100 || r > 100){
    confidence_new +=0.2;
    if ( l > 200 || c > 200 || r > 200){
      confidence_new +=0.5;
      if ( l > 300 || c > 300 || r > 300){
        confidence_new +=1;
      }
    }
    
  }

  if(confidence_new == confidence){confidence =0;}
  else{confidence = confidence_new;}

  //Serial.println(confidence);

  if (confidence > CONFIDENCE_THRESHOLD){state=ON_LINE; confidence =0;}


  
    
}

void on_line(){
  
  float left_pwm;
  float right_pwm;
  float turn;
  float forward_bias = -0.47;
  
  float l;
  float c;
  float r;
  l = abs(line_left.readCalibrated());
  c = abs(line_centre.readCalibrated());
  r = abs(line_right.readCalibrated());
  turn = line_pid.update(0.0, centre_of_line(l,c,r));  
     
  left_pwm = left_motor.update(forward_bias+turn, e1_speed);

  right_pwm = right_motor.update(forward_bias-turn, e0_speed);
  setSpeed(left_pwm, 1);
  setSpeed(right_pwm, 0);

  
 if ( l < 500 && c < 500 && r < 500){
   confidence += 0.2f;
   if ( l < 300 && c < 300 && r < 300){
    confidence += 0.5f;
     if ( l < 100 && c < 100 && r < 100){
       confidence += 1.0f;  
     }
   }
   
 }
 else{
   confidence -= 1.0f;
 }
 
  confidence = constrain(confidence, 0, CONFIDENCE_THRESHOLD);
  Serial.println(confidence);

  
  if (confidence == CONFIDENCE_THRESHOLD){
    state=OFF_LINE;
  }
}

bool right_side = false;
float angle_to_home;
float direction_to_home;


void off_line(){
  float x = kine.getX();
  float y = kine.getY();
  float theta = kine.getTheta();
  
  setSpeed(0,1);
  setSpeed(0,0);
  digitalWrite(13,HIGH); 
  angle_to_home = atan2(-y,-x) + PI;
  //if((x/y) > 0 ) {angle_to_home += PI;} else{angle_to_home -= PI;}
    
  
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print(", ");
  Serial.print(theta);
  Serial.print(", ");
  Serial.println(angle_to_home);
  if (kine.getTheta() > angle_to_home){
    right_side = false;
  }
  
  state = GO_HOME;
}
void go_home(){
  setSpeed(20,0);
  setSpeed(-20,1);
  float theta = kine.getTheta();
   if(!right_side){
    if( theta< angle_to_home){right_side=true;} 
   }
   else{
     if(theta > angle_to_home){state=HALT;}
   }
}

void halt(){
  setSpeed(0,1);
  setSpeed(0,0);
  
}


// Remmeber, loop is called again and again.
void loop(){
//kine.printCoordinates();
//Serial.println(direction_to_home);
kine.update();
  
 switch(state){
   case 0:
      find_line();
      break;
    case 1:
       on_line();
      break;
    case 2:
      off_line();
      break;
    case 3:
      go_home();
      break;

    case -1:
      halt();
      break;


    
  }

  

  
 

}
