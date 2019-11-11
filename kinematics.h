#ifndef _Kinematics
#define _Kinematics_h
#include <math.h>

//You may want to use some/all of these variables
const float WHEEL_DIAMETER    = 70; //in mm
const float WHEEL_RADIUS      = 35; //in mm
const float WHEEL_SEPERATION  = 70;
//const float GEAR_RATIO        = ??;
const float COUNTS_PER_SHAFT_REVOLUTION = 12;
//const float COUNTS_PER_WHEEL_REVOLUTION =  ??;
//const float COUNTS_PER_MM               = ??;


// Build up your Kinematics class.
class Kinematics{
  public:
    
    Kinematics(float x, float y, float theta);   // Constructor, required.
    float Kinematics::getY();
    float Kinematics::getX();
    void Kinematics::setConstructor(float x, float y, float theta);

    // Write your method functions:
    // ...
    void Kinematics::update(float distance, float theta_);  // should calucate an update to pose.
    
  private:
    
    //Private variables and methods go here
    float x; 
    float y; 
    float theta;
    float last_p;
};


// Required constructor.  Initialise variables.
Kinematics::Kinematics(float x_input, float y_input, float theta_input) {
	setConstructor(x_input, y_input, theta_input);
}

void Kinematics::update(float distance, float theta_){
  y += distance * sin(theta_);
  x += distance * cos(theta_);

  Serial.print(x);
  Serial.print(",");
  Serial.println(y);
}

void Kinematics::setConstructor(float x_input, float y_input, float theta_input){
  x = x_input;
  y = y_input;
  theta = theta_input;
}

float mmToCode(float mm){
  return 5.408 * mm; //about correct
}

float Kinematics::getX(){
  return x;
}

float Kinematics::getY(){
  return y;
}

float angleToCode(float angle){
  return angle * 8.3;
}

#endif