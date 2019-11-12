#ifndef _Kinematics
#define _Kinematics_h

//You may want to use some/all of these variables
//const float WHEEL_DIAMETER    = ??;
const float WHEEL_RADIUS      = 35;
const float WHEEL_SEPERATION  = 140;
const float MM_PER_COUNT      = (2 * PI * WHEEL_RADIUS)/1440;
//const float GEAR_RATIO        = ??;
//const float COUNTS_PER_SHAFT_REVOLUTION = ??;
const float RADS_PER_COUNT =  (TWO_PI/1440.0);
//const float COUNTS_PER_MM               = ??;


// Build up your Kinematics class.
class Kinematics{
  public:
    float x;
    float y;
    float theta;
    float e1_old = (float) count_e1;
    float e0_old = (float) count_e0;
    
    
    Kinematics();
    // Write your method functions:
    // ...
    void  update();  // should calucate an update to pose.
    void  Kinematics::printCoordinates();
    float Kinematics::getX();
    float Kinematics::getY();
    float Kinematics::getTheta();
    float Kinematics::getDistanceTravelled();
    void  Kinematics :: printTheta();
   
    
    
  private:
    
    //Private variables and methods go here
    
};


// Required constructor.  Initialise variables.
Kinematics::Kinematics() {
   x     = 0;
   y     = 0;
   theta = M_PI/2;
}

void Kinematics::update() {

    
    float e1_new      = (float) count_e1;
    float e0_new      = (float) count_e0 ;
    float delta_e0 = (e0_new - e0_old);
    float delta_e1 = (e1_new - e1_old);


    e0_old = (float)e0_new;
    e1_old = (float)e1_new;
    
    float d0 = delta_e0 * MM_PER_COUNT;
    float d1 = delta_e1 * MM_PER_COUNT;
    
    float theta_new   = theta - ((d1-d0)/WHEEL_SEPERATION);
    
    theta             = fmod(theta_new, TWO_PI);
    
    float x_new = x + (((d0 + d1)/2) * cos(theta) );
    float y_new = y + (((d0 + d1)/2) * sin(theta));
    

    /* UPDATE X AND Y*/
    x = x_new;
    y = y_new;
     
    
}



void Kinematics :: printCoordinates(){
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print(", ");
  Serial.println(theta);

}


float Kinematics::getX(){
  return x;
}

float Kinematics::getY(){
  return y;
}

float Kinematics :: getTheta(){
  return theta;  
}

void Kinematics :: printTheta(){
  Serial.println(theta);
}

float Kinematics::getDistanceTravelled(){
  return sqrt(pow(x,2) + pow(y,2));  
}

#endif
