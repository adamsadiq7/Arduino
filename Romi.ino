#include "encoders.h"
#include "pid.h"
#include "line_sensors.h"
#include "kinematics.h"
#include <math.h> 

// Note that there is no #define for E0_B:.
// it's a non-standard pin, check out setupLeftEncoder().

#define rightEncoder_A_PIN 7
#define rightEncoder_B_PIN 23
#define leftEncoder_A_PIN 26

#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN 9
#define R_DIR_PIN 15

#define BUZZER_PIN 6

#define SAFE_LEFT_SPEED 23
#define SAFE_RIGHT_SPEED 20

#define kp 400
#define ki 5
#define kd 100

/* Variables to remember our
motor speeds for Left and Right.*/

float l_speed;
float r_speed; 

float left_goal = 0;
float right_goal = 0;

float goal = 0;
bool wrongSide = false;

float last_left = left_encoder;
float last_right = right_encoder;

float left_angle_goal = 0;
float right_angle_goal = 0;

float last_timestamp = 0;
float last_timestamp_stop = 0;
float last_timestamp_buzz = 0;
float startedbuzz = 0;
 
int totalWheelsDone = 0;

int previous_right_encoder = 0;
int previous_left_encoder = 0;

float delta_right = 0;
float vel_update_t = 0;
float elapsed_time = 0;
int lastTurn = 0;

bool movementStarted = false;

bool forwardMotion = false;
bool backwardMotion = false;
bool rotateRight = false;
bool rotateLeft = false;
bool hardCode = false;
bool rotated = false;
bool calledRotate = false;
int state = 0;
bool setGoal = false;
// 0 nothing
// 1 move Forward
// 2 right turn
// 3 left turn
// 4 move backward
int commands[8] = {1, 2, 1, 2, 1, 2, 1, 2};
int currentCommand;
int command_index = 0;
bool executingCommand = false;
bool goingHome = false;

bool leftWheelDone = false;

bool rightWheelDone = false;

bool foundLine = false;
bool stop = false;
double theta = M_PI/2;

PID right_pid( kp, ki, kd );
PID left_pid( kp, ki, kd );

LineSensor left_sensor( A2 );
LineSensor middle_sensor( A3 );
LineSensor right_sensor( A4 );

Kinematics position(0,0,theta);

float threshold = -150;

// Remember, setup only runs once.
void setup(){
  /* These two function set up the pin
  change interrupts for the encoders.
  If you want to know more, find them
  at the end of this file. */

  setupLeftEncoder();
  setupRightEncoder();

  // Set our motor driver pins as outputs.
  pinMode(L_PWM_PIN, OUTPUT);
  pinMode(L_DIR_PIN, OUTPUT);
  pinMode(R_PWM_PIN, OUTPUT);
  pinMode(R_DIR_PIN, OUTPUT);
  // Set pin 6 (buzzer) to output.
  pinMode( BUZZER_PIN, OUTPUT);

  // Set initial direction for l and r
  digitalWrite(L_DIR_PIN, HIGH);
  digitalWrite(R_DIR_PIN, HIGH);

  // Set initial l_speed and r_speed values.
  l_speed = 0;
  r_speed = 0;

  left_last_timestamp = micros();
  right_last_timestamp = micros();

  previous_left_encoder = left_encoder;
  previous_right_encoder = right_encoder;

  calibrate();
  left_sensor.calibrate();
  middle_sensor.calibrate();
  right_sensor.calibrate();

  /* Initialise the Serial communication
  so that we can inspect the values of
  our encoder using the Monitor. */

  Serial.begin(9600);
  delay(1000);

  // Print reset so we can catch any reset error.
  Serial.println(" ***Reset*** ");
}

void left_motor(float l_speed){
  if (l_speed < 0){
    digitalWrite(L_DIR_PIN, HIGH); // Backwards for me
  }
  else{
    digitalWrite(L_DIR_PIN, LOW); // Forwards for me
  }
}

void right_motor(float l_speed){
  if (l_speed < 0){
    digitalWrite(R_DIR_PIN, HIGH); // Backwards for me
  }
  else{
    digitalWrite(R_DIR_PIN, LOW); // Forwards for me
  }
}

void driveForward(int mm){
  // Setting goal for encoder/wheel to reach
  left_goal = (left_encoder) + mmToCode(mm);
  right_goal = (right_encoder) + mmToCode(mm);

  //Setting speed of wheel
  l_speed = SAFE_LEFT_SPEED;
  r_speed = SAFE_RIGHT_SPEED;

  // Setting Direction
  left_motor(l_speed);
  right_motor(r_speed);
}

void turnRight(float angle){
  // Setting goal for encoder/wheel to reach
  right_goal = (right_encoder) + angleToCode(angle);

  //Setting speed of wheel
  r_speed = 20;

  // Setting Direction to be forwards
  right_motor(r_speed);
}

void turnLeft(float angle){
  // Setting goal for encoder/wheel to reach
  left_goal = (left_encoder) + angleToCode(angle);

  //Setting speed of wheel
  l_speed = 23;

  // Setting Direction to be forwards
  left_motor(l_speed);
}

void setLeftAngle(float angle){
  // Setting goal for encoder/wheel to reach
  left_angle_goal = left_encoder + angleToCode(angle);
  right_angle_goal = right_encoder - angleToCode(angle);

  //Setting speed of wheel
  l_speed = SAFE_LEFT_SPEED;
  r_speed = SAFE_RIGHT_SPEED;

  // Setting Direction to be forwards
  left_motor(l_speed);
  right_motor(-r_speed);
}

void setRightAngle(float angle){
  // Setting goal for encoder/wheel to reach
  right_angle_goal = right_encoder + angleToCode(angle);
  left_angle_goal = left_encoder - angleToCode(angle);

  //Setting speed of wheel
  r_speed = SAFE_RIGHT_SPEED;
  l_speed = SAFE_LEFT_SPEED;

  // Setting Direction
  right_motor(r_speed);
  left_motor(-l_speed);
}


void calibrate(){
  left_encoder = 0;
  right_encoder = 0;
}

void bangBang(){
  //you need to turn left or right until the middle sensor has found it

  // we are on the black line
  if (middle_sensor.readCalibrated() < threshold){
    state = 2;
    forwardMotion = true;
    last_timestamp_stop = millis();
    rotateLeft = false;
    rotateRight = false;
  }
  else if (left_sensor.readCalibrated() < threshold){
    lastTurn = 1;
    last_timestamp_stop = millis();
    state = 2;
    foundLine = true;
    rotateRight = true;


    rotateLeft = false;
    forwardMotion = false;
  }
  else if (right_sensor.readCalibrated() < threshold){
    lastTurn = 2;
    last_timestamp_stop = millis();
    state = 2;
    foundLine = true;
    rotateLeft = true;

    rotateRight = false;
    forwardMotion = false;
  }
  else{
    if (state == 2){
      // Get how much time has passed right now.
      unsigned long time_now = millis();

      // Work out how many milliseconds have gone passed by subtracting
      // our two timestamps.  time_now will always be bigger than the
      // time_of_read (except when millis() overflows after 50 days).
      unsigned long elapsed_time = time_now - last_timestamp_stop;

      if (elapsed_time > 1000){
        if (state == 2){
          state = 3;
          stop = true;
          goingHome = true;
        }
      }
      else if(lastTurn == 1){
        rotateLeft = true;
      }
      else if(lastTurn == 2){
        rotateRight = true;
      }
    }
    else {
      //whitespace, just go forward until we got itt
      forwardMotion = true;
    }
  }
}

float codeTomm(float code){
  return code * (M_PI*70)/1440;
}

float radiansToDegrees(float radian){
  return radian * 57296 / 1000;
}

void initialisingBeeps(){
  updatePosition();
  state = 1;
}

void driveForwards(){
  executingCommand = true; // do not trigger commands above (global space)
  forwardMotion = false;
  rotateLeft = false;
  rotateRight = false;
  backwardMotion = false;

  float measurement_l = 0;
  float measurement_r = 0;

  float demand = 0.3;

  measurement_l = left_velocity;
  measurement_r = right_velocity;

  float output_l = left_pid.update(demand, measurement_l);
  float output_r = right_pid.update(demand, measurement_r);

  updatePosition();

  //Once you think your error signal is correct
  //And your PID response is correct
  //Send output_r to motor

  //switch direction of motors
  if (output_r > 0){
    right_motor(1); // forwards
  }
  else if(output_r < 0){
    right_motor(-1); //backwards
  }

  if (output_l > 0){
    left_motor(1); // forwards
  }
  else if(output_l < 0){
    left_motor(-1); //backwards
  }

  output_r = constrain(output_r, 0, 255);
  output_l = constrain(output_l, 0, 255);

  bangBang();

  if (foundLine){
    state = 2;
  }

  if (!goingHome){

    if (forwardMotion){
      right_motor(1); // forwards
      left_motor(1); // forwards

      if (!stop || goingHome){
        analogWrite(R_PWM_PIN, output_r);
        analogWrite(L_PWM_PIN, output_l);
      }
      else if (stop){
        analogWrite(R_PWM_PIN, 0);
        analogWrite(L_PWM_PIN, 0);
      }
    }
    else if (rotateLeft){

      right_motor(1); // forwards
      left_motor(-1); // backwards
      if (!stop){
        analogWrite(R_PWM_PIN, output_r);
        analogWrite(L_PWM_PIN, output_l);
      }
      else{
        analogWrite(R_PWM_PIN, 0);
        analogWrite(L_PWM_PIN, 0);
      }
    }
    else if (rotateRight){
      right_motor(-1); // forwards
      left_motor(1); // backwards
      if (!stop){
        analogWrite(R_PWM_PIN, output_r);
        analogWrite(L_PWM_PIN, output_l);
      }
      else{
        analogWrite(R_PWM_PIN, 0);
        analogWrite(L_PWM_PIN, 0);
      }
    }
  }
}

void foundLineBeeps(){
  executingCommand = true; // do not trigger commands above (global space)
  forwardMotion = false;
  rotateLeft = false;
  rotateRight = false;
  backwardMotion = false;

  float measurement_l = 0;
  float measurement_r = 0;

  // Get how much time has passed right now.
  unsigned long time_now = millis();

  // Work out how many milliseconds have gone passed by subtracting
  // our two timestamps.  time_now will always be bigger than the
  // time_of_read (except when millis() overflows after 50 days).
  unsigned long elapsed_time = time_now - last_timestamp;
  float demand;
  if(elapsed_time > 30000){
    demand = 0.07;
  }
  else{
    demand = 0.2;
  }

  measurement_l = left_velocity;
  measurement_r = right_velocity;

  float output_l = left_pid.update(demand, measurement_l);
  float output_r = right_pid.update(demand, measurement_r);
  
  updatePosition();

  //Once you think your error signal is correct
  //And your PID response is correct
  //Send output_r to motor

  //switch direction of motors
  if (output_r > 0){
    right_motor(1); // forwards
  }
  else if(output_r < 0){
    right_motor(-1); //backwards
  }

  if (output_l > 0){
    left_motor(1); // forwards
  }
  else if(output_l < 0){
    left_motor(-1); //backwards
  }

  output_r = constrain(output_r, 0, 255);
  output_l = constrain(output_l, 0, 255);

  bangBang();

  if (!goingHome){

    if (forwardMotion){
      right_motor(1); // forwards
      left_motor(1); // forwards

      if (!stop || goingHome){
        analogWrite(R_PWM_PIN, output_r);
        analogWrite(L_PWM_PIN, output_l);
      }
      else if (stop){
        analogWrite(R_PWM_PIN, 0);
        analogWrite(L_PWM_PIN, 0);
      }
    }
    else if (rotateLeft){

      right_motor(1); // forwards
      left_motor(-1); // backwards
      if (!stop){
        analogWrite(R_PWM_PIN, output_r);
        analogWrite(L_PWM_PIN, output_l);
      }
      else{
        analogWrite(R_PWM_PIN, 0);
        analogWrite(L_PWM_PIN, 0);
      }
    }
    else if (rotateRight){
      right_motor(-1); // forwards
      left_motor(1); // backwards
      if (!stop){
        analogWrite(R_PWM_PIN, output_r);
        analogWrite(L_PWM_PIN, output_l);
      }
      else{
        analogWrite(R_PWM_PIN, 0);
        analogWrite(L_PWM_PIN, 0);
      }
    }
  }
}

void stopIt(){
  updatePosition();
  if (!startedbuzz){
    last_timestamp_buzz = millis();
    startedbuzz = true;
  }

  analogWrite(R_PWM_PIN, 0);
  analogWrite(L_PWM_PIN, 0);

  analogWrite(6, 15);

  float time_now = millis();
  float elapsed_time = time_now - last_timestamp_buzz;
  if (elapsed_time > 2000){ //after 2 seconds
    analogWrite(6, 0);
    state = 4;
  }
}


void setRotate(){
  updatePosition();
  float x = radiansToDegrees(position.getX());
  float y = radiansToDegrees(position.getY());
  goal = atan(y/x);
  Serial.println("Angle is set");
  state = 5;
}


void rotateUntil(){
  updatePosition();

  if (theta > goal && !wrongSide){
    wrongSide = true;
  }

  if (wrongSide){
    if (theta > goal){
      left_motor(1);
      right_motor(-1);
      analogWrite(R_PWM_PIN, abs(SAFE_RIGHT_SPEED));
      analogWrite(L_PWM_PIN, abs(SAFE_LEFT_SPEED));
    }
    else{
      state = 6;
    }
  }
  else{
    if (theta < goal){
      left_motor(-1);
      right_motor(1);
      analogWrite(R_PWM_PIN, abs(SAFE_RIGHT_SPEED));
      analogWrite(L_PWM_PIN, abs(SAFE_LEFT_SPEED));
    }
    else{
      state = 6;
    }
  }
}

void rotate(){
  /* --- Checking for right wheel ---*/
  if (right_encoder > right_angle_goal){
    analogWrite(R_PWM_PIN, abs(SAFE_RIGHT_SPEED));
  }
  else{
    //stop wheel
    rightWheelDone = true;
    analogWrite(R_PWM_PIN, 0);
    //check if other wheel is still turning
    if (leftWheelDone && rightWheelDone){
      leftWheelDone = false;
      rightWheelDone = false;
      state = 6;
    }
  }

  /* --- Checking for left wheel ---*/
  if (left_encoder < left_angle_goal){
    analogWrite(L_PWM_PIN, abs(SAFE_LEFT_SPEED));
  }
  else{
    //stop wheel
    leftWheelDone = true;
    analogWrite(L_PWM_PIN, 0);
    //check if other wheel is still turning
    if (leftWheelDone && rightWheelDone){
      rotateRight = false;
      leftWheelDone = false;
      rightWheelDone = false;
      state = 6;
    }
  }
}

void updatePosition(){
  float left_enc = left_encoder;
  float right_enc = right_encoder;
  float diff_l = codeTomm(left_enc - last_left);
  float diff_r = codeTomm(right_enc - last_right);

  theta += ((diff_l) - diff_r)/(2*WHEEL_SEPERATION);
  theta = fmod(theta,(M_PI * 2));
  float avgDistance = (diff_l + diff_r)/2;

  position.update(avgDistance, theta);

  last_left = left_enc;
  last_right = right_enc;
}

void goHome(){
  updatePosition();
}

void loop(){

  Serial.println(" ----- ");
  Serial.println(theta);
  Serial.print(",");
  Serial.print(goal);
  Serial.println(" ----- ");


  switch(state) {
      case 0:
          initialisingBeeps();
          break;
      case 1:
          driveForwards();
          break;
      case 2:
          foundLineBeeps();
          break;
      case 3:
          stopIt();
          break;
      case 4:
          setRotate();
          break;
      case 5:
          rotateUntil();
          break;
      case 6:
          goHome();
          break;
      default:
          Serial.println("System Error, Unknown state!");
          break;
  }

  /* ------ THIS IS FOR GOING HOME ------*/

  //Receive input to start moving
  if (!executingCommand){
    switch (currentCommand){
      case 1: //driveForward
        driveForward(100);
        forwardMotion = true;
        executingCommand = true;
        break;

      case 2: //turnRight
        setRightAngle(90);
        rotateRight = true;
        executingCommand = true;
        break;

      case 3: //turnLeft
        setLeftAngle(90);
        rotateLeft = true;
        executingCommand = true;
        break;

      case 4: //reverse
        driveForward(100);
        backwardMotion = true;
        executingCommand = true;
        break;
      }
  }

  delay(2);
}
