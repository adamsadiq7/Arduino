#include "encoders.h"
#include "pid.h"

// Note that there is no #define for E0_B:.
// it's a non-standard pin, check out setupLeftEncoder().

#define rightEncoder_A_PIN 7
#define rightEncoder_B_PIN 23
#define leftEncoder_A_PIN 26

#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN 9
#define R_DIR_PIN 15

#define SAFE_LEFT_SPEED 23
#define SAFE_RIGHT_SPEED 20

#define kp 0.15
#define ki 0.05
#define kd 0.00

/* Variables to remember our
motor speeds for Left and Right.*/

float l_speed;
float r_speed;

float left_goal = 0;
float right_goal = 0;

float left_angle_goal = 0;
float right_angle_goal = 0;

int totalWheelsDone = 0;

int previous_right_encoder = 0;
int previous_left_encoder = 0;

float delta_right = 0;
float vel_update = 0;

bool movementStarted = false;

bool forwardMotion = false;
bool backwardMotion = false;
bool rotateRight = false;
bool rotateLeft = false;

// 0 nothing
// 1 move Forward
// 2 right turn
// 3 left turn
// 4 move backward
int commands[8] = {1, 2, 1, 2, 1, 2, 1, 2};
int currentCommand;
int command_index = 0;
bool executingCommand = false;

bool leftWheelDone = false;

bool rightWheelDone = false;

PID right_pid( kp, ki, kd );

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

  //Setting speed of wheel
  l_speed = SAFE_LEFT_SPEED;

  // Setting Direction to be forwards
  left_motor(l_speed);
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

void commandFinished(){
  if (command_index < sizeof(commands) - 1){
    command_index++;
    currentCommand = commands[command_index];
  }
  executingCommand = false;
}

float mmToCode(float mm){
  return 5.408 * mm; //about correct
}

float angleToCode(float angle){
  return angle * 8.3;
}

void calibrate(){
  left_encoder = 0;
  right_encoder = 0;
  currentCommand = commands[0];
}

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

  /* Initialise the Serial communication
  so that we can inspect the values of
  our encoder using the Monitor. */

  Serial.begin(9600);
  delay(1000);

  // Print reset so we can catch any reset error.
  Serial.println(" ***Reset*** ");
}


void loop(){
  // output_signal <-----PID-- demand, measurement

  executingCommand = true; // do not trigger commands above (global space)

  float measurement = (float) right_velocity;
  float demand = 0.375f;

  unsigned long elapsed_time;
  elapsed_time = millis() - vel_update;

  // calculating speed
  if (elapsed_time > 50){
    vel_update = millis(); //update time

    long diff_count;
    diff_count = right_encoder - previous_right_encoder;
    previous_right_encoder = right_encoder;

    right_velocity = (float) diff_count;
    right_velocity = right_velocity / (float) elapsed_time;
  }
  // Serial.println(right_velocity);

  float output = right_pid.update(demand, right_velocity);

  //Once you think your error signal is correct
  //And your PID response is correct
  //Send output to motor

  //switch direction of motors
  if (output > 0){
    right_motor(1); // forwards
  }
  else if(output < 0){
    right_motor(-1); //backwards
  }
  else{
    Serial.println("ACHEIVED!\n");
    output = 0;
    Serial.println("ACHEIVED!\n");
  }

  output = constrain(output, 0, 255);

  analogWrite(R_PWM_PIN, output); // stop the left wheel

  
  Serial.print(measurement);
  Serial.print(", ");
  Serial.print(demand);
  Serial.print(", ");
  Serial.print(right_pid.getI());

  Serial.print("\n");


  //Consider switching this delay for a millis()
  //task-schedule block
  delay(2);

  // build your main code here. 
  // Call your pid.update() at a regular time interval.

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

  // if the current Command is to move forward

  if (forwardMotion){
    // If we haven't met the goal for the left wheel yet, we keep on moving
    if (left_encoder < left_goal){
      // Send speeds to pins, to motor drivers.
      analogWrite(L_PWM_PIN, abs(l_speed));
    }

    // we have finished our command
    else{
      leftWheelDone = true;
      //check if other wheel is still turning
      if (leftWheelDone && rightWheelDone){
        forwardMotion = false;
        leftWheelDone = false;
        rightWheelDone = false;
        commandFinished();
      }
      analogWrite(L_PWM_PIN, 0); // stop the left wheel
    }

    // If we haven't met the goal for the right wheel yet, we keep on moving
    if (right_encoder < right_goal){
      // Send speeds to pins, to motor drivers.
      analogWrite(R_PWM_PIN, abs(r_speed));
    }

    // we have met the goal
    else{
      rightWheelDone = true;
      //check if other wheel is still turning
      if (leftWheelDone && rightWheelDone){
        forwardMotion = false;
        leftWheelDone = false;
        rightWheelDone = false;
        commandFinished();
      }
      analogWrite(R_PWM_PIN, 0); //Stop the right wheel
    }
  }

  // The current command is to rotate right

  if (rotateRight){
    if (right_encoder < right_angle_goal){
      analogWrite(R_PWM_PIN, abs(r_speed));
    }
    else{
      rightWheelDone = true;
      analogWrite(R_PWM_PIN, 0);

      //check if other wheel is still turning
      if (leftWheelDone && rightWheelDone){
        rotateRight = false;
        leftWheelDone = false;
        rightWheelDone = false;
        commandFinished();
      }
    }

    if (left_encoder > left_angle_goal){
      analogWrite(L_PWM_PIN, abs(l_speed));
    }

    else{
      leftWheelDone = true;
      analogWrite(L_PWM_PIN, 0);

      //check if other wheel is still turning
      if (leftWheelDone && rightWheelDone){
        rotateRight = false;
        leftWheelDone = false;
        rightWheelDone = false;
        commandFinished();
      }
    }
  }

  // The current command is to rotate left

  if (rotateLeft){
    if (left_encoder < left_angle_goal){
      analogWrite(L_PWM_PIN, abs(l_speed));
    }
    else{
      rotateLeft = false;
      analogWrite(L_PWM_PIN, 0);
      commandFinished();
    }
  }

  // if the current command is to move backward
  if (backwardMotion){
    // If we haven't met the goal for the left wheel yet, we keep on moving
    if (left_encoder > left_goal){
      // Send speeds to pins, to motor drivers.
      analogWrite(L_PWM_PIN, abs(l_speed));
    }
    // we have met the goal
    else{
      // Check if we are not at the end of commands
      backwardMotion = false;
      analogWrite(L_PWM_PIN, 0); //Stop the left wheel
      commandFinished();
    }

    // If we haven't met the goal for the right wheel yet, we keep on moving
    if (right_encoder > right_goal){
      // Send speeds to pins, to motor drivers.
      analogWrite(R_PWM_PIN, abs(r_speed));
    }

    // we have met the goal
    else{
      // Check if we are not at the end of commands
      if (command_index < sizeof(commands) - 1){
        command_index++;
        currentCommand = commands[command_index];
        executingCommand = false;
      }

      // We have finished all commands
      else{
        analogWrite(R_PWM_PIN, 0); //Stop the right wheel
      }
    }
  }

  /* Output the count values for our encoders
  with a comma seperation, which allows for
  two lines to be drawn on the Plotter.

  NOTE: left_encoder and right_encoder values are now
  automatically updated by the ISR when the encoder pins change.

  //Serial.print(left_encoder);
  //Serial.print(", ");
  //Serial.println(right_encoder);

  // short delay so that our plotter graph keeps

  // some history.*/

  delay(2);
}
