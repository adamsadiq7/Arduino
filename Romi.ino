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

#define kp_left 0.00
#define ki_left 0.00
#define kd_left 0.00

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

float left_last_timestamp = 0;
float right_last_timestamp = 0;

double left_velocity = 0;
double right_velocity = 0;

float left_elapsed_time = 0;
float right_elapsed_time = 0;

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

// Volatile Global variables used by Encoder ISR.

volatile long right_encoder; // used by encoder to count the rotation
volatile long left_encoder; // used by encoder to count the rotation

volatile bool oldRightEncoder_A; // used by encoder to remember prior state of A
volatile bool oldRightEncoder_B; // used by encoder to remember prior state of B

volatile bool oldE0_A; // used by encoder to remember prior state of A
volatile bool oldE0_B; // used by encoder to remember prior state of B

bool leftWheelDone = false;

bool rightWheelDone = false;

PID left_pid( kp_left, ki_left, kd_left );

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


void calculateLeftVelocity(){
  left_elapsed_time = micros() - left_last_timestamp;
  left_velocity = (float)0.15f / (left_elapsed_time / 1000f); //so we are working mm per s

  Serial.print("left_velocity:");
  Serial.println(left_velocity);

  left_last_timestamp = micros(); //reset last timestamp
}

void calculateRightVelocity(){
  right_elapsed_time = micros() - right_last_timestamp;
  right_velocity = (float)0.15f / (right_elapsed_time / 1000f); //so we are working mm per s

  Serial.print("right_velocity:");
  Serial.println(right_velocity);

  right_last_timestamp = micros(); //reset last timestamp
}

  /* This ISR handles just Encoder 1
  ISR to read the Encoder1 Channel A and B pins
  and then look up based on  transition what kind of
  rotation must have occured. */

ISR(INT6_vect){

  /* First, Read in the new state of the encoder pins.
  Standard pins, so standard read functions.*/

  boolean newE1_B = digitalRead(rightEncoder_B_PIN);
  boolean newE1_A = digitalRead(rightEncoder_A_PIN);

  /* Some clever electronics combines the
  signals and this XOR restores the
  true value.*/

  newE1_A ^= newE1_B;

  /* Create a bitwise representation of our states
  We do this by shifting the boolean value up by
  the appropriate number of bits, as per our table
  header:

  State :  (bit3)  (bit2)  (bit1)  (bit0)
  State :  New A,  New B,  Old A,  Old B.*/

  byte state = 0;

  state = state | (newE1_A << 3);

  state = state | (newE1_B << 2);

  state = state | (oldRightEncoder_A << 1);

  state = state | (oldRightEncoder_B << 0);

  /* This is an inefficient way of determining
  the direction.  However it illustrates well
  against the lecture slides. */

  switch (state){
    case 1:
      right_encoder--;
      calculateRightVelocity();
      break; // anti-clockwise

    case 2:
      right_encoder++;
      calculateRightVelocity();
      break; // clockwise

    case 4:
      right_encoder++;
      calculateRightVelocity();
      break; // clockwise

    case 7:
      right_encoder--;
      calculateRightVelocity();
      break; // anti-clockwise

    case 8:
      right_encoder--;
      calculateRightVelocity();
      break; // anti-clockwise

    case 11:
      right_encoder++;
      calculateRightVelocity();
      break; // clockwise

    case 13:
      right_encoder++;
      calculateRightVelocity();
      break; // clockwise

    case 14:
      right_encoder--;
      calculateRightVelocity();
      break; // anti-clockwise
  }

  // Save current state as old state for next call.
  oldRightEncoder_A = newE1_A;
  oldRightEncoder_B = newE1_B;
}

  /* This ISR handles just Encoder 0
  ISR to read the Encoder0 Channel A and B pins
  and then look up based on  transition what kind of
  rotation must have occured. */

ISR(PCINT0_vect){

  /* First, Read in the new state of the encoder pins.
  Mask for a specific pin from the port.
  Non-standard pin, so we access the register
  directly.

  Reading just PINE would give us a number

  composed of all 8 bits.  We want only bit 2.

  B00000100 masks out all but bit 2*/

  boolean newE0_B = PINE & (1 << PINE2);

  //boolean newE0_B = PINE & B00000100;  // Does same as above.

  // Standard read fro the other pin.
  boolean newE0_A = digitalRead(leftEncoder_A_PIN); // 26 the same as A8

  /* Some clever electronics combines the
  signals and this XOR restores the
  true value.*/

  newE0_A ^= newE0_B;

  /* Create a bitwise representation of our states
  We do this by shifting the boolean value up by
  the appropriate number of bits, as per our table
  header:

  State :  (bit3)  (bit2)  (bit1)  (bit0)
  State :  New A,  New B,  Old A,  Old B.*/

  byte state = 0;
  state = state | (newE0_A << 3);
  state = state | (newE0_B << 2);
  state = state | (oldE0_A << 1);
  state = state | (oldE0_B << 0);

  /* This is an inefficient way of determining
  the direction.  However it illustrates well
  against the lecture slides.*/
  switch (state){

    case 1:
      left_encoder--;
      calculateLeftVelocity();
      break; // anti-clockwise

    case 2:
      left_encoder++;
      calculateLeftVelocity();
      break; // clockwise

    case 4:
      left_encoder++;
      calculateLeftVelocity();
      break; // clockwise

    case 7:
      left_encoder--;
      calculateLeftVelocity();
      break; // anti-clockwise

    case 8:
      left_encoder--;
      calculateLeftVelocity();
      break; // anti-clockwise

    case 11:
      left_encoder++;
      calculateLeftVelocity();
      break; // clockwise

    case 13:
      left_encoder++;
      calculateLeftVelocity();
      break; // clockwise

    case 14:
      left_encoder--;
      calculateLeftVelocity();
      break; // anti-clockwise
  }

  // Save current state as old state for next call.

  oldE0_A = newE0_A;
  oldE0_B = newE0_B;
}

  /*This setup routine enables interrupts for
  encoder1.  The interrupt is automatically
  triggered when one of the encoder pin changes.
  This is really convenient!  It means we don't
  have to check the encoder manually.*/

void setupRightEncoder(){

  // Initialise our count value to 0.
  right_encoder = 0;

  // Initialise the prior A & B signals
  // to zero, we don't know what they were.

  oldRightEncoder_A = 0;
  oldRightEncoder_B = 0;

  // Setup pins for encoder 1
  pinMode(rightEncoder_A_PIN, INPUT);
  pinMode(rightEncoder_B_PIN, INPUT);

  /* Now to set up PE6 as an external interupt (INT6), which means it can
  have its own dedicated ISR vector INT6_vector
  Page 90, 11.1.3 External Interrupt Mask Register – EIMSK
  Disable external interrupts for INT6 first
  Set INT6 bit low, preserve other bits*/

  EIMSK = EIMSK & ~(1 << INT6);

  //EIMSK = EIMSK & B1011111; // Same as above.

  /* Page 89, 11.1.2 External Interrupt Control Register B – EICRB  
  Used to set up INT6 interrupt*/

  EICRB |= (1 << ISC60); // using header file names, push 1 to bit ISC60

  //EICRB |= B00010000; // does same as above

  // Page 90, 11.1.4 External Interrupt Flag Register – EIFR

  // Setting a 1 in bit 6 (INTF6) clears the interrupt flag.
  EIFR |= (1 << INTF6);

  //EIFR |= B01000000;  // same as above

  /* Now that we have set INT6 interrupt up, we can enable

  // the interrupt to happen

  // Page 90, 11.1.3 External Interrupt Mask Register – EIMSK

  // Disable external interrupts for INT6 first

  // Set INT6 bit high, preserve other bits*/

  EIMSK |= (1 << INT6);

  //EIMSK |= B01000000; // Same as above
}

void setupLeftEncoder(){

  // Initialise our count value to 0.
  left_encoder = 0;

  /* Initialise the prior A & B signals
  to zero, we don't know what they were.*/

  oldE0_A = 0;
  oldE0_B = 0;

  /* Setting up E0_PIN_B:
  The Romi board uses the pin PE2 (port E, pin 2) which is
  very unconventional.  It doesn't have a standard
  arduino alias (like d6, or a5, for example).

  We set it up here with direct register access
  Writing a 0 to a DDR sets as input

  DDRE = Data Direction Register (Port)E

  We want pin PE2, which means bit 2 (counting from 0)
  PE Register bits [ 7  6  5  4  3  2  1  0 ]

  Binary mask      [ 1  1  1  1  1  0  1  1 ]

  // By performing an & here, the 0 sets low, all 1's preserve
  // any previous state.*/

  DDRE = DDRE & ~(1 << DDE6);

  //DDRE = DDRE & B11111011; // Same as above.

  /* We need to enable the pull up resistor for the pin

  // To do this, once a pin is set to input (as above)

  // You write a 1 to the bit in the output register*/
  PORTE = PORTE | (1 << PORTE2);

  //PORTE = PORTE | 0B00000100;

  // Encoder0 uses conventional pin 26

  pinMode(leftEncoder_A_PIN, INPUT);

  digitalWrite(leftEncoder_A_PIN, HIGH); // Encoder 0 xor

  /* Enable pin-change interrupt on A8 (PB4) for encoder0, and disable other
  pin-change interrupts.

  Note, this register will normally create an interrupt a change to any pins

  on the port, but we use PCMSK0 to set it only for PCINT4 which is A8 (PB4)

  When we set these registers, the compiler will now look for a routine called
  ISR( PCINT0_vect ) when it detects a change on the pin.  PCINT0 seems like a
  mismatch to PCINT4, however there is only the one vector servicing a change
  to all PCINT0->7 pins.

  See Manual 11.1.5 Pin Change Interrupt Control Register - PCICR

  Page 91, 11.1.5, Pin Change Interrupt Control Register*/

  //Disable interrupt first
  PCICR = PCICR & ~(1 << PCIE0);

  // PCICR &= B11111110;  // Same as above

  // 11.1.7 Pin Change Mask Register 0 – PCMSK0
  PCMSK0 |= (1 << PCINT4);

  // Page 91, 11.1.6 Pin Change Interrupt Flag Register – PCIFR
  PCIFR |= (1 << PCIF0); // Clear its interrupt flag by writing a 1.

  // Enable
  PCICR |= (1 << PCIE0);
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
  float output = left_PID.update(demand, count_e1);
  //Serial.println(output);

  //Once oyu thin your error signal is correct
  //And your PID response is cortrect
  //Send output to motor

  //Consider switching this delay for a millis()
  //task-schedule block
  delay(LOOP_DELAY);

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