#ifndef _Line_follow_h
#define _Line_follow_h

//Number of readings to take for calibration
//const int NUM_CALIBRATIONS = ????;

/* 
 *  Class to represent a single line sensor
 */
class LineSensor {
  public:

    // Required function.
    LineSensor(int pin);   //Constructor

    // Suggested functions.
    float calibrate();       //Calibrate
    int readRaw();         //Return the uncalibrated value from the sensor
    float readCalibrated();  //Return the calibrated value from the sensor

   
    
  private:
  
    int pin;
    int bias;
    
    
};


// Class Constructor: 
// Sets pin passed in as argument to input
LineSensor::LineSensor(int Line_pin) {
  pin = Line_pin;
  pinMode(pin, INPUT);
}

// Returns unmodified reading.
int LineSensor::readRaw() {
  return analogRead(pin);
}

// Write this function to measure any
// systematic error in your sensor and
// set some bias values.
float LineSensor::calibrate() {

  float reading=0;

  for( int i =0 ; i< 100 ; i++){
     reading += (float)readRaw();

  }

  bias = reading/100;
  
}


// Use the above bias values to return a
// compensated ("corrected") sensor reading.
float LineSensor::readCalibrated() {
   return readRaw()-bias;
}


#endif
