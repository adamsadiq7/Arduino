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
    void calibrate();       //Calibrate
    int readRaw();         //Return the uncalibrated value from the sensor
    int readCalibrated();  //Return the calibrated value from the sensor

    // You may wish to add other helpful functions!
    // ...
    
  private:
  
    int pin;
    /*
     * Add any variables needed for calibration here
     */
    
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
void LineSensor::calibrate() {
  /*
   * Write code to calibrate your sensor here
   */
}


// Use the above bias values to return a
// compensated ("corrected") sensor reading.
int LineSensor::readCalibrated() {
  /*
   * Write code to return a calibrated reading here
   */
}


#endif