#include <Arduino.h>
#include "PID.h"

  PID::PID(float _P, float _I, float _D, int16_t _min, int16_t _max) {
    PID::setPID(_P, _I, _D);
    PID::setLimits(_min, _max);
    PID::reset();
  }
  /*
    Set integral and previousError to 0.
  */
  void PID::reset() {
    integral = 0;
    previousError = 0;
  }
  /*
    Set output limits
  */
  void PID::setLimits(int16_t _min, int16_t _max) {
    minimum = _min;
    maximum = _max;
  }
  /*
    Set P, I, D factors
  */
  void PID::setPID(float _P, float _I, float _D) {
    PID::setP(_P);
    PID::setI(_I);
    PID::setD(_D);
  } 

  void PID::setP(float _P) { P = _P > 0 ? _P : 0; }
  void PID::setI(float _I) { I = _I > 0 ? _I : 0; }
  void PID::setD(float _D) { D = _D > 0 ? _D : 0; }

  float PID::getP() { return P; }
  float PID::getI() { return I; }
  float PID::getD() { return D; }
/*
  Takes the error as input and returns computed PID output
*/
  float PID::compute(float error) {
      
    integral += error;

    float outP = P * error;
    float outI = I * integral;
    float outD = D * (error - previousError);

    float output = outP + outI + outD;

    output = constrain(output, minimum, maximum);

    previousError = error;

    return output;
  }