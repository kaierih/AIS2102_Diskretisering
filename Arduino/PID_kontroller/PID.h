#ifndef PID_H
#define PID_H

#include <arduino.h>

/* Class to handle running integration of a repeating input value "x" 
 * in a discrete-time system with sampling time Ts.
 */
class Integrator
{
  public:
    Integrator(float Ts) : Ts(Ts), integral_value(0){};
    float update(float x);
  private:
    float integral_value;
    float Ts;
};

/* Class to handle differentiation of a repeating input value "x", 
 * with added lowpass filtering. Lowpass filter ensures frequency
 * content beyond "w_c" rad/s is no longer amplified.
 */
class Differentiator
{
  public:
    Differentiator(float Ts, float w_c);
    float update(float x);
  private:
    float Ts;
    float previous_x; // Previous input sample
    float previous_y; // Previous output sample
    float a1; // Recursive filter coefficient
    float b0; // Forward filter coefficient
    float b1; // Forward filter coefficient
};

class PID
{
  private:
    float K_p;
    float K_i;
    float K_d;
    float Ts;
    Integrator I;
    Differentiator D;
  public:
    float update(float x, float y);
    PID(float K_p, float K_i, float K_d, float Ts, float max_freq) : K_p(K_p), K_i(K_i), K_d(K_d), Ts(Ts), I(Ts), D(Ts, max_freq){};
};


#endif
