#include "PID.h"

/* Function to calculate controller output u[n] based on
 * system input x[n], and measured process output y[n].
 * Process output y[n] is passed to the differentiator instead 
 * of error e[n] to account for discontinuities in system input x[n].
 */
float PID::update(float r, float y)
{
  float e = r - y; // Error e(t)
  float u = K_p * e + K_i * I.update(e) + K_d * D.update(y);
  return u;
}

/* Function to update internal integrator value based on integrator input x. 
 * Uses rectangular integration as discrete integration method.
 */
float Integrator::update(float x)
{
  integral_value += x * Ts;
  return integral_value;
}

/* Constructor takes system parameters Ts and w_c, and calculates
 * filter coefficients for the LP-filtered differentiator.
 */
Differentiator::Differentiator(float Ts, float w_c) : previous_x(0), previous_y(0)
{
  a1 = -1.0/(w_c*Ts + 1);
  b0 = w_c/(w_c*Ts + 1);
  b1 = -b0;
}
/* Update function takes sampled differentiator input "x", calculates 
 * output "y", and updates state variables.
 */
float Differentiator::update(float x)
{
  float y = b0*x + b1*previous_x - a1*previous_y;
  previous_x = x;
  previous_y = y;
  return y;
}
