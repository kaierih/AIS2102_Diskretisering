#include <MCP_DAC.h>
#include "PID.h"

// Input pin for analog signal
#define SIG_INPUT = A0

// Configuration of output to external DAC
MCP4911 MCP(10, 11);  // Declare an object for handling of DAC
#define CS_PIN 13  // "Chip Select" output pin on Arduino Uno

// Set sampling period
#define T_s 10 // Sample time is 5 milliseconds
unsigned long next_sample_checkpoint = 0; // Global checkpoint variable for sample times

#define K_p 1.0
#define K_i 0.0
#define K_d 0.0
#define max_freq 100.0 // rad/s
PID controller(K_p, K_i, K_d, (float)T_s/1000.0, max_freq);

void setup()
{
  MCP.begin(CS_PIN);
  next_sample_checkpoint = millis();
}

void loop()
{
  /* If millis() counter exceeds next sample checkpoint, 
   * get a new sample for processing. */
  if(millis() > next_sample_checkpoint)
  {
    next_sample_checkpoint += T_s; // Set new checkpoint for next sample.
    int sensor_value = analogRead(A0)-512; // Measured process output
    int setpoint = analogRead(A1);     // Process value setpoint r(t)
    float controller_output = controller.update(setpoint, sensor_value);
    MCP.analogWrite(round(controller_output) + 512, 0);   // Write output value to DAC
  }
}
