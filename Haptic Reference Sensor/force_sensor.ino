#include <Arduino.h>

#define SUPPLY_VOLTAGE 3.3
#define ANALOG_RESOLUTION 12
#define NUM_BITS pow(2, ANALOG_RESOLUTION)

// slope and intercept are from a linear fit
float read_force(float slope = 8, float intercept = -0.400987);

void setup()
{
  Serial.begin(9600);
  analogReadResolution(ANALOG_RESOLUTION);
  pinMode(A10, INPUT);
}

int count = 0;
int count_to_print = 10; // this determines when we print a value to the terminal for python to read

void loop()
{
  float force = read_force();
  if (count == count_to_print)
  {
    Serial.print(micros());
    Serial.print(",");
    Serial.println(force, 4);
    count = 0;
  }
  count++;
  // put your main code here, to run repeatedly:
}

// put function definitions here:
float read_force(float slope, float intercept)
{
  float Vout = SUPPLY_VOLTAGE * analogRead(A10) / NUM_BITS;
  float force = Vout * slope + intercept;
  return force;
}