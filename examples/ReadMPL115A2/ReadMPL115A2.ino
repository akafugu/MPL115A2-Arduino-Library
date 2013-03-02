/****************************
 * Read MPL115A2
 *  An example sketch that prints the
 *  local pressure to the PC's serial port
 *
 *  Tested with the MPL115A2-Breakout
 *  Pressure Sensor from Misenso Electronics
 *****************************/

//http://dsscircuits.com/articles/arduino-i2c-master-library.html
#include "I2C.h"
#include <MPL115A2.h>

int led = 5;

void setup()
{
  I2c.begin();
  I2c.pullup(true);
  I2c.setSpeed(1); //400kHz
  MPL115A2.begin();
  Serial.begin(9600);
  pinMode(led, OUTPUT);  
}

void loop()
{
  digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
  MPL115A2.ReadSensor();
  MPL115A2.shutdown();
  
  Serial.print("Pressure(kPa): ");
  Serial.print(MPL115A2.GetPressure());
  Serial.print("     Temperature(C): ");
  Serial.println(MPL115A2.GetTemperature());
  digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);
}
