/*
 * Bug0 algorithm implemented on standard RedBot
 */
#include <RedBot.h>

// Instantiate the motor control class. This only needs to be done once
//  and indeed SHOULD only be done once!
RedBotMotors motor;
// Instantiate the sensors. Sensors can only be created for analog input
//  pins; the Xbee software serial uses pins A0 and A1 and the
//  accelerometer uses pins A4 and A5.
RedBotSensor lSen = RedBotSensor(A3);
RedBotSensor cSen = RedBotSensor(A6);
RedBotSensor rSen = RedBotSensor(A7);
// Instantiate the accelerometer. It can only be connected to pins A4
//  and A5, since those are the I2C pins for this board.
RedBotAccel xl;
// Variable used to wait on a bump before starting operation.
boolean start = false;

void setup()
{
  Serial.begin(57600);
  // Enable bump detection. Once a bump occurs, xl.checkBump() can be
  //  used to detect it. We'll use that to start moving.
  xl.enableBump();
}

void loop()
{
  // triggers start if accel has detected a bump
  if  (xl.checkBump() && start == false) start = true;
  
  
  if (start)
  {
  }  
}
