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


#define SENSOR_VALUE 240 // You'll need to adjust these for your surface.

// TODO actually calibrate robot and set these
// there will be only two commands: go forward (at a constant rate v (in m/s)) and turn about center (at a constant rate omega (in rad/s)).
#define FORWARD_VELOCITY 0.074
#define ROTATIONAL_VELOCITY 0.94247779607
// command speeds for both wheels when going forward
#define LEFT_WHEEL_FORWARD 50
#define RIGHT_WHEEL_FORWARD 46
// command speeds for both wheels when turning right
#define LEFT_WHEEL_TURN_RIGHT 50
#define RIGHT_WHEEL_TURN_RIGHT -50
// command speeds for both wheels when turning left
#define LEFT_WHEEL_TURN_LEFT -50
#define RIGHT_WHEEL_TURN_LEFT 50


unsigned long start_time = 0;
unsigned long current_time = 0;

void setup()
{
  // Enable bump detection. Once a bump occurs, xl.checkBump() can be
  //  used to detect it. We'll use that to start moving.
  xl.enableBump();
}

void loop()
{
  
  // triggers start if accel has detected a bump
  if(xl.checkBump() && start == false) start = true;
  
  // start
  if(start) {
    start_time = millis();
    current_time = millis();
    while(current_time - start_time < 5000) {
      // go forward
      motor.rightDrive(RIGHT_WHEEL_FORWARD);
      motor.leftDrive(LEFT_WHEEL_FORWARD);

      // turn left
//      motor.rightDrive(RIGHT_WHEEL_TURN_LEFT);
//      motor.leftDrive(LEFT_WHEEL_TURN_LEFT);
      
      
        // turn right
//        motor.rightDrive(RIGHT_WHEEL_TURN_RIGHT);
//        motor.leftDrive(LEFT_WHEEL_TURN_RIGHT);


      current_time = millis();
    }
    motor.brake();
    while(true);  // loop forever

  }
  

}
