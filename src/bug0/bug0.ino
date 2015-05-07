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


// variables for odometry
float x = 0;      // in meters
float y = 0;      // in meters
float theta = 0;  // in radians

#define darkLevel  100  // You'll need to adjust these for your surface.

// TODO actually calibrate robot and set these
// there will be only two commands: go forward (at a constant rate v (in m/s)) and turn about center (at a constant rate omega (in rad/s)).
#define FORWARD_VELOCITY 1.00f
#define ROTATIONAL_VELOCITY 1.00f
// command speeds for both wheels when going forward
#define LEFT_WHEEL_FORWARD 50
#define RIGHT_WHEEL_FORWARD 50
// command speeds for both wheels when turning right
#define LEFT_WHEEL_TURN_RIGHT 50
#define RIGHT_WHEEL_TURN_RIGHT 50
// command speeds for both wheels when turning left
#define LEFT_WHEEL_TURN_LEFT 50
#define RIGHT_WHEEL_TURN_LEFT 50

boolean turning = false;  // when false, this means the robot is moving forward


// TODO set this
// goal position relative to start (in meters)
#define GOAL_X 1.0
#define GOAL_Y 1.0

// uses a no-delay timer (cannot use delay() because TIMER1 is already in use)
unsigned long current_millis = 0;
unsigned long previous_millis = 0;

void setup()
{
  Serial.begin(57600);
  // Enable bump detection. Once a bump occurs, xl.checkBump() can be
  //  used to detect it. We'll use that to start moving.
  xl.enableBump();
  
  previous_millis = millis();
}

void loop()
{
  current_millis = millis();
  int delta_t = current_millis - previous_millis;
  // triggers start if accel has detected a bump
  if(xl.checkBump() && start == false) start = true;
  
  
  if(start) {
    
    // calculate omega when turning
    if(turning) {
      theta += ROTATIONAL_VELOCITY * delta_t;
      
    // calculate x and y when going forward
    } else {
      x += FORWARD_VELOCITY * delta_t * cos(theta);
      y += FORWARD_VELOCITY * delta_t * sin(theta);
    }
  }
  
  previous_millis = millis();
}
