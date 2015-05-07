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

#define SENSOR_VALUE 240 // You'll need to adjust these for your surface.

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

enum movement_states {
  turning_left,
  turning_right,
  forward,
  stopped
};

enum bug_states {
  heading_to_goal,
  following_line
};

// start off by heading straight to goal
bug_states bug_state = heading_to_goal;
// start off being stopped
movement_states movement_state = stopped;

// TODO set this
// goal position relative to start (in meters)
#define GOAL_X 1.0
#define GOAL_Y 1.0

unsigned long current_millis = 0;
unsigned long previous_millis = 0;

// threshold values (to allow for errors in measurements)
#define GOAL_ANGLE_THRESHOLD 0.017453292519  // 1 degree expressed in radians



void setup()
{
  Serial.begin(57600);
  // Enable bump detection. Once a bump occurs, xl.checkBump() can be
  //  used to detect it. We'll use that to start moving.
  xl.enableBump();
  
  previous_millis = millis();
}

// returns angle from corrent orientation to goal
float get_goal_angle() {
  // equation for phi is inversetan of (y2-y1) / (x2-x1)
  float phi = atan((GOAL_Y - y) / (GOAL_X - x));
  return phi - theta;
}

// returns true if we need to turn right to stay on line, false otherwise
// if only the left sensor is on (sees something), then we don't need to turn. Otherwise, we do.
boolean need_to_turn_right() {
  // if center or right sensors see something, we need to turn right
  if(cSen.read() > SENSOR_VALUE || rSen.read() > SENSOR_VALUE) return true;
  // else if center and right don't see anything, (regardless of left), return false
  else return false;
}


// returns true if we lost the line (not on any of our sensors), false otherwise
boolean check_lost_line() {
  if(lSen.read() <= SENSOR_VALUE && cSen.read() <= SENSOR_VALUE && rSen.read() <= SENSOR_VALUE)
    return true;
  return false;
}

void loop()
{
  current_millis = millis();
  int delta_t = current_millis - previous_millis;
  // triggers start if accel has detected a bump
  if(xl.checkBump() && start == false) start = true;
  
  // start
  if(start) {
    
    if(bug_state == heading_to_goal) {
      float goal_angle = get_goal_angle();
      
      // if we are off from the goal by threshold, turn towards goal
      if(abs(goal_angle) > GOAL_ANGLE_THRESHOLD) {
        if(goal_angle > 0) {
          // TODO turn left
          movement_state = turning_left;
        } else {
          // TODO turn right
          movement_state = turning_right;
        }
        
      // else we are pointing at goal, then go forward
      } else {
        // TODO go forward
        movement_state = forward;
      }
      
    // else we are following a line (tracing an obstacle outline)
    } else if(bug_state == following_line) {
      // see if we need to turn right to stay on object
      if(need_to_turn_right()) {
        // TODO turn right
        movement_state = turning_right;
        
      // otherwise, just go forward until you can reach the goal or lost the line
      } else {
        
        // TODO go forward
        movement_state = forward;
        
        // if we can reach the goal (it's on our left), go towards goal
        if(get_goal_angle() < 0) {
          bug_state = heading_to_goal;
          // TODO stop robot
          movement_state = stopped;
          
          
        // else if we lost the line, turn left to find it again
        } else if (check_lost_line()) {
          // TODO turn left
          movement_state = turning_left;
        }
        
        // otherwise keep following line by going forward
        
      }
      
    }
    
    
    
    
    // TODO fix turning directions
    // calculate odometry according to movement_state (regardless of bug_state)
    // calculate omega when turning left
    if(movement_state == turning_left) {
      theta += ROTATIONAL_VELOCITY * delta_t;
    // calculate omega when turning right
    } else if(movement_state == turning_right) {
      theta -= ROTATIONAL_VELOCITY * delta_t;
    // calculate x and y when going forward
    } else {
      x += FORWARD_VELOCITY * delta_t * cos(theta);
      y += FORWARD_VELOCITY * delta_t * sin(theta);
    }
    
    
    
  }
  
  previous_millis = millis();
}
