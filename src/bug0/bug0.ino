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
double x = 0;      // in mm
double y = 0;      // in mm
double theta = 0;  // in radians

// TODO actually calibrate robot and set these
// there will be only two commands: go forward (at a constant rate v (in mm/ms)) and turn about center (at a constant rate omega (in rad/ms)).
#define FORWARD_VELOCITY 0.086
#define ROTATIONAL_VELOCITY 0.00094247779607
// command speeds for both wheels when going forward
#define LEFT_WHEEL_FORWARD 50
#define RIGHT_WHEEL_FORWARD 46
// command speeds for both wheels when turning right
#define LEFT_WHEEL_TURN_RIGHT 50
#define RIGHT_WHEEL_TURN_RIGHT -50
// command speeds for both wheels when turning left
#define LEFT_WHEEL_TURN_LEFT -50
#define RIGHT_WHEEL_TURN_LEFT 50
// goal position relative to start (in mm)
#define GOAL_X 500
#define GOAL_Y 500
// threshold values (to allow for errors in measurements)
#define GOAL_ANGLE_THRESHOLD 0.0523598776  // 3 degrees expressed in radians
#define GOAL_THRESHOLD  50  // distance threshold within goal in mm
#define SENSOR_VALUE 240 // You'll need to adjust these for your surface.

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

unsigned long current_millis = 0;
unsigned long previous_millis = 0;

/* Assumptions for coordinate system:
 *
 * positive x-axis is towards the right
 * positive y-axis is towards the top
 * positive theta orientation is towards counter-clockwise (turn left about center)
 * theta starts at 0 pointing with the x-axis
 *
 * All units are: distance in mm, velocity in mm/ms, angles in radians, rotational velocity in rad/ms
 */


void setup()
{
  Serial.begin(57600);
  // Enable bump detection. Once a bump occurs, xl.checkBump() can be
  //  used to detect it. We'll use that to start moving.
  xl.enableBump();
  
  pinMode(13, OUTPUT);
  previous_millis = millis();
}

// returns angle from corrent orientation to goal
double get_goal_angle() {
  // equation for phi is inversetan of (y2-y1) / (x2-x1)
  double phi = atan((double)(GOAL_Y - y) / (double)(GOAL_X - x));
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

// returns true if close enough to goal within threshold, false otherwise
bool found_goal() {
  if(abs(x - GOAL_X) < GOAL_THRESHOLD && abs(y - GOAL_Y) < GOAL_THRESHOLD)
    return true;
  return false;
}

void loop()
{
  current_millis = millis();
  double delta_t = (double)(current_millis - previous_millis);  // in milliseconds
  
  // triggers start if accel has detected a bump
  if(xl.checkBump() && start == false) start = true;
  
  // start
  if(start) {
    
    if(bug_state == heading_to_goal) {
      double goal_angle = get_goal_angle();
      Serial.println(goal_angle);
      
      // if we found goal, stop motors and enter infinite loop (turn on LED 13)
      if(found_goal()) {
        digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
        motor.brake();
        while(true);
      // if we are off from the goal by threshold, turn towards goal
      } else if(abs(goal_angle) > GOAL_ANGLE_THRESHOLD) {
        if(goal_angle > 0) {
          // turn left
          motor.rightDrive(RIGHT_WHEEL_TURN_LEFT);
          motor.leftDrive(LEFT_WHEEL_TURN_LEFT);
          movement_state = turning_left;
        } else {
          // turn right
          motor.rightDrive(RIGHT_WHEEL_TURN_RIGHT);
          motor.leftDrive(LEFT_WHEEL_TURN_RIGHT);
          movement_state = turning_right;
        }
        
      // else we are pointing at goal, then go forward
      } else {
        // go forward
        motor.rightDrive(RIGHT_WHEEL_FORWARD);
        motor.leftDrive(LEFT_WHEEL_FORWARD);
        movement_state = forward;
      }
      
    // else we are following a line (tracing an obstacle outline)
    } else if(bug_state == following_line) {
      // see if we need to turn right to stay on object
      if(need_to_turn_right()) {
        // turn right
        motor.rightDrive(RIGHT_WHEEL_TURN_RIGHT);
        motor.leftDrive(LEFT_WHEEL_TURN_RIGHT);
        movement_state = turning_right;
        
      // otherwise, just go forward until you can reach the goal or lost the line
      } else {
        
        // go forward
        motor.rightDrive(RIGHT_WHEEL_FORWARD);
        motor.leftDrive(LEFT_WHEEL_FORWARD);
        movement_state = forward;
        
        // if we can reach the goal (it's on our left), go towards goal
        if(get_goal_angle() < 0) {
          bug_state = heading_to_goal;
          // stop robot
          motor.brake();
          movement_state = stopped;
          
          
        // else if we lost the line, turn left to find it again
        } else if (check_lost_line()) {
          // turn left
          motor.rightDrive(RIGHT_WHEEL_TURN_LEFT);
          motor.leftDrive(LEFT_WHEEL_TURN_LEFT);
          movement_state = turning_left;
        }
        
        // otherwise keep following line by going forward
        
      }
      
    }
    
    
    
    
    // calculate odometry according to movement_state (regardless of bug_state)
    // calculate omega when turning left
    if(movement_state == turning_left) {
      theta += ROTATIONAL_VELOCITY * delta_t;
    // calculate omega when turning right
    } else if(movement_state == turning_right) {
      theta -= ROTATIONAL_VELOCITY * delta_t;
    // calculate x and y when going forward
    } else {
      double delta_x = FORWARD_VELOCITY * delta_t * cos(theta);
      double delta_y = FORWARD_VELOCITY * delta_t * sin(theta);

      x += delta_x;
      y += delta_y;
    }
    
    
    
  }
  
  Serial.println(x);
  Serial.println(y);
  Serial.println(theta);
  Serial.println(movement_state);
  previous_millis = current_millis;
}
