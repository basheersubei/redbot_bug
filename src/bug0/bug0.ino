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

#define SENSOR_VALUE 400 // You'll need to adjust these for your surface.
// TODO actually calibrate robot and set these
// there will be only two commands: go forward (at a constant rate v (in m/s)) and turn about center (at a constant rate omega (in rad/s)).
#define FORWARD_VELOCITY 0.1111
#define ROTATIONAL_VELOCITY 0.0020420352
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
#define GOAL_X 250
#define GOAL_Y 0
// threshold values (to allow for errors in measurements)
#define GOAL_ANGLE_THRESHOLD 0.0523598776  // 3 degrees expressed in radians
#define GOAL_THRESHOLD  50  // distance threshold within goal in mm

#define DEBOUNCE_THRESHOLD 10  // sensor debouncing time interval in ms


#define LEFT_LED 3
#define CENTER_LED 9
#define RIGHT_LED 10

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
unsigned long last_debounced = 0;
bool debouncing = false;

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
  pinMode(LEFT_LED, OUTPUT);
  pinMode(CENTER_LED, OUTPUT);
  pinMode(RIGHT_LED, OUTPUT);
  previous_millis = millis();
}

// lights up LEDs based on sensor data
void sensor_led_debug() {
  if(lSen.read() > SENSOR_VALUE) digitalWrite(LEFT_LED, HIGH);
  else digitalWrite(LEFT_LED, LOW);
  
  if(cSen.read() > SENSOR_VALUE) digitalWrite(CENTER_LED, HIGH);
  else digitalWrite(CENTER_LED, LOW);
  
  if(rSen.read() > SENSOR_VALUE) digitalWrite(RIGHT_LED, HIGH);
  else digitalWrite(RIGHT_LED, LOW);
}

// returns angle from corrent orientation to goal
double get_goal_angle() {
  // equation for phi is inversetan of (y2-y1) / (x2-x1)
  double phi = atan((double)(GOAL_Y - y) / (double)(GOAL_X - x));
  return phi - theta;
}

// TODO need to add debouncing for sensors
// returns true if we need to turn right to stay on line, false otherwise
// if only the left sensor is on (sees something), then we don't need to turn. Otherwise, we do.
boolean need_to_turn_right() {
  // if we're not currently debouncing, then start debouncing
//  if(!debouncing) {
//    last_debounced = millis();
//    debouncing = true;
//    return false;
//  }
  
  // if it hasn't been that long (sensors haven't stabilized yet), return false
//  if(millis() - last_debounced < DEBOUNCE_THRESHOLD) {
//    return false;
//  // if center or right sensors see something, we need to turn right
//  } else 
  if(cSen.read() > SENSOR_VALUE || rSen.read() > SENSOR_VALUE) {
//    last_debounced = millis();
//    debouncing = false;
//    digitalWrite(13, HIGH-digitalRead(13));   // blink the LED
    return true;
  // else if center and right don't see anything, (regardless of left), return false
  } else {
    return false;
  }
  
}

// TODO need to add debouncing for sensors
// returns true if we lost the line (not on any of our sensors), false otherwise
boolean check_lost_line() {
  // if we're not currently debouncing, then start debouncing
//  if(!debouncing) {
//    last_debounced = millis();
//    debouncing = true;
//    return false;
//  }

  // if it hasn't been that long, return false
//  if(millis() - last_debounced < DEBOUNCE_THRESHOLD) {
//    return false;
//  // otherwise, check if we lost line
//  } else 
  if(lSen.read() <= SENSOR_VALUE && cSen.read() <= SENSOR_VALUE && rSen.read() <= SENSOR_VALUE) {
//    last_debounced = millis();
//    debouncing = false;
    return true;
  // we have debounced (stabilized our sensors) but we haven't lost the line  
  } else {
    return false;
  }
  
}

// returns true if any sensors see anything, false otherwise
bool check_for_obstacles() {
  // if we're not currently debouncing, then start debouncing
//  if(!debouncing) {
//    last_debounced = millis();
//    debouncing = true;
//    return false;
//  }

  // if it hasn't been that long, return false
//  if(millis() - last_debounced < DEBOUNCE_THRESHOLD) {
//    return false;
//  // otherwise, check if sensors see any lines
//  } else 
  if(lSen.read() > SENSOR_VALUE || cSen.read() > SENSOR_VALUE || rSen.read() > SENSOR_VALUE) {
//    last_debounced = millis();
//    debouncing = false;
    return true;
  // we have debounced (stabilized our sensors) but we don't see a line
  } else {
    return false;
  }
  
}

// returns true if close enough to goal within threshold, false otherwise
bool found_goal() {    
  if(abs(x - GOAL_X) < GOAL_THRESHOLD && abs(y - GOAL_Y) < GOAL_THRESHOLD)
    return true;
  return false;
}

void head_to_goal() {
  double goal_angle = get_goal_angle();
  Serial.println(goal_angle);
  
  // if we found goal, stop motors and enter infinite loop (turn on LED 13)
  if(found_goal()) {
    motor.brake();
    while(true) {
      digitalWrite(13, HIGH-digitalRead(13));   // blink the LED
      delay(100);
    }
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
}

void follow_line() {
  // see if we need to turn right to stay on object
  if(need_to_turn_right()) {
    // turn right
    motor.rightDrive(RIGHT_WHEEL_TURN_RIGHT);
    motor.leftDrive(LEFT_WHEEL_TURN_RIGHT);
    movement_state = turning_right;
    
    
  // otherwise go forward and check if we either can reach the goal or we lost the line while tracing it
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
      bug_state = heading_to_goal;
      
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

void update_odometry() { 
  double delta_t = (double)(current_millis - previous_millis);  // in milliseconds
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

void loop()
{
  
  sensor_led_debug();
  current_millis = millis();
  // triggers start if accel has detected a bump
  if(xl.checkBump() && start == false) start = true;
  
  // start
  if(start) {
    
    // if we are heading to goal
    if(bug_state == heading_to_goal) {
      // head to goal if there are no obstacles
      if(!check_for_obstacles())
        head_to_goal();
      // change state to following line if you see obstacles
      else {
        bug_state = following_line;
        // stop robot
        motor.brake();
        movement_state = stopped;
      }
    // else we are following a line (tracing an obstacle outline)
    } else if(bug_state == following_line) {
      follow_line();
      
    // sanity check
    } else {
      motor.brake();
      while(true) {
        digitalWrite(13, HIGH-digitalRead(13));   // blink the LED
        delay(500);
      }
    }
    
    // update odometry based on the choices it made above
    update_odometry();
  }
  
  Serial.print("x:\t"); Serial.print(x); Serial.print("\t"); 
  Serial.print("y:\t"); Serial.print(y); Serial.print("\t"); 
  Serial.print("theta:\t"); Serial.print(theta); Serial.print("\t");
  Serial.print("m_state\t:");Serial.print(movement_state); Serial.print("\t");
  Serial.println();
  previous_millis = current_millis;
}
