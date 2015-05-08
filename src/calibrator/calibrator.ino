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


#define SENSOR_VALUE 400 // You'll need to adjust these for your surface.

// TODO actually calibrate robot and set these
// there will be only two commands: go forward (at a constant rate v (in m/s)) and turn about center (at a constant rate omega (in rad/s)).
#define FORWARD_VELOCITY 0.1111
#define ROTATIONAL_VELOCITY 0.0020420352

// command speeds for both wheels when going forward
#define LEFT_WHEEL_FORWARD 52
#define RIGHT_WHEEL_FORWARD 44
// command speeds for both wheels when turning right
#define LEFT_WHEEL_TURN_RIGHT 55
#define RIGHT_WHEEL_TURN_RIGHT -55
// command speeds for both wheels when turning left
#define LEFT_WHEEL_TURN_LEFT -55
#define RIGHT_WHEEL_TURN_LEFT 55

#define LEFT_LED 3
#define CENTER_LED 9
#define RIGHT_LED 10

unsigned long start_time = 0;
unsigned long current_time = 0;

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
}

// lights up LEDs based on sensor data
void sensor_led_debug(int left, int center, int right) {
  if(left > SENSOR_VALUE) digitalWrite(LEFT_LED, HIGH);
  else digitalWrite(LEFT_LED, LOW);
  
  if(center > SENSOR_VALUE) digitalWrite(CENTER_LED, HIGH);
  else digitalWrite(CENTER_LED, LOW);
  
  if(right > SENSOR_VALUE) digitalWrite(RIGHT_LED, HIGH);
  else digitalWrite(RIGHT_LED, LOW);
}

void loop()
{
  // triggers start if accel has detected a bump
  if(xl.checkBump() && start == false) start = true;
  
  // start
  if(start) {
    
    // sensor calibration
    while(true) {
      int left = lSen.read();
      int center = cSen.read();
      int right = rSen.read();

      Serial.print(left);
      Serial.print(" ");
      Serial.print(center);
      Serial.print(" ");
      Serial.print(right);
      Serial.println();
      
      delay(50);
      
      sensor_led_debug(left, center, right);
    }
    
    start_time = millis();
    current_time = millis();
    while(current_time - start_time < 1000) {
      // go forward
//      motor.rightDrive(RIGHT_WHEEL_FORWARD);
//      motor.leftDrive(LEFT_WHEEL_FORWARD);

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
