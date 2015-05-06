#define darkLevel  700  // You'll need to adjust these for your surface.

// Speed levels for looking for line and following once it's found. Go
//  too fast and you're liable to lose the line.
#define seekSpd    50
#define fwdSpd     90

void followNoCal()
{
  static boolean centerOnLine = false;
  static boolean onLine = false;
  
  int rLevel, lLevel, cLevel;
  
  // read sensors
  rLevel = rSen.read();
  lLevel = lSen.read();
  cLevel = cSen.read();

  // Go straight
  motor.rightDrive(46.27);
  motor.leftDrive(58);
  
  // if line on right, turn left
  if(rLevel >= darkLevel){
   motor.brake();
   delay(1000);
 
  while(rLevel >= darkLevel) {
    rLevel = rSen.read();
    lLevel = lSen.read();
    cLevel = cSen.read();
    motor.rightDrive(60);
    motor.leftDrive(-60);
  }
     delay(400);
     motor.brake();
     while(1);
    // if line on left, turn right
  } else if (lLevel >= darkLevel) {
    motor.brake();
    delay(1000);
    while(lLevel >= darkLevel) {
      rLevel = rSen.read();
      lLevel = lSen.read();
      cLevel = cSen.read();
      motor.rightDrive(-60);
      motor.leftDrive(60);
    }
    delay(400);
    motor.brake();
    while(1);
  }
 
}

