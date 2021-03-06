#include <DmxSimple.h>

#define CH_X_GROSS 1
#define CH_X_FINE 2
#define CH_Y_GROSS 3
#define CH_Y_FINE 4
#define CH_SPEED 5
#define CH_DIMMING 6
#define CH_STROBE 7
#define CH_LED_RED 8
#define CH_LED_GREEN 9 
#define CH_LED_BLUE 10
#define CH_LED_WHITE 11
#define CH_RESET 13

#define X_FINE_STEPS_PER_GROSS_STEP 105
#define X_DEGREES_PER_GROSS_STEP (540 / 255.0)
#define X_WORLD_ZERO_IN_DEGREES 90
//TODO: check Y fine steps per gross
#define Y_FINE_STEPS_PER_GROSS_STEP 105
#define Y_DEGREES_PER_GROSS_STEP (180 / 205.0)
#define Y_WORLD_ZERO_IN_DEGREES 25

#define Y_FINE_TO_X_FINE_RATIO 33/11.75

#define MIN_X_GROSS 0
#define MAX_X_GROSS 85
#define MIN_Y_GROSS 0
#define MAX_Y_GROSS 131

void resetLamp() {
  DmxSimple.write(CH_RESET,255);
  delay(3000);
}

void setup() {
  Serial.begin(115200);
  
  DmxSimple.usePin(3);
  //resetLamp();
  lightsOn();
  //while (true) {}
  gotoXY(0,0);
  delay(1500);
}
//
//int fromDegreesX(float degrees) {
//  return round(degrees/);
//}
//
//int fromDegreesY(float degrees) {
//  return round(degrees/);
//}
//
//int convertWorldDegreesX(float degrees) {
//  return degrees + 90;
//}
//
//int convertWorldDegreesY(float degrees) {
//  return degrees + 29;
//}

int xGross, xFine, yGross, yFine;

void setXGross(int x) {
  xGross = x;
  DmxSimple.write(CH_X_GROSS, xGross);
}

int getXGross() {
  return xGross;
}

void setYGross(int y) {
  yGross = y;
  DmxSimple.write(CH_Y_GROSS, yGross);
}

int getYGross() {
  return yGross;
}

void setXFine(int x) {
  xFine = x;
  DmxSimple.write(CH_X_FINE, xFine);
}

int getXFine() {
  return xFine;
}

void setYFine(int y) {
  yFine = y;
  DmxSimple.write(CH_Y_FINE, yFine);  
}

int getYFine() {
  return yFine;
}

void moveIncremental(int x, int y) {
  //TODO: take care of proper behavior when hitting the limits of gross motion.                       

  int newXFine = (getXFine() + x) % X_FINE_STEPS_PER_GROSS_STEP;
  int newXGross = getXGross() + (getXFine() + x) / X_FINE_STEPS_PER_GROSS_STEP;   
  while (newXFine<0) {
    newXGross--;
    newXFine+=X_FINE_STEPS_PER_GROSS_STEP;
  }

  int newYFine = (getYFine() + y) % Y_FINE_STEPS_PER_GROSS_STEP;
  int newYGross = getYGross() + (getYFine() + y) / Y_FINE_STEPS_PER_GROSS_STEP;   
  while (newYFine<0) {
    newYGross--;
    newYFine+=Y_FINE_STEPS_PER_GROSS_STEP;
  }

  setXFine(newXFine);
  setXGross(min(MAX_X_GROSS,max(MIN_X_GROSS,newXGross)));
  setYFine(newYFine);
  setYGross(min(MAX_Y_GROSS,max(MIN_Y_GROSS,newYGross)));
}


/**
 * goto given coordinate (x,y) given in degrees in world coordinate system (where 0,0 is the lamp pointing straight ahead (y level with ground) at a -90 degree offset on X from home).
 */
void gotoXY(double x, double y) {
  int xGross, xFine, yGross, yFine;
  double xSteps = (X_WORLD_ZERO_IN_DEGREES - x) / X_DEGREES_PER_GROSS_STEP;
  double ySteps = (Y_WORLD_ZERO_IN_DEGREES + y) / Y_DEGREES_PER_GROSS_STEP;

  xGross = floor(xSteps);
  xFine = (xSteps-xGross) * X_FINE_STEPS_PER_GROSS_STEP;

  yGross = floor(ySteps);
  yFine = (ySteps-yGross) * Y_FINE_STEPS_PER_GROSS_STEP;

  Serial.print("Going to [");
  Serial.print(x,2);
  Serial.print(",");
  Serial.print(y,2);
  Serial.print("] : [");
  Serial.print(xGross);
  Serial.print(":");
  Serial.print(xFine);
  Serial.print(",");
  Serial.print(yGross);
  Serial.print(":");
  Serial.print(yFine);
  Serial.println("]");

  setXGross(xGross);
  setXFine(xFine);
  setYGross(yGross);
  setYFine(yFine);  
}

void lightsOn() {
  DmxSimple.write(6, 255);
  DmxSimple.write(8, 255);
  DmxSimple.write(9, 255);
  DmxSimple.write(10, 255);
  DmxSimple.write(11, 255);  
}

void lightsOff() {
  DmxSimple.write(6, 0);
}

String inString = "";    // string to hold input
void handleInput() {
  while (Serial.available() > 0) {
    int inChar = Serial.read();
    inString += (char)inChar;

    if (inChar == '\n') {
      if ('c'==inString[0]) {
        gotoXY(0,0);
      } else if ('x'==inString[0]) {
        int steps = inString.substring(1).toInt();
        moveIncremental(steps, 0);
      } else if ('y'==inString[0]) {
        int steps = inString.substring(1).toInt();
        moveIncremental(0, steps);        
      } else {
        Serial.print(" Unknown command: '");
        Serial.print(inString);
        Serial.println("'");
      }
      // clear the string for new input:
      inString = "";
    }
  }
}

void loop() {
  handleInput();
}
