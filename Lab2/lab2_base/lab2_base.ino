#include <Sparki.h>

#define CYCLE_TIME .100  // seconds

// Program States
#define CONTROLLER_FOLLOW_LINE 1
#define CONTROLLER_DISTANCE_MEASURE 2


int current_state = CONTROLLER_FOLLOW_LINE; // Change this variable to determine which controller to run
const int threshold = 700;
int line_left = 1000;
int line_center = 1000;
int line_right = 1000;

float pose_x = 0., pose_y = 0., pose_theta = 0.;

void setup() {
  pose_x = 0.;
  pose_y = 0.;
  pose_theta = 0.;
}

void readSensors() {
  line_left = sparki.lineLeft();
  line_right = sparki.lineRight();
  line_center = sparki.lineCenter();
  // distance = sparki.ping();
}


void followLine() {
  readSensors();
 
  if ( line_left < threshold ) // if line is below left line sensor
  {  
    sparki.moveLeft(); // turn left
  }
 
  if ( line_right < threshold ) // if line is below right line sensor
  {  
    sparki.moveRight(); // turn right
  }
 
  // if the center line sensor is the only one reading a line
  if ( (line_center < threshold) && (line_left > threshold) && (line_right > threshold) )
  {
    sparki.moveForward(); // move forward
  }  
 
  sparki.clearLCD(); // wipe the screen
 
  sparki.print("Line Left: "); // show left line sensor on screen
  sparki.println(line_left);
 
  sparki.print("Line Center: "); // show center line sensor on screen
  sparki.println(line_center);
 
  sparki.print("Line Right: "); // show right line sensor on screen
  sparki.println(line_right);
 
  sparki.updateLCD(); // display all of the information written to the screen
}


void measure_30cm_speed() {
  unsigned long startTime = millis();
  sparki.moveForward(30);
  unsigned long endTime = millis();

  sparki.clearLCD();
  sparki.println("Time Taken:");
  sparki.println(endTime - startTime);
  sparki.updateLCD();
}


void updateOdometry() {
  // TODO
}

void displayOdometry() {
  // TODO
}

void loop() {

  // TODO: Insert loop timing/initialization code here
  unsigned long startTime = millis();
  
  switch (current_state) {
    case CONTROLLER_FOLLOW_LINE: {
      followLine();
      break;
    }
    case CONTROLLER_DISTANCE_MEASURE: {
      measure_30cm_speed();
      current_state++;
      break;
    }
  }

  unsigned long endTime = millis();

  // Ensure loop lasts 100ms every loop
  delay(1000*CYCLE_TIME - (endTime - startTime));
}
