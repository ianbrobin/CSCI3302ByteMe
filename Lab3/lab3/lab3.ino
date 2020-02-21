#include <Sparki.h>
#include <math.h>

#define M_PI 3.14159
#define ROBOT_SPEED 0.0275 // meters/second
#define CYCLE_TIME .050 // Default 50ms cycle time
#define AXLE_DIAMETER 0.0857 // meters
#define WHEEL_RADIUS 0.03 // meters
#define CONTROLLER_FOLLOW_LINE 1
#define CONTROLLER_GOTO_POSITION_PART2 2
#define CONTROLLER_GOTO_POSITION_PART3 3
#define ROTATIONAL_VELOCITY 0.66
#define ULONG_MAX 9999999999

// Given defines...
#define FWD 1
#define NONE 0
#define BCK -1

// Our Defines from lab 2
#define FORWARD 0
#define RIGHT 1
#define LEFT 2

float p1 = .1;
float p2 = .1;
float p3 = .01;


int robot_motion=FORWARD;

// Line following configuration variables
const int threshold = 700;
int line_left = 1000;
int line_center = 1000;
int line_right = 1000;

// Controller and dTheta update rule settings
const int current_state = CONTROLLER_GOTO_POSITION_PART2;
//const int current_state = CONTROLLER_FOLLOW_LINE;

// Odometry bookkeeping
float orig_dist_to_goal = 0.0;
float pose_x = 0., pose_y = 0., pose_theta = 0.;
float dest_pose_x = 0., dest_pose_y = 0., dest_pose_theta = 0.;
float d_err = 0., b_err = 0., h_err = 0.; // Distance error (m), bearing error (rad), heading error (rad)
float phi_l = 0., phi_r = 0.; // Wheel rotation (radians)


// Wheel rotation vars
int left_speed_pct = 0.;
int right_speed_pct = 0.;
int left_dir = DIR_CCW;
int right_dir = DIR_CW;
int left_wheel_rotating = NONE;
int right_wheel_rotating = NONE;

// X and Theta Updates (global for debug output purposes)
// and their respective feedback controller gains
const float distance_gain = 1.;
const float theta_gain = 1.;
float dX  = 0., dTheta = 0.;

float to_radians(double deg) {
  return  deg * 3.1415/180.;
}

float to_degrees(double rad) {
  return  rad * 180 / 3.1415;
}

void setup() {
  pose_x = 0.;
  pose_y = 0.;
  pose_theta = 0.;
  left_wheel_rotating = NONE;
  right_wheel_rotating = NONE;

  // Set test cases here!
  set_pose_destination(0.15,0.05, to_radians(180));  // Goal_X_Meters, Goal_Y_Meters, Goal_Theta_Radians
}

// Sets target robot pose to (x,y,t) in units of meters (x,y) and radians (t)
void set_pose_destination(float x, float y, float t) {
  dest_pose_x = x;
  dest_pose_y = y;
  dest_pose_theta = t;
  if (dest_pose_theta > M_PI) dest_pose_theta -= 2*M_PI;
  if (dest_pose_theta < -M_PI) dest_pose_theta += 2*M_PI;
  orig_dist_to_goal = 0; // TODO
}

void readSensors() {
  line_left = sparki.lineLeft();
  line_right = sparki.lineRight();
  line_center = sparki.lineCenter();
}

void resetOdometry(){
  pose_x = 0;
  pose_y = 0;
  pose_theta = 0;  
}

void updateOdometry() {
  float deltaX = 0;
  float deltaY = 0;
  float deltaTheta = 0;
  
  float leftSpeed = (left_speed_pct / 100) * ROBOT_SPEED;
  float rightSpeed = (right_speed_pct / 100) * ROBOT_SPEED;
  float leftDist = leftSpeed * CYCLE_TIME;
  float rightDist = rightSpeed * CYCLE_TIME;

  deltaTheta = atan((leftDist - rightDist) / AXLE_DIAMETER);
  float deltaDist = (leftDist + rightDist) / 2;
  
  pose_theta += deltaTheta;
  
  deltaX = deltaDist * cos(pose_theta);
  deltaY = deltaDist * sin(pose_theta);

  pose_x += deltaX;
  pose_y += deltaY;

//  sparki.clearLCD();
//  sparki.println("Update Odometry");
//  sparki.print("deltaDist:");
//  sparki.println(deltaDist);
//  sparki.print("deltaTheta:");
//  sparki.println(deltaTheta);
//  sparki.updateLCD();
//  delay(1000);
}

void displayOdometry() {
  sparki.print("X: ");
  sparki.print(pose_x);
  sparki.print(" Xg: ");
  sparki.println(dest_pose_x);
  sparki.print("Y: ");
  sparki.print(pose_y);
  sparki.print(" Yg: ");
  sparki.println(dest_pose_y);
  sparki.print("T: ");
  sparki.print(to_degrees(pose_theta));
  sparki.print(" Tg: ");
  sparki.println(to_degrees(dest_pose_theta));

  sparki.print("dX : ");
  sparki.print(dX );
  sparki.print("   dT: ");
  sparki.println(dTheta);
  sparki.print("phl: "); sparki.print(phi_l); sparki.print(" phr: "); sparki.println(phi_r);
  sparki.print("p: "); sparki.print(d_err); sparki.print(" a: "); sparki.println(to_degrees(b_err));
  sparki.print("h: "); sparki.println(to_degrees(h_err));
}

float posError(){
  return sqrt(pow(dest_pose_x - pose_x, 2) + pow(dest_pose_y - pose_y, 2));
}

float bearingError(){
  return atan( (dest_pose_y - pose_y) / (dest_pose_x - pose_x) );
}

float headingError(){
  return dest_pose_theta - pose_theta;  
}

void rotateMotors(int rightSpeed, int leftSpeed){
  left_speed_pct = leftSpeed;
  right_speed_pct = rightSpeed;
  
  if (rightSpeed > 0){
    right_wheel_rotating = FWD;
    right_dir = DIR_CW;
  }
  else{
    right_wheel_rotating = BCK;
    rightSpeed *= -1;
    right_dir = DIR_CCW;
  }
  if (leftSpeed > 0){
    left_wheel_rotating = FWD;
    left_dir = DIR_CCW;
  }
  else{
    left_wheel_rotating = BCK;
    leftSpeed *= -1;
    left_dir = DIR_CW;
  }

  /*
  sparki.clearLCD();
  sparki.print("left speed:");
  sparki.println(leftSpeed);
  sparki.print("Right speed:");
  sparki.println(rightSpeed);
  sparki.print("left speed pct:");
  sparki.println(left_speed_pct);
  sparki.print("Right speed pct:");
  sparki.println(right_speed_pct);
  sparki.updateLCD();
  */
  sparki.motorRotate(MOTOR_LEFT, left_dir, leftSpeed, ULONG_MAX);
  sparki.motorRotate(MOTOR_RIGHT, right_dir, rightSpeed, ULONG_MAX);
  
}

void loop() {
  unsigned long begin_time = millis();
  unsigned long end_time = 0;
  unsigned long delay_time = 0;

  float deltaDist = 0;
  float deltaTheta = 0;
  float ld = 0;
  float rd = 0;
  float lp = 0;
  float rp = 0;

  float maxDist = ROBOT_SPEED * CYCLE_TIME;
  float maxTheta = (2 * ROBOT_SPEED * CYCLE_TIME) / AXLE_DIAMETER;

  switch (current_state) {
    case CONTROLLER_FOLLOW_LINE:
      // Useful for testing odometry updates
      readSensors();
      if (line_center < threshold) {
        robot_motion = FORWARD;
        sparki.moveForward();
      } else if (line_left < threshold) {
        robot_motion = LEFT;
        sparki.moveLeft();
      } else if (line_right < threshold) {
        robot_motion = RIGHT;
        sparki.moveRight();
      } else {
        sparki.moveStop();
      }

      // Check for start line, use as loop closure
      if (line_left < threshold && line_right < threshold && line_center < threshold) {
        pose_x = 0.;
        pose_y = 0.;
        pose_theta = 0.;
      }
      
      break;
    case CONTROLLER_GOTO_POSITION_PART2:
      // TODO: Implement solution using moveLeft, moveForward, moveRight functions
      // This case should arrest control of the program's control flow (taking as long as it needs to, ignoring the 100ms loop time)
      // and move the robot to its final destination
      
      sparki.moveLeft(to_degrees(bearingError()));
      sparki.moveForward(posError() * 100);
      sparki.moveLeft(to_degrees(headingError() - bearingError()));
      sparki.moveStop();
      pose_x = dest_pose_x;
      pose_y = dest_pose_y;
      pose_theta = dest_pose_theta;
      break;
    case CONTROLLER_GOTO_POSITION_PART3:
      deltaDist = p1 * posError();
      deltaTheta = p2 * bearingError() + p3 * headingError();
      // Normalize deltaDist and deltaTheta
      if (deltaDist > maxDist)
        deltaDist = maxDist;
      if (deltaTheta > maxTheta)
        deltaTheta = maxTheta;
        
      ld = (deltaDist * 2 + AXLE_DIAMETER * deltaTheta) / 2;
      rd = (deltaDist * 2 - AXLE_DIAMETER * deltaTheta) / 2;
      lp = (100 * ld) / (CYCLE_TIME * ROBOT_SPEED);
      rp = (100 * rd) / (CYCLE_TIME * ROBOT_SPEED);
      
      sparki.clearLCD();
      sparki.println("In Loop");
      sparki.print("deltaDist:");
      sparki.println(deltaDist * 100);
      sparki.print("deltaTheta:");
      sparki.println(deltaTheta * 100);

      sparki.print("X: ");
      sparki.print(pose_x);
      sparki.print(" Xg: ");
      sparki.println(dest_pose_x);
      sparki.print("Y: ");
      sparki.print(pose_y);
      sparki.print(" Yg: ");
      sparki.println(dest_pose_y);
      sparki.print("T: ");
      sparki.print(to_degrees(pose_theta));
      sparki.print(" Tg: ");
      sparki.println(to_degrees(dest_pose_theta));
      
      sparki.updateLCD();
      
      rotateMotors((int) rp, (int) lp);
      break;
  }
  updateOdometry();
  sparki.clearLCD();
  displayOdometry();
  sparki.updateLCD();

  end_time = millis();
  delay_time = end_time - begin_time;
  if (delay_time < 1000*CYCLE_TIME)
    delay(1000*CYCLE_TIME - delay_time); // each loop takes CYCLE_TIME ms
  else
    delay(10);
}
