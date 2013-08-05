#include <ZumoMotors.h>
#include <LSM303.h>
#include <Pushbutton.h>
#include <Wire.h>
#include "structs.h"
#include <math.h>

#define LED_PIN 13
#define CRA_REG_M_220HZ 0x1C    // CRA_REG_M value for magnetometer 220 Hz update rate

int start_heading = 0;

ZumoMotors motors;
LSM303 compass;
Pushbutton button(ZUMO_BUTTON);

void setup()
{
  int calibration_speed = 200;
  int calibration_samples = 70;
  unsigned char index;
  
  pinMode(LED_PIN, OUTPUT);
  
  // The highest possible magnetic value to read in any direction is 2047
  // The lowest possible magnetic value to read in any direction is -2047
  LSM303::vector running_min = {2047, 2047, 2047}, running_max = {-2048, -2048, -2048};

  Serial.begin(9600);

  // Initiate the Wire library and join the I2C bus as a master
  Wire.begin();

  // Initiate LSM303
  compass.init();

  // Enables accelerometer and magnetometer
  compass.enableDefault();

  compass.setMagGain(compass.magGain_25);                  // +/- 2.5 gauss sensitivity to hopefully avoid overflow problems
  compass.writeMagReg(LSM303_CRA_REG_M, CRA_REG_M_220HZ);  // 220 Hz compass update rate

  float min_x_avg[calibration_samples];
  float min_y_avg[calibration_samples];
  float max_x_avg[calibration_samples];
  float max_y_avg[calibration_samples];

  button.waitForButton();
  Serial.println("setting up...");
  // To calibrate the magnetometer, the Zumo spins to find the max/min
  // magnetic vectors. This information is used to correct for offsets
  // in the magnetometer data.
  
  motors.setLeftSpeed(calibration_speed);
  motors.setRightSpeed(-calibration_speed);

  for(index = 0; index < calibration_samples; index ++)
  {
    // Take a reading of the magnetic vector and store it in compass.m
    compass.read();
    
    running_min.x = min(running_min.x, compass.m.x);
    running_min.y = min(running_min.y, compass.m.y);
    running_max.x = max(running_max.x, compass.m.x);
    running_max.y = max(running_max.y, compass.m.y);
    delay(50);
  }

  motors.setLeftSpeed(0);
  motors.setRightSpeed(0);
  // Set calibrated values to compass.m_max and compass.m_min
  compass.m_max.x = running_max.x;
  compass.m_max.y = running_max.y;
  compass.m_min.x = running_min.x;
  compass.m_min.y = running_min.y;

  Serial.println("waiting for button...");
  button.waitForButton();
  start_heading = averageHeading();

}

/*
 * Returns the zero-d out "map" that the robot will draw on
 */
int**
get_mat() {
  /*
   * area_mat is a bit matrix corresponding to 1cm square areas.
   * If there's a "1" in area_mat, then the robot will draw there
   */
  int** area_mat;
  int i;
  int j;
  int size;
  
  size = 10;
   Serial.println("getting matrix...");
   area_mat = new int*[size];
   //only need 10 * 10 bits but idk how to make this work now so wasting a shitton of space
  for(i = 0; i < size; i ++) {
    area_mat[i] = (int*)calloc(size, sizeof(int));  //should init to zero, riiight?
  }
  return area_mat;
}



/*
 * Fill in the desired pattern in the area matrix
 * TODO: GUI
 */
void
fill_pattern(int** area_mat) {
  //for now, hard-code in a pattern
  /* vertical line
  for(int i = 0; i < 10; i ++) {
    area_mat[i][0] = 1;
  }
  */
  /* horizontal line
  for(int j = 0; j < 10; j ++) {
    area_mat[0][j] = 1;
  }
  */
  /* zig zag */
  area_mat[0][0] = 1;
  area_mat[1][0] = 1;
  area_mat[2][0] = 1;
  area_mat[3][0] = 1;
  area_mat[3][1] = 1;
  area_mat[3][2] = 1;
  area_mat[3][3] = 1;
  area_mat[4][3] = 1;
  area_mat[5][3] = 1;
  area_mat[6][3] = 1;
  area_mat[6][4] = 1;
  area_mat[6][5] = 1;
  area_mat[6][6] = 1;
  area_mat[7][6] = 1;
  area_mat[8][6] = 1;
  area_mat[9][6] = 1;
  area_mat[9][7] = 1;
  area_mat[9][9] = 1;

  //print matrix
  for(int i = 0; i < 10; i ++) {
    for(int j=0; j < 10; j++) {
      Serial.print(area_mat[i][j]);
    }
    Serial.println("");
  }
  return;
}

/*
 * Sends the robot from curr_pos to end_pos
 * Requires: start_pos and end_pos are valid positions,
 */
void
send_to_pos(pos start_pos, pos end_pos) {
  int dist;
  int rotate_degrees;
  int curr_dir;
  int target_dir;
  int deviation_threshold = 5;
  int turn_speed = 100;
  
  Serial.print("sending from ");
  Serial.print(pos_string(start_pos));
  Serial.print(" to ");
  Serial.println(pos_string(end_pos));
  //get data needed for moving there
  dist = get_distance(start_pos, end_pos);
  rotate_degrees = angle_from_zero(start_pos, end_pos);
  Serial.print("rotating degress: ");
  Serial.println(rotate_degrees); 
  curr_dir = averageHeading();
  target_dir = start_heading + rotate_degrees;
  Serial.print("avg heading: ");
  Serial.print(curr_dir);
  Serial.print(", target heading: ");
  Serial.println(target_dir);  
  //turn!
  Serial.println("turning...");
  turnToHeading(target_dir);
  Serial.println("done turning...");
  
  //move!
  move_forward(dist);
 
  return;
}

/*
 * gets the angle from the positive y-axis to turn from curr_pos to end_pos
 */
float
angle_from_zero(pos curr_pos, pos end_pos) {
  float vert_dist;
  float total_dist;
  
  vert_dist = end_pos.y - curr_pos.y;
  total_dist = get_distance(curr_pos, end_pos);
  float degrees = acos(vert_dist/total_dist) * 180 / 3.14159265359;
  return degrees;
}

float
get_distance(pos from_pos, pos to_pos) {
  return sqrt(pow(to_pos.x - from_pos.x, 2) + pow(to_pos.y - from_pos.y, 2));
}

/*
 * Moves the robot dist cm forward
 * TODO: Tune this to use accelerometer? / Something more accurate
 * than hard-coded ms_per_cm value
 */
void
move_forward(int dist) {
  int speed;
  int ms_per_cm;
  
  Serial.print("moving forward ");
  Serial.print(dist);
  Serial.println("cm");
  speed = 100;
  ms_per_cm = 1000;
  motors.setSpeeds(speed, speed);
  delay(ms_per_cm * dist); //for now just assume constant time
  motors.setSpeeds(0, 0);
  return;
}


/*
 * Returns the closest position yet to be filled in area_mat
 * 
 * Requires: area_mat valid bit matrix,
 * area_mat is square
 */
pos 
find_closest_pos(int** area_mat, int area_mat_size, pos curr_pos) {
  float min_dist = 100000;
  pos min_pos;
  int found_pos = 0;
  Serial.println("finding closest position");
  /* this is inefficient and assumes area_mat is square */
  for(int i = 0; i < area_mat_size; i ++) {
    for(int j = 0; j < area_mat_size; j++) {
     if(area_mat[i][j] == 1) {
        pos new_pos;
        new_pos.x = j;
        new_pos.y = i;
        float new_dist = get_distance(curr_pos, new_pos);
        if(new_dist < min_dist) {
          found_pos = 1;
          min_pos = new_pos;
          min_dist = new_dist;
        }
     }
   }
  }
  if(!found_pos) {
    Serial.println("error: no closest pos");
    exit(-1);
  }
  return min_pos;
}

/*
 * Marks the current position as filled
 *
 * Requires: area_mat is valid bit matrix, curr_pos valid position in area_mat
 */
void
mark_filled(int **area_mat, pos to_mark) {
  area_mat[to_mark.y][to_mark.x] = 0;
  return;
}

void 
loop() {
  pos curr_pos;
  int** area_mat;
  int area_mat_size;
  Serial.println("beginning loop...");

  digitalWrite(LED_PIN, HIGH);

  curr_pos.x = curr_pos.y = 0;
  area_mat = get_mat();
  fill_pattern(area_mat);
  area_mat_size = 10;  //don't hard code this duhhh
  while(true) {
    Serial.println("looooooping...");
    pos next_pos = find_closest_pos(area_mat, area_mat_size, curr_pos);
    Serial.print("closest pos: "); 
    Serial.println(pos_string(next_pos));
    send_to_pos(curr_pos, next_pos);
    curr_pos = next_pos;
    Serial.print("updated curr_pos to: ");
    Serial.println(pos_string(curr_pos));
    mark_filled(area_mat, curr_pos);
      //print matrix
  for(int i = 0; i < area_mat_size; i ++) {
    for(int j=0; j< area_mat_size; j++) {
      Serial.print(area_mat[i][j]);
    }
    Serial.println("");
  }
  }
}

// Converts x and y components of a vector to a heading in degrees.
// This function is used instead of LSM303::heading() because we don't
// want the acceleration of the Zumo to factor spuriously into the
// tilt compensation that LSM303::heading() performs. This calculation
// assumes that the Zumo is always level.
int heading(LSM303::vector v)
{
  float x_scaled =  2.0*(float)(v.x - compass.m_min.x) / ( compass.m_max.x - compass.m_min.x) - 1.0;
  float y_scaled =  2.0*(float)(v.y - compass.m_min.y) / (compass.m_max.y - compass.m_min.y) - 1.0;

  int angle = round(atan2(y_scaled, x_scaled)*180 / M_PI);
  if (angle < 0)
    angle += 360;
  return angle;
}


// Average 10 vectors to get a better measurement and help smooth out
// the motors' magnetic interference.
int averageHeading()
{
  LSM303::vector avg = {0, 0, 0};

  for(int i = 0; i < 10; i ++)
  {
    compass.read();
    avg.x += compass.m.x;
    avg.y += compass.m.y;
  }
  avg.x /= 10.0;
  avg.y /= 10.0;

  // avg is the average measure of the magnetic vector.
  return heading(avg);
}

String
pos_string(pos position) {
  String s1 = "(";
  s1 += String(position.x);
  s1 += ", ";
  s1 += String(position.y);
  s1 += ")";
  return s1;
}

// turns the Zumo to face the requested magnetic heading
void turnToHeading(int angle)
{
  // make sure angle is a number from 0 to 360
  while (angle < 0)
    angle += 360;
  while (angle >= 360)
    angle -= 360;
    
  int currentAngle;
  long diffSum = 0;
  int lastDiff = 1000;  // dummy value outside allowed -180 to 180 range indicates uninitialized
  unsigned char count = 0;
  
  while (1)
  {
    currentAngle = averageHeading();
    Serial.println(currentAngle);
    int diff = currentAngle - angle;
    if (diff < 0)
      diff += 360;
    if (diff > 180)
      diff -= 360;
    
    if (lastDiff == 1000);
      lastDiff = diff;
      
    if (diff < 5 && diff > -5)
    {
      // if we are within 10 degrees of our target heading
      count++;
      if (count > 6)
      {
        // if we have been within 10 degrees of our target for 6 consecutive readings
        //  we have reached our target; stop motors and return
        motors.setSpeeds(0, 0);
        return;
      }
    }
    else
      count = 0;

    // use PID on the heading error (diff) to determine what the motor speed should be
    // proportional constant is 5/2, derivative constant is 30, integral constant is 1/7   
    int motor_speed = diff*5/2 + (diff - lastDiff)*30 + diffSum/7;
    motors.setSpeeds(-motor_speed, motor_speed);  // update speed of motors

    lastDiff = diff;
    
    // only include small errors in error sum (integral term only grows when Zumo is close to its target)
    if (diff > -40  && diff < 40)
      diffSum += diff;
    
    // cap the magnitude of the error sum at 100,000
    if (diffSum > 100000)
      diffSum == 100000;
    if (diffSum < -100000)
      diffSum = -100000;
    
    /*
    // the following code is an alternate approach that doesn't use PID
    //  it doesn't work as well, but it's easier to understand
    if (diff < -40)
      setMotors(150, -150);
    else if (diff < -30)
      setMotors(100, -100);
    else if (diff < -20)
      setMotors(80, -80);
    if (diff > 40)
      setMotors(-150, 150);
    else if (diff > 30)
      setMotors(-100, 100);
    else if (diff > 20)
      setMotors(-80, 80);
    */
    
    delay(5);
  }
}
