#include <BasicLinearAlgebra.h>
#include <ElementStorage.h>

#include <xArmServoController.h>

xArmServoController myArm = xArmServoController(xArm, Serial1);

#define SW_PIN_1 4
#define SW_PIN_2 5
#define VRX_PIN_1 0
#define VRY_PIN_1 1
#define VRX_PIN_2 2
#define VRY_PIN_2 3

#define sw_input_count 2
#define axis_input_count 4

int sw_input[sw_input_count * 2];
int axis_input[axis_input_count * 2];

int sw_position[sw_input_count];
int last_sw_position[sw_input_count];

int axis_position[axis_input_count];
int last_axis_position[axis_input_count];

#define GRIPPER 0
#define GRIPPER_ROT 1
#define SERVO_3 2
#define SERVO_4 3
#define BASE_ARM 4
#define BASE 5

#define SERVO_1 GRIPPER
#define SERVO_2 GRIPPER_ROT
#define SERVO_5 BASE_ARM
#define SERVO_6 BASE

/*
 * 1. 0 - 700, open to closed
 * 2. 0 - 1000, ccw to cw
 * 3. 500 - 105, inside backwards
 * 4. 886 - 500 - 883, inside forwards
 * 5. 880 - 500 - 148, inside forwards
 * 6. 870 - 500 - 100, -90* to 0* to 90*
 * 
 */

unsigned int robot_axis_raw[6];
float robot_axis_angle[6];
float robot_90_deg_calib_value[6] = {
    500,500,395,383,383,400
};
float robot_axis_length[6];
typedef BLA::Matrix<3,1> vec3d;
vec3d robot_axis_position[6], robot_axis_direction[6];

float DistSqrd(vec3d a, vec3d b) {
  float x = b(0,0)-a(0,0), y = b(1,0)-a(1,0), z = b(2,0)-a(2,0);
  return (x * x) + (y * y) + (z * z);
}

float Dist(vec3d a, vec3d b) {
  return sqrtf(DistSqrd(a,b));
}

float Sum(float values[], int length) {
  float tmp = 0;
  for (int i = 0; i < length; i++)
    tmp += values[i];
  return tmp;
}

int clip(int value, int vmin, int vmax) {
  if (value < vmin) value = vmin;
  if (value > vmax) value = vmax;
  return value;
}

unsigned int angle_to_raw(float angle, int servo, bool flip = false) {
  float yy = (int((angle / 6.283f) * 1000.0f) % 1000) - 500.0f;
  Serial.println(yy);
  //float conv = (yy / robot_90_deg_calib_value[servo]) * 500.0f;
  float conv = (yy / 500.0f) * robot_90_deg_calib_value[servo];
  //int value = clip((int((angle / (6.283f * 4.0f)) * robot_90_deg_calib_value[servo]) % 1000), 100, 900);
  //int svalue = 500 + ;
  //if (flip)
  //  value += svalue;
  //return value;
  return (unsigned int)(conv);
}

float rad_to_deg(float angle) {
  return (angle / 3.14159f) * 180.0f;
}

void print_vec3d(vec3d vec) {
  Serial.print("x: ");
  Serial.print(vec(0,0));
  Serial.print(" y: ");
  Serial.print(vec(1,0));
  Serial.print(" z: ");
  Serial.println(vec(2,0));
}

void print_robot_debug() {
  Serial.println("Robo-debug");
  for (int i = 0; i < 6; i++) {
    Serial.print("Servo ");
    Serial.println(i+1);
    Serial.print("  pos: ");
    print_vec3d(robot_axis_position[i]);
    Serial.print("  dir: ");
    print_vec3d(robot_axis_direction[i]);
    Serial.print("  raw: ");
    Serial.print(robot_axis_raw[i]);
    Serial.print(" angle: ");
    Serial.print(robot_axis_angle[i]);
    Serial.print(" angle (deg): ");
    Serial.print(rad_to_deg(robot_axis_angle[i]));
    Serial.print(" arm length: ");
    Serial.println(robot_axis_length[i]);
  }
}

void setup_home() {
  robot_axis_length[SERVO_1] = 0.0f;
  robot_axis_length[SERVO_2] = 0.0f; //In line with axis
  robot_axis_length[SERVO_3] = 103.0f; //103mm
  robot_axis_length[SERVO_4] = 95.0f; //95mm
  robot_axis_length[SERVO_5] = 101.0f; //101mm
  robot_axis_length[SERVO_6] = 0.0f; //In line with axis

  robot_axis_position[SERVO_6] = {0,0,0};
  robot_axis_position[SERVO_5] = {0,0,65.0f};

  /*
   * SERVO_5 is 65mm from base of robot
   * SERVO_5 is 364mm from tip of gripper
   * Robot is 429mm tall when home
   * Base is 140mm,143mm,127mm,154mm
   * Gripper is 55mm fully opened
   * Gripper is 96mm at its widest point
   */
}

void move_robot(vec3d target, vec3d front) {
  //find servo 3 coordinates from the front. servo 3 is final direction servo
  vec3d servo_3_displacement = front * robot_axis_length[SERVO_3];  
  robot_axis_direction[SERVO_3] = front;
  robot_axis_position[SERVO_3] = target + servo_3_displacement;

  //does not change because its the base of the robot
  vec3d servo_6_xy = {robot_axis_position[BASE](0,0), robot_axis_position[BASE](1,0), 0};
  vec3d servo_3_xy = {robot_axis_position[SERVO_3](0,0), robot_axis_position[SERVO_3](1,0), 0};

  //base rotation is front, find direction, find rotation degrees using arcos brsin
  vec3d servo_6to3_xy_dir = servo_3_xy - servo_6_xy;
  robot_axis_direction[BASE] = servo_6to3_xy_dir / Norm(servo_6to3_xy_dir);

  //servo 5, base arm, find coords
  vec3d servo_5_displacement = robot_axis_direction[BASE] * robot_axis_length[BASE];
  robot_axis_position[BASE_ARM] = robot_axis_position[BASE] + servo_5_displacement;
  robot_axis_position[BASE_ARM](2,0) = 65.0f;

  /*
   * Known coords
   *  BASE/SERVO_6
   *  BASE_ARM/SERVO_5
   *  SERVO_3
   *  Target
   *  
   * Known directions
   *  BASE_ARM/SERVO_5
   *  SERVO_3 (Front)
   *  
   * Known servo positions
   *  BASE/SERVO_6
   */

  
   { //SERVO_4 Calculation using law of cosines
     float c = Dist(robot_axis_position[SERVO_5], robot_axis_position[SERVO_3]);
     float a = robot_axis_length[SERVO_4];
     float b = robot_axis_length[SERVO_5];
  
     float denom = 2 * a * b;
     float num = (a * a) + (b * b) - (c * c);
     float divisor = num / denom;
  
     robot_axis_angle[SERVO_4] = powf(cosf(divisor),-1);
  
     robot_axis_raw[SERVO_4] = angle_to_raw(robot_axis_angle[SERVO_4], 4, true);

     robot_axis_direction[SERVO_4] = {cosf(robot_axis_angle[SERVO_4]),sinf(robot_axis_angle[SERVO_4]),0};
   }

   { //SERVO_3 Calculation using dot product of servo 4 and front
      float dp0 = robot_axis_direction[SERVO_4](0,0) * robot_axis_direction[SERVO_3](1,0);
      float dp1 = robot_axis_direction[SERVO_4](1,0) * robot_axis_direction[SERVO_3](2,0);

      robot_axis_direction[SERVO_3] = {dp0,dp1,0};
      //float theta = powf(tanf(robot_axis_direction[SERVO_3](1,0) / robot_axis_direction[SERVO_3](0,0)),-1);
      //float theta = powf(tanf(dp1/dp0),-1);
      robot_axis_position[SERVO_4] = robot_axis_position[SERVO_3]-(robot_axis_direction[SERVO_3] * robot_axis_length[SERVO_4]);
      float theta = powf(cosf(dp1*dp0),-1);
      robot_axis_angle[SERVO_3] = theta;
      robot_axis_raw[SERVO_3] = angle_to_raw(robot_axis_angle[SERVO_3], 3);
   }
   
   { //SERVO_5 Calculation using law of cosines
     //float c = robot_axis_length[SERVO_4];
     //float a = robot_axis_length[SERVO_5];
     //float b = Dist(robot_axis_position[SERVO_5], robot_axis_position[SERVO_3]);
     //robot_axis_angle[SERVO_5] = powf(cosf(((a * a) + (b * b) - (c * c)) / (2 * a * b)),-1);
     //robot_axis_raw[SERVO_5] = angle_to_raw(robot_axis_angle[SERVO_5]);
     
     //robot_axis_direction[SERVO_5] = {cosf(robot_axis_angle[SERVO_5]),sinf(robot_axis_angle[SERVO_5]),0};

     vec3d mag = robot_axis_position[SERVO_4] - robot_axis_position[SERVO_5];
     mag(2,0) = 0;
     robot_axis_direction[SERVO_5] = mag / Norm(mag);
     robot_axis_angle[SERVO_5] = powf(tanf(robot_axis_direction[SERVO_5](1,0)/robot_axis_direction[SERVO_5](0,0)),-1);
     robot_axis_raw[SERVO_5] = angle_to_raw(robot_axis_angle[SERVO_5], 5);
   }

   
   { //SERVO_6 Calculation
     float theta = powf(tanf(robot_axis_direction[BASE](1,0)/robot_axis_direction[BASE](0,0)),-1);
     robot_axis_angle[SERVO_6] = theta;
     robot_axis_raw[SERVO_6] = angle_to_raw(robot_axis_angle[SERVO_6], 6);
   }

   /*
    * Known servo positions
    *  SERVO_3/Front
    *  SERVO_4
    *  BASE_ARM/SERVO_5
    *  BASE/SERVO_6
    */

    xArmServo newposition[] = {
      {1, robot_axis_raw[SERVO_1]},
      {2, robot_axis_raw[SERVO_2]},
      {3, robot_axis_raw[SERVO_3]},
      {4, robot_axis_raw[SERVO_4]},
      {5, robot_axis_raw[SERVO_5]},
      {6, robot_axis_raw[SERVO_6]},
    };

    myArm.setPosition(newposition, 6, 5000, true);
    
    print_robot_debug();
}

void setup_joysticks() {
  for (int i = 0; i < 4; i++)
    sw_input[i] = false;
  for (int i = 0; i < 8; i++)
    axis_input[i] = 0;
}

void read_joysticks(int offset) {
  int sw_offset = offset * sw_input_count;
  sw_input[0 + sw_offset] = analogRead(SW_PIN_1);
  sw_input[1 + sw_offset] = analogRead(SW_PIN_2);

  int axis_offset = offset * axis_input_count;
  axis_input[0 + axis_offset] = analogRead(VRX_PIN_1);
  axis_input[1 + axis_offset] = analogRead(VRY_PIN_1);
  axis_input[2 + axis_offset] = analogRead(VRX_PIN_2);
  axis_input[3 + axis_offset] = analogRead(VRY_PIN_2);
}

void setup() {
  setup_joysticks();
  setup_home();
  Serial1.begin(9600);
  Serial.begin(9600);
  print_robot_debug();

  xArmServo home[] = {{1, 500},
                      {2, 900},
                      {3, 500},
                      {4, 500},
                      {5, 500},
                      {6, 500}};
 
  myArm.setPosition(home, 6, 1000, true);

  axis_position[0] = 500;
  axis_position[1] = 500;
  axis_position[2] = 500;
  axis_position[3] = 500;
  sw_position[0] = 500;
  sw_position[1] = 500;
  read_joysticks(0);
}

void loop() {
  move_robot(vec3d{30,10,10}, vec3d{0,0,1});
  delay(4000);
  move_robot(vec3d{10, 5, 25}, vec3d{0,0,1});
  delay(4000);
  //delay(-1);
}

/*
void loop() {
  // Your code here.
  delay(5);
  read_joysticks(1);

  Serial.print("Axis_positions: ");
  for (int i = 0; i < axis_input_count; i++) {
    axis_position[i] = ((axis_input[i + axis_input_count] - axis_input[i]) / 1024.0f) * 700.0f + 500.0f;
    Serial.print(axis_position[i]);
    Serial.print(" ");
  }
  Serial.println();

  for (int i = 0; i < sw_input_count; i++) {
    sw_position[i] = 1000.0f - (((sw_input[i + sw_input_count] - sw_input[i]) / 1024.0f) * 1000.0f);
  }

  bool tolerance = false;
  bool sw_tolerance = false;

  for (int i = 0; i < axis_input_count; i++) {
    if (abs(last_axis_position[i] - axis_position[i]) > 10)
      tolerance = true;
  }

  for (int i = 0; i < sw_input_count; i++) {
    if (abs(last_sw_position[i] - sw_position[i]) > 10)
      sw_tolerance = true;
  }

  typedef unsigned int nar;

  xArmServo position[4] = {
    {3, (nar) axis_position[0]},
    {4, (nar) axis_position[1]},
    {5, (nar) axis_position[2]},
    {6, (nar) axis_position[3]}
  };

  xArmServo gripper[2] = {
    {1, (nar) sw_position[0]},
    {2, (nar) sw_position[1]}
  };

  if (tolerance) {
    myArm.setPosition(position, 4, 200);
    //set last axis position
    for (int i = 0; i < axis_input_count; i++) {
      last_axis_position[i] = axis_position[i];
    }
  }
  if (sw_tolerance) {
    myArm.setPosition(gripper, 2, 1000);
    for (int i = 0; i < sw_input_count; i++) {
      last_sw_position[i] = sw_position[i];
    }
  }
}
*/
