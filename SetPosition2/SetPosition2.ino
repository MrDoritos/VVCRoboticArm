#include <xArmServoController.h>
//#include <SoftwareSerial.h>

// To use SoftwareSerial:
// 1. Uncomment include statement above and following block.
// 2. Update xArmServoController with mySerial.
// 3. Change Serial1.begin to mySerial.begin.
/* 
#define rxPin 2
#define txPin 3

SoftwareSerial mySerial = SoftwareSerial(rxPin, txPin);
*/
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

void setup_joysticks() {
  //pinMode(SW_PIN_1, INPUT);
  //pinMode(SW_PIN_2, INPUT);
  
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
  Serial1.begin(9600);
  Serial.begin(9600);

  // xArm servo positions
   xArmServo home[] = {{1, 500},
                      {2, 900},
                      {3, 500},
                      {4, 500},
                      {5, 500},
                      {6, 500}};
  xArmServo bow[] = {{1, 650},
                     {3, 130},
                     {4, 845},
                     {5, 650}};
 

  //myArm.setPosition(home, 6, 500, true);
  //delay(1000);
  //myArm.setPosition(bow, 4, 500, true);
  //delay(1000);
  myArm.setPosition(home, 6, 1000, true);

  // Your setup here.
  axis_position[0] = 500;
  axis_position[1] = 500;
  axis_position[2] = 500;
  axis_position[3] = 500;
  sw_position[0] = 500;
  sw_position[1] = 500;
  read_joysticks(0);
}

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

  xArmServo position[4] = {
    {3, axis_position[0]},
    {4, axis_position[1]},
    {5, axis_position[2]},
    {6, axis_position[3]}
  };

  xArmServo gripper[2] = {
    {1, sw_position[0]},
    {2, sw_position[1]}
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
