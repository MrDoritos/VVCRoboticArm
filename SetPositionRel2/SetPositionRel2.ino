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

#define switch_input_count 2
#define joystick_input_count 4
#define axis_count 6

int joystick_input[joystick_input_count];
int joystick_input_home[joystick_input_count];
int joystick_input_last[joystick_input_count];

int switch_input[switch_input_count];
int switch_input_home[switch_input_count];
int switch_input_last[switch_input_count];

int axis_position[axis_count];
int axis_position_last[axis_count];

void setup_joysticks() {
  //pinMode(SW_PIN_1, INPUT);
  //pinMode(SW_PIN_2, INPUT);
  
  for (int i = 0; i < switch_input_count; i++) {
    switch_input[i] = switch_input_home[i] = switch_input_last[i] = false;
  }
  for (int i = 0; i < joystick_input_count; i++) {
    joystick_input[i] = joystick_input_home[i] = joystick_input_last[i] = 0;
  }
}

void read_input(int *joysticks, int *switches) {
  switches[0] = analogRead(SW_PIN_1);
  switches[1] = analogRead(SW_PIN_2);

  joysticks[0] = analogRead(VRX_PIN_1);
  joysticks[1] = analogRead(VRY_PIN_1);
  joysticks[2] = analogRead(VRX_PIN_2);
  joysticks[3] = analogRead(VRY_PIN_2);
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

  myArm.setPosition(home, 6, 1000, true);

  for (int i = 0; i < axis_count; i++)
    axis_position[i] = 500;

  read_input(&joystick_input_home[0], &switch_input_home[0]);
}

void print_debug(int i, float v, float lv, int axis_position) {
    Serial.print(i);
    Serial.print(":");
    Serial.print(axis_position);
    Serial.print(":");
    Serial.print(v);
    Serial.print(":");
    Serial.print(lv);
    Serial.println();
}

void loop() {
  delay(5);
  read_input(&joystick_input[0], &switch_input[0]);

  for (int i = 0; i < joystick_input_count; i++) {
    float v = ((joystick_input[i] - joystick_input_home[i])) * 0.03f;
    //float lv = axis_position_last[i + 2] - (axis_position[i + 2] + v);
    //l600 h500 i550 = -50
    //l400 h500 i450 = -50
    //l400 h500 i350 = 50
    //l600 h500 i650 = 50
    //float lv = abs(joystick_input[i] - joystick_input_home[i]) - abs(joystick_input_last[i] - joystick_input_home[i]);
    float lv = abs(joystick_input_last[i] - joystick_input_home[i]) - abs(joystick_input[i] - joystick_input_home[i]);
    bool f = (joystick_input[i] - joystick_input_home[i]) > 0;

    //if (f && lv > 0)
    //  continue;
    
    if (abs(v) < 0.4f)
      continue;
    //if (lv < 0.0f)
    //  continue;

    joystick_input_last[i] = joystick_input[i];
      
    axis_position[i + 2] += v;
    
    if (axis_position[i + 2] > 1000)
      axis_position[i + 2] = 1000;
    if (axis_position[i + 2] < 0)
      axis_position[i + 2] = 0;

    print_debug(i, v, lv, axis_position[i + 2]);
  }

  for (int i = 0; i < switch_input_count; i++) {
    float v = 1000.0f - (((switch_input_home[i] - switch_input[i]) / 1024.0f) * 1000.0f);
    float lv = abs(switch_input[i] - switch_input_home[i]) - abs(switch_input_last[i] - switch_input_home[i]);

    if (abs(lv) < 5.0f)
      continue;

    axis_position[i] = v;

    switch_input_last[i] = switch_input[i];

    if (axis_position[i] > 1000)
      axis_position[i] = 1000;
    if (axis_position[i] < 0)
      axis_position[i] = 0;

    //print_debug(i, v, lv, axis_position[i + 2]);    
  }

  bool tolerance = false;
  
  for (int i = 0; i < axis_count; i++) {
    if (abs(axis_position_last[i] - axis_position[i]) > 0)
      tolerance = true;
  }

  xArmServo position[6] = {
    {1, axis_position[0]},
    {2, axis_position[1]},
    {3, axis_position[2]},
    {4, axis_position[3]},
    {5, axis_position[4]},
    {6, axis_position[5]}
  };

  if (tolerance) {
    myArm.setPosition(position, 6, 200);
    Serial.println("Setposition");
    //set last axis position
    for (int i = 0; i < axis_count; i++) {
      axis_position_last[i] = axis_position[i];
    }
  }
}
