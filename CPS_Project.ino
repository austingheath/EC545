// Master Hand CPS Project
// EC 545

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Braccio.h>
#include <Servo.h>

Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_rot;
Servo wrist_ver;
Servo gripper;

// Pin setup
const int glove_fsr_pin = A11;  
const int robot_fsr_pin =  A10;  
const int button_pin = 47;

Adafruit_BNO055 bno = Adafruit_BNO055(55);

// Global values
int glove_fsr_reading;
int robot_fsr_reading;

int prev_euler[3];
int prev_processed[3];

int grip = 30;

void setup(void) 
{ 
  
  Serial.begin(9600);
  
  // Init the robot
  Braccio.begin();

  if(!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  bno.setExtCrystalUse(true);

  // Setup the pin for the button
  pinMode(button_pin, INPUT_PULLUP);
}

// Determines if the system should be in a running state
// by viewing the button's value
bool poll_button() 
{
  static const unsigned long debounce_dur = 500;
  static unsigned long last_time = 0;
  static bool running = false;

  // Debounce button input
  // If we are within a certain time tolerance, since last press
  // output running an return.
  if ((millis() - last_time) <= debounce_dur) {
    return running;
  }

  last_time = millis();

  // Get current button value
  bool button_pressed = digitalRead(button_pin) == HIGH;

  if (!running && button_pressed) {
    // Startup routine
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    prev_euler[0] = euler.x();
    prev_euler[1] = euler.y();
    prev_euler[2] = euler.z();
    prev_processed[0] = 90;
    prev_processed[1] = 90;
    prev_processed[2] = 90;

    // Move robot into initial position
    Braccio.ServoMovement(20, 90, 90, 90, 0, 90, 73);

    // Wait for next loop iteration before starting by returning false
    // but setting running to true
    running = true;

    return false;
 } else if (running && button_pressed) {
    running = false;
 }

  return running;
}

// Calculate the position the robot joints should be in
// from the IMU data
int* get_robot_angles()
{
  const double weight = 0.1;
  static int processed[3];

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  // Low Pass Filter on y only. The other axis don't really need it.
  int x = euler.x();
  int y = (prev_euler[1] * (1 - weight)) + (euler.y() * weight);
  int z = euler.z();

  // Set the previous raw values to the new raw values
  prev_euler[0] = x;
  prev_euler[1] = y;
  prev_euler[2] = z;
  
  // IMU x goes 0 to 360 (initial near 90)
  // Robot M1 goes 0 to 180
  // Ignore values greater than 180
  // Must be witin a tolerance
  if (x < 0 || x > 180 || abs(x - prev_processed[0]) > 30) {
    x = prev_processed[0];
  }
  
  // IMU y goes -90 to 90 (initial near 0)
  // Robot M3 goes 0 to 180 (initial 90)
  if (y <= 0 && y >= -90) {
    // Going up is negative
    y = abs(y) + 90;
  } else if (y > 0 && y <= 90) {
    // Going down is positive
    y = 90 - y;
  } else {
    y = prev_processed[1];
  }

  // IMU z goes 0 to 180 or -180 to 0 (initial near 180/-180)
  // Robot M5 goes 0 to 180 (initial 90)
  if (z >= 90 && z <= 180) {
    z = z - 90;
    z = 180 - z;
  } else if (z >= -180 && z <= -90) {
    z = (180 - abs(z)) + 90;
    z = 180 - z;
  } else {
    z = prev_processed[2];
  }

  // Set the previous processed values.
  prev_processed[0] = x;
  prev_processed[1] = y;
  prev_processed[2] = z;

  // Set the new x, y, z values for the joint angles.
  processed[0] = x;
  processed[1] = y;
  processed[2] = z;

  return processed;
}

void loop(void) 
{
  // Wait for button press before starting
  // Or stop running if the button is pressed again
  if (!poll_button()) {
    return;
  }

  // Get sensor data
  glove_fsr_reading = analogRead(glove_fsr_pin);
  robot_fsr_reading = analogRead(robot_fsr_pin);
  
  // Process orientation
  int* robot_angles = get_robot_angles();

  // Process force
  // Try get the two fsr values to be withing 10 of each other.
  if (glove_fsr_reading - robot_fsr_reading > 10 && grip < 69){
    grip += 5;
  } else if (glove_fsr_reading < 10) {
    grip -= 10;
  }

  // Move robot accordingly
  Braccio.ServoMovement(10,robot_angles[0], 90, robot_angles[1], 0, robot_angles[2], grip);

  // Delay so the robot has a bit of time to move.
  delay(100);
}
