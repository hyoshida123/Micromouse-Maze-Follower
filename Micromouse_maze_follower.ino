#include <PID_v1.h>

// Define Variables we'll be connecting to
double velocity_linear_setpoint;
double velocity_linear;
double velocity_linear_power;

double velocity_angular_setpoint;
double velocity_angular;
double velocity_angular_power;

double distance_left;
double distance_left_setpoint;
double left_stat = 0;
double distance_center;
double distance_right;

// double braking_power;
int count = 0;

// Specify the links and initial tuning parameters
PID velocity_linear_pid(&velocity_linear, &velocity_linear_power, &velocity_linear_setpoint, .007, .0005, 0, DIRECT); //.007 .0005
PID velocity_angular_pid(&velocity_angular, &velocity_angular_power, &velocity_angular_setpoint, .4, .05, 0, DIRECT);
// PID distance_left_pid(&distance_left, &braking_power, &distance_left_setpoint, .007, .0005, 0, DIRECT);

void setup()
{
  velocity_linear = 0;
  velocity_linear_setpoint = 200; // We want to go 200 mm/sec
  velocity_linear_pid.SetOutputLimits(-1.0, 1.0);
  velocity_linear_pid.SetSampleTime(10);

  velocity_angular = 0;
  velocity_angular_setpoint = 0;
  velocity_angular_pid.SetOutputLimits(-1.0, 1.0);
  velocity_angular_pid.SetSampleTime(10);

  // distance_left = 0;
  // distance_left_setpoint = 10; // 10 cm from left wall
  // distance_left_pid.SetOutputLimits(-0.5, 0.5);
  // distance_left_pid.SetSampleTime(10);

  // turn the PID on
  velocity_linear_pid.SetMode(AUTOMATIC);
  velocity_angular_pid.SetMode(AUTOMATIC);
  // distance_left_pid.SetMode(AUTOMATIC);
  pinSetup();
  Serial.begin(9600);
}

void loop()
{
  velocity_linear = getLinearVelocity();
  velocity_linear_pid.Compute();
  velocity_angular = getAngularVelocity();
  velocity_angular_pid.Compute();
  distance_left = readDistanceLeft();
  distance_center = readDistanceCenter();
  distance_right = readDistanceRight();
  // distance_left_pid.Compute();
  
  // print(velocity_linear_power);

 if (distance_center > 0 && distance_center < 6) {
    applyPowerLeft(0);
    applyPowerRight(0);
    delay(1000);
     
    if (distance_left < 8 && distance_right < 8) {
      applyPowerLeft(0.3);
      applyPowerRight(-0.3);
      delay(360);
    } else if (distance_left < 8) {
      applyPowerLeft(-0.3);
      applyPowerRight(0.3);
      delay(160);
    } else {
      applyPowerLeft(0.3);
      applyPowerRight(-0.3);
      delay(160);
    }
    applyPowerLeft(0);
    applyPowerRight(0);
    delay(100);
  }

  if (count % 2 == 0) {
    applyPowerLeft(velocity_linear_power - velocity_angular_power);
    applyPowerRight(velocity_linear_power + velocity_angular_power);
    // applyBrakeRight(braking_power);
    count = 0;
  }else {
   alter_velocity_perception((distance_left - 15) * 0.03, velocity_linear_power);
  }
  //Serial.print("center");
  //Serial.println(distance_center);
  /*if (millis() % 200 == 0){
    Serial.print("left");
    Serial.println(distance_left);
    Serial.println(distance_left);
    Serial.println((distance_left - 15) * 0.1);
    Serial.println((velocity_linear_power - velocity_angular_power));  
    Serial.println();
    Serial.println(velocity_linear);
    Serial.println(velocity_linear_power);
     
    Serial.println(distance_center);
  }*/
  
  count++;
  checkEncodersZeroVelocity();
}
