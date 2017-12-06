
#define PIN_DISTANCE_LEFT A0
#define PIN_DISTANCE_CENTER A6
#define PIN_DISTANCE_RIGHT A7

#define PIN_LED1 A3
#define PIN_LED2 A2
#define PIN_BUTTON A1

#define PIN_ENCODER_LEFT_A 11
#define PIN_ENCODER_LEFT_B 2
#define PIN_ENCODER_RIGHT_A 12
#define PIN_ENCODER_RIGHT_B 3

#define PIN_MOTOR_LEFT_1 9
#define PIN_MOTOR_LEFT_2 10
#define PIN_MOTOR_RIGHT_1 5
#define PIN_MOTOR_RIGHT_2 6

// Invert encoder directions if needed
const boolean INVERT_ENCODER_LEFT = true;
const boolean INVERT_ENCODER_RIGHT = true;

// Invert motor directions if needed
const boolean INVERT_MOTOR_LEFT = true;
const boolean INVERT_MOTOR_RIGHT = false;

// Loop count, used for print statements
// int count = 0;
// double left_power = 0.5;
// double right_power = 0.5;

double left_motor_constant = 1;
double right_motor_constant = 1;
double past_brake = 0;
double past_brake2 = 0;
double past_brake3 = 0;
double average_brake = 0;


/*

   Some helper functions we've written to simplify your code
   These were all taken from your previous labs!

   Functions that you might find useful:

    pinSetup()            - sets up pins (modes, interrupts, etc)

    applyPowerLeft()      - applies a power (-1 to 1) to the left wheel
    applyPowerRight()     - applies a power (-1 to 1) to the right wheel
    applyBrakeLeft()      - applies a braking force (0 to 1) to the left wheel
    applyBrakeRight()     - applies a braking force (0 to 1) to the right wheel

    getLinearVelocity()   - computes and returns our forward velocity (mm/sec)
    getAngularVelocity()  - computes and returns our turning velocity (rad/sec)

    readDistanceLeft()    - reads a distance from the left IR sensor (cm)
    readDistanceRight()   - reads a distance from the right IR sensor (cm)
    readDistanceCenter()  - reads a distance from the center IR sensor (cm)

*/

///////////////////////////////
// Helper for hardware setup //
///////////////////////////////

void pinSetup() {
  pinMode(PIN_DISTANCE_LEFT, INPUT);
  pinMode(PIN_DISTANCE_RIGHT, INPUT);
  pinMode(PIN_DISTANCE_CENTER, INPUT);

  pinMode(PIN_LED1, OUTPUT);
  pinMode(PIN_LED2, OUTPUT);
  pinMode(PIN_BUTTON, INPUT);

  pinMode(PIN_MOTOR_LEFT_1, OUTPUT);
  pinMode(PIN_MOTOR_LEFT_2, OUTPUT);
  pinMode(PIN_MOTOR_RIGHT_1, OUTPUT);
  pinMode(PIN_MOTOR_RIGHT_2, OUTPUT);

  pinMode(PIN_ENCODER_LEFT_A, INPUT);
  pinMode(PIN_ENCODER_LEFT_B, INPUT);
  pinMode(PIN_ENCODER_RIGHT_A, INPUT);
  pinMode(PIN_ENCODER_RIGHT_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_LEFT_B), leftEncoderRisingEdge, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_RIGHT_B), rightEncoderRisingEdge, RISING);
}

void alter_velocity_perception(double brake, double power) {
  if (brake > -0.3 && brake < 0.4) { 
    average_brake = 1 + brake + past_brake + past_brake2 + past_brake3;
    average_brake = (average_brake + 1) / 2;
    left_motor_constant = average_brake;
    Serial.println(average_brake);
    //Serial.println(power);
    //Serial.println((brake + past_brake) / 2);
    past_brake3 = past_brake2;
    past_brake2 = past_brake;
    past_brake = brake;
    /*Serial.println();
    Serial.println("brake");
    Serial.println(brake);
    Serial.println();
    Serial.println();
    Serial.println("average brake");
    Serial.println(((brake + past_brake + past_brake2 + past_brake3)/8));
    Serial.println();
    */
  } else {
    Serial.println("bad");
    //Serial.println(brake);
  }
}


////////////////////////////////////
// Motor driving helper functions //
////////////////////////////////////

// Apply power to a motor
// power = 0 means no power is applied
// power = 1 means 100% power forward
// power = -1 means 100% power backwards

void applyPowerLeft(double power) {
  if (INVERT_MOTOR_LEFT)
    power *= -1;

  if (power > 0) {
    analogWrite(PIN_MOTOR_LEFT_1, constrain(power * 255.0, 0, 255));
    analogWrite(PIN_MOTOR_LEFT_2, 0);
  } else {
    analogWrite(PIN_MOTOR_LEFT_1, 0);
    analogWrite(PIN_MOTOR_LEFT_2, constrain(power * -255.0, 0, 255));
  }
}

void applyPowerRight(double power) {
  if (INVERT_MOTOR_RIGHT)
    power *= -1;

  if (power > 0) {
    analogWrite(PIN_MOTOR_RIGHT_1, constrain(power * 255.0, 0, 255));
    analogWrite(PIN_MOTOR_RIGHT_2, 0);
  } else {
    analogWrite(PIN_MOTOR_RIGHT_1, 0);
    analogWrite(PIN_MOTOR_RIGHT_2, constrain(power * -255.0, 0, 255));
  }
}


// Apply a braking force to the motor
// power = 0 means no braking force
// power = 1 means 100% braking force

void applyBrakeLeft(double power) {
  analogWrite(PIN_MOTOR_LEFT_1, constrain(power * 255.0, 0, 255));
  analogWrite(PIN_MOTOR_LEFT_2, constrain(power * 255.0, 0, 255));
}

void applyBrakeRight(double power) {
  analogWrite(PIN_MOTOR_RIGHT_1, constrain(power * 255.0, 0, 255));
  analogWrite(PIN_MOTOR_RIGHT_2, constrain(power * 255.0, 0, 255));
}


////////////////////////////////
// IR sensor helper functions //
////////////////////////////////

double readDistanceLeft() {
  return (1 / (0.000413153 * analogRead(PIN_DISTANCE_LEFT) - 0.0055266887));
}

double readDistanceRight() {
  return (1 / (0.000413153 * analogRead(PIN_DISTANCE_RIGHT) - 0.0055266887));
}

double readDistanceCenter() {
  return (1 / (0.000413153 * analogRead(PIN_DISTANCE_CENTER) - 0.0055266887));
}


//////////////////////////////
// Encoder helper functions //
//////////////////////////////

// Mouse physical parameters; these are used for velocity calculations
const double ENCODER_TICKS_PER_REVOLUTION = 420.0 / 2.0; // blaze it
const double WHEELBASE_DIAMETER = 95.0; // mm
const double WHEEL_DIAMETER = 34.0; // mm

// Precomputed constant used for calculating wheel velocities
// (VELOCITY_COEFF / encoder pulse width in microseconds) will give us the linear velocity of a wheel
const double VELOCITY_COEFF = WHEEL_DIAMETER * PI / ENCODER_TICKS_PER_REVOLUTION * 1000000.0;

// Encoder helper variables
unsigned long prev_pulse_time_right;
unsigned long prev_pulse_time_left;

// Encoder state variables
long ticks_left = 0; // ticks
long ticks_right = 0; // ticks
double velocity_left = 0; // millimeters/sec
double velocity_right = 0; // millimeters/sec

double getLinearVelocity() {
  return 0.5 * (velocity_left + velocity_right);
}

double getLeftVelocity() {
  return velocity_left;
}

double getRightVelocity() {
  return velocity_right;
}

double getAngularVelocity() {
  return (velocity_right - velocity_left) / WHEELBASE_DIAMETER;
}

void checkEncodersZeroVelocity(void) {
  // Sets the wheel velocity to 0 if we haven't see an edge in a while
  unsigned long curr_time = micros();
  if (curr_time > prev_pulse_time_left + 100000) {
    velocity_left = 0;
  }
  if (curr_time > prev_pulse_time_right + 100000) {
    velocity_right = 0;
  }
}

void leftEncoderRisingEdge(void) {
  unsigned long curr_time = micros();

  int direction;
  if (digitalRead(PIN_ENCODER_LEFT_A) == !INVERT_ENCODER_LEFT) {
    direction = 1;
  } else {
    direction = -1;
  }

  if (direction * velocity_left < 0) {
    velocity_left = 0;
  } else {
    // Otherwise, convert the period of our pulse in mm/second
    velocity_left = left_motor_constant * direction * VELOCITY_COEFF / (curr_time - prev_pulse_time_left);
  }
  ticks_left += direction;

  prev_pulse_time_left = curr_time;
}

void rightEncoderRisingEdge(void) {
  unsigned long curr_time = micros();

  int direction;
  if (digitalRead(PIN_ENCODER_RIGHT_A) == !INVERT_ENCODER_RIGHT) {
    direction = -1;
  } else {
    direction = 1;
  }

  if (direction * velocity_right < 0) {
    velocity_right = 0;
  } else {
    // Otherwise, convert the period of our pulse in mm/second
    velocity_right = direction * VELOCITY_COEFF / (curr_time - prev_pulse_time_right);
  }
  ticks_right += direction;

  prev_pulse_time_right = curr_time;
}
