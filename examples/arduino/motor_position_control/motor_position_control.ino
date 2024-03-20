/*
  Position control example of a motor.
  An example of controlling a motor/gearbox's shaft position using an absolute encoder, an optional speed sensor and PID.

  theta = angular position (angle)
  omega_dot = angular velocity

  Todo: add control rate and sample time
*/
#include <Arduino.h>
#include <Servo.h>

// Todo: declare global variables and constants
float RAD2DEG = 0.0;
float DEG2RAD = 0.0;

float DEGREES_PER_SECOND2RADS_PER_SECOND = 0.0;
float RADS_PER_SECONDS2DEGREES_PER_SECOND = 0.0;

float RADS_PER_SECOND2REVS_PER_MINUTE = 0.0;
float REVS_PER_MINUTE2RADS_PER_SECONDS = 0.0;

float DEGREES_PER_SECOND2REVS_PER_MINUTE = DEGREES_PER_SECOND2RADS_PER_SECOND * RADS_PER_SECOND2REVS_PER_MINUTE;
float REVS_PER_MINUTE2DEGREES_PER_SECOND = REVS_PER_MINUTE2RADS_PER_SECONDS * RADS_PER_SECONDS2DEGREES_PER_SECOND;

// todo: setup limits for saturation, angle limits (from calibration with limit switches)
int throttleControlPin = 10;  // use a PWM pin
const int minPWM = 1200;  // 1000. To saturate the PWM commands
const int maxPWM = 1800;  //  2000. To saturate the PWM commands

// todo: setup fmap function

// todo: setup calibration routing

// todo: (optional) setup encoder interrupt function

// setup the motor
Servo electronicSpeedController;

void setup() {
  // This setup function is run once.
  Serial.begin(115200); // init serial communication

  // Initialize the motor driver
  electronicSpeedController.attach(throttleControlPin);

  // Setup direction, gains and ratios to make directions uniform
  bool encoder_inverted = false;  // if the encoder does not read positive for left movement, set to True
  bool motor_inverted = false;  // if the motor does not turn left for positive commands, set to True
  bool gearbox_inverted = false;  // if the gearbox output shaft does not turn left for positive commands, set to True

  float encoder_gain = 1.0; // 1.0 if encoder +ve corresponds to left movement, else -1.0
  float motor_gain = 1.0; // 1.0 if the motor moves left for positive commands, else -1.0
  float gearbox_gain = 1.0; // 1.0 if the gearbox output shaft moves left for positive commands, else -1.0

  if (encoder_inverted) == false {encoder_gain = -1.0;}
  if (motor_inverted) == false {motor_gain = -1.0;}
  if (gearbox_inverted) == false {gearbox_gain = -1.0;}

  float gear_ratio = 0.0;
  int n_teeth_motor_shaft = 0; // optionally used to calculate the gear ratio
  int n_teeth_gearbox_shaft = 0; // optionally used to calculate the gear ratio
  if (n_teeth_motor_shaft ~= 0) && (n_teeth_gearbox_shaft ~= 0) {gear_ratio = n_teeth_motor_shaft/n_teeth_gearbox_shaft;}

  // Setup absolute position encoder. todo
  float current_motor_direction = 1.0; // if current_omega < or > 0. Could be used for motors with separate direction enable pins.
  float desired_motor_direction = 1.0; // if current_omega < or > 0. Could be used for motors with separate direction enable pins.
  // setup position to velocity mapping, i.e differentiate position, optionally add a low pass filter

  // Setup offsets, i.e if the angle shaft position is not the same as the absolute encoders position
  float angle_offset = 0.0;

  // state estimates
  int current_pwm = 1500;
  float current_speed_encoder = 0.0f;  // the speed reported by the encoder either directly or indirectly.
  float current_speed_esc = 0.0f;  // the speed reported by the ESC. Contains speed fused from hall sensors and FOC current measurements 
  float current_speed_motor = 0.0f;  // the speed of the motor shaft either measured directly or indirectly
  float current_speed_gearbox = 0.0f;  // the speed of the gearboxes shaft either measured directly or indirectly.
  float current_speed_fused = 0.0f;  // the final speed estimate from fusing all measurements

  float current_angle_encoder = 0.0f;  // the angle/position reported by the encoder either directly or indirectly.
  float current_angle_esc = 0.0f;  // the angle/position reported by the ESC. Contains angle/position fused from hall sensors and FOC current measurements 
  float current_angle_motor = 0.0f;  // the angle/position of the motor shaft either measured directly or indirectly
  float current_angle_gearbox = 0.0f;  // the angle/position of the gearboxes shaft either measured directly or indirectly.
  float current_angle_fused = 0.0f;   // the final angle/position estimate from fusing all measurements

  // Motor input parameters. Todo: setup servo, pwm, voltage, pwm_mapping, normalized_pwm, angular_velocity,
  bool motor_enabled = true;
  int pwm_frequency = 0; // might not be configurable
  int target_pwm = 1500;  // the input to the motor
  float normalized_pwm_min = -1.0;  // [-1, 1] or [-100, 0]
  float normalized_pwm_max = 1.0;  // [-1, 1] or [-100, 0]
  float motor_voltage_limit = 12.0f;

  // Setup Limit switches
  bool left_limit_switch_pressed = false;
  bool right_limit_switch_pressed = false;

  // Motor speed control parameters. 
  // In this example, refers to the speed of the motors shaft. 
  // Obtained from (1) the ESC, (2) an encoder or (3) estimated by differentiating the position
  float speed_Kp = 0.1;
  float speed_Ki = 0.0;
  float speed_Kd = 0.0;
  float speed_Kd_filter = 0.0;
  float speed_feedforward = 0.0f;
  float speed_output_ramp = 0.0f;
  float target_speed = 0.0f;  // [rads/s]. Desired motor shaft speed

  // Motor position control parameters. 
  // In this example, refers to the angle of the gearbox's shaft.
  // In this example, refers to the speed of the motors shaft. 
  // Obtained from (1) an absolute or ABI encoder [recommended], (2) the ESC or (3) estimated by integrating the speed.
  float angle_Kp = 0.1;
  float angle_Ki = 0.0;
  float angle_Kd = 0.0;
  float angle_Kd_filter = 0.0;
  float angle_feedforward = 0.0f;
  float angle_output_ramp = 0.0f;
  float target_angle = 0.0f;  // [rads]. Desired gearbox shaft angle
  
  electronicSpeedController.writeMicroseconds(target_pwm);
  delay(1000);
  Serial.println("Initialization complete.");
}

void loop() {
  // put your main code here, to run repeatedly:

  // get position. todo
  // get speed. todo
  // get limit switches state. todo, only run the loop if the limit switches are not pressed and/or if the angle is within the valid range.
  

  // convert speed and position to reference frame, e.g to motor shaft or gearbox shaft. todo

  // get target speed and position in reference frame and run PID. todo
  target_speed = PositionPID();  // takes in the desired position and outputs the desired angular velocity
  target_normalized_pwm = SpeedPID();  // takes in the desired angular velocity and outputs the normalized PWM

  // map PID command to motor units. todo
  target_pwm = map();  // unnormalized the PWM signal and optionally set enable pin

  // send motor input command to motor
  electronicSpeedController.writeMicroseconds(target_pwm);
  delay(50);
  current_pwm = target_pwm;
}
