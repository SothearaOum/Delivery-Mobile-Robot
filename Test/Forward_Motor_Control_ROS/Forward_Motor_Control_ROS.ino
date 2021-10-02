// Import library
// ROS library
//#include <ros.h>
//#include <geometry_msgs/Twist.h>

// START INITIALIZATION ======================================================
//ros::NodeHandle nodehandle;         // initialize ros node handle name: "nodehandle"
//ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &onTwist);  // initialize ros subscriber to twist msg type
//ros::Subscriber<msg_categorys::msgs > sub("/topics ", &callback function);  // syntax "&"is necessary

void onTwist(const geometry_msgs::Twist &msg)
{
  // Cap values at [-1 .. 1]
  float x = max(min(msg.linear.x, 1.0f), -1.0f);
  float z = max(min(msg.angular.z, 1.0f), -1.0f);

  // Calculate the intensity of left and right wheels. Simple version.
  float l = (msg.linear.x - msg.angular.z) / 2;
  float r = (msg.linear.x + msg.angular.z) / 2;

  // Then map those values to PWM intensities. PWMRANGE = full speed, while PWM_MIN = the minimal amount of power at which the motors begin moving.
  uint16_t lPwm = mapPwm(fabs(l), PWM_MIN, PWMRANGE);
  uint16_t rPwm = mapPwm(fabs(r), PWM_MIN, PWMRANGE);

  // Set direction pins and PWM
  digitalWrite(L_FORW, l > 0);
  digitalWrite(L_BACK, l < 0);
  digitalWrite(R_FORW, r > 0);
  digitalWrite(R_BACK, r < 0);
  analogWrite(L_PWM, lPwm);
  analogWrite(R_PWM, rPwm);
}

// define motor pin
#define left_motor_Rpwm_pin 5
#define left_motor_Lpwm_pin 4
#define right_motor_Rpwm_pin 7
#define right_motor_Lpwm_pin 6

// preassign variable
int left_pwm;
int right_pwm;

// mobile robot parameter
float V;      // linear velocity
float omega;  // angular velocity
const float L = 0.5;  // robot base length in m
const float r = 0.2;  // robot wheel radius in m
float left_speed;
float right_speed;
// END INITIALIZATION ======================================================

void setup() {
  // set pins motor output
  pinMode(left_motor_Rpwm_pin, OUTPUT);
  pinMode(left_motor_Lpwm_pin, OUTPUT);
  pinMode(right_motor_Rpwm_pin, OUTPUT);
  pinMode(right_motor_Lpwm_pin, OUTPUT);
}

void loop() {

  // get V and omega from controller
  V = 0.2;
  omega = 0.01;

  // convert V and omega to each wheel speed
  left_speed = (2 * V - L * omega) / (2 * r);
  right_speed = (2 * V + L * omega) / (2 * r);

  // convert each wheel speed to pwm value, y=Ax+b, A=0.22,b=0.1
  left_pwm = 0.22 * left_speed + 0.1;
  right_pwm = 0.22 * right_speed + 0.1;

  // limit pwm
  if (left_pwm < -250) {
    left_pwm = -250;
  }
  if (left_pwm > 250) {
    left_pwm = 250;
  }
  if (right_pwm < -250) {
    right_pwm = -250;
  }
  if (right_pwm > 250) {
    right_pwm = 250;
  }

  // write pwm to pin
  if ( left_pwm < 0 && right_pwm < 0) {
    // left reverse , right reverse
    analogWrite(left_motor_Rpwm_pin, 0);
    analogWrite(left_motor_Lpwm_pin, left_pwm * -1);
    analogWrite(right_motor_Rpwm_pin, 0);
    analogWrite(right_motor_Lpwm_pin, right_pwm * -1);
  }
  else if ( left_pwm > 0 && right_pwm > 0) {
    // left forward , right forward
    analogWrite(left_motor_Rpwm_pin, left_pwm);
    analogWrite(left_motor_Lpwm_pin, 0);
    analogWrite(right_motor_Rpwm_pin, right_pwm);
    analogWrite(right_motor_Lpwm_pin, 0);
  }
  else if ( left_pwm < 0 && right_pwm > 0) {
    // left reverse , right forward
    analogWrite(left_motor_Rpwm_pin, 0);
    analogWrite(left_motor_Lpwm_pin, left_pwm * -1);
    analogWrite(right_motor_Rpwm_pin, right_pwm);
    analogWrite(right_motor_Lpwm_pin, 0);
  }
  else if ( left_pwm > 0 && right_pwm < 0) {
    // left forward , right reverse
    analogWrite(left_motor_Rpwm_pin, left_pwm);
    analogWrite(left_motor_Lpwm_pin, 0);
    analogWrite(right_motor_Rpwm_pin, 0);
    analogWrite(right_motor_Lpwm_pin, right_pwm * -1);
  }
  else {
    // neutral
    analogWrite(left_motor_Rpwm_pin, 0);
    analogWrite(left_motor_Lpwm_pin, 0);
    analogWrite(right_motor_Rpwm_pin, 0);
    analogWrite(right_motor_Lpwm_pin, 0);
  }
}
