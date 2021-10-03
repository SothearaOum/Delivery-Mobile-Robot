// Import library, ROS library
#include <ros.h>
#include <geometry_msgs/Twist.h>

// define motor pin
#define left_motor_Rpwm_pin 5
#define left_motor_Lpwm_pin 4
#define right_motor_Rpwm_pin 7
#define right_motor_Lpwm_pin 6

// preassign variable
int left_pwm; // pwm for left wheel
int right_pwm; // pwm for right wheel

// mobile robot parameter
float V;      // linear velocity
float omega;  // angular velocity
const float L = 0.5;  // robot base length in m
const float r = 0.2;  // robot wheel radius in m
float left_speed;  // omega left wheel
float right_speed; // omega right wheel

// callback function for subcribe to twist msg
void twist_cb(const geometry_msgs::Twist &msg)
{
  // get linear and angular velocity from twist
  V = msg.linear.x;
  omega = msg.angular.z;

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

ros::NodeHandle nodehandle; // create ros node handle name: "nodehandle"
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &twist_cb); // create ros sub, syntax--> ros::Subscriber<msg_categorys::msgs > sub("/topics ", &callback function); "&"is necessary

void setup() {
  // set pins motor output
  pinMode(left_motor_Rpwm_pin, OUTPUT);
  pinMode(left_motor_Lpwm_pin, OUTPUT);
  pinMode(right_motor_Rpwm_pin, OUTPUT);
  pinMode(right_motor_Lpwm_pin, OUTPUT);

  // ROS nodehandle
  nodehandle.initNode(); // init nodehandle
  nodehandle.subscribe(sub); // start subscribe to sub
}

void loop() {
  nodehandle.spinOnce();
  dalay(1);
}
