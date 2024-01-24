#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

#define LED_PIN 2
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

#include <math.h>
#define L_FORW 16
#define L_BACK 17
#define R_FORW 32
#define R_BACK 33
#define enable2Pin1 15
#define enable1Pin1 5

float mapPwm(float x, float out_min, float out_max);

int PWM_MIN =181;
int PWMRANGE =255;

  const int freq = 30000;
  const int pwmChannel1 = 0;
  const int pwmChannel2 = 1;
  const int resolution = 8;

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

//twist message cb
void subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  // if velocity in x direction is 0 turn off LED, if 1 turn on LED
  digitalWrite(LED_PIN, (msg->linear.x == 0) ? LOW : HIGH);
  float linear = msg->linear.x;
  float angular = msg->angular.z;
  float x = max(min(linear, 1.0f), -1.0f);
  float z = max(min(angular, 1.0f), -1.0f);

  float l = (x - z) / 2;
  float r = (x + z) / 2;

  uint16_t lPwm = mapPwm(fabs(l), PWM_MIN, PWMRANGE);
  uint16_t rPwm = mapPwm(fabs(r), PWM_MIN, PWMRANGE);

    digitalWrite(L_FORW, l > 0);
    digitalWrite(L_BACK, l < 0);
    digitalWrite(R_FORW, r > 0);
    digitalWrite(R_BACK, r < 0);
    ledcWrite(pwmChannel1, lPwm);
    ledcWrite(pwmChannel2, rPwm);
}

void setup() {

    pinMode(L_FORW,OUTPUT);
    pinMode(L_BACK,OUTPUT);
    pinMode(R_FORW,OUTPUT);
    pinMode(R_BACK,OUTPUT);
    pinMode(enable1Pin1,OUTPUT);
    pinMode(enable2Pin1,OUTPUT);

    ledcSetup(pwmChannel1, freq, resolution);
    ledcAttachPin(enable1Pin1, pwmChannel1);

    ledcSetup(pwmChannel2, freq, resolution);
    ledcAttachPin(enable2Pin1, pwmChannel2);


  set_microros_transports();
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(2000);

  allocator = rcl_get_default_allocator();

   //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

}

void loop() {
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}

float mapPwm(float x, float out_min, float out_max)
{
  return x * (out_max - out_min) + out_min;
}