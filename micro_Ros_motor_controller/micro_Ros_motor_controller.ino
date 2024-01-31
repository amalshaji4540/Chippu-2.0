#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32.h>

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

rcl_publisher_t publisher1;
rcl_publisher_t publisher2;
std_msgs__msg__Int32 encodervalue_l;
std_msgs__msg__Int32 encodervalue_r;
rcl_timer_t timer;

#define LED_PIN 2
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

#include <math.h>
#define L_FORW 27
#define L_BACK 26
#define R_FORW 32
#define R_BACK 33
#define enable2Pin1 5
#define enable1Pin1 25

int encoderPin1 = 18; //Encoder Output 'A' must connected with intreput pin of arduino.
int encoderPin2 = 21; //Encoder Otput 'B' must connected with intreput pin of arduino.
int encoderPin3 = 23;
int encoderPin4 = 15;
volatile long EncoderCount_l=0;
volatile long EncoderCount_r=0;


int PWM_MIN =100;
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

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher1, &encodervalue_l, NULL));
    RCSOFTCHECK(rcl_publish(&publisher2, &encodervalue_r, NULL));
    
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

  uint16_t lPwm = map(l,-1,1,PWM_MIN,PWMRANGE);
  uint16_t rPwm = map(r,-1,1,PWM_MIN,PWMRANGE);

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
    pinMode(enable1Pin1,INPUT);
    pinMode(enable2Pin1,INPUT);
    attachInterrupt(digitalPinToInterrupt(encoderPin1), updateEncoder_l,RISING);
    pinMode(encoderPin3, INPUT); 
    pinMode(encoderPin4, INPUT);
    attachInterrupt(digitalPinToInterrupt(encoderPin3), updateEncoder_r,RISING);


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

  // create publisher for left wheel
  RCCHECK(rclc_publisher_init_default(
    &publisher1,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "left_wheel_tick"));
    //create a publisher for right wheel
    RCCHECK(rclc_publisher_init_default(
    &publisher2,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "right_wheel_tick"));

  const unsigned int timer_timeout = 500;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  encodervalue_l.data = 0;
  encodervalue_r.data = 0;

}

void loop() {
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}

void updateEncoder_l(){
  if(digitalRead(encoderPin1)>digitalRead(encoderPin2))
  {
    EncoderCount_l++;
  }
  else
  {
    EncoderCount_l--;
  }

  encodervalue_l.data =EncoderCount_l;
  
}

void updateEncoder_r(){
  if(digitalRead(encoderPin3)>digitalRead(encoderPin4))
  {
    EncoderCount_r++;
  }
  else
  {
    EncoderCount_r--;
  }

  encodervalue_r.data =EncoderCount_r;
}

