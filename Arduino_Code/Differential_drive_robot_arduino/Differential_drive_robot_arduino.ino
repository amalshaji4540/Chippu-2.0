/*Motor controller using micro_ros serial set_microros_transports*/
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
rcl_timer_t ControlTimer;

int callbackCount;

#define LED_PIN 2
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void timer_callback(rcl_timer_t* timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher1, &encodervalue_l, NULL));
    RCSOFTCHECK(rcl_publish(&publisher2, &encodervalue_r, NULL));
    
  }
}
void subscription_callback(const void *msgin) {
  callbackCount++;    

}
//pin declaration
//Left wheel
int8_t L_FORW =26;
int8_t L_BACK =27;
int8_t L_enablePin= 25;
int8_t L_encoderPin1 = 18; //Encoder Output 'A' must connected with intreput pin of arduino.
int8_t L_encoderPin2 = 21; //Encoder Otput 'B' must connected with intreput pin of arduino.
//right Wheel pins initialization
int8_t R_FORW =32;
int8_t R_BACK =33;
int8_t R_enablePin= 5;
int8_t R_encoderPin1 = 23;
int8_t R_encoderPin2 = 15;


float requiredrpm_LW=15;
int tickPerRevolution_LW=1050;
float requiredrpm_RW=15;
int tickPerRevolution_RW=1055;
int threshold=150;

//pwm parameters setup
const int freq = 30000;
const int pwmChannelL = 0;
const int pwmChannelR = 1;
const int resolution = 8;

class MotorController {
  public:
  int8_t Forward;
  int8_t Backward;
  int8_t Enable;
  int8_t EncoderPinA;
  int8_t EncoderPinB;
  volatile long EncoderCount ;
  volatile long CurrentPosition;
  volatile long PreviousPosition;
  volatile long CurrentTime;
  volatile long PreviousTime;
  volatile long CurrentTimeforError;
  volatile long PreviousTimeForError;
  float rpmFilt;
  float eintegral;
  float ederivative;
  float rpmPrev;
  float kp;
  float ki;
  float kd;
  float error;
  float previousError=0;
  int tick;

  MotorController(int8_t ForwardPin, int8_t BackwardPin, int8_t EnablePin, int8_t EncoderA, int8_t EncoderB,int tickPerRevolution){
    this->Forward = ForwardPin;
    this->Backward = BackwardPin;
    this->Enable = EnablePin;
    this->EncoderPinA = EncoderA;
    this->EncoderPinB = EncoderB;
    this->tick=tickPerRevolution;
    pinMode(Forward, OUTPUT);
    pinMode(Backward,OUTPUT);
    pinMode(EnablePin,OUTPUT);
    pinMode(EncoderPinA, INPUT);
    pinMode(EncoderPinB, INPUT);
  }
    void initPID(float proportionalGain, float integralGain, float derivativeGain){
    kp=proportionalGain;
    ki=integralGain;
    kd=derivativeGain;
  }

  float getRpm(){
    CurrentPosition=EncoderCount;
    CurrentTime=millis();
    float delta1=((float) CurrentTime-PreviousTime)/1.0e3;
    float velocity=((float)CurrentPosition-PreviousPosition)/delta1;
    float rpm=(velocity/tick)*60;
    rpmFilt = 0.854 * rpmFilt + 0.0728 * rpm + 0.0728 * rpmPrev;
    float rpmPrev = rpm;
    PreviousPosition=CurrentPosition;
    PreviousTime=CurrentTime;
    // Serial.println(rpmFilt);
    return rpmFilt;
  }

  float pid(float setpoint,float feedback){
      CurrentTimeforError=millis();
      float delta2=((float)CurrentTimeforError-PreviousTimeForError)/1.0e3;
      error = setpoint - feedback;
      eintegral=eintegral+(error*delta2);
      ederivative=(error-previousError)/delta2;
      float control_signal = (kp * error)+(ki*eintegral)+(kd*ederivative);

      previousError=error;
      PreviousTimeForError=CurrentTimeforError;
      // Serial.println(control_signal);
      return control_signal;
  }
  void moveBase(float ActuatingSignal,int threshold,int pwmChannel)
  { 
    if(ActuatingSignal>0){
      digitalWrite(Forward, HIGH);
      digitalWrite(Backward, LOW);
    }
    else{
      digitalWrite(Forward, LOW);
      digitalWrite(Backward, HIGH);
    }
    int pwm =threshold+(int)fabs(ActuatingSignal);
    if (pwm >255) 
      pwm=255;
    ledcWrite(pwmChannel, pwm);
  

  }
  void plot(float Value1, float Value2){
      Serial.print("Value1:");
      Serial.print(Value1);
      Serial.print(",");
      Serial.print("value2:");
      Serial.println(Value2);


  }
};
MotorController leftWheel(L_FORW,L_BACK,L_enablePin,L_encoderPin1,L_encoderPin2,tickPerRevolution_LW);
MotorController rightWheel(R_FORW,R_BACK,R_enablePin,R_encoderPin1,R_encoderPin2,tickPerRevolution_RW);



void setup() {
  // put your setup code here, to run once:

  leftWheel.initPID(1.8,5,0);
  rightWheel.initPID(1.8,5,0);
  attachInterrupt(digitalPinToInterrupt(leftWheel.EncoderPinB),updateEncoderL, RISING);
  attachInterrupt(digitalPinToInterrupt(rightWheel.EncoderPinA),updateEncoderR, RISING);
  ledcSetup(pwmChannelL, freq, resolution);
  ledcAttachPin(leftWheel.Enable, pwmChannelL);
  ledcSetup(pwmChannelR, freq, resolution);
  ledcAttachPin(rightWheel.Enable, pwmChannelR);

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
  
  const unsigned int samplingT = 10;
  RCCHECK(rclc_timer_init_default(
    &ControlTimer,
    &support,
    RCL_MS_TO_NS(samplingT),
    MotorControll_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_timer(&executor, &ControlTimer));


}

void loop() {
  // put your main code here, to run repeatedly:
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}

void MotorControll_callback(rcl_timer_t* timer, int64_t last_call_time){
    float linearVelocity=msg.linear.x;
    float angularVelocity=msg.angular.z;
    float vL=(linearVelocity-(angularVelocity*1/2))*20;
    float vR=(linearVelocity+(angularVelocity*1/2))*20;
    float currentRpmL=leftWheel.getRpm();
    float currentRpmR=rightWheel.getRpm();
  // leftWheel.plot(a,requiredrpm_LW);
  // Serial.println(a);
    // Serial.println(leftWheel.EncoderCount);
    float actuating_signal_LW=leftWheel.pid(vL,currentRpmL);
    float actuating_signal_RW=rightWheel.pid(vR,currentRpmR);
    // Serial.printiln(leftWheel.getRpm());
    leftWheel.moveBase(actuating_signal_LW,threshold,pwmChannelL);
    rightWheel.moveBase(actuating_signal_RW,threshold,pwmChannelR);
}

void updateEncoderL() {
    if (digitalRead(leftWheel.EncoderPinB) > digitalRead(leftWheel.EncoderPinA))
      leftWheel.EncoderCount++;
    else
      leftWheel.EncoderCount--;
  }
void updateEncoderR() {
    if (digitalRead(rightWheel.EncoderPinA) > digitalRead(rightWheel.EncoderPinB))
      rightWheel.EncoderCount++;
    else
      rightWheel.EncoderCount--;
  }


