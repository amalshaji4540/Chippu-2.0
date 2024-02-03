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

  leftWheel.initPID(1.8,5,0);
  rightWheel.initPID(1.8,5,0);
  attachInterrupt(digitalPinToInterrupt(leftWheel.EncoderPinB),updateEncoderL, RISING);
  attachInterrupt(digitalPinToInterrupt(rightWheel.EncoderPinA),updateEncoderR, RISING);
  ledcSetup(pwmChannelL, freq, resolution);
  ledcAttachPin(leftWheel.Enable, pwmChannelL);
  ledcSetup(pwmChannelR, freq, resolution);
  ledcAttachPin(rightWheel.Enable, pwmChannelR);
  Serial.begin(9600);



}

void loop() {
  
  float a=leftWheel.getRpm();
  float b=rightWheel.getRpm();
  // leftWheel.plot(a,requiredrpm_LW);
  // Serial.println(a);
  // Serial.println(leftWheel.EncoderCount);
  float actuating_signal_LW=leftWheel.pid(requiredrpm_LW,a);
  float actuating_signal_RW=rightWheel.pid(requiredrpm_RW,b);
  // Serial.println(leftWheel.getRpm());
  leftWheel.moveBase(actuating_signal_LW,threshold,pwmChannelL);
  rightWheel.moveBase(actuating_signal_RW,threshold,pwmChannelR);
  delay(10);

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





