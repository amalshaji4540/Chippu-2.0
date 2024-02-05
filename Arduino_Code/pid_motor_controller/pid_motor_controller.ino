
/*Closed loop pid motor speed Controller.*/
#define encoderPin1 23
#define encoderPin2 15
#define forward 32
#define backward 33
#define enable 5

int pwm = 0;
volatile long EncoderCount_l;
volatile long prev_t;
volatile long prev_p;
float eintegral=0;
float ederivative=0;
float prev_e;

long prevT_i;
float velocity_2;
float v1;
float v1Filt = 0;
float v1Prev = 0;
float rpmFilt = 0;
float rpmPrev = 0;

// put your setup code here, to run once:
const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;
const int tick_per_revolution = 1057;


void setup() {


  pinMode(forward, OUTPUT);
  pinMode(backward, OUTPUT);
  pinMode(enable, OUTPUT);
  pinMode(encoderPin1, INPUT);
  pinMode(encoderPin2, INPUT);

  ledcSetup(pwmChannel, freq, resolution);
  ledcAttachPin(enable, pwmChannel);

  attachInterrupt(digitalPinToInterrupt(encoderPin1), updateEncoder_l, RISING);

  Serial.begin(9600);
}

void loop() {


  volatile long pos = EncoderCount_l;
  volatile long current_t = millis();
  float delta = ((float)current_t - prev_t) / 1.0e3;
  // Serial.println(delta);
  //encoder count per second
  v1 = ((float)pos - prev_p) / delta;
  prev_p = pos;
  prev_t = current_t;

  //digital filter for counter
  // v1Filt = 0.854*v1Filt + 0.0728*v1 + 0.0728*v1Prev;
  // v1Prev = v1;
  //Comparing filtered and unfiltered velocity by serial plotter
  // Serial.print("vel1:");
  // Serial.print(v1);
  // Serial.print(",");
  // Serial.print("Vel2:");
  // Serial.println(v1Filt);
  //increase pwm from 0 to 255 in 20 seconds.
  // pwm=255/20*micros()/1.0e6;
  delay(20);
  //count/second to rpm conversion
  float rpm = (v1*60 /1056);
  Serial.println(rpm);
  //rpm filter
  rpmFilt = 0.854 * rpmFilt + 0.0728 * rpm + 0.0728 * rpmPrev;
  rpmPrev = rpm;
  // Serial.print("RPM:");
  // Serial.print(rpm);
  // Serial.print(",");
  // Serial.print("RPM_f:");
  // Serial.println(rpmFilt);
  // setpoint (we are creating a square wave)
  // float vt = 15 * (sin(current_t / 1.0e3) > 0);
  float vt = 3;
  //error
  float e = vt - rpmFilt;
  //integral gain
  float ki=10;
  //integration of error
  eintegral=eintegral+e*delta;
  //derivative gain
  float kd=0.01;
  //derivative of error
  ederivative=(e-prev_e)/delta;
  prev_e=e;
  //proportional gain
  float kp =1.8;
  // u is the control output
  float u = (kp * e)+(ki*eintegral)+(kd*ederivative);

// Serial.println(u);

  int dir = 1;
  if (u < 0) {
    dir = -1;
  }
  int pwr =160+(int)fabs(u);
  if(pwr>255)
  {
    pwr=255;
  }
  movebase(dir, pwr);
  Serial.print("RPM:");
  Serial.print(rpmFilt);
  Serial.print(",");
  Serial.print("setpoint:");
  Serial.println(vt);
  // put your main code here, to run repeatedly:
}


void updateEncoder_l() {
  if (digitalRead(encoderPin1) > digitalRead(encoderPin2)) {
    EncoderCount_l++;
  } else {
    EncoderCount_l--;
  }
}

void movebase(int dir, int pwm) {
  ledcWrite(pwmChannel, pwm);
    if (dir == 1) {
    digitalWrite(forward, HIGH);
    digitalWrite(backward, LOW);
  } else if(dir==-1) {

    digitalWrite(forward, LOW);
    digitalWrite(backward, HIGH);

  }

}
