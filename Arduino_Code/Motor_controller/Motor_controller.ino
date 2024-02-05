/*Program for controlling a motor speed using pwm
Motor used : N20 motor
Development board :DOIT Esp32 DevKit v1*/

//pin connections
#define L_FORW 26
#define L_BACK 27
#define R_FORW 32
#define R_BACK 33
#define enable2Pin1 5
#define enable1Pin1 25

int rightWheelPwm=200; //can have values from 0 to 255
int leftWheelPwm=200;
void setup() {
    //pinmode definition
    pinMode(L_FORW,OUTPUT);
    pinMode(L_BACK,OUTPUT);
    pinMode(R_FORW,OUTPUT);
    pinMode(R_BACK,OUTPUT);
    pinMode(enable1Pin1,OUTPUT);
    pinMode(enable2Pin1,OUTPUT);
    //setup pwm parameters
    const int freq = 30000;
    const int pwmChannel1 = 0;
    const int pwmChannel2 = 1;
    const int resolution = 8;
    int dutyCycle = 200;

  
    ledcSetup(pwmChannel1, freq, resolution);
    ledcAttachPin(enable1Pin1, pwmChannel1);

    ledcSetup(pwmChannel2, freq, resolution);
    ledcAttachPin(enable2Pin1, pwmChannel2);

    Serial.begin(9600);


    digitalWrite(L_FORW,HIGH);
    digitalWrite(L_BACK,LOW);
    digitalWrite(R_FORW,HIGH);
    digitalWrite(R_BACK, LOW);
    ledcWrite(pwmChannel1,leftWheelPwm);
    ledcWrite(pwmChannel2,rightWheelPwm);

}



void loop() {
  // put your main code here, to run repeatedly:

}

