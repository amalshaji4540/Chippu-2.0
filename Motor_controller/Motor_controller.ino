#include <math.h>

#define L_FORW 26
#define L_BACK 27
#define R_FORW 32
#define R_BACK 33
#define enable2Pin1 5
#define enable1Pin1 25

float mapPwm(float x, float out_min, float out_max);

int PWM_MIN =181;
int PWMRANGE =255;

void setup() {
  // put your setup code here, to run once:
    pinMode(L_FORW,OUTPUT);
    pinMode(L_BACK,OUTPUT);
    pinMode(R_FORW,OUTPUT);
    pinMode(R_BACK,OUTPUT);
    pinMode(enable1Pin1,OUTPUT);
    pinMode(enable2Pin1,OUTPUT);

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
    // float x = max(min(0.5f, 1.0f), -1.0f);
    // float z = max(min(0.0f, 1.0f), -1.0f);

    // float l = (x - z) / 2;
    // float r = (x + z) / 2;

    // uint16_t lPwm = map(l,-1,1,181,255);
    // uint16_t rPwm = map(r,-1,1,181,255);

    digitalWrite(L_FORW,HIGH);
    digitalWrite(L_BACK,LOW);
    digitalWrite(R_FORW,HIGH);
    digitalWrite(R_BACK, LOW);
    ledcWrite(pwmChannel1,188);
    ledcWrite(pwmChannel2, 188);


    
    // Serial.println(x);
    // Serial.println(z);

}



void loop() {
  // put your main code here, to run repeatedly:

}

float mapPwm(float x, float out_min, float out_max)
{
  return x * (out_max - out_min) + out_min;
}