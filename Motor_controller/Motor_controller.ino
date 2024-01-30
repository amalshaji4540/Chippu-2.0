#include <math.h>

#define L_FORW 25
#define L_BACK 26
#define R_FORW 32
#define R_BACK 33
#define enable2Pin1 15
#define enable1Pin1 5

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
    float x = max(min(0.5f, 1.0f), -1.0f);
    float z = max(min(0.0f, 1.0f), -1.0f);

    float l = (x - z) / 2;
    float r = (x + z) / 2;

    uint16_t lPwm = map(l,-1,1,181,255);
    uint16_t rPwm = map(r,-1,1,181,255);

    digitalWrite(L_FORW, l > 0);
    digitalWrite(L_BACK, l < 0);
    digitalWrite(R_FORW, r > 0);
    digitalWrite(R_BACK, r < 0);
    ledcWrite(pwmChannel1, lPwm);
    ledcWrite(pwmChannel2, rPwm);


    
    Serial.println(x);
    Serial.println(z);

}



void loop() {
  // put your main code here, to run repeatedly:

}

float mapPwm(float x, float out_min, float out_max)
{
  return x * (out_max - out_min) + out_min;
}