#define Motor1Pin1 16
#define Motor1Pin2 17
#define enable1Pin1 5
void setup() {
  // put your setup code here, to run once:
    pinMode(Motor1Pin1,OUTPUT);
    pinMode(Motor1Pin2,OUTPUT);
    pinMode(enable1Pin1,OUTPUT);

    const int freq = 30000;
    const int pwmChannel = 0;
    const int resolution = 8;
    int dutyCycle = 200;

    digitalWrite(Motor1Pin1,HIGH);
    digitalWrite(Motor1Pin2,LOW);

    ledcSetup(pwmChannel, freq, resolution);
    ledcAttachPin(enable1Pin1, pwmChannel);
    ledcWrite(pwmChannel, 160);


    
}

void loop() {
  // put your main code here, to run repeatedly:

}
