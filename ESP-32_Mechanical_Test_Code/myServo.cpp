#include "myServo.h"

myServo::myServo(int sda, int scl,int min_freq, int max_freq) 
{
    MINFREQ = min_freq;
    MAXFREQ = max_freq;
    SCL_PIN = scl;
    SDA_PIN = sda;
}

void myServo::start()
{
    Wire.begin(SDA_PIN,SCL_PIN);

    pwm1.begin();
    pwm2.begin();

    pwm1.setPWMFreq(50);
    pwm2.setPWMFreq(50);
}

void myServo::move(int servo_num, int angle)
{
    int pulseLength = map(angle, -90, 90, MINFREQ, MAXFREQ);

    if (servo_num < 9) {
        pwm1.setPWM(servo_num, 0, pulseLength);
    } else {
        pwm2.setPWM(servo_num - 9, 0, pulseLength);
    }
}
