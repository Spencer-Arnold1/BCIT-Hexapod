#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

class myServo {
    public:
        myServo(int sda,int scl,int min_freq,int max_freq);
        void start();
        void move(int servo_num, int pos);
   
    private:
        int MINFREQ;
        int MAXFREQ;
        int SCL_PIN;
        int SDA_PIN;
        Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);
        Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41);

};