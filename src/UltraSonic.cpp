#include <Arduino.h>
class UltraSonic {
    public:
        int tpin;
        int epin;
        UltraSonic(int trigger, int echo) {
            tpin = trigger;
            epin = echo;
        }

        void Setup() {
            pinMode(tpin, OUTPUT);
            pinMode(epin, INPUT);
        }

        int Read() {
            digitalWrite(tpin, LOW);
            delayMicroseconds(2);
            digitalWrite(tpin, HIGH);
            delayMicroseconds(10);
            digitalWrite(tpin, LOW);

            const unsigned long duration = pulseIn(epin, HIGH);
            return duration/29/2;
        }
};