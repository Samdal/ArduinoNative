#define AN_IMPL
#include "ArduinoNative.hpp"

void setup()
{
        Serial.begin(9600);
#ifdef ArduinoNative
        Serial.an_take_input();
#endif
}

void loop()
{
        while(Serial.available()) {
                Serial.println((char)Serial.read());
        }
}
