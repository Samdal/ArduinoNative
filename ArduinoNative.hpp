#ifndef ArduinoNative
#define ArduinoNative

#include <iostream>
#include <algorithm>
#include <sstream>
#include <chrono>
#include <thread>
#include <stdint.h>
#include <cstring>
#include <cstdlib>
#include <ctype.h>
#include <cmath>
#include <string>

/* CONSTANTS */
#define String std::string
#define LOW 0
#define HIGH 1
enum {
        INPUT,
        OUTPUT,
        INPUT_PULLUP
} an_pin_mode;

#define byte uint8_t
#define word uint16_t

/* DEBUG MESSAGES */
#ifdef AN_DEBUG_ALL
#define AN_DEBUG_DIGITALREAD
#define AN_DEBUG_DIGITALWRITE
#define AN_DEBUG_ANALOGREAD
#define AN_DEBUG_ANALOGWRITE
#endif

/* BOARD DEFINITIONS */
#if defined(AN_BOARD_NANO) || defined(AN_BOARD_PRO_MINI)

#define MAX_PINS 21

#define LED_BUILTIN 13
#define A0 14
#define A1 15
#define A2 16
#define A3 17
#define A4 18
#define A5 19
#define A6 20
#define A7 21

#else // Default is Arduino Uno

#define MAX_PINS 19

#define LED_BUILTIN 13
#define A0 14
#define A1 15
#define A2 16
#define A3 17
#define A4 18
#define A5 19

#endif

uint8_t an_pin_cycle[MAX_PINS] = {0};
float an_pin_voltage[MAX_PINS] = {0};

/* FUNCTION DEFINITIONS */

// Digital I/O
bool digitalRead(uint8_t pin);
void digitalWrite(uint8_t pin, bool value);
#define pinMode(pin, mode)

// Analog I/O
uint16_t analogRead(uint8_t pin);
// void analogReference();
void analogWrite(uint8_t pin, uint8_t value);
void an_set_voltage(uint8_t pin, float voltage);

// Advanced I/O
// void noTone(uint8_t pin);
// unsigned long pulseIn(uint8_t pin, bool value);
// unsigned long pulseInLong(uint8_t pin, bool value);
// unsigned long tone(uint8_t pin, unsigned frequenzy);

//Time
inline void delay(unsigned long milliseconds);
inline void delayMicroseconds(unsigned long microseconds);
unsigned long micros(void);
unsigned long millis(void);

// Math
#define constrain(x, a, b) ({x < a ? a : x; x > b ? b : x;})
#define map(x, fL, fH, tL, tH) (lround((x - fL) * (tH - tL) / (fH - fL) + tL))
#define max(a, b) (a > b ? a : b)
#define min(a, b) (a < b ? a : b)
#define sq(x) (x*x)
// the rest are included in math.h
// the same with all of the trigonomitry functions

// Characthers
inline bool isAlpha(char thisChar);
inline bool isAlphaNumeric(char thisChar);
inline bool isAscii(char thisChar);
inline bool isControl(char thisChar);
inline bool isDigit(char thisChar);
inline bool isGraph(char thisChar);
inline bool isHexadecimalDigit(char thisChar);
inline bool isLowerCase(char thisChar);
inline bool isPrintable(char thisChar);
inline bool isPunct(char thisChar);
inline bool isSpace(char thisChar);
inline bool isUpperCase(char thisChar);
inline bool isWhitespace(char thisChar);

// Random Numbers
inline long random(long max);
inline long random(long min, long max);
inline void randomSeed(unsigned long seed);

// Bits and Bytes
#define bit(n) (1 << n)
#define bitClear(x, n) (x & ~(1 << n))
#define bitRead(x, n) (x & (1 << n))
#define bitSet(x, n) (x |= (1 << n))
#define bitWrite(x, n, b) (x = ((x & ~(1 << n)) | (b << n)))
#define highByte(x) ((uint8_t) ((x) >> 8))
#define lowByte(x) ((uint8_t) ((x) & 0xff))

// External Interrupts

// Interrupts

// Implimentation
#ifdef AN_IMPL

// Communication
class an_serial
{
private:
        std::string buffer;
public:
        void an_take_input()
        {
                std::cout << "ArduinoNative is requesting Serial input: ";
                std::cin >> buffer;
        }

        inline size_t available() {return buffer.length();}
        inline size_t availableForWrite() {return SIZE_MAX;}
        inline void begin(unsigned speed) {}
        inline void begin(unsigned speed, int config) {}
        inline void end() {}
        inline void flush() {}
        inline void setTimeout(long new_time) {}
        int peek()
        {
                std::string buffrev = buffer;
                std::reverse(buffrev.begin(), buffrev.end());
                return buffrev.length() > 0 ? int(buffrev.back()) : 0;
        }
        int read()
        {
                std::string buffrev = buffer;
                std::reverse(buffrev.begin(), buffrev.end());
                int readByte = buffrev.length() > 0 ? int(buffrev.back()) : 0;
                buffrev.pop_back();
                std::reverse(buffrev.begin(), buffrev.end());
                buffer = buffrev;
                return readByte;
        }
        size_t readBytes(char readbuffer[], unsigned length)
        {
                std::stringstream buff;
                for (; length != 0; length--) {
                        if (!this->available())
                                break;
                        buff << (char)this->read();
                }
                std::strcpy(readbuffer, buff.str().c_str());
                return buff.str().length();
        }
        size_t readBytesUntill(char readbuffer[], unsigned length)
        {
                std::stringstream buff;
                for (; length != 0; length--) {
                        if (!this->available())
                                break;
                        buff << (char)this->read();
                }
                std::strcpy(readbuffer, buff.str().c_str());
                return buff.str().length();
        }
        //float parseFloat() {}
        template <typename T>
        size_t print(T val)
        {
                std::cout << val;
                std::stringstream s;
                s << val;
                return s.str().length();
        }
        template <typename T>
        size_t println(T val)
        {
                std::cout << val << std::endl;
                std::stringstream s;
                s << val;
                return s.str().length() + 1;
        }
};

void setup();
void loop();

unsigned long an_start_time_ms;
unsigned long an_start_time_µs;
an_serial Serial;

// start program
int main()
{
        // Initialize
        an_start_time_ms = millis();
        an_start_time_µs = micros();

        // run setup() and loop()
        setup();
        for (;;) loop();
}

// Digital I/O
bool digitalRead(uint8_t pin)
{
        if (pin > MAX_PINS) {
                std::cout << "ERROR: PIN " << std::to_string(pin) << " IS NOT DEFINED\n";
                exit(1);
        }
#ifdef AN_DEBUG_DIGITALREAD
        std::cout << "Read pin: " << std::to_string(pin) << " is " << std::to_string(an_pin_voltage[pin] > 3) << "\n";
#endif
        return an_pin_voltage[pin] > 3;
}

void digitalWrite(uint8_t pin, bool val)
{
        if (pin > MAX_PINS) {
                std::cout << "ERROR: PIN " << std::to_string(pin) << " IS NOT DEFINED\n";
                exit(1);
        }
        an_pin_cycle[pin] = val * 255;
        an_pin_voltage[pin] = val * 5.0;
#ifdef AN_DEBUG_DIGITALWRITE
        std::cout << "Pin: " << std::to_string(pin) << " is now " << std::to_string(an_pin_voltage[pin] > 3)  << "\n";
#endif
}

// Analog I/O
uint16_t analogRead(uint8_t pin)
{
        if (pin > MAX_PINS) {
                std::cout << "ERROR: PIN " << pin << " IS NOT DEFINED\n";
                exit(1);
        }
        uint16_t val = map(an_pin_voltage[pin], 0.0, 5.0, 0, 1023);
        val = constrain(val, 0, 1023);
#ifdef AN_DEBUG_ANALOGREAD
        std::cout << "Analog pin: " << std::to_string(pin) << " is " << std::to_string(val) << "\n";
#endif
        return val;
}

void analogWrite(uint8_t pin, uint8_t val)
{
        if (pin > MAX_PINS) {
                std::cout << "ERROR: PIN " << pin << " IS NOT DEFINED\n";
                exit(1);
        }
        val = constrain(val, 0, 255);
        an_pin_cycle[pin] = val;
        an_pin_voltage[pin] = map(val, 0, 255, 0.0, 5.0);
#ifdef AN_DEBUG_ANALOGWRITE
        std::cout << "Duty cycle on pin: " << std::to_string(pin) << " is now " << std::to_string(an_pin_cycle[pin]) << "\n";
#endif
}

void an_set_voltage(uint8_t pin, float voltage)
{
        if (pin > MAX_PINS) {
                std::cout << "ERROR: PIN " << std::to_string(pin) << " IS NOT DEFINED\n";
                exit(1);
        }
        an_pin_voltage[pin] = voltage;
}

void an_request_voltage(uint8_t pin)
{
        if (pin > MAX_PINS) {
                std::cout << "ERROR: PIN " << std::to_string(pin) << " IS NOT DEFINED\n";
                exit(1);
        }
        std::cout << "set voltage of pin " << std::to_string(pin) << " to: ";
        std::cin >> an_pin_voltage[pin];
}

// Time
void delay(unsigned long milliseconds)
{
        std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}
void delayMicroseconds(unsigned long microseconds)
{
        std::this_thread::sleep_for(std::chrono::microseconds(microseconds));
}

unsigned long micros()
{
        auto duration = std::chrono::system_clock::now().time_since_epoch();
        return std::chrono::duration_cast<std::chrono::microseconds>(duration).count() - an_start_time_µs;
}
unsigned long millis()
{
        auto duration = std::chrono::system_clock::now().time_since_epoch();
        return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() - an_start_time_ms;
}

// Characters
bool isAlpha(char thisChar)            {return isalpha(thisChar);}
bool isAlphaNumeric(char thisChar)     {return isalnum(thisChar);}
bool isAscii(char thisChar)            {return true;}
bool isControl(char thisChar)          {return iscntrl(thisChar);}
bool isDigit(char thisChar)            {return isdigit(thisChar);}
bool isGraph(char thisChar)            {return isgraph(thisChar);}
bool isHexadecimalDigit(char thisChar) {return isxdigit(thisChar);}
bool isLowerCase(char thisChar)        {return islower(thisChar);}
bool isPrintable(char thisChar)        {return isprint(thisChar);}
bool isPunct(char thisChar)            {return ispunct(thisChar);}
bool isSpace(char thisChar)            {return isspace(thisChar);}
bool isUpperCase(char thisChar)        {return isupper(thisChar);}
bool isWhitespace(char thisChar)       {return isspace(thisChar);}

// Random Numbers
long random(long max) {return rand() % max;}
long random(long min, long max) {return min + rand() % (max - min);}
void randomSeed(long seed) {srand(seed);}

#undef AN_IMPL
#endif // AN_IMPL

#endif // ArduinoNative
