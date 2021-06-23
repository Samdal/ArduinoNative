#ifndef ArduinoNative
#define ArduinoNative

#include <algorithm>
#include <bitset>
#include <cstring>
#include <cstdlib>
#include <ctype.h>
#include <cmath>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <stdint.h>
#include <string>
#include <thread>
#include <unordered_map>

/* CONSTANTS */
#define String std::string
#define LOW 0
#define HIGH 1
typedef enum {
        INPUT,
        OUTPUT,
        INPUT_PULLUP
} an_pin_mode_t;
typedef enum  {
        BIN,
        OCT,
        DEC,
        HEX
} an_print_format_t;
typedef enum : uint64_t {
        CHANGE,
        RISING,
        FALLING
} an_int_mode_t;
#define byte uint8_t
#define word uint16_t

/* DEBUG MESSAGES */
#ifdef AN_DEBUG_ALL
#define AN_DEBUG_DIGITALREAD
#define AN_DEBUG_DIGITALWRITE
#define AN_DEBUG_ANALOGREAD
#define AN_DEBUG_ANALOGWRITE
#define AN_DEBUG_TIMESTAMP
#endif

/* BOARD DEFINITIONS */
#ifdef AN_BOARD_PRO_MINI
#define AN_BOARD_PRO
#endif

/* ↓ Arduino PRO / Pro Mini and Arduino NANO ↓ */
#if defined(AN_BOARD_NANO) || defined(AN_BOARD_PRO)

#define AN_MAX_PINS 21

enum {
        LED_BUILTIN = 13,
        A0 = 14,
        A1 = 15,
        A2 = 16,
        A3 = 17,
        A4 = 18,
        A5 = 19,
        A6 = 20,
        A7 = 21,
};

/* ↑ Arduino PRO / Pro Mini and Arduino NANO ↑ */
#else // Default is Arduino Uno
/* ↓ Arduino UNO ↓ */

#define AN_MAX_PINS 19

enum {
        LED_BUILTIN = 13,
        A0 = 14,
        A1 = 15,
        A2 = 16,
        A3 = 17,
        A4 = 18,
        A5 = 19,
};

/* ↑ Arduino UNO ↑ */
#endif

// pin voltages
float an_pin_voltage[AN_MAX_PINS] = {0};

/* FUNCTION DEFINITIONS */

// non-arduino functions
void an_set_voltage(const uint8_t pin, const float voltage);

// Digital I/O
bool digitalRead(const uint8_t pin);
void digitalWrite(const uint8_t pin, const bool value);
void pinMode(const uint8_t pin, const an_pin_mode_t mode);

// Analog I/O
uint16_t analogRead(const uint8_t pin);
// void analogReference();
void analogWrite(const uint8_t pin, const uint8_t value);

// Advanced I/O

//Time
inline void delay(const unsigned long milliseconds);
inline void delayMicroseconds(const unsigned long microseconds);
unsigned long micros(void);
unsigned long millis(void);

// Math
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define map(x, fL, fH, tL, tH) ((x - fL) * (tH - tL) / (fH - fL) + tL)
#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#define sq(x) ((x)*(x))

// Characthers
#define isAlpha(thisChar)            (isalpha(thisChar))
#define isAlphaNumeric(thisChar)     (isalnum(thisChar))
#define isAscii(thisChar)            (true)
#define isControl(thisChar)          (iscntrl(thisChar))
#define isDigit(thisChar)            (isdigit(thisChar))
#define isGraph(thisChar)            (isgraph(thisChar))
#define isHexadecimalDigit(thisChar) (isxdigit(thisChar))
#define isLowerCase(thisChar)        (islower(thisChar))
#define isPrintable(thisChar)        (isprint(thisChar))
#define isPunct(thisChar)            (ispunct(thisChar))
#define isSpace(thisChar)            (isspace(thisChar))
#define isUpperCase(thisChar)        (isupper(thisChar))
#define isWhitespace(thisChar)       (isspace(thisChar))

// Random Numbers
inline long random(long max);
inline long random(long min, long max);
inline void randomSeed(unsigned long seed);

// Bits and Bytes
#define bit(b) (1UL << (b))
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitToggle(value, bit) ((value) ^= (1UL << (bit)))
#define bitWrite(value, bit, bitvalue) ((bitvalue) ? bitSet(value, bit) : bitClear(value, bit))
#define highByte(w) ((uint8_t) ((w) >> 8))
#define lowByte(w) ((uint8_t) ((w) & 0xff))

// Interrupts
#define digitalPinToInterrupt(pin) (pin)
void attachInterrupt(const uint8_t pin, void(*intpointer)(void), const an_int_mode_t mode);
void  detachInterrupt(const uint8_t pin);
inline void interrupts(void);
inline void noInterrupts(void);

// Implimentation
#ifdef AN_IMPL

unsigned long an_start_time_ms;
unsigned long an_start_time_µs;
typedef enum {
        an_analog,
        an_digital,
        an_pwm,
        an_int_pin,
} an_pin_types_t;
typedef struct an_int {
        void (*intpointer)(void);
        an_int_mode_t mode;
} an_int_t;
std::unordered_map<uint8_t, an_int_t> an_ints;
bool an_interrupts_enabled = true;

void setup(void);
void loop(void);
void an_is_pin_defined(const uint8_t pin, const an_pin_types_t = an_digital);

class an_serial
{
        std::string buffer;
public:
        inline size_t available() {return buffer.length();}
        inline size_t availableForWrite() {return SIZE_MAX;}
        inline void begin(unsigned speed) {}
        inline void begin(unsigned speed, int config) {}
        inline void end() {}
        inline void flush() {}
        inline void setTimeout(long new_time) {}
        void an_take_input()
        {
                std::cout << "ArduinoNative is requesting Serial input: ";
                std::cin >> buffer;
        }
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
        template <typename T>
        size_t print(T val)
        {
                std::cout << val;
                std::stringstream s;
                s << val;
                return s.str().length();
        }
        template <typename T>
        size_t print(const T val, an_print_format_t format)
        {
                std::stringstream s;
                switch (format) {
                case BIN: {
                        std::bitset<sizeof(val)*8> bits(val);
                        std::cout << bits;
                        s << bits;
                        return s.str().length();
                } case DEC: {
                        long value = (long)val;
                        std::cout << value;
                        s << value;
                        return s.str().length();
                } case HEX: {
                        long value = (long)val;
                        std::cout << std::hex << value;
                        s << std::hex << value;
                        return s.str().length();
                } case OCT: {
                        long value = (long)val;
                        std::cout << std::oct << value;
                        s << std::oct << value;
                        return s.str().length();
                }}
                return 0;
        }
        size_t print(const float val, const uint8_t decimals)
        {
                std::cout << std::fixed << std::setprecision(decimals) << val;
                std::stringstream s;
                s << std::fixed << std::setprecision(decimals) << val;
                return s.str().length();
        }
        template <typename T>
        size_t println(const T val)
        {
                size_t byteswritten = this->print(val);
                std::cout << "\n";
                return byteswritten + 1;
        }
        template <typename T>
        size_t println(const T val, const an_print_format_t format)
        {
                size_t byteswritten = this->print(val, format);
                std::cout << "\n";
                return byteswritten + 1;
        }
        size_t println(const float val, const uint8_t format)
        {
                size_t byteswritten = this->print(val, format);
                std::cout << "\n";
                return byteswritten + 1;
        }
        size_t println() {std::cout << std::endl; return 1;}
};

an_serial Serial;

// start program
int main()
{
        an_start_time_ms = millis();
        an_start_time_µs = micros();

        setup();
        for (;;) loop();
}

void an_is_pin_defined(uint8_t pin, an_pin_types_t type)
{
        if (pin > AN_MAX_PINS) {
                std::cout << "ERROR: PIN " << std::to_string(pin) << " IS NOT DEFINED\n";
                exit(1);
        }
}

// Digital I/O
bool digitalRead(uint8_t pin)
{
        bool res = an_pin_voltage[pin] > 3;
#ifdef AN_DEBUG_DIGITALREAD
#ifdef AN_DEBUG_TIMESTAMP
        std::cout << millis() << "ms | ";
#endif
        std::cout << "Read pin: " << pin << " is " << res ? "HIGH\n" : "LOW\n";
#endif
        return res;
}

void digitalWrite(uint8_t pin, bool val)
{
        an_set_voltage(pin, val * 5.0f);
#ifdef AN_DEBUG_DIGITALWRITE
#ifdef AN_DEBUG_TIMESTAMP
        std::cout << millis() << "ms | ";
#endif
        std::cout << "Pin: " << pin << " is now " << val ? "HIGH\n" : "LOW\n";
#endif
}

void pinMode(uint8_t pin, an_pin_mode_t mode)
{
        if (mode == INPUT_PULLUP)
                an_pin_voltage[pin] = 5.0f;
}

// Analog I/O
uint16_t analogRead(uint8_t pin)
{
        an_is_pin_defined(pin);
        uint16_t val = map(an_pin_voltage[pin], 0.0f, 5.0f, 0, 1023);
        val = constrain(val, 0, 1023);
#ifdef AN_DEBUG_ANALOGREAD
#ifdef AN_DEBUG_TIMESTAMP
        std::cout << millis() << "ms | ";
#endif
        std::cout << "Analog pin: " << pin << " is " << val << "\n";
#endif
        return val;
}

void analogWrite(uint8_t pin, uint8_t val)
{
        val = constrain(val, 0, 255);
        an_set_voltage(pin,  map(val, 0, 255, 0.0f, 5.0f));
#ifdef AN_DEBUG_ANALOGWRITE
#ifdef AN_DEBUG_TIMESTAMP
        std::cout << millis() << "ms | ";
#endif
        std::cout << "Duty cycle on pin: " << pin << " is now " << val << "\n";
#endif
}

void an_set_voltage(uint8_t pin, float voltage)
{
        an_is_pin_defined(pin);
        bool is_on = an_pin_voltage[pin] > 3;
        bool turn_on = voltage > 3;
        if (an_interrupts_enabled && an_ints.find(pin) != an_ints.end())
                switch(an_ints[pin].mode) {
                case CHANGE:
                        if (is_on != turn_on)
                                an_ints[pin].intpointer();
                        break;
                case RISING:
                        if (!is_on && turn_on)
                                an_ints[pin].intpointer();
                        break;
                case FALLING:
                        if (is_on && !turn_on)
                                an_ints[pin].intpointer();
                        break;
                }
        an_pin_voltage[pin] = voltage;
}

void an_request_voltage(uint8_t pin)
{
        std::cout << "set voltage of pin " << std::to_string(pin) << " to: ";
        float voltage;
        std::cin >> voltage;
        an_set_voltage(pin, voltage);
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

// Random Numbers
long random(long max) {return rand() % max;}
long random(long min, long max) {return min + rand() % (max - min);}
void randomSeed(long seed) {srand(seed);}

// External Interrupts
void attachInterrupt(uint8_t pin, void (*intpointer)(), an_int_mode_t mode)
{
        an_is_pin_defined(pin, an_int_pin);
        an_ints[pin] = {intpointer, mode};
}
void  detachInterrupt(const uint8_t pin)
{
        auto int_pos = an_ints.find(pin);
        if (int_pos != an_ints.end())
                an_ints.erase(int_pos);
}
void interrupts() {an_interrupts_enabled = true;}
void noInterrupts() {an_interrupts_enabled = false;}

#undef AN_IMPL
#endif // AN_IMPL

#endif // ArduinoNative
