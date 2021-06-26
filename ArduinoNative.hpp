#ifndef ArduinoNative
#define ArduinoNative

#include <algorithm>
#include <bitset>
#include <cstring>
#include <ctype.h>
#include <cmath>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>
#include <charconv>
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
} an_print_fmt_t;
typedef enum : uint64_t {
        CHANGE,
        RISING,
        FALLING
} an_int_mode_t;
typedef enum {
        SKIP_NONE,
        SKIP_WHITESPACE,
        SKIP_ALL,
} LookaheadMode;
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
inline void an_print_timestamp();
#ifndef _WIN32
void serialEvent() __attribute__((weak));
#endif

class an_serial
{
        std::string buffer;
        inline bool skip_alpha(LookaheadMode lookahead, bool is_float, char ignore)
        {
                while(available()) {
                        char c = peek();
                        if (c == ignore){
                                buffer.erase(buffer.begin());
                                continue;
                        }
                        if ((c == '-') || (c >= '0' && c <= '9') || (is_float && c == '.'))
                                return true;
                        switch (lookahead) {
                        case SKIP_ALL:
                                buffer.erase(buffer.begin());
                                break;
                        case SKIP_NONE:
                        case SKIP_WHITESPACE:
                               return true;
                        }
                }
                return false;
         }
        inline void remove_digit(bool is_float)
        {
                while(available()) {
                        char c = peek();
                        if ((c == '-') || (c >= '0' && c <= '9') || (is_float && c == '.'))
                                buffer.erase(buffer.begin());
                        else
                                return;
                }
        }
public:
        inline size_t available() {return buffer.length();}
        inline size_t availableForWrite() {return SIZE_MAX;}
        inline void begin(unsigned speed) {}
        inline void begin(unsigned speed, int config) {}
        inline void end() {}
        inline void flush() {std::cout << std::flush;}
        inline void setTimeout(const long new_time) {}
        inline std::string readString() {std::string str = buffer; buffer.clear(); return str;}
        inline std::string readStringUntil(const char terminator)
        {
                size_t t_pos = buffer.find(terminator);
                if (t_pos == std::string::npos)
                        return readString();

                std::string str = buffer;
                buffer.erase(0, t_pos);
                str.erase(str.begin() + t_pos, str.end());
                return str;
        }
        void an_take_input()
        {
                std::cout << "ArduinoNative is requesting Serial input: ";
                std::cin >> buffer;
#ifndef _WIN32
                if (serialEvent)
                        serialEvent();
#endif
        }
        uint8_t peek() {return buffer.length() > 0 ? uint8_t(buffer.c_str()[0]) : 0;}
        inline uint8_t read()
        {
                uint8_t read_byte = peek();
                buffer.erase(buffer.begin());
                return read_byte;
        }
        size_t readBytes(char* buffer, const unsigned length, const bool is_until = false, const char terminator = '\0')
        {
                size_t count = 0;
                for(; count < length; count++) {
                        uint8_t c = read();
                        if (c < 0 || (is_until && c == terminator))
                                break;
                        *buffer++ = (char)c;
                }
                return count;
        }
        inline size_t readBytesUntil(const char terminator, char* buffer, const unsigned length)
        {
                return readBytes(buffer, length, true, terminator);
        }
        bool find(const char* target, const size_t len = 1)
        {
                size_t t_pos = buffer.find(target);
                if (t_pos == std::string::npos) {
                        buffer.clear();
                        return false;
                }
                buffer.erase(0, t_pos);
                return true;
        }
        bool findUntil(const char* target, const char* terminal)
        {
                bool res = find(target);
                if (res)
                        find(terminal);
                return res;
        }
        long parseInt(const LookaheadMode lookahead = SKIP_ALL, const char ignore = '\n')
        {
                if (!skip_alpha(lookahead, false, ignore))
                        return 0;
                double res = 0.0;
                std::from_chars(buffer.data(), buffer.data() + buffer.size(), res);
                remove_digit(false);
                return (long)res;
        }
        float parseFloat(const LookaheadMode lookahead = SKIP_ALL, const char ignore = '\n')
        {
                if (!skip_alpha(lookahead, true, ignore))
                        return 0.0f;
                double res = 0.0;
                std::from_chars(buffer.data(), buffer.data() + buffer.size(), res);
                remove_digit(true);
                return (float)res;
        }

        template <typename T>
        size_t print(const T val)
        {
                std::cout << val;
                std::stringstream s;
                s << val;
                return s.str().length();
        }
        template <typename T>
        size_t print(const T val, const an_print_fmt_t fmt)
        {
                std::stringstream s;
                switch (fmt) {
                case BIN: {
                        std::bitset<sizeof(val) * 8> bits(val);
                        return print(bits);
                } case DEC: {
                        return print((long)val);
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
        size_t print(const char* buff, const an_print_fmt_t fmt)
        {
                size_t res = 0;
                for (unsigned int i = 0; i < strlen(buff); i++)
                        res += print((uint8_t)buff[i], fmt);
                return res;
        }
        size_t print(const double val, const uint8_t decimals)
        {
                std::cout << std::fixed << std::setprecision(decimals) << val;
                std::stringstream s;
                s << std::fixed << std::setprecision(decimals) << val;
                return s.str().length();
        }
        template <typename T>
        inline size_t write(const T val) {return print(val, HEX) / 2;}

        inline size_t println() {std::cout << "\n"; return 1;}

        template <typename T>
        inline size_t println(const T val) {return print(val) + println();}

        template <typename T>
        inline size_t println(const T val, const an_print_fmt_t fmt)
        {
                return print(val, fmt) + println();
        }

        inline size_t println(const double val, const uint8_t fmt)
        {
                return print(val, fmt) + println();
        }
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

/* ArduinoNative reused functions */
void an_is_pin_defined(uint8_t pin, an_pin_types_t type)
{
        if (pin > AN_MAX_PINS) {
                std::cout << "ERROR: PIN " << std::to_string(pin) << " IS NOT DEFINED\n";
                exit(1);
        }
}
void an_print_timestamp()
{
#ifdef AN_DEBUG_TIMESTAMP
        std::cout << millis() << "ms | ";
#endif
}

// Digital I/O
bool digitalRead(uint8_t pin)
{
        bool res = an_pin_voltage[pin] > 3;
#ifdef AN_DEBUG_DIGITALREAD
        an_print_timestamp();
        std::cout << "Read pin: " << std::to_string(pin) << " is " << (res ? "HIGH\n" : "LOW\n");
#endif
        return res;
}

void digitalWrite(uint8_t pin, bool val)
{
        an_set_voltage(pin, val * 5.0f);
#ifdef AN_DEBUG_DIGITALWRITE
        an_print_timestamp();
        std::cout << "Pin: " << std::to_string(pin) << " is now " << (val ? "HIGH\n" : "LOW\n");
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
        uint16_t val = (uint16_t)lround(map(an_pin_voltage[pin], 0.0f, 5.0f, 0, 1023));
        val = constrain(val, 0, 1023);
#ifdef AN_DEBUG_ANALOGREAD
        an_print_timestamp();
        std::cout << "Analog pin: " << std::to_string(pin) << " is " << val << "\n";
#endif
        return val;
}

void analogWrite(uint8_t pin, uint8_t val)
{
        val = constrain(val, 0, 255);
        an_set_voltage(pin,  map(val, 0, 255, 0.0f, 5.0f));
#ifdef AN_DEBUG_ANALOGWRITE
        an_print_timestamp();
        std::cout << "Duty cycle on pin: " << std::to_string(pin) << " is now " << val << "\n";
#endif
}

void an_set_voltage(uint8_t pin, float voltage)
{
        an_is_pin_defined(pin);
        bool is_on = an_pin_voltage[pin] > 3;
        bool turn_on = voltage > 3;

        /* If pin has interrupt attached */
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
void delay(unsigned long ms)
{
        std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}
void delayMicroseconds(unsigned long µs)
{
        std::this_thread::sleep_for(std::chrono::microseconds(µs));
}

unsigned long micros()
{
        auto duration = std::chrono::system_clock::now().time_since_epoch();
        return (unsigned long)std::chrono::duration_cast<std::chrono::microseconds>(duration).count() - an_start_time_µs;
}
unsigned long millis()
{
        auto duration = std::chrono::system_clock::now().time_since_epoch();
        return (unsigned long)std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() - an_start_time_ms;
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
