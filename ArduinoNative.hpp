#ifndef ArduinoNative_H_
#define ArduinoNative_H_

#ifndef ArduinoNative
#define ArduinoNative
#endif // ArduinoNative

#include <algorithm>
#include <bitset>
#include <cstring>
#include <ctype.h>
#include <cmath>
#include <chrono>
#include <iostream>
#include <stdio.h>
#include <iomanip>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>

/* CONSTANTS */
#define LOW 0
#define HIGH 1
#define MSBFIRST 0
#define LSBFIRST 1
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
} an_num_fmt_t;
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
typedef enum {
        DEFAULT,
        INTERNAL,
        INTERNAL1V1,
        INTERNAL2V56,
        EXTERNAL,
} an_reference_t;
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
#ifdef AN_BOARD_PRO_MINI
#define AN_BOARD_PRO
#endif

#if defined(AN_TEENSY_41)

#define AN_MAX_PINS 42

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
        A8 = 22,
        A9 = 23,
        A10 = 24,
        A11 = 25,
        A12 = 26,
        A13 = 27,
        A14 = 38,
        A15 = 39,
        A16 = 40,
        A17 = 41,
};


/* ↓ Arduino PRO / Pro Mini and Arduino NANO ↓ */
#elif defined(AN_BOARD_NANO) || defined(AN_BOARD_PRO)

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

#define AREF 255
float an_pin_voltage[AN_MAX_PINS] = {0};


/* FUNCTION DEFINITIONS */

// non-arduino functions
void an_set_voltage(const uint8_t pin, const float voltage);
void an_request_voltage(const uint8_t pin);
inline void an_print_timestamp();
void an_attach_sine(const uint8_t pin, const unsigned hz = 1, const float amp = 2.5, const float dc = 2.5, const bool abs = false);
void an_remove_sine(const uint8_t pin);
void an_attach_square(const uint8_t pin, const unsigned hz = 1, const float duty = 0.5);
void an_remove_square(const uint8_t pin);

// Digital I/O
bool digitalRead(const uint8_t pin);
void digitalWrite(const uint8_t pin, const bool value);
void pinMode(const uint8_t pin, const an_pin_mode_t mode);

// Analog I/O
uint16_t analogRead(const uint8_t pin);
void analogReference(an_reference_t type);
void analogWrite(const uint8_t pin, const uint8_t value);

// Advanced I/O
void noTone(const uint8_t pin);
unsigned long pulseIn(const uint8_t pin, const bool val, const unsigned long timeout);
unsigned long pulseInLong(const uint8_t pin, const bool val, const unsigned long timeout);
uint8_t shiftIn(const uint8_t data_pin, const uint8_t clock_pin, const bool bit_order);
void shiftOut(const uint8_t data_pin, const uint8_t clock_pin, const bool bit_order, const byte value);
void tone(const uint8_t pin, unsigned hz, unsigned long dur = 0);

//Time
inline void delay(const unsigned long milliseconds);
inline void delayMicroseconds(const unsigned long microseconds);
unsigned long micros(void);
unsigned long millis(void);

// Math
#define constrain(x, a, b) ({x = x < a ? a : x; x = x > b ? b : x;})
#define map(x, fL, fH, tL, tH) ((x - fL) * (tH - tL) / (fH - fL) + tL)
#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#define sq(x) ((x)*(x))
#define PI 3.14159265

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

class String : public std::string {
public:
        String() {};
        template<typename T>
        String(const T val)
        {
                std::stringstream s;
                s << val;
                append(s.str());
        }
        template <typename T>
        String(const T val, const an_num_fmt_t fmt)
        {
                std::stringstream s = {};
                switch (fmt) {
                case BIN: {
                        std::bitset<sizeof(val) * 8> bits(val);
                        s << bits;
                        append(s.str());
                        return;
                } case DEC: {
                        break;
                } case HEX: {
                        s << std::hex;
                        break;
                } case OCT: {
                        s << std::oct;
                        break;
                }}
                s << (long)val;
                append(s.str());
        }
        String(const char* buff, const an_num_fmt_t fmt)
        {
                for (unsigned int i = 0; i < strlen(buff); i++)
                        append(String((uint8_t)buff[i], fmt));
        }
        String(const double val, const uint8_t decimals)
        {
                std::stringstream s;
                s << std::fixed << std::setprecision(decimals) << val;
                append(s.str());
        }
        inline float toFloat() {return std::stof(c_str());}
        inline int toInt() {return (int)toFloat();}
        inline double toDouble() {return std::atof(c_str());}

        inline void getBytes(byte* buf, unsigned len) {strncpy((char*)buf, substr(0, len).c_str(), len);}
        inline void toCharArray(unsigned char* buf, const unsigned len) {getBytes((byte*)buf, len); buf[len] = '\0';}
        inline String substring(size_t pos = 0, size_t len = npos) {return substr(pos, len);}
        inline void toLowerCase()
        {
                std::transform(begin(), end(), begin(),
                        [](char c){return std::tolower(c);});
        }
        inline void toUpperCase()
        {
                std::transform(begin(), end(), begin(),
                        [](char c){return std::toupper(c);});
        }
        inline char charAt(const unsigned int n) {return at(n);}
        inline int compareTo(const String str2) {return compare(str2);}
        template <typename T>
        inline bool concat(const T val) {append(String(val)); return true;}
        inline bool startsWith(const String substr) {return rfind(substr, 0) == 0;}
        inline bool endsWith(const String str) {return compare(length() - str.length(), str.length(), str) == 0;}
        inline bool equals(const String str2) {return compare(str2) == 0;}
        bool equalsIgnoreCase(const String str2)
        {
                String strlwr = String(c_str());
                String str2lwr = String(str2.c_str());
                strlwr.toLowerCase();
                str2lwr.toLowerCase();
                return strlwr.compare(str2lwr) == 0;
        }
        inline size_t indexOf(const char* val, const size_t from = 0) {return find(val, from);}
        inline size_t lastIndexOf(const char* val, const size_t from = 0) {return rfind(val, from);}
        inline void remove(const size_t index, const size_t count = 1) {erase(index, count);}
        void replace(const String from, const String to)
        {
                size_t start_pos = 0;
                while ((start_pos = find(from, start_pos)) != npos) {
                        std::string::replace(start_pos, from.length(), to);
                        start_pos += to.length();
                }
        }
        inline void setCharAt(const size_t index, const char c) {at(index) = c;}
        void trim()
        {
                erase(begin(), std::find_if(begin(), end(), [](char ch) {
                        return !std::isspace(ch);
                }));
                erase(std::find_if(rbegin(), rend(), [](char ch) {
                        return !std::isspace(ch);
                }).base(), end());
        }
};

#ifndef _WIN32
void serialEvent() __attribute__((weak));
#endif

class an_serial
{
private:
        String buffer;
        inline bool skip_alpha(LookaheadMode lookahead, bool is_float, char ignore)
        {
                while(available()) {
                        char c = peek();
                        if (c == ignore){
                                buffer.erase(buffer.begin());
                                continue;
                        }
                        if ((c == '-') || (c >= '0' && c <= '9') ||
                            (is_float && c == '.'))
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
                        if ((c == '-') || (c >= '0' && c <= '9') ||
                            (is_float && c == '.'))
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
        inline String readString() {String str = buffer; buffer.clear(); return str;}
        inline String readStringUntil(const char terminator)
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
                        *buffer++ = c;
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
        int parseInt(const LookaheadMode lookahead = SKIP_ALL, const char ignore = '\n')
        {
                if (!skip_alpha(lookahead, false, ignore))
                        return 0;
                int res = buffer.toInt();
                remove_digit(false);
                return res;
        }
        float parseFloat(const LookaheadMode lookahead = SKIP_ALL, const char ignore = '\n')
        {
                if (!skip_alpha(lookahead, true, ignore))
                        return 0.0f;
                float res = buffer.toFloat();
                remove_digit(true);
                return res;
        }
        template <typename T> inline size_t print(const T val)
        {
                String s = val;
                std::cout << s;
                return s.length();
        }
        template <typename V, typename F>
        inline size_t print(const V val, const F fmt)     {return print(String(val, fmt));}

        template <typename T>
        inline size_t write(const T val)                  {return print(val, HEX) / 2;}
        inline size_t write(const uint8_t* data, int data_len) {return data_len;} // TODO: implement

        template <typename V, typename F>
        inline size_t println(const V val, const F fmt)   {return print(val, fmt) + println();}
        template <typename T>
        inline size_t println(const T val)                {return print(val) + println();}
        inline size_t println()                           {std::cout << "\n"; return 1;}
};

// TODO: add debug functionality
// maybe add the option to add emulated hardware?
class an_wire
{
public:
        void begin() {};
        void begin(uint8_t adr);
        int requestFrom(uint8_t adr, int quant);
        int requestFrom(uint8_t adr, int quant, bool stop);
        void beginTransmission(uint8_t adr);
        void endTransmission();
        void endTransmission(bool stop);
        int write(uint8_t val);
        int write(String str);
        int write(const uint8_t* data, int len);
        int available();
        int read();
        void setClock(int hz);
        void onReceive(void(*handler)(int num_bytes));
        void onRequest(void(*handler)(void));
};

an_serial Serial;
an_wire Wire;

#ifdef AN_TEENSY_41
an_serial Serial1;
an_serial Serial2;
an_wire Wire1;
an_wire Wire2;
#endif

// Implimentation
#ifdef AN_IMPL

unsigned long an_start_time_ms;
unsigned long an_start_time_micros;
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
std::unordered_map<uint8_t, std::thread> an_sines;
std::unordered_map<uint8_t, bool> an_sines_terminate;
std::unordered_map<uint8_t, std::thread> an_squares;
std::unordered_map<uint8_t, bool> an_squares_terminate;
bool an_interrupts_enabled = true;
float an_reference_v = 5.0;

void setup(void);
void loop(void);
void an_is_pin_defined(const uint8_t pin, const an_pin_types_t = an_digital);

// start program
int main()
{
        an_start_time_ms = millis();
        an_start_time_micros = micros();

        setup();
        for (;;) loop();
}

/* ArduinoNative reused functions */
void an_is_pin_defined(uint8_t pin, an_pin_types_t type)
{
        if (pin > AN_MAX_PINS && pin != AREF) {
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
        uint16_t val = (uint16_t)lround(map(an_pin_voltage[pin], 0.0f, an_reference_v, 0, 1023));
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

void analogReference(an_reference_t type)
{
        switch(type) {
        case DEFAULT:
                an_reference_v = 5.0;
                break;
        case INTERNAL:
                an_reference_v = 1.1;
                break;
        case INTERNAL1V1:
                an_reference_v = 1.1;
                break;
        case INTERNAL2V56:
                an_reference_v = 2.56;
                break;
        case EXTERNAL:
                break;
        }
}

void an_set_voltage(uint8_t pin, float voltage)
{
        an_is_pin_defined(pin);
        bool is_on = an_pin_voltage[pin] > 3;
        bool turn_on = voltage > 3;
        if (pin == AREF) {
                an_reference_v = voltage;
                return;
        }


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
void delayMicroseconds(unsigned long micros)
{
        std::this_thread::sleep_for(std::chrono::microseconds(micros));
}

unsigned long micros()
{
        auto duration = std::chrono::system_clock::now().time_since_epoch();
        return (unsigned long)std::chrono::duration_cast<std::chrono::microseconds>(duration).count() - an_start_time_micros;
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
        detachInterrupt(pin);
        an_ints[pin] = {intpointer, mode};
}
void  detachInterrupt(const uint8_t pin)
{
        an_is_pin_defined(pin, an_int_pin);
        auto int_pos = an_ints.find(pin);
        if (int_pos != an_ints.end())
                an_ints.erase(int_pos);
}
void interrupts() {an_interrupts_enabled = true;}
void noInterrupts() {an_interrupts_enabled = false;}

// Advanced I/O
inline void noTone(const uint8_t pin)
{
        an_is_pin_defined(pin);
#ifndef _WIN32
        system("killall \"ffplay\" -q");
#endif
}
unsigned long pulseIn(const uint8_t pin, const bool val, const unsigned long timeout)
{
        an_is_pin_defined(pin);
        while (digitalRead(pin) != val);
        unsigned long before = micros();
        while (digitalRead(pin) == val && (!timeout && micros() - before >= timeout));
        unsigned long after = micros();
        return after - before;
}
inline unsigned long pulseInLong(const uint8_t pin, const bool val, const unsigned long timeout)
{
        return pulseIn(pin, val, timeout);
}
uint8_t shiftIn(const uint8_t data_pin, const uint8_t clock_pin, const uint8_t bit_order)
{
        an_is_pin_defined(data_pin);
        an_is_pin_defined(clock_pin);

        uint8_t value = 0;
        uint8_t i;

        for (i = 0; i < 8; ++i) {
                digitalWrite(clock_pin, HIGH);
                if (bit_order == LSBFIRST)
                        value |= digitalRead(data_pin) << i;
                else
                        value |= digitalRead(data_pin) << (7 - i);
                digitalWrite(clock_pin, LOW);
        }
        return value;
}
void shiftOut(const uint8_t data_pin, const uint8_t clock_pin, const bool bit_order, byte val)
{
        an_is_pin_defined(data_pin);
        an_is_pin_defined(clock_pin);

        uint8_t i;

        for (i = 0; i < 8; i++)  {
                if (bit_order == LSBFIRST) {
                        digitalWrite(data_pin, val & 1);
                        val >>= 1;
                } else {
                        digitalWrite(data_pin, (val & 128) != 0);
                        val <<= 1;
                }

                digitalWrite(clock_pin, HIGH);
                digitalWrite(clock_pin, LOW);
        }
}
void tone(const uint8_t pin, unsigned hz, unsigned long dur)
{
        noTone(pin);
#ifndef _WIN32
        hz = constrain(hz, 0, 20000);
        char ffplay[70];
        snprintf(ffplay, sizeof(ffplay), "ffplay -f lavfi -i \"sine=frequency=%d\" -nodisp -loglevel quiet &", hz);
        system(ffplay);
#endif
}
void an_play_sine(const uint8_t pin, const unsigned hz, const float amp, const float dc)
{
        for (;;) {
                if (an_sines_terminate[pin])
                        return;
                an_set_voltage(pin, sin(((float)millis() / (1000.0f / (2.0f * PI))) * hz) * amp + dc);
        }
}
void an_play_sine_abs(const uint8_t pin, const unsigned hz, const float amp, const float dc)
{
        for (;;) {
                if (an_sines_terminate[pin])
                        return;
                an_set_voltage(pin, fabs(sin(((float)millis() / (1000.0f / (2.0f * PI))) * hz) * amp + dc));
        }
}

void an_attach_sine(const uint8_t pin, const unsigned hz, const float amp, const float dc, const bool is_abs)
{
        an_remove_sine(pin);
        an_sines_terminate[pin] = false;
        if (is_abs) {
                std::thread sine(an_play_sine_abs, pin, hz, amp, dc);
                an_sines[pin] = move(sine);
        } else {
                std::thread sine(an_play_sine, pin, hz, amp, dc);
                an_sines[pin] = move(sine);
        }
}
void an_remove_sine(const uint8_t pin)
{
        an_is_pin_defined(pin);
        auto sine_pos = an_sines.find(pin);
        if (sine_pos != an_sines.end()) {
                an_sines_terminate[pin] = true;
                an_sines[pin].join();
                an_sines.erase(sine_pos);
        }
}

void an_play_square(const uint8_t pin, const unsigned hz, const float duty)
{
        bool top = true;
        for (;;) {
                if (an_sines_terminate[pin])
                        return;
                float sine = sin((millis() / (1000.0f / (2.0f * PI))) * hz);
                float triangle = 1.0f-acos(sine)/PI;

                float new_top = triangle <= duty;
                if (new_top != top) {
                        an_set_voltage(pin, new_top * 5.0f);
                        top = new_top;
                }
        }
}

void an_attach_square(const uint8_t pin, const unsigned hz, const float duty)
{
        an_remove_square(pin);
        an_squares_terminate[pin] = false;
        std::thread square(an_play_square, pin, hz, duty);
        an_squares[pin] = move(square);
}
void an_remove_square(const uint8_t pin)
{
        an_is_pin_defined(pin);
        auto square_pos = an_squares.find(pin);
        if (square_pos != an_squares.end()) {
                an_squares_terminate[pin] = true;
                an_squares[pin].join();
                an_squares.erase(square_pos);
        }
}
#undef AN_IMPL
#endif // AN_IMPL

#endif // ArduinoNative_H_
