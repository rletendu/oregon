#ifndef OREGON_H
#define OREGON_H

#include "Arduino.h"

//#define OREGON_DEBUG

// Setup debug printing macros.
#ifdef OREGON_DEBUG
// Define where debug output will be printed.
#ifndef DEBUG_OREGON_PRINTER
#define DEBUG_OREGON_PRINTER Serial
#endif
#define DEBUG_OREGON_PRINT(...) { DEBUG_OREGON_PRINTER.print(__VA_ARGS__); }
#define DEBUG_OREGON_PRINTLN(...) { DEBUG_OREGON_PRINTER.println(__VA_ARGS__); }
#else
#define DEBUG_OREGON_PRINT(...) {}
#define DEBUG_OREGON_PRINTLN(...) {}
#endif

#ifndef OREGON_MAX_MESSAGE_SIZE
#define OREGON_MAX_MESSAGE_SIZE 11
#endif

class Oregon
{
  public:
    Oregon(void);
    void begin(byte tx_pin);
    void end(void);
    void send_temperature(uint8_t ch, uint8_t id, float temperature, byte battery);
    void send_temperature_hum(uint8_t ch, uint8_t id, float temperature, byte hum, byte battery);
    void send_temperature_from_ds18(uint8_t ch, uint8_t id, int temperature, byte battery);

  private:
    byte OregonMessage[OREGON_MAX_MESSAGE_SIZE];
    byte OregonMessageSize;
    byte OregonTxPin;
    void sendMessage(void);
    void setTemperature(float temp);
    void setTemperature_ds18(int temp);
    uint8_t dec2bcd2(uint8_t n);
    void calculateAndSetChecksum(void);
    void setHumidity(byte hum);
    void setPressure(float pres);
    int Sum(byte count);
    void sendZero(void);
    void sendOne(void);
    void sendOregon(void);
    void sendQuarterLSB(const byte data);
    void sendQuarterMSB(const byte data);
};

// Very Scrappy way to add modules in arduino subfolder ...
// But the only one found using ARDUINO IDE
#if defined(ARDUINO) && defined (OREGON_SRC_INCLUDE)
#include "oregon.cpp"
#endif

#endif //OREGON_H
