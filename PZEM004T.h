#ifndef PZEM004T_H
#define PZEM004T_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

// #define PZEM004_NO_SWSERIAL

#if (not defined(PZEM004_NO_SWSERIAL)) && (defined(__AVR__) || defined(ESP8266))
#define PZEM004_SOFTSERIAL
#endif

#if defined(PZEM004_SOFTSERIAL)
#include <SoftwareSerial.h>
#endif

#include <IPAddress.h>

#define PZEM_DEFAULT_READ_TIMEOUT 1000
#define PZEM_ERROR_VALUE -1.0

struct PZEMCommand
{
    uint8_t command;
    uint8_t addr[4];
    uint8_t data;
    uint8_t crc;
};

class PZEM004T
{
public:
    PZEM004T(uint8_t receivePin, uint8_t transmitPin);
    PZEM004T(HardwareSerial *port);
    ~PZEM004T();

    void setReadTimeout(unsigned long msec);
    unsigned long readTimeout() { return _readTimeOut; }

#if (ESP32_INTERRUPT)
    // Call this function to fetch the latest and greatest from the PZEM004T
    bool update();

    bool voltage(float &output);
    bool current(float &output);
    bool power(float &output);
    bool energy(float &output);

    bool setAddress(const IPAddress &newAddr);
    bool setPowerAlarm(uint8_t threshold);

    void setUpdateInterval(unsigned long interval);
#else
    float voltage(const IPAddress &addr);
    float current(const IPAddress &addr);
    float power(const IPAddress &addr);
    float energy(const IPAddress &addr);

    bool setAddress(const IPAddress &newAddr);
    bool setPowerAlarm(const IPAddress &addr, uint8_t threshold);
#endif

private:
    Stream *serial;

    bool _isSoft;
    unsigned long _readTimeOut = PZEM_DEFAULT_READ_TIMEOUT;

    bool send(const IPAddress &addr, uint8_t cmd, uint8_t data = 0);

#if (ESP32_INTERRUPT)
    // DO NOT CALL THIS FUNCTION DIRECTLY
    bool processQueue();

    bool recieve(uint8_t *data);
    
    unsigned long updateInterval = PZEM_DEFAULT_READ_TIMEOUT;
    unsigned long lastUpdate = 0;

    uint8_t nextRequest = 0;
#else
    bool recieve(uint8_t resp, uint8_t *data = 0);
#endif

    uint8_t crc(uint8_t *data, uint8_t sz);

#if (ESP32_INTERRUPT)
    float mVoltage;
    float mCurrent;
    float mPower;
    float mEnergy;
    uint8_t mPowerAlarm;
    IPAddress mIp;
#endif
};

#endif // PZEM004T_H
