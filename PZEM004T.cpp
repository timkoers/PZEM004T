#include "PZEM004T.h"

#if defined(ESP32)
#include <esp_task_wdt.h>
#endif

#if (ESP32_INTERRUPT)
#warning "Using ESP32 uart-interrupt"
#endif

// Mutexes
xSemaphoreHandle voltageSemaphore, currentSemaphore, powerSemaphore, energySemaphore, addressSemaphore, powerAlarmSemaphore = NULL;

// http://wiki.bernardino.org/index.php/Electric_monitoring_and_communication_module_power_energy_meter

#define PZEM_VOLTAGE (uint8_t)0xB0
#define RESP_VOLTAGE (uint8_t)0xA0

#define PZEM_CURRENT (uint8_t)0xB1
#define RESP_CURRENT (uint8_t)0xA1

#define PZEM_POWER (uint8_t)0xB2
#define RESP_POWER (uint8_t)0xA2

#define PZEM_ENERGY (uint8_t)0xB3
#define RESP_ENERGY (uint8_t)0xA3

#define PZEM_SET_ADDRESS (uint8_t)0xB4
#define RESP_SET_ADDRESS (uint8_t)0xA4

#define PZEM_POWER_ALARM (uint8_t)0xB5
#define RESP_POWER_ALARM (uint8_t)0xA5

#define RESPONSE_SIZE sizeof(PZEMCommand)
#define RESPONSE_DATA_SIZE RESPONSE_SIZE - 2

#define PZEM_BAUD_RATE 9600

#define SEMAPHORE_ACQ_TICKS 0

#ifdef PZEM004_SOFTSERIAL
PZEM004T::PZEM004T(uint8_t receivePin, uint8_t transmitPin)
{
    SoftwareSerial *port = new SoftwareSerial(receivePin, transmitPin);
    port->begin(PZEM_BAUD_RATE);
    this->serial = port;
    this->_isSoft = true;
}
#endif

#if (ESP32_INTERRUPT)
QueueHandle_t serialQueue;
static void IRAM_ATTR handleInterrupt(uint8_t c, void *pzemObject)
{
    BaseType_t xHigherPriorityTaskWoken;

    if (!xQueueSendFromISR(serialQueue, &c, &xHigherPriorityTaskWoken))
    {
        ESP_LOGE("PZEM", "Failed to push uart IRQ char onto queue");
    }
}
#endif

PZEM004T::PZEM004T(HardwareSerial *port)
{
    port->begin(PZEM_BAUD_RATE);
#if (ESP32_INTERRUPT)

    // Initialize all the semaphores
    voltageSemaphore = xSemaphoreCreateBinary();
    currentSemaphore = xSemaphoreCreateBinary();
    powerSemaphore = xSemaphoreCreateBinary();
    energySemaphore = xSemaphoreCreateBinary();
    addressSemaphore = xSemaphoreCreateBinary();
    powerAlarmSemaphore = xSemaphoreCreateBinary();

    assert(voltageSemaphore != NULL);
    assert(currentSemaphore != NULL);
    assert(powerSemaphore != NULL);
    assert(energySemaphore != NULL);
    assert(addressSemaphore != NULL);
    assert(powerAlarmSemaphore != NULL);

    // Take all semaphores once to clear them, which seems to work
    xSemaphoreTake(voltageSemaphore, 0);
    xSemaphoreTake(currentSemaphore, 0);
    xSemaphoreTake(powerSemaphore, 0);
    xSemaphoreTake(energySemaphore, 0);
    xSemaphoreTake(addressSemaphore, 0);
    xSemaphoreTake(powerAlarmSemaphore, 0);

    xSemaphoreGive(voltageSemaphore);
    xSemaphoreGive(currentSemaphore);
    xSemaphoreGive(powerSemaphore);
    xSemaphoreGive(energySemaphore);
    xSemaphoreGive(addressSemaphore);
    xSemaphoreGive(powerAlarmSemaphore);

    ESP_LOGD("PZEM", "Created semaphores");

    // First create the queue and then set the interrupt handler, store 4 messages in the queue
    serialQueue = xQueueCreate(RESPONSE_SIZE * 4, sizeof(uint8_t));
    assert(serialQueue != NULL);
    xQueueReset(serialQueue);

    ESP_LOGD("PZEM", "Queue created");

    // Add the interrupt function again
    port->setRXInterrupt(handleInterrupt, (void *)this);

    ESP_LOGD("PZEM", "Assigned uart interrupt handler");

    nextRequest = PZEM_VOLTAGE;
#endif

    this->serial = port;
    this->_isSoft = false;
}

PZEM004T::~PZEM004T()
{
    if (_isSoft)
        delete this->serial;
}

#if (ESP32_INTERRUPT)
// This function gets called from outside of the interrupt
bool PZEM004T::processQueue()
{
    bool ok = false;

    uint8_t buffer[RESPONSE_SIZE];
    uint8_t len = 0, c;
    // ok should be true when entering this loop
    ok = uxQueueMessagesWaiting(serialQueue) > 0 && !((int)uxQueueMessagesWaiting(serialQueue) % RESPONSE_SIZE);

    while ((ok = ((int)uxQueueMessagesWaiting(serialQueue) > 0 && !((int)uxQueueMessagesWaiting(serialQueue) % RESPONSE_SIZE))), ok)
    {
        // Clear the buffer as items where leaking into the rx buffer
        memset(buffer, '\0', RESPONSE_SIZE);

        len = 0;
        c = 0;        

        ESP_LOGD("PZEM", "Processing queue (%d items)", uxQueueMessagesWaiting(serialQueue));

        // The serial interrupt handler should handle it, we only wait until the queue is full
        while (ok && len < RESPONSE_SIZE)
        {
            // Receive from the queue
            if (xQueueReceive(serialQueue, &c, 0) != pdTRUE) // Don't block
                continue;                                    // Skip this iteration, since no valuable data is received

            if (!c && !len)
                continue; // skip 0 at startup

            buffer[len++] = c;
        }

        if (!ok)
        {
            ESP_LOGE("PZEM", "Queue got messed up");
            // Might reset it
        }

        if (len > 0)
        {
            ok = (len == RESPONSE_SIZE);

            // Convert the buffer into values
            if (ok)
            {
                ok = recieve(buffer);

                if (!ok)
                    ESP_LOGE("PZEM", "Failed to parse buffer");
            }
            else
            {
                ESP_LOGE("PZEM", "Failed handling the queue (len %d), resetting..", len);
                xQueueReset(serialQueue);
            }
        }
    }

    return ok;
}

void PZEM004T::setUpdateInterval(unsigned long interval)
{
    updateInterval = interval;
}

// Call as often as possible
bool PZEM004T::update()
{
    // Flush everything
    this->serial->flush();

    // Request everything
    bool ok = processQueue();

    if ((millis() - lastUpdate) > updateInterval)
    {
        lastUpdate = millis();

        ok = mIp[0] != 0 && mIp[1] != 0 && mIp[2] != 0 && mIp[3] != 0;
        if (ok && this->serial->available() == 0)
        {
            // Don't need the queue semaphore for this
            //xQueueReset(serialQueue);

            // Just send everything, the response is handled by the interrupt handler, and put into the queue which can hold 4 answers
            
            ESP_LOGD("PZEM", "Requesting %02x", nextRequest);

            send(mIp, nextRequest);

            if(nextRequest != PZEM_ENERGY){
                nextRequest++;
            }else{
                nextRequest = PZEM_VOLTAGE;
            }
        }
    }

    return ok;
}

#endif

void PZEM004T::setReadTimeout(unsigned long msec)
{
    _readTimeOut = msec;
}
#if (ESP32_INTERRUPT)
bool PZEM004T::voltage(float &output)
{
    bool ok = false;

    if (ok = (xSemaphoreTake(voltageSemaphore, SEMAPHORE_ACQ_TICKS) == pdTRUE), ok)
    {
        output = mVoltage;
        xSemaphoreGive(voltageSemaphore);
    }

    return ok;
}

bool PZEM004T::current(float &output)
{
    bool ok = false;

    if (ok = (xSemaphoreTake(currentSemaphore, SEMAPHORE_ACQ_TICKS) == pdTRUE), ok)
    {
        output = mCurrent;
        xSemaphoreGive(currentSemaphore);
    }

    return ok;
}

bool PZEM004T::power(float &output)
{
    bool ok = false;

    if (ok = (xSemaphoreTake(powerSemaphore, SEMAPHORE_ACQ_TICKS) == pdTRUE), ok)
    {
        output = mPower;
        xSemaphoreGive(powerSemaphore);
    }

    return ok;
}

bool PZEM004T::energy(float &output)
{
    bool ok = false;

    if (ok = (xSemaphoreTake(energySemaphore, SEMAPHORE_ACQ_TICKS) == pdTRUE), ok)
    {
        output = mEnergy;
        xSemaphoreGive(energySemaphore);
    }

    return ok;
}

bool PZEM004T::setAddress(const IPAddress &newAddr)
{
    bool ok = false;

    // First check if the address is already the same
    if (xSemaphoreTake(addressSemaphore, SEMAPHORE_ACQ_TICKS) == pdTRUE)
    {
        ESP_LOGD("PZEM", "Got addr. sem");
        ok = (mIp == newAddr);
        xSemaphoreGive(addressSemaphore);

        if (ok)
            ESP_LOGD("PZEM", "Matching");
    }

    if (!ok)
        send(newAddr, PZEM_SET_ADDRESS);

    return ok;
}

uint8_t mTempThreshold;

bool PZEM004T::setPowerAlarm(uint8_t threshold)
{
    bool ok = false;
    uint8_t thres;
    mTempThreshold = threshold;

    send(mIp, PZEM_POWER_ALARM, threshold);
    delay(100); // Hopefully wait for the response

    if (ok = (xSemaphoreTake(powerAlarmSemaphore, SEMAPHORE_ACQ_TICKS) == pdTRUE), ok)
    {
        thres = mPowerAlarm;
        xSemaphoreGive(powerAlarmSemaphore);
        ok = threshold == thres;
    }

    return ok;
}
#else
float PZEM004T::voltage(const IPAddress &addr)
{
    uint8_t data[RESPONSE_DATA_SIZE];

    send(addr, PZEM_VOLTAGE);
    if (!recieve(RESP_VOLTAGE, data))
        return PZEM_ERROR_VALUE;

    return (data[0] << 8) + data[1] + (data[2] / 10.0);
}

float PZEM004T::current(const IPAddress &addr)
{
    uint8_t data[RESPONSE_DATA_SIZE];

    send(addr, PZEM_CURRENT);
    if (!recieve(RESP_CURRENT, data))
        return PZEM_ERROR_VALUE;

    return (data[0] << 8) + data[1] + (data[2] / 100.0);
}

float PZEM004T::power(const IPAddress &addr)
{
    uint8_t data[RESPONSE_DATA_SIZE];

    send(addr, PZEM_POWER);
    if (!recieve(RESP_POWER, data))
        return PZEM_ERROR_VALUE;

    return (data[0] << 8) + data[1];
}

float PZEM004T::energy(const IPAddress &addr)
{
    uint8_t data[RESPONSE_DATA_SIZE];

    send(addr, PZEM_ENERGY);
    if (!recieve(RESP_POWER, data))
        return PZEM_ERROR_VALUE;

    return ((uint32_t)data[0] << 16) + ((uint16_t)data[1] << 8) + data[2];
}

bool PZEM004T::setAddress(const IPAddress &newAddr)
{
    send(newAddr, PZEM_SET_ADDRESS);
    return recieve(RESP_SET_ADDRESS);
}

bool PZEM004T::setPowerAlarm(const IPAddress &addr, uint8_t threshold)
{
    send(addr, PZEM_POWER_ALARM, threshold);
    return recieve(RESP_POWER_ALARM);
}
#endif

IPAddress mTempIp;

bool PZEM004T::send(const IPAddress &addr, uint8_t cmd, uint8_t data)
{
    //ESP_LOGD("PZEM", "Messages waiting in queue %d", uxQueueMessagesWaiting(serialQueue));
    PZEMCommand pzem;

    pzem.command = cmd;

    for (int i = 0; i < sizeof(pzem.addr); i++)
    {
        pzem.addr[i] = addr[i];
        mTempIp[i] = pzem.addr[i];
    }
    pzem.data = data;

    uint8_t *bytes = (uint8_t *)&pzem;
    pzem.crc = crc(bytes, sizeof(pzem) - 1);

    serial->write(bytes, sizeof(pzem));

    return true;
}

#if (ESP32_INTERRUPT)
// This function gets called from an ISR
bool PZEM004T::recieve(uint8_t *data)
{
    bool ok = data[6] == crc(data, RESPONSE_SIZE - 1);

    uint8_t header = data[0];

    data++;

    // Check the CRC first before loading in the values
    if (!ok)
    {
        ESP_LOGE("PZEM", "CRC check failed");
        return false;
    }

    ESP_LOGD("PZEM", "Parsing header %02x", header);

    // switch the first byte of the buffer.
    switch (header)
    {
    case RESP_SET_ADDRESS:
    {
        if (data[0] == data[1] == data[2] == data[3] == data[4] == 0)
        {
            if (ok = (xSemaphoreTake(addressSemaphore, portMAX_DELAY) == pdTRUE), ok)
            {
                mIp = mTempIp;
                xSemaphoreGive(addressSemaphore);
            }
        }
        else
        {
            // Zero out the IP address
            mTempIp[0] = mTempIp[1] = mTempIp[2] = mTempIp[3] = 0;
        }
        break;
    }
    case RESP_VOLTAGE:
    {
        if (ok = (xSemaphoreTake(voltageSemaphore, portMAX_DELAY) == pdTRUE), ok)
        {
            ESP_LOGD("PZEM", "Calculating voltage float");
            mVoltage = (float)((data[0] << 8) + data[1] + (data[2] / 10.0));
            xSemaphoreGive(voltageSemaphore);
        }
        break;
    }
    case RESP_CURRENT:
    {
        if (ok = (xSemaphoreTake(currentSemaphore, portMAX_DELAY) == pdTRUE), ok)
        {
            ESP_LOGD("PZEM", "Calculating current float");
            mCurrent = (float)((data[0] << 8) + data[1] + (data[2] / 100.0));
            xSemaphoreGive(currentSemaphore);
        }
        break;
    }

    case RESP_ENERGY:
    {
        if (ok = (xSemaphoreTake(energySemaphore, portMAX_DELAY) == pdTRUE), ok)
        {
            ESP_LOGD("PZEM", "Calculating energy float");
            mEnergy = (float)((uint32_t)(data[0] << 16) + ((uint16_t)(data[1]) << 8) + data[2]);
            xSemaphoreGive(energySemaphore);
        }
        break;
    }

    case RESP_POWER:
    {
        if (ok = (xSemaphoreTake(powerSemaphore, portMAX_DELAY) == pdTRUE), ok)
        {
            ESP_LOGD("PZEM", "Calculating power float");
            mPower = (float)((data[0] << 8) + data[1]);
            xSemaphoreGive(powerSemaphore);
        }
        break;
    }
    case RESP_POWER_ALARM:
    {
        if (ok = (xSemaphoreTake(powerAlarmSemaphore, portMAX_DELAY) == pdTRUE), ok)
        {
            {
                if (data[0] == data[1] == data[2] == data[3] == data[4] == 0)
                {
                    mPowerAlarm = mTempThreshold;
                }
                else
                {
                    mTempThreshold = 0;
                }
            }

            xSemaphoreGive(powerAlarmSemaphore);
        }
    }
    default:
    {
        ESP_LOGE("PZEM", "Unkown response header %02x", header);
        break;
    }
    }

    return ok;
}
#else
bool PZEM004T::recieve(uint8_t resp, uint8_t *data)
{

    uint8_t buffer[RESPONSE_SIZE];

    // Clear the buffer as items where leaking into the rx buffer
    memset(buffer, '\0', RESPONSE_SIZE);

#ifdef PZEM004_SOFTSERIAL
    if (_isSoft)
        ((SoftwareSerial *)serial)->listen();
#endif

    uint8_t len = 0;

    uint8_t c;

    unsigned long startTime = millis();
    while ((len < RESPONSE_SIZE) && (millis() - startTime < _readTimeOut))
    {
        if (serial->available() > 0)
        {
            uint8_t c = (uint8_t)serial->read();
            if (!c && !len)
                continue; // skip 0 at startup

            ESP_LOGD("PZEM", "Read %02x (%d)", c, len);
            buffer[len++] = c;
        }
    }

    ESP_LOGD("PZEM", "len %d, expecting %d", len, RESPONSE_SIZE);

    if (len != RESPONSE_SIZE)
        return false;

    ESP_LOGD("PZEM", "Response size matching");

    if (buffer[6] != crc(buffer, len - 1))
        return false;

    ESP_LOGD("PZEM", "CRC matching");

    if (buffer[0] != resp)
        return false;

    ESP_LOGD("PZEM", "Response matching");

    if (data)
    {
        for (int i = 0; i < RESPONSE_DATA_SIZE; i++)
            data[i] = buffer[1 + i];
    }

    ESP_LOGD("PZEM", "recieve() will finish succesfully!");

    return true;
}
#endif

uint8_t PZEM004T::crc(uint8_t *data, uint8_t sz)
{
    uint16_t crc = 0;
    for (uint8_t i = 0; i < sz; i++)
        crc += *data++;
    return (uint8_t)(crc & 0xFF);
}
