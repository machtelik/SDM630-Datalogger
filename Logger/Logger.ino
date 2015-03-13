/**
 *
 * Copyright (c) 2015, Mike Achtelik
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **/

#include <Wire.h>
#include <SPI.h>
#include <stdint.h>
#include <util/crc16.h>

#include <RTClib.h>
#include <SdFat.h>

// ------------------------ DEFINES------------------------

//#define DEBUG

static const uint16_t LOG_INTERVAL = 10000;
static const uint16_t S_TO_MS = 1000;

// -- System status --

static const uint8_t PIN_LED_GREEN = 5;
static const uint8_t PIN_LED_RED = 6;
static const uint8_t PIN_DC_POWER = 4;
static const uint8_t PIN_CARD_INSERTED = 9;

static const uint8_t OK = 0;
static const uint8_t NO_POWER = 1;
static const uint8_t NO_SDCARD = 2;
static const uint8_t RTC_ERROR = 4;
static const uint8_t SD_INIT_UNSUCCESSFUL = 8;
static const uint8_t FILE_OPEN_UNSUCCESSFUL = 16;
static const uint8_t MODBUS_ERROR = 32;

// -- S0 --

static const uint8_t PIN_S0 = 7;
static const uint8_t PIN_S0_INTERRUPT = 4;

static const uint8_t MIN_SO_PULSE_DURATION = 120;

// -- Logging --

static const uint8_t MODBUS_VALUE_COUNT = 86;
static const uint8_t MODBUS_REQUEST_COUNT = 15;

// -- Modbus --

static const uint8_t PIN_UEXT_POWER = 8;
static const uint8_t PIN_RECEIVER_ENABLE = 13;
static const uint8_t PIN_SENDER_ENABLE = 15;

static const uint16_t MODBUS_BAUDRATE = 9600;
static const uint16_t MODBUS_CHARACTER_TIMEOUT = 200;

static const uint8_t READ_INPUT_REGISTER_FUNCTION = 4;
static const uint8_t DATA_BUFFER_SIZE = 128;
static const uint8_t MAX_CLEAR_COUNT = 128;

static const uint8_t MODBUS_SLAVE_ADDRESS = 1;

static const uint8_t MODBUS_OK = 0;
static const uint8_t MISSING_DATA = 1;
static const uint8_t WRONG_SLAVE = 2;
static const uint8_t WRONG_FUNCTION = 3;
static const uint8_t MODBUS_REQUEST_ERROR = 4;
static const uint8_t WRONG_BYTE_SIZE = 5;
static const uint8_t WRONG_REGISTER_COUNT = 6;
static const uint8_t CRC_ERROR = 7;

// ------------------------ VARIABLES ----------------------------

SdFat SD;
RTC_DS1307 RTC;

// -- Setup/Loop --

DateTime _currentDateTime;

// --- S0 ---

volatile uint8_t _currentS0PulseCount = 0;
volatile uint32_t _pulseTimestamp = 0;

// -- Logging --

uint32_t _timestamp = 0;
uint8_t _s0PulseCount = 0;
float _modbusData[MODBUS_VALUE_COUNT];
uint8_t _modbusDataPos = 0;

static const uint16_t _requestRegisterStart[MODBUS_REQUEST_COUNT] = {0x0000,0x002E,0x0034,0x0038,0x003C,0x0042,0x0046,0x0064,0x00C8,0x00E0,0x00EA,0x00F8,0x00FE,0x0102,0x014E};
static const uint16_t _requestRegisterEnd[MODBUS_REQUEST_COUNT] =   {0x002A,0x0030,0x0034,0x0038,0x003E,0x0042,0x0056,0x006A,0x00CE,0x00E0,0x00F4,0x00FA,0x00FE,0x010C,0x017C};

// -- SDCard --

File _logfile;
File _errorlogfile;

// -- Modbus --

uint8_t _dataBuffer[DATA_BUFFER_SIZE];
uint8_t _dataSize = 0;

uint8_t _modbusError = 0;

union BFConvert {
    uint8_t bVal[4];
    float fVal;
} _convert;

// ------------------------ FUNCTIONS ----------------------------

// -- Setup/Loop --

void setup()
{
    pinMode(PIN_LED_GREEN, OUTPUT);
    pinMode(PIN_LED_RED, OUTPUT);

    pinMode(PIN_DC_POWER, INPUT);
    pinMode(PIN_CARD_INSERTED, INPUT_PULLUP);

    modbusInit();

    Wire.begin();

    uint8_t systemStatus = OK;
    while(systemStatus = getSystemStatus() != OK) {
        displaySystemStatus(systemStatus);
        delay(1000);
    }
    displaySystemStatus(systemStatus);

    enableS0Interrupts();
    delay(LOG_INTERVAL - (millis() % LOG_INTERVAL));
}

void loop()
{
    uint8_t systemStatus = getSystemStatus();

    if(systemStatus == OK) {
        _currentDateTime = RTC.now();

        systemStatus |= logData();
    } else {
        resetPulseCounter();
    }

    if(systemStatus == OK) {
        systemStatus |= writeLogData();
    } 
    
#ifdef DEBUG
    if( systemStatus != OK && !( systemStatus & (NO_POWER | NO_SDCARD) ) ) {
        systemStatus |= writeErrorData(systemStatus, _modbusError);
    }
#endif

    displaySystemStatus(systemStatus);

    delay(LOG_INTERVAL - (millis() % LOG_INTERVAL));
}

// -- System status --

uint8_t getSystemStatus()
{
    if(!digitalRead(PIN_DC_POWER)) {
        return NO_POWER;
    }

    if(digitalRead(PIN_CARD_INSERTED)) {
        return NO_SDCARD;
    }

    if(!RTC.isrunning()) {
        return RTC_ERROR;
    }

    return OK;
}

void displaySystemStatus(uint8_t systemStatus)
{
    if(systemStatus == OK) {
        digitalWrite(PIN_LED_GREEN, LOW);
        digitalWrite(PIN_LED_RED, LOW);
        return;
    }

    if(systemStatus & (NO_POWER | NO_SDCARD) ) {
        digitalWrite(PIN_LED_GREEN, HIGH);
        digitalWrite(PIN_LED_RED, LOW);
    } else if(systemStatus & (SD_INIT_UNSUCCESSFUL | FILE_OPEN_UNSUCCESSFUL | RTC_ERROR ) ) {
        digitalWrite(PIN_LED_GREEN, LOW);
        digitalWrite(PIN_LED_RED, HIGH);
    } else if(systemStatus & ( MODBUS_ERROR )) {
        digitalWrite(PIN_LED_GREEN, HIGH);
        digitalWrite(PIN_LED_RED, HIGH);
    }
}

// -- S0 --

inline void enableS0Interrupts()
{
    pinMode(PIN_S0, INPUT_PULLUP);
    delay(5);
    attachInterrupt(PIN_S0_INTERRUPT, handleS0Pulse, FALLING);
}

void handleS0Pulse()
{
    if(!digitalRead(PIN_S0)) {
        if(millis() >= _pulseTimestamp + MIN_SO_PULSE_DURATION) {
            ++_currentS0PulseCount;
        }
        _pulseTimestamp = millis();
    }
}

// -- Logging --

inline uint8_t logData()
{
    uint8_t systemStatus = OK;

    _timestamp = _currentDateTime.unixtime();
    systemStatus |= logPulseData();
    systemStatus |= logModbusData();

    return systemStatus;
}

inline uint8_t logPulseData()
{
    _s0PulseCount = _currentS0PulseCount;
    resetPulseCounter();

    return OK;
}

inline void resetPulseCounter()
{
    _currentS0PulseCount = 0;
}

inline uint8_t logModbusData()
{
    uint8_t systemStatus = OK;
    _modbusDataPos = 0;

    modbusEnableModRS485(true);
    for(uint8_t i = 0; i < MODBUS_REQUEST_COUNT && systemStatus == OK; ++i) {
        systemStatus |= modbusReadValues(MODBUS_SLAVE_ADDRESS, _requestRegisterStart[i], _requestRegisterEnd[i], _modbusData, &_modbusDataPos);
    }
    modbusEnableModRS485(false);

    return systemStatus;
}

// -- SDCard --

#ifdef DEBUG

uint8_t writeErrorData(uint8_t systemStatus, uint8_t modbusError)
{
    if(SD.begin()) {
        char *filename = "ERRORLOG.TXT";

        if(!SD.exists(filename)) {
            _errorlogfile = SD.open(filename, O_WRITE | O_CREAT);
        } else {
            _errorlogfile = SD.open(filename, O_WRITE | O_APPEND);
        }

        if(_errorlogfile.isOpen()) {

            _errorlogfile.print(_currentDateTime.unixtime());
            _errorlogfile.print(": ");
            _errorlogfile.print(systemStatus);
            
            if(systemStatus == MODBUS_ERROR) {
                _errorlogfile.print(" ");
                _errorlogfile.print(modbusError);
                _errorlogfile.print(" -- ");
                for(uint8_t i = 0; i < _dataSize; ++i) {
                  _errorlogfile.print(_dataBuffer[i], HEX);
                  _errorlogfile.print(" ");
                }
            }
            
            _errorlogfile.println();

            _errorlogfile.close();
        } else {
            return FILE_OPEN_UNSUCCESSFUL;
        }
    } else {
        return SD_INIT_UNSUCCESSFUL;
    }

    return OK;
}

#endif

bool openLogfile()
{
    char *filename = "20000000.TXT";
    filename[2] = (_currentDateTime.year() / 10) % 10 + '0';
    filename[3] = _currentDateTime.year() % 10 + '0';
    filename[4] = _currentDateTime.month() / 10 + '0';
    filename[5] = _currentDateTime.month() % 10 + '0';
    filename[6] = _currentDateTime.day() / 10 + '0';
    filename[7] = _currentDateTime.day() % 10 + '0';

    if(!SD.exists(filename)) {
        _logfile = SD.open(filename, O_WRITE | O_CREAT);

        _logfile.print(F("Timestamp\tS0Pulses"));
        for(uint8_t i = 0; i < MODBUS_REQUEST_COUNT; ++i) {
            for(uint16_t reg = _requestRegisterStart[i]; reg <= _requestRegisterEnd[i]; reg += 2) {
                _logfile.print("\t");
                _logfile.print(reg, HEX);
            }
        }

        _logfile.println();
    } else {
        _logfile = SD.open(filename, O_WRITE | O_APPEND);
    }

    if(!_logfile.isOpen()) {
        return false;
    }

    return true;
}

uint8_t writeLogData()
{
    if(SD.begin()) {
        if(openLogfile()) {
            _logfile.print(_timestamp);
            _logfile.print("\t");
            _logfile.print(_s0PulseCount);

            for(uint8_t i = 0; i < _modbusDataPos; ++i) {
                _logfile.print("\t");
                _logfile.print(_modbusData[i]);
            }

            _logfile.println();
            _logfile.close();

        } else {
            return FILE_OPEN_UNSUCCESSFUL;
        }
    } else {
        return SD_INIT_UNSUCCESSFUL;
    }

    return OK;
}

// -- Modbus --

inline void modbusInit()
{
    pinMode(PIN_UEXT_POWER, OUTPUT);
    pinMode(PIN_SENDER_ENABLE, OUTPUT);
    pinMode(PIN_RECEIVER_ENABLE, OUTPUT);

    digitalWrite(PIN_RECEIVER_ENABLE, LOW);
    digitalWrite(PIN_SENDER_ENABLE, LOW);

    digitalWrite(PIN_UEXT_POWER, HIGH);

    Serial1.begin(MODBUS_BAUDRATE);
}

void modbusEnableModRS485(bool enable)
{
    pinMode(PIN_RECEIVER_ENABLE, OUTPUT);

    digitalWrite(PIN_RECEIVER_ENABLE, LOW);
    digitalWrite(PIN_SENDER_ENABLE, LOW);

    digitalWrite(PIN_UEXT_POWER, !enable);

    delay(5);
}

uint8_t modbusReadValues(uint8_t slaveAdress, uint16_t readValueStart, uint16_t readValueEnd, float *data, uint8_t *dataPos)
{
    uint16_t readRegisterCount = readValueEnd - readValueStart + 2;
    uint8_t readValueCount = readRegisterCount / 2;
    uint16_t dataCRC;

    _dataSize = 0;
    _dataBuffer[_dataSize++] = slaveAdress;
    _dataBuffer[_dataSize++] = READ_INPUT_REGISTER_FUNCTION;
    _dataBuffer[_dataSize++] = highByte(readValueStart);
    _dataBuffer[_dataSize++] = lowByte(readValueStart);
    _dataBuffer[_dataSize++] = highByte(readRegisterCount);
    _dataBuffer[_dataSize++] = lowByte(readRegisterCount);

    dataCRC = 0xFFFF;
    for (uint8_t i = 0; i < _dataSize; ++i) {
        dataCRC = _crc16_update(dataCRC, _dataBuffer[i]);
    }

    _dataBuffer[_dataSize++] = lowByte(dataCRC);
    _dataBuffer[_dataSize++] = highByte(dataCRC);

//Clear receive buffer
    for(uint8_t i = 0; i < MAX_CLEAR_COUNT && Serial1.available(); ++i) {
        Serial1.read();
    }

    digitalWrite(PIN_SENDER_ENABLE, HIGH);
    delay(5);

    for (uint8_t i = 0; i < _dataSize; ++i) {
        Serial1.write(_dataBuffer[i]);
    }
    Serial1.flush();

    digitalWrite(PIN_SENDER_ENABLE, LOW);
    
    _dataSize = 0;
    uint32_t lastReceive = millis();
    while(millis() <= lastReceive + MODBUS_CHARACTER_TIMEOUT && _dataSize < DATA_BUFFER_SIZE) {
        if(Serial1.available()) {
            _dataBuffer[_dataSize++] = Serial1.read();
            lastReceive = millis();
        }
    }

    if (_dataSize < 13) {
        _modbusError = MISSING_DATA;
        return MODBUS_ERROR;
    }

    if (_dataBuffer[8] != slaveAdress) {
        _modbusError = WRONG_SLAVE;
        return MODBUS_ERROR;
    }

    if ((_dataBuffer[9] & 0x7F) != READ_INPUT_REGISTER_FUNCTION) {
        _modbusError = WRONG_FUNCTION;
        return MODBUS_ERROR;
    }

    if (bitRead(_dataBuffer[9], 7)) {
        _modbusError = MODBUS_REQUEST_ERROR;
        return MODBUS_ERROR;
    }

    if((_dataSize - 13) != _dataBuffer[10]) {
        _modbusError = WRONG_BYTE_SIZE;
        return MODBUS_ERROR;
    }

    if((readRegisterCount * 2) != _dataBuffer[10]) {
        _modbusError = WRONG_REGISTER_COUNT;
        return MODBUS_ERROR;
    }

    dataCRC = 0xFFFF;
    for (uint8_t i = 8; i < (_dataSize -2); ++i) {
        dataCRC = _crc16_update(dataCRC, _dataBuffer[i]);
    }
    if (lowByte(dataCRC) != _dataBuffer[_dataSize - 2] || highByte(dataCRC) != _dataBuffer[_dataSize - 1]) {
        _modbusError = CRC_ERROR;
        return MODBUS_ERROR;
    }

    for(uint8_t i = 0; i < readValueCount; ++i) {
        _convert.bVal[3] = _dataBuffer[11 + i * 4];
        _convert.bVal[2] = _dataBuffer[12 + i * 4];
        _convert.bVal[1] = _dataBuffer[13 + i * 4];
        _convert.bVal[0] = _dataBuffer[14 + i * 4];
        data[(*dataPos)++] = _convert.fVal;
    }

    _modbusError = MODBUS_OK;
    return OK;
}

