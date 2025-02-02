#include <Wire.h>
#include <HardwareSerial.h>
#include <Esp.h>
#include <esp_task_wdt.h>
#include "Actimetre.h"

#define MPU6500_FIFO_CNT_H  0x72
#define MPU6500_FIFO_CNT_L  0x73
#define MPU6500_FIFO_DATA   0x74
#define MPU6500_INT_STATUS  0x3A
#define MPU6500_DATA_RDY    0x01
#define MPU6500_FIFO_OVER   0x10

// GENERAL

static void writeByte(int memory, unsigned char cmd) {
    Wire.beginTransmission(MPU6500_ADDR);
    if (Wire.write((unsigned char)memory) != 1) ERROR_FATAL3("writeByte() -> write(memory)");
    if (Wire.write(cmd) != 1)                   ERROR_FATAL3("writeByte() -> write(cmd)");
    if (Wire.endTransmission(true) != 0)        ERROR_FATAL3("writeByte() -> endTransmission");
}

static unsigned char readByte(int memory) {
    unsigned char data;
    
    Wire.beginTransmission(MPU6500_ADDR);
    if (Wire.write((unsigned char)memory) != 1) ERROR_FATAL3("readByte() -> write");
    if (Wire.endTransmission(false) != 0)       ERROR_FATAL3("readByte() -> endTransmission0");
    if (Wire.requestFrom(MPU6500_ADDR, 1) != 1) ERROR_FATAL3("readByte() -> requestFrom");
    data = Wire.read();
    if (Wire.endTransmission(true) != 0)        ERROR_FATAL3("readByte() -> endTransmission1");

    return data;
}

static int readWord(int memory) {
    byte bytes[2];
    
    Wire.beginTransmission(MPU6500_ADDR);
    if (Wire.write((unsigned char)memory) != 1) ERROR_FATAL3("readWord() -> write");
    if (Wire.endTransmission(false) != 0)       ERROR_FATAL3("readWord() -> endTransmission0");
    if (Wire.requestFrom(MPU6500_ADDR, 2) != 2) ERROR_FATAL3("readWord() -> requestFrom");
    if (Wire.readBytes(bytes, 2) != 2)          ERROR_FATAL3("readWord() -> readBytes");
    if (Wire.endTransmission(true) != 0)        ERROR_FATAL3("readWord() -> endTransmission1");

    return (int)bytes[0] << 8 | bytes[1];
}

// SENSORS (MPU-6050)

void clearSensor() {
     int fifoBytes = readWord(MPU6500_FIFO_CNT_H) & 0x1FFF;
     
    Serial.printf("Reset sensor FIFO %d", fifoBytes);
    
    writeByte(0x6A, 0x04); // reset FIFO
    writeByte(0x6A, 0x40); // enable FIFO
    fifoBytes = readWord(MPU6500_FIFO_CNT_H) & 0x1FFF;
    Serial.printf(" -> %d bytes\n", fifoBytes);
    clearNextCycle();
}

void setSensorFrequency() {
    writeByte(0x6A, 0x04); // reset FIFO
    
    Serial.printf("Set frequency %dHz\n", my.sampleFrequency);
    if (my.sampleFrequency == 1000) {
        writeByte(0x1D, 0x00); // A_FCHOICE_B = b0, A_DLPF = 0     (1kHz)
    } else { 
        writeByte(0x1D, 0x08); // A_FCHOICE_B = b1, Disable A_DLPF (4kHz)
    }
            
    writeByte(0x6A, 0x40); // enable FIFO
}

int setSamplingMode() {
    my.cycleMicroseconds = 30 * 1000000 / my.sampleFrequency;
    Serial.printf("Sampling rate %dusec\n", my.cycleMicroseconds);
    return my.sampleFrequency * BYTES_IN_FIFO * 2;
}

static int detectSensor() {
    Wire.beginTransmission(MPU6500_ADDR);
    if (Wire.endTransmission(true) == 0) {
        Serial.print("Initializing");
        writeByte(0x6B, 0x80);    // Reset
    } else {
        Serial.println("No device found");
        RESTART(2);
    }
    delay(100);
    
    byte sensorType = readByte(0x75);
    Serial.printf(" WAI=0x%02X, ", sensorType);
    if (sensorType == 0x74) sensorType = WAI_6500;
    if (sensorType != WAI_6500 && sensorType != WAI_6050) {
        Serial.println("BAD. Rebooting");
        RESTART(2);
    }

    my.lastMessage = 0;

    writeByte(0x6B, 0x08); // Disable temperature, osc clock source
    writeByte(0x6C, 0x07); // Disable gyro
//    writeByte(0x1A, 0x00); // DLPF = 0
//    writeByte(0x1B, 0x00); // FCHOICE_B = b00
    writeByte(0x1C, 0x08); // Accel range +/-4g
    writeByte(0x1D, 0x00); // A_FCHOICE_B = b0, A_DLPF = 0 (1kHz)
    writeByte(0x23, 0x08); // enable FIFO for accel only (6 bytes per sample)
    writeByte(0x38, 0x11); // enable interrupts
    writeByte(0x6A, 0x40); // enable FIFO

    int fifoBytes = readWord(MPU6500_FIFO_CNT_H) & 0x1FFF;
    Serial.printf(" FIFO %d", fifoBytes);
    writeByte(0x6A, 0x04); // reset FIFO
    fifoBytes = readWord(MPU6500_FIFO_CNT_H) & 0x1FFF;
    Serial.printf(" -> %d", fifoBytes);
    
    Serial.println(" OK!");
    return sensorType;
}

// return -1 if OK, >=0 code otherwise
static int fifoError(byte *buffer, int fifoBytes) {
    if (fifoBytes < 12) return -1;
    byte *span = buffer + fifoBytes - 12;
    byte check = span[0];
    int i;
    for (i = 1; span[i] == check && i < 12; i++);
    if (i >= 12) return check;
    else return -1;
}

static int makeInt12(byte msb, byte lsb) {
    int word = msb * 256 + lsb;
    if (word >= 32768) word -= 65536;
    return word / (1 << 4);
}

// Dedicated buffer for FIFO
static byte fifoBuffer[512];
static byte sigBuffer[MAX_MEASURES];

int readFifo(byte *message) {
    byte *buffer = message + HEADER_LENGTH;
    int fifoBytes = readWord(MPU6500_FIFO_CNT_H) & 0x1FFF;
    int fifoCheck = readWord(MPU6500_FIFO_CNT_H) & 0x1FFF;
    if (fifoCheck < fifoBytes) {
        clearSensor();
        my.nMissed[Core1I2C]++;
        char error[64];
        sprintf(error, "FIFO read error: %d != %d", fifoBytes, fifoCheck);
        Serial.println(error);
        ERROR_REPORT(error);
        return 0;
    }
    
    if (fifoBytes > FIFO_OVERFLOW) {
        clearSensor();
        my.nMissed[Core1I2C]++;
        char error[64];
        sprintf(error, "FIFO overflow");
        Serial.println(error);
        ERROR_REPORT(error);
        return 0;
    }

    if (fifoBytes < 2 * BYTES_IN_FIFO) {
        Serial.printf("Too little data %d bytes\n", fifoBytes);
        return 0;
    }
    
    int timeOffset = 0;
    int moreToRead = fifoBytes - MAX_MEASURES * BYTES_IN_FIFO;
    if (moreToRead > 0) {
        timeOffset = (moreToRead / BYTES_IN_FIFO) * (1000000 / my.sampleFrequency) ;
        fifoBytes = MAX_MEASURES * BYTES_IN_FIFO;
    } else {
        fifoBytes = (fifoBytes / BYTES_IN_FIFO - 1) * BYTES_IN_FIFO;
    }
    
    int fifoCount = fifoBytes / BYTES_IN_FIFO;
    int64_t now = formatHeader(message, fifoCount, timeOffset);

    if (my.lastMessage != 0) {
        int64_t span = (now - my.lastMessage) / (1000000 / my.sampleFrequency);
        if (fifoCount > (int)span + FIFO_THRESHOLD) {
            clearSensor();
            my.nMissed[Core1I2C]++;
            char error[64];
            sprintf(error, "FIFO mix-up: %d samples / %d cycles", fifoCount, (int)span);
            Serial.println(error);
            ERROR_REPORT(error);
            return 0;
        }
    } else {
        my.startClock = now - fifoCount * (1000000 / my.sampleFrequency);
    }
    my.lastMessage = now;

    Wire.beginTransmission(MPU6500_ADDR);
    if (Wire.write(MPU6500_FIFO_DATA) != 1) {
        ERROR_FATAL3("readFifo() -> write");
        return 0;
    }
    if (Wire.endTransmission(false) != 0) {
        ERROR_FATAL3("readFifo() -> endTransmission0");
        return 0;
    }
    if (Wire.requestFrom(MPU6500_ADDR, fifoBytes) != fifoBytes) {
        ERROR_FATAL3("readFifo() -> requestFrom");
        return 0;
    }
    if (Wire.readBytes(fifoBuffer, fifoBytes) != fifoBytes) {
        ERROR_FATAL3("readFifo() -> readBytes");
        return 0;
    }
    if (Wire.endTransmission(true) != 0) {
        ERROR_FATAL3("readFifo() -> endTransmission1");
        return 0;
    }
    
    int errorCode = fifoError(fifoBuffer, fifoBytes);
    if (errorCode >= 0) {
        clearSensor();
        my.nMissed[Core1I2C]++;
        char error[64];
        sprintf(error, "FIFO data 12/%d bytes 0x%X", fifoBytes, errorCode);
        Serial.println(error);
        ERROR_REPORT3(error);
        return 0;
    }

    readSignals(sigBuffer, fifoCount);

    // Weave FIFO data and Signals
    byte *fifoPtr = fifoBuffer;
    for (int i = 0; i < fifoCount; i++) {
        memcpy(buffer, fifoPtr, BYTES_IN_FIFO);
        buffer[BYTES_IN_FIFO] = sigBuffer[i];
        fifoPtr += BYTES_IN_FIFO;
        buffer += BYTES_IN_RECORD;
    }

    if (moreToRead / BYTES_IN_FIFO > FIFO_THRESHOLD) {
        Serial.printf("FIFO more to read %d\n", moreToRead / BYTES_IN_FIFO);
        return 2;
    }
    return 1;
}

// GLOBAL

void deviceScanInit() {
    Serial.print("Checking I2C devices\n");

    if (!detectSensor()) {
        Serial.println("No sensors found, rebooting");
        blinkLed(COLOR_RED);
        RESTART(5);
    }

    int budget = setSamplingMode();
    if (budget > I2C_BAUDRATE) {
        Serial.printf("Requires %d baud > %d. Stop\n", budget, I2C_BAUDRATE);
        blinkLed(COLOR_RED);
        RESTART(5);
    }
    
    setSensorFrequency();
    clearSensor();
}
