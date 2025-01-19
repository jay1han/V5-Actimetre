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

int DATA_LENGTH[] = {12, 6, 6, 12};

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

static int clearSensor() {
    int fifoBytes = readWord(MPU6500_FIFO_CNT_H) & 0x1FFF;
    
    Serial.printf("Reset sensor FIFO %d", fifoBytes);
    
    writeByte(0x6A, 0x04); // reset FIFO
    writeByte(0x6A, 0x40); // enable FIFO
    fifoBytes = readWord(MPU6500_FIFO_CNT_H) & 0x1FFF;
    Serial.printf(" -> %d bytes\n", fifoBytes);
    return fifoBytes;
}

static void setSensorFrequency() {
    if (my.sampleFrequency <= 1000) {
        int divider = 1000 / my.sampleFrequency - 1;
        Serial.printf("Sampling rate divider %d\n", divider);
        writeByte(0x6A, 0x04); // reset FIFO
        writeByte(0x6B, 0x09); // Disable temperature, Gx clock source
        writeByte(0x6C, 0x00); // Enable accel
        writeByte(0x19, (byte)divider); // Sampling rate divider
        writeByte(0x1C, 0x08); // Accel range +/-4g
        writeByte(0x1A, 0x01); // DLPF = 1
        writeByte(0x1B, 0x00); // FCHOICE_B = b00
        writeByte(0x1D, 0x00); // A_FCHOICE_B = b0, A_DLPF = 0
        writeByte(0x23, 0x78); // enable FIFO for gx, gy, gz, accel (12 bytes per sample)
        writeByte(0x6A, 0x40); // enable FIFO
    } else { 
        writeByte(0x6A, 0x04); // reset FIFO

        if (my.sampleFrequency <= 4000) {  // only accel
            writeByte(0x6B, 0x08); // Disable temperature, osc clock source
            writeByte(0x6C, 0x07); // Disable gyro
            writeByte(0x1A, 0x00); // DLPF = 0
            writeByte(0x1B, 0x00); // FCHOICE_B = b00
            writeByte(0x1C, 0x08); // Accel range +/-4g
            writeByte(0x1D, 0x08); // A_FCHOICE_B = b1, Disable A_DLPF
            writeByte(0x23, 0x08); // enable FIFO for accel (6 bytes per sample)
        } else if (my.sampleFrequency == 8000) { // only gyro
            writeByte(0x6B, 0x09); // Disable temperature, Gx clock source
            writeByte(0x6C, 0x38); // Disable accel
            writeByte(0x1A, 0x07); // DLPF = 7
            writeByte(0x1B, 0x00); // FCHOICE_B = b00
            writeByte(0x1C, 0x08); // Accel range +/-4g
            writeByte(0x1D, 0x08); // A_FCHOICE_B = b1, Disable A_DLPF
            writeByte(0x23, 0x70); // enable FIFO for gx, gy, gz (6 bytes per sample)
        } else {
            Serial.printf("Sensor unhandled frequency %d\n", my.sampleFrequency);
            RESTART(2);
        }
            
        writeByte(0x6A, 0x40); // enable FIFO
    }
}

void setSensorsFrequency() {
    setSensorFrequency();
    clearSensor();
}

int setSamplingMode() {
    int perCycle = 100;
    int budget = 0;
    int samplingMode;
    
        samplingMode = SAMPLE_ACCEL;
    my.sensor.samplingMode = samplingMode;

        my.sensor.maxMeasures = 40;
        my.sensor.fifoThreshold = 10;
        my.sensor.dataLength  = 6;
        if (perCycle > 30) perCycle = 30;
        
    budget = my.sampleFrequency * my.sensor.dataLength;
    Serial.printf("mode %d (max %d, sample %d bytes)\n",
                  my.sensor.samplingMode,
                  my.sensor.maxMeasures,
                  my.sensor.dataLength);

    my.cycleMicroseconds = perCycle * 1000000 / my.sampleFrequency;
    Serial.printf("Sampling frequency %dHz(code %d), cycle time %dus.",
                  my.sampleFrequency, my.frequencyCode, my.cycleMicroseconds);
    Serial.printf(" Req. %d baud\n", budget);
    my.I2Cbudget = budget;
    return budget;
}

void clearSensors() {
    clearSensor();
    my.sensor.nSamples = 0;
}

static int detectSensor() {
    Wire.beginTransmission(MPU6500_ADDR);
    if (Wire.endTransmission(true) == 0) {
        Serial.print("Initializing");
        writeByte(0x6B, 0x80);    // Reset
    }
    delay(100);
    
    byte sensorType = readByte(0x75);
    Serial.printf(" WAI=0x%02X, ", sensorType);
    if (sensorType == 0x74) sensorType = WAI_6500;
    if (sensorType != WAI_6500) {
        Serial.println("BAD. Rebooting");
        RESTART(2);
    }

    my.sensor.type = sensorType;
    my.sensor.lastMessage = 0;
    my.sensor.nSamples = 0;
    my.sensor.nCycles = 0;
    my.sensor.fifoOverflow = 500;
//  writeByte(0x6C, 0x01); // Disable gz
    writeByte(0x6B, 0x08); // Disable temperature, osc clock source
    writeByte(0x19, 0);    // Sampling rate divider
    writeByte(0x1C, 0x08); // Accel range +/-4g
    writeByte(0x1A, 0x01); // DLPF = 1
    writeByte(0x1B, 0x00); // FCHOICE_B = b00
    writeByte(0x1D, 0x00); // A_FCHOICE_B = b0, A_DLPF = 0
    writeByte(0x23, 0x78); // enable FIFO for gx, gy, gz, accel (12 bytes per sample)
//  writeByte(0x38, 0x11); // enable interrupts
//  writeByte(0x6A, 0x40); // enable FIFO

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

static char *checkData(byte *buffer, int samplingMode, int count) {
    int dataLength = DATA_LENGTH[samplingMode];
    char *result = NULL;
    int ax, ay, az, vector = 0;
    int rx = 0, ry = 0, rz = 0;
    for (int i = 0; i < count; i++) {
        byte *dataPoint = buffer + i * dataLength;
        ax = makeInt12(dataPoint[0], dataPoint[1]);
        ay = makeInt12(dataPoint[2], dataPoint[3]);
        az = makeInt12(dataPoint[4], dataPoint[5]);
        vector += ax * ax + ay * ay + az * az;
        dataPoint += 6;
    }
    
    if (vector > 0) {
        if (vector > 0x400000 * count) {
            result = "Accel > 4g";
        } else if (vector < 0x1000 * count) {
            result = "Accel < 0.125g";
        }
    }

    if (abs(rx) > 410 * count || abs(ry) > 410 * count || abs(rz) > 410 * count) {
        if (result != NULL) result = "Acc & Rot > limits";
        else result = "Rotation > 50deg/s";
    }

    // return result;
    // Unverified, so no check for Prod
    return NULL;
}

int readFifo(byte *message) {
    byte *buffer = message + HEADER_LENGTH;
    int dataLength = my.sensor.dataLength;

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
    
    if (fifoBytes > my.sensor.fifoOverflow) {
        clearSensor();
        my.nMissed[Core1I2C]++;
        char error[64];
        sprintf(error, "FIFO overflow");
        Serial.println(error);
        ERROR_REPORT(error);
        return 0;
    }

    if (fifoBytes < 2 * dataLength) {
        Serial.print("Too little data\n");
        return 0;
    }
    
    int maxMeasures = my.sensor.maxMeasures;
    int timeOffset = 0;
    int moreToRead = fifoBytes - maxMeasures * dataLength;
    if (moreToRead > 0) {
        timeOffset = (moreToRead / dataLength) * (1000000 / my.sampleFrequency) ;
        fifoBytes = maxMeasures * dataLength;
    } else {
        fifoBytes = (fifoBytes / dataLength) * dataLength;
        fifoBytes -= dataLength;
    }
    
    int fifoCount = fifoBytes / dataLength;
    int64_t now = formatHeader(message, fifoCount, timeOffset);

    if (my.sensor.lastMessage != 0) {
        int64_t span = (now - my.sensor.lastMessage) / (1000000 / my.sampleFrequency);
        if (fifoCount > (int)span + my.sensor.fifoThreshold) {
            clearSensor();
            my.nMissed[Core1I2C]++;
            char error[64];
            sprintf(error, "FIFO mix-up: %d samples / %d cycles", fifoCount, (int)span);
            Serial.println(error);
            ERROR_REPORT(error);
            return 0;
        }
        my.sensor.nCycles = (now - my.sensor.startClock) / (1000000 / my.sampleFrequency);
    } else {
        my.sensor.startClock = now - fifoCount * (1000000 / my.sampleFrequency);
        my.sensor.nCycles = fifoCount;
    }
    my.sensor.lastMessage = now;
    my.sensor.nSamples += fifoCount;

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
    if (Wire.readBytes(buffer, fifoBytes) != fifoBytes) {
        ERROR_FATAL3("readFifo() -> readBytes");
        return 0;
    }
    if (Wire.endTransmission(true) != 0) {
        ERROR_FATAL3("readFifo() -> endTransmission1");
        return 0;
    }
    
    int errorCode = fifoError(buffer, fifoBytes);
    if (errorCode >= 0) {
        clearSensor();
        my.nMissed[Core1I2C]++;
        char error[64];
        sprintf(error, "FIFO data 12/%d bytes 0x%X", fifoBytes, errorCode);
        Serial.println(error);
        ERROR_REPORT3(error);
        return 0;
    }

    char *dataSanity = checkData(buffer, my.sensor.samplingMode, fifoCount);
    if (dataSanity != NULL) {
        clearSensor();
        my.nMissed[Core1I2C]++;
        Serial.println(dataSanity);
        ERROR_REPORT3(dataSanity);
        return 0;
    }
    
    if (moreToRead / dataLength > my.sensor.fifoThreshold) {
        Serial.printf("FIFO more to read %d\n", moreToRead / dataLength);
        return 2;
    }
    return 1;
}

// GLOBAL

void deviceScanInit() {
    Serial.print("Checking I2C devices\n");

    int port, address;
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
}
