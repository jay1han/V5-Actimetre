#ifndef ACTIMETRE_H
#define ACTIMETRE_H

#include <Wire.h>

#define VERSION_STR "512"

static void _test(int);
//#define TEST_LOCAL(t)     _test(t)
#define TEST_LOCAL(t)

// CONSTANTS

#define ACTISERVER  "Actis"
#define LONGPRESS_MILLIS  2000L

#define I2C_BAUDRATE      400000

#define MPU6500_ADDR 0x68
#define WAI_6500     0x70
#define WAI_6050     0x68
#define SAMPLE_ACCEL 1

#define MEASURE_SECS       60
#define MAX_MISSED1        3
#define MAX_MISSED0        3
#define MAX_MISSED0_SINGLE 30

#define MAX_MEASURES    40
#define FIFO_THRESHOLD  10
#define FIFO_OVERFLOW   500

#define BYTES_IN_FIFO   6
#define BYTES_IN_RECORD (BYTES_IN_FIFO + 1)

#define HEADER_LENGTH    8     // epoch(3), count(1), rssi(high)+freq(low) (1), usec(3)
#define BUFFER_LENGTH    (BYTES_IN_RECORD * MAX_MEASURES + HEADER_LENGTH)
#define QUEUE_SIZE       200

typedef enum {
    PIN_BUTTON   = 0,
//    PIN_LEDZ     = 21,
    PIN_LEDM     = 47,

    PIN_I2C0_SDA  = 13,
    PIN_I2C0_SCL  = 11,

    PIN_I2C_SDA  = 37,
    PIN_I2C_SCL  = 33,

    PIN_CAM_REC  = 2,
    PIN_CAM_1    = 4,
    PIN_CAM_2    = 12,
    PIN_GATE     = 17,
    PIN_DRAIN    = 34,

    PIN_PULLUP   = 14,
    PIN_PULLDOWN = 15
} PinName;

// TYPES

typedef enum {Core0Net, Core1I2C, CoreNumMax} CoreNum;

// GLOBALS

typedef struct {
    unsigned char mac[6];
    char macString[15];
    unsigned int clientId;
    char clientName[12];

    unsigned int serverId;
    char ssid[10];
    char serverIP[20];
    int rssi;
    time_t bootTime;
    int frequencyCode;
    int sampleFrequency;
    unsigned long cycleMicroseconds;

    int64_t lastMessage;
    int64_t startClock;
    
    float queueFill;
    int nMissed[2];
    float avgCycleTime[2];
    unsigned int upTime;
    TaskHandle_t core0Task, core1Task;

    bool isStopped;
} MyInfo;

extern MyInfo my;

// INTERFACES

// reseau.cpp
void netInit();
int isConnected();
void queueIndex(int);
void netWork();
void netCore0(void *dummy_to_match_argument_signature);
extern byte msgQueueStore[QUEUE_SIZE][BUFFER_LENGTH];

// devices.cpp
extern TwoWire &wire;
int readFifo(byte *buffer);
void clearSensor();
void setSensorFrequency();
int setSamplingMode();
void deviceScanInit();

#define REMOTE_COMMAND   0xF0
#define REMOTE_BUTTON    0x10
#define REMOTE_STOP      0x30
#define REMOTE_RESTART   0xF0

// boards.cpp
void setupBoard();
void blinkLed(int color);
void manageButton(int set);
void setupCore0(void (*core0Loop)(void*));

//                      BBGGRR
#define COLOR_WHITE   0x3F3F3F
#define COLOR_RED     0x00003F
#define COLOR_GREEN   0x003F00
#define COLOR_BLUE    0x3F0000
#define COLOR_BLACK   0x000000
#define COLOR_SWAP    0x8000000
#define COLOR_FREQ    0x1000000
#define COLOR_BLINK   0x2000000

// clock.cpp
void initClock(time_t bootEpoch);
void getTimeSinceBoot(time_t *sec, int *usec);
int64_t getAbsMicros();
unsigned long millis_diff_10(unsigned long end, unsigned long start);
unsigned long micros_diff(unsigned long end, unsigned long start);
void waitNextCycle();
void clearNextCycle();
void logCycleTime(CoreNum coreNum, unsigned long time_spent);
void clearCycleTime();

// Actimetre.ino
void dump(void *address, int size);
void ERROR_REPORT(char *what);
void ERROR_REPORT3(char *what);
void ERROR_FATAL(char *where);
void ERROR_FATAL3(char *where);
extern bool FATAL_ERROR;
void RESTART(int);
int64_t formatHeader(unsigned char *message, int count, int timeOffset);

// signal.cpp
void readSignals(byte *buffer, int take, int leave);
void setupSignals();
void clearSignals();

#endif //ACTIMETRE_H
