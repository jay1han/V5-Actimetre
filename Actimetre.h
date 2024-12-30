#ifndef ACTIMETRE_H
#define ACTIMETRE_H

#define VERSION_STR "411"

//#define PROFILE_DISPLAY
//#define LOG_DISPLAY
//#define PROFILE_NETWORK
//#define LOG_HEARTBEAT
//#define LOG_STACK
#define STATIC_STACK
#define STATIC_QUEUE
//#define TIGHT_QUEUE
//#define LOG_QUEUE
#define INFO_DISPLAY       0   // 0 = none, 1 = fifo, 2 = stack, 3 = display
//#define STOP_FATAL

static void _test(int);
//#define TEST_LOCAL(t)     _test(t)
#define TEST_LOCAL(t)

// CONSTANTS

#define ACTISERVER  "Actis"
#define LONGPRESS_MILLIS  2000L

#define MPU_BAUDRATE     400000
#define DISPLAY_BAUDRATE 400000
#define LOW_BAUDRATE     400000

#define SSD1306_ADDR 0x3C
#define MPU6050_ADDR 0x68
#define WAI_6050     0x68
#define WAI_6500     0x70

#define LCD_H_RES 128
#define LCD_V_RES 64

#define MEASURE_SECS       60
#define MAX_MISSED1        3
#define MAX_MISSED0        3
#define MAX_MISSED0_SINGLE 30

#define HEADER_LENGTH    8     // epoch(3), count(1), rssi(high)+freq(low) (1), usec(3)
#define BUFFER_LENGTH    (240 + HEADER_LENGTH)
#if CONFIG_IDF_TARGET_ESP32S3
#define QUEUE_SIZE       800
#else
#define QUEUE_SIZE       100
#endif

#define SAMPLE_ACCEL     1
#define SAMPLE_GYRO      2
#define SAMPLE_ALL       0
extern int DATA_LENGTH[];

// TYPES

typedef enum {Core0Net, Core1I2C, CoreNumMax} CoreNum;

typedef enum {
    BOARD_BAD = 0,
    BOARD_S3_I2C,
    BOARD_S3_NEWBOX,
    BOARD_S3_ZERO,
    BOARD_S3_NEWBOX2,
    BOARD_S2_SOLO,
    BOARD_C3_SOLO,
    BOARD_S3_SOLO,
    BOARD_TYPES
} BoardType;

// GLOBALS

typedef struct {
    byte type;
    int samplingMode;
    int dataLength;
    int maxMeasures;
    int fifoThreshold;
    int fifoOverflow;
    uint64_t nSamples;
    uint64_t nCycles;
    int64_t lastMessage;
    int64_t startClock;
} SensorDesc;

typedef struct {
    BoardType boardType;
    bool hasI2C[2];
    int ledRGB;
    char boardName[4];
    unsigned char mac[6];
    char macString[15];
    unsigned int clientId;
    char clientName[12];
    bool dualCore;

    unsigned int serverId;
    char ssid[10];
    char serverIP[20];
    int rssi;
    time_t bootTime;
    int frequencyCode;
    int sampleFrequency;
    unsigned long cycleMicroseconds;
    int I2Cbudget;
    
    int displayPort;
    SensorDesc sensor[2][2];
    unsigned char sensorBits;
    int nSensors;
    char sensorList[10];

    float queueFill;
    int nMissed[2];
    float avgCycleTime[2];
    unsigned int upTime;
    TaskHandle_t core0Task, core1Task;

    bool isStopped;
} MyInfo;

extern MyInfo my;

// INTERFACES

// display.cpp
void initDisplay();
void displayTitle(char *title);
void displaySensors();
void displayLoop(int firstLoop);
void writeLine(char *message);

// reseau.cpp
void netInit();
int isConnected();
void queueIndex(int);
void netWork();
void netCore0(void *dummy_to_match_argument_signature);
extern byte msgQueueStore[QUEUE_SIZE][BUFFER_LENGTH];

// devices.cpp
int readFifo(int port, int address, byte *buffer);
void clearSensors();
void setSensorsFrequency();
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
//void longPress();
//void shortPress();

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
void ERROR_REPORT3(int port, int address, char *what);
void ERROR_FATAL(char *where);
void ERROR_FATAL3(int port, int address, char *where);
extern bool FATAL_ERROR;
void RESTART(int);
int64_t formatHeader(int port, int address, unsigned char *message, int count, int timeOffset);

#endif //ACTIMETRE_H
