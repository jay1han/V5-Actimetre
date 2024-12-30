#include <Wire.h>
#include <WiFi.h>
#include <HardwareSerial.h>
#include <Esp.h>
#include <esp_task_wdt.h>
#include <time.h>
#include "Actimetre.h"

// GLOBALS

MyInfo my;

// MAIN SETUP

#ifdef STATIC_STACK
static StaticTask_t core1Task;
static StackType_t core1Stack[16384];
#endif

void setup() {
    memset(&my, 0x00, sizeof(MyInfo));
    
    setupBoard();
    if (my.boardType == BOARD_BAD) {
        Serial.println("Unsupported board type");
        writeLine("Unsupported");
        RESTART(30);
    }
    
#ifdef LOG_STACK
    Serial.printf("Loop stack size %d\n", getArduinoLoopTaskStackSize());
#endif    

    delay(100);
    deviceScanInit();

    char title[16];
    sprintf(title, "v%s", VERSION_STR);
    displayTitle(title);
    displaySensors();
    
    netInit();
    clearSensors();
    blinkLed(COLOR_FREQ | 1);

#ifdef STATIC_STACK
    if (my.dualCore) {
        my.core1Task = xTaskCreateStaticPinnedToCore(Core1Loop, "Core1", 16384, NULL, 2, core1Stack, &core1Task, 1);
    } else {
        my.core1Task = xTaskCreateStatic(Core1Loop, "Core", 16384, NULL, 2, core1Stack, &core1Task);
    }
    if (my.core1Task == NULL) {
        Serial.println("Error starting Main task");
        ESP.restart();
    }
#endif        
}

// MAIN LOOP

int64_t formatHeader(int port, int address, byte *message, int count, int timeOffset) {
    time_t msgBootEpoch;
    int msgMicros;
    getTimeSinceBoot(&msgBootEpoch, &msgMicros);
    msgMicros -= timeOffset;
    if (msgMicros < 0) {
        msgMicros += 1000000;
        msgBootEpoch -= 1;
    }
    message[0] = (msgBootEpoch >> 16) & 0xFF;
    message[1] = (msgBootEpoch >> 8) & 0xFF;
    message[2] = msgBootEpoch & 0xFF;

    if (count > 63) {
        char error[64];
        sprintf(error, "FIFO %d%c count %d > 64", 1 + port, 'A' + address, count);
        ERROR_FATAL(error);
    }
    message[3] = count | (port << 7) | (address << 6);
    message[4] = ((byte)my.rssi << 5) | ((byte)my.sensor[port][address].samplingMode << 3) | (byte)my.frequencyCode;
    message[5] = (msgMicros >> 16) & 0x0F;
    if (my.sensor[port][address].type == WAI_6500)
        message[5] |= 0x80;
    message[6] = (msgMicros >> 8) & 0xFF;
    message[7] = msgMicros & 0xFF;
    return (int64_t)msgBootEpoch * 1000000 + msgMicros;
}

byte msgQueueStore[QUEUE_SIZE][BUFFER_LENGTH];
static int nextIndex() {
    static int msgIndex = 1;
    int index = msgIndex++;
    if (msgIndex >= QUEUE_SIZE) msgIndex = 1;
    return index;
}

#ifdef STATIC_STACK
static void Core1Loop(void *dummy_to_match_argument_signature) {
    Serial.printf("Core %d started\n", xPortGetCoreID());
    for (;;) {
        MainLoop();
        delayMicroseconds(1);
    }
}
void loop() {}
static void MainLoop()
#else
void loop()    
#endif
{
//    TEST_LOCAL(1);
    if (processError()) return;
    if (!isConnected()) RESTART(2);
    manageButton(0);
    
    waitNextCycle();
    
    unsigned long cycle_time = micros();

    if (my.isStopped) {
        queueIndex(1);
    } else {
        int fifoState;
        for (int port = 0; port <= 1; port++) {
            for (int address = 0; address <= 1; address++) {
                if (my.sensor[port][address].type) {
                    do {
                        int index = nextIndex();
                        fifoState = readFifo(port, address, msgQueueStore[index]);
                        if (fifoState > 0) {
                            queueIndex(index);
                        }
                    } while (fifoState > 1);
                }
            }
        }
    }
    
    logCycleTime(Core1I2C, micros_diff(micros(), cycle_time));
}

// UTILITY FUNCTION

bool FATAL_ERROR = false;
static char errorDisplay[32] = "";

void RESTART(int seconds) {
    FATAL_ERROR = true;
    Serial.printf("RESTART in %d\n", seconds);
    blinkLed(COLOR_RED);
    delay(1000 * seconds);
    blinkLed(COLOR_BLACK);
    ESP.restart();
}

void ERROR_REPORT(char *what) {
    Serial.printf("REPORT:%s\n", what);
    
    int index = nextIndex();
    byte *message = msgQueueStore[index];
    int msgLength = strlen(what);
    if (msgLength > 62) {
        what[62] = 0;
        msgLength = 62;
    }
    formatHeader(0, 0, message, msgLength / 4, 0);
    strcpy((char*)message + HEADER_LENGTH, what);
    memset(message + HEADER_LENGTH + msgLength, 0, 4 - msgLength % 4);
    message[0] = 0xFF;
    queueIndex(index);
}

void ERROR_REPORT3(int port, int address, char *what) {
    Serial.printf("REPORT(%d%c):%s\n", 1 + port, 'A' + address, what);
    
    int index = nextIndex();
    byte *message = msgQueueStore[index];
    int msgLength = strlen(what) + 3;
    if (msgLength > 255) {
        what[252] = 0;
        msgLength = 255;
    }
    formatHeader(port, address, message, msgLength / 4, 0);
    message[5] |= 0x10;
    sprintf((char*)message + HEADER_LENGTH, "%d%c %s", 1 + port, 'A' + address, what);
    memset(message + HEADER_LENGTH + msgLength, 0, 4 - msgLength % 4);
    queueIndex(index);
}

void ERROR_FATAL(char *where) {
    int coreId = xPortGetCoreID();
    if (coreId == 1) { // no re-rentry for main loop
        while (FATAL_ERROR) delay(1);
    }
    
    FATAL_ERROR = true;
    Serial.printf("FATAL#%d\n", coreId);
    
#ifdef STOP_FATAL
    memset(errorDisplay, 0, sizeof(errorDisplay));
    sprintf(errorDisplay, "FATAL#%d", coreId);
    strcpy(errorDisplay + 8, where);
    blinkLed(COLOR_RED);
    Wire.endTransmission(true);
    Wire1.endTransmission(true);
    if (coreId == 1) processError();
    while (true) delay(1);
#else
    if (coreId == 1) ERROR_REPORT(where);
    RESTART(5);
#endif    
}

void ERROR_FATAL3(int port, int address, char *where) {
    int coreId = xPortGetCoreID();
    if (coreId == 1) { // no re-rentry for main loop
        while (FATAL_ERROR) delay(1);
    }
    
    FATAL_ERROR = true;
    Serial.printf("FATAL#%d\n", coreId);
    
#ifdef STOP_FATAL
    memset(errorDisplay, 0, sizeof(errorDisplay));
    sprintf(errorDisplay, "FATAL#%d", coreId);
    strcpy(errorDisplay + 8, where);
    blinkLed(COLOR_RED);
    Wire.endTransmission(true);
    Wire1.endTransmission(true);
    if (coreId == 1) processError();
    while (true) delay(1);
#else
    if (coreId == 1) ERROR_REPORT3(port, address, where);
    RESTART(5);
#endif    
}

static bool processError() {
    if (FATAL_ERROR) {
        if (errorDisplay[0] != 0) {
            Serial.printf("processError:%s\n", errorDisplay);
            char *error = errorDisplay;
            for (int line = 0; line < 2; line++) {
                int linelen = 0;
                while (error[linelen] != 0) linelen++;
                writeLine(error);
                error += linelen + 1;
            }
            memset(errorDisplay, 0, sizeof(errorDisplay));
        }
        return true;
    }
    return false;
}

void dump(void *pointer, int size) {
    byte *address = (byte*)pointer;
    for(int line = 0; line < size; line += 16) {
        for (int i = 0; (i < 16) && (line + i < size); i++) {
            Serial.printf("%02X ", address[line + i]);
        }
        Serial.print("    ");
        for (int i = 0; (i < 16) && (line + i < size); i++) {
            byte c = address[line + i];
            if (c < 0x20) c = '.';
            Serial.printf("%c", c);
        }
        Serial.println();
    }
}

static void _test(int type) {
    switch (type) {
    case 1:
        if (getAbsMicros() > 10000000) {
            ERROR_FATAL("Test FATAL1");
        }
    }
}
