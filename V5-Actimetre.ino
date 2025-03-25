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

static StaticTask_t core1Task;
static StackType_t core1Stack[16384];

void setup() {
    memset(&my, 0x00, sizeof(MyInfo));
    
    setupBoard();
    
    delay(100);
    deviceScanInit();
    netInit();
    blinkLed(COLOR_FREQ | 1);

    my.core1Task = xTaskCreateStaticPinnedToCore(Core1Loop, "Core1", 16384, NULL, 2, core1Stack, &core1Task, 1);
    if (my.core1Task == NULL) {
        Serial.println("Error starting Main task");
        ESP.restart();
    }
}

// MAIN LOOP

int64_t formatHeader(byte *message, int count, int timeOffset) {
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
        sprintf(error, "FIFO count %d > 64", count);
        ERROR_FATAL(error);
    }
    message[3] = count;
    message[4] = ((byte)my.rssi << 5) | (byte)my.frequencyCode | (SAMPLE_ACCEL << 3);
    message[5] = 0xA0 | ((msgMicros >> 16) & 0x0F);
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

static void Core1Loop(void *dummy_to_match_argument_signature) {
    Serial.printf("Core %d started\n", xPortGetCoreID());
    setupSignals();
    clearSensor();
    for (;;) {
        MainLoop();
        delayMicroseconds(1);
    }
}
void loop() {}
static void MainLoop()
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
        do {
            int index = nextIndex();
            fifoState = readFifo(msgQueueStore[index]);
            if (fifoState > 0) {
                queueIndex(index);
            }
        } while (fifoState > 1);
    }
    
    logCycleTime(Core1I2C, micros_diff(micros(), cycle_time));
}

// UTILITY FUNCTION

bool FATAL_ERROR = false;

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
    formatHeader(message, msgLength / 4, 0);
    strcpy((char*)message + HEADER_LENGTH, what);
    memset(message + HEADER_LENGTH + msgLength, 0, 4 - msgLength % 4);
    message[0] = 0xFF;
    queueIndex(index);
}

void ERROR_REPORT3(char *what) {
    Serial.printf("REPORT3:%s\n", what);
    
    int index = nextIndex();
    byte *message = msgQueueStore[index];
    int msgLength = strlen(what) + 3;
    if (msgLength > 255) {
        what[252] = 0;
        msgLength = 255;
    }
    formatHeader(message, msgLength / 4, 0);
    message[5] |= 0x10;
    strcpy((char*)message + HEADER_LENGTH, what);
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
    Serial.println(where);
    
    if (coreId == 1) ERROR_REPORT(where);
    RESTART(5);
}

void ERROR_FATAL3(char *where) {
    int coreId = xPortGetCoreID();
    if (coreId == 1) { // no re-rentry for main loop
        while (FATAL_ERROR) delay(1);
    }
    
    FATAL_ERROR = true;
    Serial.printf("FATAL#%d\n", coreId);
    Serial.println(where);
    
    if (coreId == 1) ERROR_REPORT3(where);
    RESTART(5);
}

static bool processError() {
    if (FATAL_ERROR) {
        return true;
    }
    return false;
}

void dump(void *pointer, int size) {
    char text[64] = "";
    byte *address = (byte*)pointer;
    for(int line = 0; line < size; line += 16) {
        text[0] = 0;
        for (int i = 0; (i < 16) && (line + i < size); i++) {
            sprintf(text + i * 3, "%02X ", address[line + i]);
        }
        Serial.println(text);
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
