#include <Wire.h>
#include <HardwareSerial.h>
#include <esp_cpu.h>
#include <Esp.h>
#include <esp32-hal.h>
#include <esp_chip_info.h>
#include "Actimetre.h"

typedef enum {
    PIN_BUTTON   = 0,
    PIN_LEDZ     = 21,
    PIN_LEDM     = 47,

    PIN_I2C_SDA  = 12,
    PIN_I2C_SCL  = 13,
    PIN_I2C_GND  = 11,
    PIN_I2C_VCC  = 10,

    PIN_CAM_1    = 6,
    PIN_CAM_2    = 7,
    PIN_CAM_REC  = 8,
    
    PIN_SYNC     = 1
} PinName;

// BOARD DEFINITION

static char BoardName[4] = "S3x";

#define FREQ_COUNT   2
static int freqCode =  0;
static int Frequencies[8] = {100, 500, 1000, 2000, 4000, 8000};
static int FrequencyCode[FREQ_COUNT] = {2, 4};

static void switchFrequency() {
    do {
        freqCode = (freqCode + 1) % FREQ_COUNT;
        my.frequencyCode = FrequencyCode[freqCode];
        my.sampleFrequency = Frequencies[my.frequencyCode];
    } while (setSamplingMode() > I2C_BAUDRATE);
    
    setSensorFrequency();
    clearSensor();
    clearCycleTime();
    clearNextCycle();
    int kHz = my.sampleFrequency / 1000;
    int rank = 0;
    while (kHz > 0) {
        rank ++;
        kHz >>= 1;
    }
    blinkLed(COLOR_FREQ | rank);
}

// LED AND BUTTON

static void longPress() {
    Serial.println("Button long-press");
    ERROR_REPORT("Long press");
    // Do nothing
}

static void shortPress() {
    Serial.println("Button press");
    ERROR_REPORT("Button press");
    switchFrequency();
}

void manageButton(int set) {
    static unsigned long buttonDown;
    static int prevButton = 0;
    int nowButton  = 0;
    static int actioned   = 0;

    if (set == 1) { // Force a short press
        prevButton = 1;
        buttonDown = millis() - LONGPRESS_MILLIS;
        return;
    }
    
    nowButton = 1 - digitalRead(PIN_BUTTON);
    if (nowButton) { // pressed
        if (prevButton) { // still pressed
            if (millis_diff_10(millis(), buttonDown) > LONGPRESS_MILLIS) { // it's a long press
                if (!actioned){  // only action once
                    longPress();
                    actioned = 1;
                }
            } else { // wait until timeout or release
            }
        } else { // was not pressed, start counter
            buttonDown = millis();
            actioned = 0;
        }
    } else { // now released
        if (prevButton) { // was pressed
            if (!actioned) { // short press
                shortPress();
            } else { // already actioned, so do nothing
                actioned = 0;
            }
        } else {
            // nothing...
        }
    }

    prevButton = nowButton;
}

#define RMT_SIZE (8 * 3)
static rmt_data_t RmtBuffer[RMT_SIZE];
static void setupLED(int pin) {
    Serial.printf("RGB pin %d init ", pin);
    if (!rmtInit(pin, RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_1, 20000000))
        Serial.println("FAILED");
    else Serial.println("OK");
}
    
static rmt_data_t *stuffBits(rmt_data_t *data, int level) {
    for (int bit = 7; bit >= 0; bit--) {
        if (level & (1 << bit)) {
            data->level0    = 1;
            data->duration0 = 12;
            data->level1    = 0;
            data->duration1 = 5;
        } else {
            data->level0    = 1;
            data->duration0 = 5;
            data->level1    = 0;
            data->duration1 = 12;
        }
        data++;
    }
    return data;
}

//                                     <1K,      1K,       2K,       4K,       8K
//                                     magenta,  yellow,   cyan,     green,    blue
static int COLORS[]                 = {0x0F0007, 0x00070F, 0x0F0700, 0x000F00, 0x170000};
static unsigned long BLINK_MILLIS[] = {2000,     1000,     500,      200,      200     };

void blinkLed(int command) {
    static int saved = COLOR_WHITE;
    static int frequency = 1;
    static bool state = false;
    static unsigned long blinkTime = millis();
    int color;
    rmt_data_t *data = RmtBuffer;

    if (command == COLOR_BLINK) {
        unsigned long blinkPeriod = 1000;
        if (my.isStopped) blinkPeriod = 50;
        if (millis_diff_10(millis(), blinkTime) > blinkPeriod) {
            blinkTime = millis();
            state = !state;
            if (state) color = saved;
            else color = COLOR_BLACK;
        } else return;
    } else if (command == COLOR_SWAP) {
        state = !state;
        if (state) color = saved;
        else color = COLOR_BLACK;
    } else if (command & COLOR_FREQ) {
        frequency = command & 0x07;
        color = saved = COLORS[frequency];
        state = true;
    } else {
        color = command;
        if (color == COLOR_BLACK) {
            state = false;
        } else {
            saved = color;
            state = true;
        }
    }
    
    data = RmtBuffer;
    data = stuffBits(data, color & 0xFF);
    data = stuffBits(data, (color >> 8) & 0xFF);
    data = stuffBits(data, (color >> 16) & 0xFF);
    rmtWrite(PIN_LEDM, RmtBuffer, RMT_SIZE, RMT_WAIT_FOR_EVER);

    data = RmtBuffer;
    data = stuffBits(data, (color >> 8) & 0xFF);
    data = stuffBits(data, color & 0xFF);
    data = stuffBits(data, (color >> 16) & 0xFF);
    rmtWrite(PIN_LEDZ, RmtBuffer, RMT_SIZE, RMT_WAIT_FOR_EVER);
}

void setupBoard() {
    Serial.setTxTimeoutMs(0);
    Serial.begin(2000000);
    delay(2000);

    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    if (chip_info.model != CHIP_ESP32S3) {
        // Error
    }
    
    Serial.printf("\nSoftware v%s Board Type S3x\n", VERSION_STR);

    pinMode(PIN_I2C_GND, OUTPUT);
    digitalWrite(PIN_I2C_GND, 0);
    pinMode(PIN_I2C_VCC, OUTPUT);
    digitalWrite(PIN_I2C_VCC, 1);
    
    pinMode(PIN_BUTTON, INPUT_PULLUP);
    setupLED(PIN_LEDZ);
    setupLED(PIN_LEDM);

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_BAUDRATE);
    Wire.setTimeout(0);
    Serial.printf("I2C started %d baud\n", Wire.getClock());
    
    blinkLed(COLOR_WHITE);

    my.frequencyCode = FrequencyCode[0];
    my.sampleFrequency = Frequencies[my.frequencyCode];
}

#ifdef STATIC_STACK
static StaticTask_t core0Task;
static byte core0Stack[16384];
#endif
void setupCore0(void (*core0Loop)(void*)) {
#ifdef STATIC_STACK
    my.core0Task = xTaskCreateStaticPinnedToCore(core0Loop, "Core0", 16384, NULL, 2, core0Stack, &core0Task, 0);
    if (my.core0Task == NULL) {
#else    
    if (xTaskCreatePinnedToCore(core0Loop, "Core0", 16384, NULL, 2, &my.core0Task, 0) != pdPASS) {
#endif        
        Serial.println("Error starting Network task");
        ESP.restart();
    }
}
