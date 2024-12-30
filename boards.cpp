#include <Wire.h>
#include <HardwareSerial.h>
#include <esp_cpu.h>
#include <Esp.h>
#include <esp32-hal.h>
#include <esp_chip_info.h>
#include "Actimetre.h"

typedef enum {
    _PIN_BUTTON = 0,
    _PIN_LED,

    _PIN_I2C0_SDA,
    _PIN_I2C0_SCL,
    _PIN_I2C0_GND,
    _PIN_I2C0_VCC,

    _PIN_I2C1_SDA,
    _PIN_I2C1_SCL,
    _PIN_I2C1_GND,
    _PIN_I2C1_VCC,

    _PIN_MORE_GND,
    _PIN_MORE_VCC,

    PIN_MAX
} PinName;

// BOARD DEFINITIONS (See readme.txt)

static char BoardName[BOARD_TYPES][4] = {"BAD", "S3i", "S3n", "S3z", "S3m", "S2o", "C3o", "S3o"};

#define DISABLE_I2C   0xFF
#define UNPOWERED_PIN 0xFF
const uint8_t PINS[BOARD_TYPES][PIN_MAX] = {
    // SDA SCL GND VCC
    // Dummy for bad
    {0xFF, 0xFF,
     0xFF, 0xFF, 0xFF, 0xFF,
     0xFF, 0xFF, 0xFF, 0xFF,
     0xFF, 0xFF},
    // Board Type 1 (S3 mini with I2C) S3i
    {0, 47,
     21, 17, 0xFF, 15,
     7, 8, 9, 14,
     10, 0xFF},  // Pin 10 is also GND for the display
    // Board Type 2 (S3 mini with new box) S3n
    {0, 47,
     13, 11, 10, 0xFF, 
     44, 36, 35, 18,
     0xFF, 0xFF}, 
    // Board Type 3 (S3 zero) S3z
    {0, 21,
     10, 9, 8, 7,
     3, 4, 5, 6,
     11, 12},
    // Board Type 4 (S3 mini alternate for new box) S3m
    {0, 47,
     2, 4, 12, 13,
     44, 36, 35, 18,
     0xFF, 16},   // Pin 16 is also 3V
    // Board Type 5 (S2 mini Solo) S2o
    {0, 15,
     40, 38, 36, 34,
     6, 4, 2, 1,
     0xFF, 0xFF},
    // Board Type 6 (C3 Solo) C3o
    {9, 8,
     9, 10, 20, 21,
     0xFF, 0xFF, 0xFF, 0xFF,
     0xFF, 0xFF},
    // Board Type 7 (S3 mini Solo) S3o
    {0, 47,
     33, 37, 38, 34,
     1, 3, 5, 6,
     0xFF, 0xFF},
};
static uint8_t PIN_BUTTON, PIN_LED,
    PIN_I2C0_SDA, PIN_I2C0_SCL, PIN_I2C0_GND, PIN_I2C0_VCC,
    PIN_I2C1_SDA, PIN_I2C1_SCL, PIN_I2C1_GND, PIN_I2C1_VCC,
    PIN_MORE_GND, PIN_MORE_VCC;

#define FREQ_COUNT   3
static int freqCode =  0;
static int Frequencies[8] = {100, 500, 1000, 2000, 4000, 8000};
static int FrequencyCode[FREQ_COUNT] = {2, 4, 5};

static void switchFrequency() {
    do {
        freqCode = (freqCode + 1) % FREQ_COUNT;
        my.frequencyCode = FrequencyCode[freqCode];
        my.sampleFrequency = Frequencies[my.frequencyCode];
    } while (setSamplingMode() > (my.dualCore ? MPU_BAUDRATE : (MPU_BAUDRATE / 2)));
    
    setSensorsFrequency();
    displaySensors();
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
static void setupLED() {
    Serial.printf("RGB pin %d init ", PIN_LED);
    if (!rmtInit(PIN_LED, RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_1, 20000000))
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
#define LED_MONO  0
#define LED_RGB   1
#define LED_GRB   2

void blinkLed(int command) {
    static int saved = COLOR_WHITE;
    static int frequency = 1;
    static bool state = false;
    static unsigned long blinkTime = millis();
    int color;

    if (command == COLOR_BLINK) {
        unsigned long blinkPeriod = (my.ledRGB != LED_MONO) ? 1000 : BLINK_MILLIS[frequency];
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
    
    if (my.ledRGB == LED_RGB) {
        rmt_data_t *data = RmtBuffer;
        data = stuffBits(data, color & 0xFF);
        data = stuffBits(data, (color >> 8) & 0xFF);
        data = stuffBits(data, (color >> 16) & 0xFF);
        rmtWrite(PIN_LED, RmtBuffer, RMT_SIZE, RMT_WAIT_FOR_EVER);
    } else if (my.ledRGB == LED_GRB) {
        rmt_data_t *data = RmtBuffer;
        data = stuffBits(data, (color >> 8) & 0xFF);
        data = stuffBits(data, color & 0xFF);
        data = stuffBits(data, (color >> 16) & 0xFF);
        rmtWrite(PIN_LED, RmtBuffer, RMT_SIZE, RMT_WAIT_FOR_EVER);
    } else {
        if (state) digitalWrite(PIN_LED, 1);
        else digitalWrite(PIN_LED, 0);
    }
}

void setupBoard() {
    Serial.setTxTimeoutMs(0);
    Serial.begin(115200);
    delay(2000);

    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    switch (chip_info.model) {
    case CHIP_ESP32S3:
        my.dualCore = true;
        my.ledRGB = LED_RGB;
        pinMode(14, INPUT_PULLDOWN);
        pinMode(15, INPUT_PULLDOWN);
        pinMode(1, INPUT_PULLDOWN);
        pinMode(17, INPUT_PULLUP);
        if (digitalRead(1) == 1) {
            my.boardType = BOARD_S3_ZERO;
            my.ledRGB = LED_GRB;
        }
        else if (digitalRead(17) == 0) my.boardType = BOARD_S3_SOLO;
        else if (digitalRead(14) == 1 and digitalRead(15) == 1) my.boardType = BOARD_S3_I2C;
        else if (digitalRead(14) == 1 and digitalRead(15) == 0) my.boardType = BOARD_S3_NEWBOX2;
        else if (digitalRead(14) == 0 and digitalRead(15) == 0) my.boardType = BOARD_S3_NEWBOX;
        else {
            my.boardType = BOARD_BAD;
        }
        break;

    case CHIP_ESP32S2:
        my.dualCore = false;
        my.boardType = BOARD_S2_SOLO;
        my.ledRGB = LED_MONO;
        break;

    case CHIP_ESP32C3:
        my.dualCore = false;
        my.boardType = BOARD_C3_SOLO;
        my.ledRGB = LED_MONO;
        break;

    default:
        my.boardType = BOARD_BAD;
    }
    strcpy(my.boardName, BoardName[my.boardType]);
    Serial.printf("\nSoftware v%s Board Type %s(%d)\n",
                  VERSION_STR, my.boardName, my.boardType);

    pinMode(PIN_BUTTON    = PINS[my.boardType][_PIN_BUTTON],   INPUT_PULLUP);
    pinMode(PIN_LED       = PINS[my.boardType][_PIN_LED],      OUTPUT);
    if (my.ledRGB > LED_MONO) setupLED();

    if (PINS[my.boardType][_PIN_I2C0_SDA] == DISABLE_I2C) {
        my.hasI2C[0] = false;
    } else {
        my.hasI2C[0] = true;
        PIN_I2C0_SDA                = PINS[my.boardType][_PIN_I2C0_SDA];
        PIN_I2C0_SCL                = PINS[my.boardType][_PIN_I2C0_SCL];
        PIN_I2C0_GND                = PINS[my.boardType][_PIN_I2C0_GND];
        PIN_I2C0_VCC                = PINS[my.boardType][_PIN_I2C0_VCC];
        if (PIN_I2C0_GND != 0xFF) {
            pinMode(PIN_I2C0_GND, OUTPUT);
            digitalWrite(PIN_I2C0_GND, 0);
        }
        if (PIN_I2C0_VCC != 0xFF) {
            pinMode(PIN_I2C0_VCC, OUTPUT);
            digitalWrite(PIN_I2C0_VCC, 1);
        }
        Wire.begin(PIN_I2C0_SDA, PIN_I2C0_SCL, LOW_BAUDRATE);
        Wire.setTimeout(0);
        Serial.printf("I2C0 started %d baud\n", Wire.getClock());
    }
    
    if (PINS[my.boardType][_PIN_I2C1_SDA] == DISABLE_I2C) {
        my.hasI2C[1] = false;
    } else {
        my.hasI2C[1] = true;
        PIN_I2C1_SDA                = PINS[my.boardType][_PIN_I2C1_SDA];
        PIN_I2C1_SCL                = PINS[my.boardType][_PIN_I2C1_SCL];
        PIN_I2C1_GND                = PINS[my.boardType][_PIN_I2C1_GND];
        PIN_I2C1_VCC                = PINS[my.boardType][_PIN_I2C1_VCC];
        if (PIN_I2C1_GND != 0xFF) {
            pinMode(PIN_I2C1_GND, OUTPUT);
            digitalWrite(PIN_I2C1_GND, 0);
        }
        if (PIN_I2C1_VCC != 0xFF) {
            pinMode(PIN_I2C1_VCC, OUTPUT);
            digitalWrite(PIN_I2C1_VCC, 1);
        }
        Wire1.begin(PIN_I2C1_SDA, PIN_I2C1_SCL, LOW_BAUDRATE);
        Wire1.setTimeout(0);
        Serial.printf("I2C1 started %d baud\n", Wire1.getClock());
    }

    if ((PIN_MORE_GND = PINS[my.boardType][_PIN_MORE_GND]) != 0xFF) {
        pinMode(PIN_MORE_GND, OUTPUT);
        digitalWrite(PIN_MORE_GND, 0);
    }
    if ((PIN_MORE_VCC = PINS[my.boardType][_PIN_MORE_VCC]) != 0xFF) {
        pinMode(PIN_MORE_VCC, OUTPUT);
        digitalWrite(PIN_MORE_VCC, 1);
    }
    
    blinkLed(COLOR_WHITE);

    my.frequencyCode = FrequencyCode[0];
    my.sampleFrequency = Frequencies[my.frequencyCode];
    if (my.boardType == BOARD_BAD) {
        my.cycleMicroseconds = 100000;
    } else {
        setSamplingMode();
    }
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
