#include <cstring>
#include <HardwareSerial.h>
#include <Esp.h>
#include <Wire.h>
#include "Actimetre.h"

// SSD1306 DRIVER

#define LCD_PAGES (LCD_V_RES / 8)
#define LCD_BUFFER_SIZE (LCD_H_RES * LCD_PAGES)

#include "fontdata.h"

#define FONT_PITCH_16 (FONT_WIDTH_16 + 1)
#define CHAR_PER_LINE_16 (LCD_H_RES / FONT_PITCH_16)
static const char EMPTY_LINE[] = "              ";

static void write_cmd(unsigned char cmd) {
    if (my.displayPort < 0) return;
    TwoWire &wire = (my.displayPort == 0) ? Wire : Wire1;
    
    wire.beginTransmission(SSD1306_ADDR);
    wire.write(0x80);
    wire.write(cmd);
    wire.endTransmission(true);
}

static const unsigned char ssd1306_init_cmd[] = {
    0xAE | 0x00,          // SET_DISP            off
    0xA0 | 0x01,          // SET_REG_REMAP       horizontal reverse start
    0xA8, LCD_V_RES - 1,  // SET_MUX_RATIO
    0xC0 | 0x08,          // SET_COM_OUT_DIR     horizontal reverse scan
    0xDA, 0x12,           // SET_COM_PIN_CFG     must be 0x02 if aspect ratio > 2:1
    0x20, 0x02,           // SET_MEM_ADDRESS     page mode
    0xD9, 0xF1,           // SET_PRECHARGE
    0xDB, 0x30,           // SET_VCOM_DESEL
    0x81, 0x7F,           // SET_CONTRAST
    0xA4,                 // SET_ENTIRE_ON
    0xA6 | 0x00,          // SET_NORM_INV (0x01 for inverse)
    0x8D, 0x14,           // SET_CHARGE_PUMP
    0xAE | 0x01,          // SET_DISP            on
};

static const byte init_buffer[2] = {0, 0}; // dummy two bytes for right-margin
static void ssd1306_init() {
    if (my.displayPort < 0) return;
    
    int i;
    for (i = 0; i < sizeof(ssd1306_init_cmd); i++) {
        write_cmd(ssd1306_init_cmd[i]);
    }

    for (int page = 0; page < LCD_PAGES; page++) {
        TwoWire &wire = (my.displayPort == 0) ? Wire : Wire1;

        write_cmd(0xB0 | page);
        write_cmd(0x00 | 0x0F);
        write_cmd(0x10 | 0x07);
        wire.beginTransmission(SSD1306_ADDR);
        wire.write(0x40);
        wire.write(init_buffer, 2);
        wire.endTransmission(true);
    }
}

static void ssd1306_off() {
    write_cmd(0xAE);
}

static void ssd1306_on() {
    write_cmd(0xAF);
}

static unsigned char displayBuffer[LCD_BUFFER_SIZE];
static char textBuffer[2][CHAR_PER_LINE_16 + 1];

static void write_page(int page) {
    if (my.displayPort < 0) return;
    TwoWire &wire = (my.displayPort == 0) ? Wire : Wire1;

    wire.beginTransmission(SSD1306_ADDR);
    wire.write(0x40);
    wire.write(displayBuffer + page * LCD_H_RES, LCD_H_RES);
    wire.endTransmission(true);
}

static void ssd1306_showpages(int page0, int page1) {
    int page;

    for (page = page0; page <= page1; page++) {
        write_cmd(0xB0 | page);
        write_cmd(0x00);
        write_cmd(0x10);
        write_page(page);
    }
}

static void ssd1306_showall() {
    ssd1306_showpages(0, LCD_PAGES - 1);
}

static void write_char16(int x, int y, unsigned char c) {
    int col, target, index = c - 32;

    target = y * LCD_H_RES + x * FONT_PITCH_16;
    for (col = 0; col < FONT_WIDTH_16; col++) {
        displayBuffer[target + col] = fontdata_top[index][col];
        displayBuffer[target + col + LCD_H_RES] = fontdata_bot[index][col];
    }
    displayBuffer[target + col] = 0x00;
    displayBuffer[target + col + LCD_H_RES] = 0x00;
}

static void writeLine16(int line, const char *message) {
    if (my.displayPort < 0) return;

    int i;
    for (i = 0; i < strlen(message) && i < CHAR_PER_LINE_16; i++) {
        write_char16(i, line, message[i]);
    }
    int j;
    for (j = i * FONT_PITCH_16; j < LCD_H_RES; j++) {
        displayBuffer[line * LCD_H_RES + j] = 0x00;
        displayBuffer[(line + 1) * LCD_H_RES + j] = 0x00;
    }
    ssd1306_showpages(line, line + 1);
}

void displayTitle(char *title) {
    if (my.displayPort < 0) return;
    TwoWire &wire = (my.displayPort == 0) ? Wire : Wire1;
    wire.setClock(DISPLAY_BAUDRATE);
    
    writeLine16(0, title);
}

void displaySensors() {
    if (my.displayPort < 0) return;
    TwoWire &wire = (my.displayPort == 0) ? Wire : Wire1;
    wire.setClock(DISPLAY_BAUDRATE);
    
    char sensorLine[20];
    if (my.sampleFrequency >= 1000)
        sprintf(sensorLine, "%-6s %s@%dk", my.sensorList, my.boardName, my.sampleFrequency / 1000);
    else
        sprintf(sensorLine, "%-6s %s@%d", my.sensorList, my.boardName, my.sampleFrequency);
    writeLine16(2, sensorLine);
}

void writeLine(char *message) {
    if (my.displayPort < 0) return;
    TwoWire &wire = (my.displayPort == 0) ? Wire : Wire1;
    wire.setClock(DISPLAY_BAUDRATE);
    
    int scroll;
    for (scroll = 4; scroll < 6; scroll++)
        memcpy(displayBuffer + scroll * LCD_H_RES,
               displayBuffer + (scroll + 2) * LCD_H_RES,
               LCD_H_RES);
    ssd1306_showpages(4, 5);
    writeLine16(6, message);
}

void initDisplay() {
    if (my.displayPort < 0) return;
    TwoWire &wire = (my.displayPort == 0) ? Wire : Wire1;

    wire.setClock(DISPLAY_BAUDRATE);
    ssd1306_init();
    memset(displayBuffer, 0, sizeof(displayBuffer));
    ssd1306_showall();
}

static void write_block(int x, int y) {
    if (my.displayPort < 0) return;
    TwoWire &wire = (my.displayPort == 0) ? Wire : Wire1;
    wire.setClock(DISPLAY_BAUDRATE);

    int pixel_x = x * FONT_PITCH_16;
    write_cmd(0xB0 | y);
    write_cmd(0x00 | (pixel_x & 0x0F));
    write_cmd(0x10 | (pixel_x >> 4));

    wire.beginTransmission(SSD1306_ADDR);
    wire.write(0x40);
    wire.write(displayBuffer + (y * LCD_H_RES) + pixel_x, FONT_WIDTH_16);
    wire.endTransmission(true);
}

#define RSSI_POS   10
#define RSSI_X     (FONT_PITCH_16 * RSSI_POS)
#define RSSI_STEPS 3
#define TEXT_STEPS 5
#define BLINK_POS  4

#define TOTAL_SCAN_LINE  (RSSI_STEPS + TEXT_STEPS + CHAR_PER_LINE_16 * 2 * 3 + 1)
#ifdef PROFILE_DISPLAY
static float avgDisplay;
static int maxDisplay[TOTAL_SCAN_LINE];
static int maxDisplayMax, maxDisplayLine;
#endif

static void textPanel(int step) {
    switch(step) {
    case 0:
        if (my.upTime >= 6000)
            snprintf(textBuffer[0], CHAR_PER_LINE_16 + 1, "%dh %.1f %.1f", my.upTime / 60,
                    my.avgCycleTime[1] / 1000.0, my.avgCycleTime[0] / 1000.0);
        else
            snprintf(textBuffer[0], CHAR_PER_LINE_16 + 1, "%dh%02d %.1f %.1f", my.upTime / 60, my.upTime % 60,
                    my.avgCycleTime[1] / 1000.0, my.avgCycleTime[0] / 1000.0);
        if (strlen(textBuffer[0]) < CHAR_PER_LINE_16)
            strncat(textBuffer[0], EMPTY_LINE, CHAR_PER_LINE_16 - strlen(textBuffer[0]));
        break;
        
    case 1:
        if (my.isStopped) {
            strcpy(textBuffer[1], "Stopped");
        } else {
#if INFO_DISPLAY == 1
            float rating = 0.0;
            for (int port = 0; port < 2; port++) {
                for (int address = 0; address < 2; address++) {
                    if (my.sensor[port][address].type) {
                        if (my.sensor[port][address].nCycles > 0 &&
                            my.sensor[port][address].nSamples <= my.sensor[port][address].nCycles) 
                            rating += 1.0 - (float)my.sensor[port][address].nSamples / my.sensor[port][address].nCycles;
                    }
                }
            }
            rating /= my.nSensors;
            snprintf(textBuffer[1], CHAR_PER_LINE_16 + 1, "%.3f%% M%d Q%.0f%%", rating, my.nMissed[1], my.queueFill);
#endif    
#if INFO_DISPLAY == 2
            snprintf(textBuffer[1], CHAR_PER_LINE_16 + 1, "%d %d",
                     uxTaskGetStackHighWaterMark(my.core1Task),
                     uxTaskGetStackHighWaterMark(my.core0Task));
#endif    
#if INFO_DISPLAY == 3
            snprintf(textBuffer[1], CHAR_PER_LINE_16 + 1, "%.0f %d:%d",
                     avgDisplay, maxDisplayLine, maxDisplayMax);
#endif    
#if INFO_DISPLAY == 0
            snprintf(textBuffer[1], CHAR_PER_LINE_16 + 1, "%dms %dkB",
                     my.cycleMicroseconds / 1000, my.I2Cbudget / 1000);
#endif
        }
        if (strlen(textBuffer[1]) < CHAR_PER_LINE_16)
            strncat(textBuffer[1], EMPTY_LINE, CHAR_PER_LINE_16 - strlen(textBuffer[1]));
        break;

    case 2:
        static unsigned char blink = ' ';
        static time_t stopwatch = time(NULL);
        if (stopwatch != time(NULL)) {
            if (blink == ' ') blink = '>';
            else blink = ' ';
            write_char16(BLINK_POS, 0, blink);
            stopwatch = time(NULL);
        }
        break;

    case 3:
        write_block(BLINK_POS, 0);
        break;

    case 4:
        write_block(BLINK_POS, 1);
        break;
    }
}

static void displayRssi(int step) {
    
    switch(step) {
    case 0:
        write_char16(RSSI_POS, 0, 0x81 + my.rssi);
        break;

    case 1:
        write_block(RSSI_POS, 0);
        break;

    case 2:
        write_block(RSSI_POS, 1);
        break;
    }
}

static void displayScanLine(int scanLine) {
    int line = scanLine / (CHAR_PER_LINE_16 * 3);
    int y = (line + 2) * 2;
    int x = (scanLine % (CHAR_PER_LINE_16 * 3)) / 3;
    int step = (scanLine % (CHAR_PER_LINE_16 * 3)) % 3;

    switch (step) {
    case 0:
        write_char16(x, y, textBuffer[line][x]);
        break;

    case 1:
        write_block(x, y);
        break;

    case 2:
        write_block(x, y + 1);
        break;
    }
}

static void displayScan(int scanLine) {
    if (scanLine < TOTAL_SCAN_LINE - 1) {
        if (scanLine < RSSI_STEPS) displayRssi(scanLine);
        else if (scanLine < RSSI_STEPS + TEXT_STEPS) textPanel(scanLine - RSSI_STEPS);
        else displayScanLine(scanLine - RSSI_STEPS - TEXT_STEPS);
        return;
    }
    
#ifdef LOG_HEARTBEAT        
    static time_t heartbeat = time(NULL);
    if (scanLine == TOTAL_SCAN_LINE - 1 && heartbeat != time(NULL)) {
        heartbeat = time(NULL);
        Serial.printf("%dh%02d %.1f %.1f (%.1f) ",
                      my.upTime / 60, my.upTime % 60,
                      my.avgCycleTime[1] / 1000.0, my.avgCycleTime[0] / 1000.0,
                      (float)my.cycleMicroseconds / 1000.0);
        Serial.printf("M%d,%d Q%.0f%%\n", my.nMissed[1], my.nMissed[0], my.queueFill);
        return;
    }
#endif

#ifdef LOG_STACK
    static time_t breath = time(NULL);
    if ((scanLine == TOTAL_SCAN_LINE - 1) && (time(NULL) - breath > 10)) {
        breath = time(NULL);
        Serial.printf("Stack loop:%d net:%d\n",
                      uxTaskGetStackHighWaterMark(NULL),
                      uxTaskGetStackHighWaterMark(my.core0Task));
        return;
    }
#endif    
}    

void displayLoop(int force) {
    if (my.displayPort < 0) {
#ifdef LOG_HEARTBEAT        
        static time_t logTimer = 0;
        if (time(NULL) != logTimer) {
            logTimer = time(NULL);
            Serial.printf("%dh%02d %.1f %.1f (%.1f) ",
                          my.upTime / 60, my.upTime % 60,
                          my.avgCycleTime[1] / 1000.0, my.avgCycleTime[0] / 1000.0,
                          (float)my.cycleMicroseconds / 1000.0);
            Serial.printf("M%d,%d Q%.0f%%\n", my.nMissed[1], my.nMissed[0], my.queueFill);
        }
#endif        
        return;
    }
    TwoWire &wire = (my.displayPort == 0) ? Wire : Wire1;

    wire.setClock(DISPLAY_BAUDRATE);
    static int scanLine = 0;
#ifdef PROFILE_DISPLAY    
    int64_t stopwatch = getAbsMicros();
#endif
    
    if (my.displayPort >= 0) {
        if (force == 1) {
            ssd1306_on();
            for (scanLine = 0; scanLine < TOTAL_SCAN_LINE; scanLine++)
                displayScan(scanLine);
            scanLine = 0;
        } else {
            scanLine = (scanLine + 1) % TOTAL_SCAN_LINE;
            displayScan(scanLine);
        }
    }

#ifdef PROFILE_DISPLAY    
    static int profiling = 0;
    static int entries   = 0;
    static int reporting = time(NULL);
    int microseconds = (int)(getAbsMicros() - stopwatch);
    if (microseconds > maxDisplay[scanLine]) maxDisplay[scanLine] = microseconds;
    if (microseconds > maxDisplayMax) {
        maxDisplayMax = microseconds;
        maxDisplayLine = scanLine;
    }
    profiling += microseconds;
    entries ++;
    if (time(NULL) != reporting) {
        avgDisplay = (float)profiling / entries;
#ifdef LOG_DISPLAY        
        Serial.printf("displayLoop() %dus/%d = %.1fms avg, max %d:%d\n",
                      profiling, entries, avgDisplay, maxDisplayLine, maxDisplayMax);
#endif        
        profiling = 0;
        entries = 0;
        reporting = time(NULL);
    }
#endif
}
