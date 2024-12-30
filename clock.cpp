#include <time.h>
#include <Esp.h>
#include <time.h>
#include <esp_sleep.h>
#include "Actimetre.h"

#define TEN_KILO    10000L
#define ONE_MEGA    1000000L

#define ROLLOVER_MILLIS      296L
#define ROLLOVER_TEN_MILLIS  7296L
#define ROLLOVER_MICROS      967296L
#define ROLLOVER_TEN_MICROS  4967296L

static bool init_complete = false;
static int64_t nextMicros;

int64_t getAbsMicros() {
    struct timeval timeofday;
    gettimeofday(&timeofday, NULL);
    return (int64_t)timeofday.tv_sec * 1000000L + (int64_t)timeofday.tv_usec;
}

static int timeRemaining() {
    int64_t remain = nextMicros - getAbsMicros();
    return (int)remain;
}

void waitNextCycle() {
    blinkLed(COLOR_BLINK);
    my.upTime = (time(NULL) - my.bootTime) / 60;
    if (!my.dualCore) {
        netWork();
        while (timeRemaining() > 2500) netWork();
    }
    while (timeRemaining() > 500) displayLoop(0);
    while (timeRemaining() > 5);
    nextMicros = getAbsMicros() + (int64_t)my.cycleMicroseconds;
}

void clearNextCycle() {
    nextMicros = getAbsMicros();
}

void initClock(time_t bootEpoch) {
    struct timeval timeofday = {bootEpoch, 0};
    settimeofday(&timeofday, 0);
    nextMicros = (int64_t)bootEpoch * 1000000L;
    
    my.bootTime = bootEpoch;
    init_complete = true;

    struct tm timeinfo;
    gmtime_r(&bootEpoch, &timeinfo);
    Serial.printf("%04d/%02d/%02d %02d:%02d:%02d UTC\n",
                  timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                  timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    
    for (int core = 0; core < 2; core++) {
        my.nMissed[core] = 0;
        my.avgCycleTime[core] = 0.0;
    }
    my.upTime = 0;
}

void getTimeSinceBoot(time_t *r_sec, int *r_usec) {
    int64_t clock = getAbsMicros() - (int64_t)my.bootTime * 1000000L;
    int64_t sec = clock / 1000000L;
    int64_t usec = clock % 1000000L;
    if (r_sec != NULL) *r_sec = (time_t)sec;
    if (r_usec != NULL) *r_usec = (int)usec;
}

unsigned long millis_diff_10(unsigned long end, unsigned long start) {
    if (end >= start)
        return (((end % TEN_KILO) + TEN_KILO) - (start % TEN_KILO)) % TEN_KILO;
    else
        return (((end % TEN_KILO) + ROLLOVER_TEN_MILLIS + TEN_KILO) - (start % TEN_KILO)) % TEN_KILO;
}

unsigned long micros_diff(unsigned long end, unsigned long start) {
    if (end >= start)
        return (((end % ONE_MEGA) + ONE_MEGA) - (start % ONE_MEGA)) % ONE_MEGA;
    else
        return (((end % ONE_MEGA) + ROLLOVER_MICROS + ONE_MEGA) - (start % ONE_MEGA)) % ONE_MEGA;
}

static unsigned long nCycles[CoreNumMax] = {0, 0};
static time_t clear = time(NULL);

void logCycleTime(CoreNum coreNum, unsigned long time_spent) {
    time_t life;
    getTimeSinceBoot(&life, NULL);
    if (life > 0xFEFFFF) {
        ERROR_FATAL("Alive over 6 months, rebooting");
    }

    my.avgCycleTime[coreNum] = (my.avgCycleTime[coreNum] * nCycles[coreNum] + time_spent) / (nCycles[coreNum] + 1);
    nCycles[coreNum] ++;

    if (coreNum == Core1I2C &&
        (my.nMissed[1] >= MAX_MISSED1 || my.nMissed[0] >= (my.dualCore ? MAX_MISSED0 : MAX_MISSED0_SINGLE))) {
        char error[64];
        sprintf(error, "M%d,%d Q%.1f Avg %.1f,%.1f",
                my.nMissed[1], my.nMissed[0], my.queueFill,
                my.avgCycleTime[1] / 1000.0, my.avgCycleTime[0] / 1000.0);
        Serial.println(error);
        ERROR_FATAL(error);
    }
    
    if (time(NULL) - clear > MEASURE_SECS) clearCycleTime();
}

void clearCycleTime() {
    nCycles[0] = nCycles[1] = 0;
    my.nMissed[0] = my.nMissed[1] = 0;
    clear = time(NULL);
}
