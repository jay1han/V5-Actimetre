#include <Esp.h>
#include "Actimetre.h"

#define FIFO_SIZE (MAX_MEASURES * 4)
static byte signalFifo[FIFO_SIZE];
static int head = 0, tail = 0;
static struct timeval ticker = {0,0};

static long int elapsed(struct timeval *from) {
    struct timeval now;
    gettimeofday(&now, NULL);
    return (now.tv_sec - from->tv_sec) * 1e6 + (now.tv_usec - from->tv_usec);
}

static volatile SemaphoreHandle_t timerSem;
static portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void readSignals(byte *buffer, int take, int leave) {
    if (tail == head) {
        Serial.println("No signal data");
        memset(buffer, signalFifo[tail], take);
    } else {
        portENTER_CRITICAL(&timerMux);
        int target = 0;
        int count = head - tail;
        if (count < 0) count += FIFO_SIZE;
        int fill = take + leave - count;
        int tail0 = tail;

        if (fill > 0) {
            for (; target < fill; target++)
                buffer[target] = signalFifo[tail];
        } else if (fill < -1) {
            tail = tail - fill - 1;
        }
        
        for (; target < take ; tail++, target++) {
            if (tail >= FIFO_SIZE) tail -= FIFO_SIZE;
            buffer[target] = signalFifo[tail];
        }
        portEXIT_CRITICAL(&timerMux);

        if (fill > 0 || fill < -1)
            Serial.printf("Signal FIFO take %d leave %d head %d tail %d fill %d\n",
                          take, leave, head, tail0, fill);
    }
}

void signalISR() {
    byte signal = 0;
    if (digitalRead(PIN_CAM_REC)) signal |= 1;
    if (digitalRead(PIN_CAM_1)) signal |= 2;
    if (digitalRead(PIN_CAM_2)) signal |= 4;

    portENTER_CRITICAL_ISR(&timerMux);
    signalFifo[head] = signal;
    if (++head >= FIFO_SIZE) head = 0;
    if (head == tail) {
        if (++tail >= FIFO_SIZE) tail = 0;
    }
    portEXIT_CRITICAL_ISR(&timerMux);
}

void setupSignals() {
    timerSem = xSemaphoreCreateBinary();
    hw_timer_t *timer = timerBegin(1000000);
    timerAttachInterrupt(timer, &signalISR);
    timerAlarm(timer, 997, true, 0);
}    
