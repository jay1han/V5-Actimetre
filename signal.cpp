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

static portMUX_TYPE signalMux = portMUX_INITIALIZER_UNLOCKED;

void readSignals(byte *buffer, int take, int leave) {
    taskENTER_CRITICAL(&signalMux);
    if (tail == head) {
        taskEXIT_CRITICAL(&signalMux);
        Serial.println("No signal data");
        memset(buffer, 0, take);
    } else {
        int target = 0;
        int count = head - tail;
        if (count < 0) count += FIFO_SIZE;
        int fill = take + leave - count;
        byte tail_data = signalFifo[tail];

        if (fill > 0) {
            for (; target < fill; target++)
                buffer[target] = tail_data;
        } else if (fill < -1) {
            tail = tail - fill - 1;
        }
        
        for (; target < take ; tail++, target++) {
            if (tail >= FIFO_SIZE) tail -= FIFO_SIZE;
            buffer[target] = signalFifo[tail];
        }
        taskEXIT_CRITICAL(&signalMux);
        if (fill > 0)
            Serial.printf("Filled %d bytes\n", fill);
        if (fill < -1)
            Serial.printf("Left %d bytes\n", - (fill + 1));
    }
}

void clearSignals() {
    taskENTER_CRITICAL(&signalMux);
    tail = head;
    taskEXIT_CRITICAL(&signalMux);
}

void signalISR() {
    byte signal = 0;
    if (digitalRead(PIN_CAM_REC)) signal |= 1;
    if (digitalRead(PIN_CAM_1)) signal |= 2;
    if (digitalRead(PIN_CAM_2)) signal |= 4;

    signalFifo[head] = signal;
    if (++head >= FIFO_SIZE) head = 0;
    if (head == tail) {
        if (++tail >= FIFO_SIZE) tail = 0;
    }
}

void setupSignals() {
    attachInterrupt(PIN_DRAIN, signalISR, FALLING);

    Serial.println("Signal ISR attached");
}    
