#include <Esp.h>
#include "Actimetre.h"

#define FIFO_SIZE (MAX_MEASURES * 2)
static byte signalFifo[FIFO_SIZE];
static int head = 0, tail = 0;
static struct timeval ticker;

static long int elapsed(struct timeval *from) {
    struct timeval now;
    gettimeofday(&now, NULL);
    return (now.tv_sec - from->tv_sec) * 1e6 + (now.tv_usec - from->tv_usec);
}

void readSignals(byte *buffer, int count) {
    if (tail == head) memset(buffer, 0, count);
    else {
        int target;
        int fill = count - (head - tail);
        
        if (tail > head) fill -= FIFO_SIZE;
        for (target = 0; target < fill; target++)
            buffer[target] = signalFifo[tail];
        for (; target < count ; tail++, target++) {
            if (tail >= FIFO_SIZE) tail -= FIFO_SIZE;
            buffer[target] = signalFifo[tail];
        }
    }
}

void signalTick() {
    if (elapsed(&ticker) >= 1000000L / my.sampleFrequency) {
        gettimeofday(&ticker, NULL);
        
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
}
