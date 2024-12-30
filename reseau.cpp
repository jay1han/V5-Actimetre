#include <WiFi.h>
#include <Esp.h>
#include <esp_task_wdt.h>
#include <time.h>
#include <esp_wifi.h>
#include <esp_mac.h>
#include "Actimetre.h"

#define ACTI_PORT 2883
#define SIDE_PORT 2882

static WiFiClient wifiClient;
static QueueHandle_t msgQueue;

#ifdef STATIC_QUEUE
static StaticQueue_t msgQueueStatic;
static byte msgQueueItems[QUEUE_SIZE * sizeof(int)];
#endif

#define INIT_LENGTH      13   // boardName = 3, MAC = 6, Sensors = 1, version = 3 : Total 13
#define RESPONSE_LENGTH  6    // actimId = 2, time = 4 : Total 6

// Messaging functions

static bool sendMessage(byte *message) {
    int epochSec = message[0] << 16 | message[1] << 8 | message[2];
    int count = message[3] & 0x3F;
    int msgLength;
    if (message[0] == 0xFF) {
        msgLength = HEADER_LENGTH + (count + 1) * 4;
        Serial.printf("REPORT message length %d\n", msgLength);
    } else if (message[5] & 0x10) {
        msgLength = HEADER_LENGTH + (count + 1) * 4;
    } else if (message[5] & 0x40) {
        msgLength = HEADER_LENGTH;
    } else {
        int dataLength = DATA_LENGTH[(message[4] >> 3) & 0x03];
        msgLength = HEADER_LENGTH + dataLength * count;
    }
    
    int sent = 0;
    unsigned long timeout = micros();
    while (sent < msgLength && micros_diff(micros(), timeout) < (my.dualCore ? 1000000L : my.cycleMicroseconds / 3)) {
        sent += wifiClient.write(message + sent, msgLength - sent);
    }

    if (sent != msgLength) {
        my.nMissed[Core0Net] ++;
        if (message[0] == 0xFF) {
            Serial.printf("Timeout sending message\n");
            return false;
        } else {
            Serial.printf("Timeout sending data\n");
            if (my.dualCore) {
                writeLine("Timeout");
                RESTART(2);
            } else {
                return false;
            }
        }
    }

#ifdef PROFILE_NETWORK
    int port = (message[3] >> 7) & 1;
    int address = (message[3] >> 6) & 1;
    static int inSec[2][2];
    static int thisSec[2][2];
    static int nMessages[2][2];
    nMessages[port][address] ++;
    inSec[port][address] += count;
    if (epochSec != thisSec[port][address]) {
        Serial.printf("%d%c: %d samples in %d packets (mode %d, %d bytes), avg. %.1f/s\n",
                      port + 1, address + 'A', inSec[port][address], nMessages[port][address],
                      my.sensor[port][address].samplingMode, my.sensor[port][address].dataLength,
                      (float)inSec[port][address] / nMessages[port][address]);
        inSec[port][address] = 0;
        nMessages[port][address] = 0;
        thisSec[port][address] = epochSec;
    }
#endif
    return true;
}

static byte heartbeatMessage[HEADER_LENGTH];

static void sendHeartbeat() {
    static time_t heartbeat = time(NULL);
    if (time(NULL) != heartbeat) {
        formatHeader(0, 0, heartbeatMessage, 0, 0);
        heartbeatMessage[5] |= 0x40;
        sendMessage(heartbeatMessage);
        heartbeat = time(NULL);
        Serial.println("Heartbeat");
    }
}

void queueIndex(int index) {
    if (index <= 0 || index >= QUEUE_SIZE) {
        char error[16];
        sprintf(error, "Q %X", index);
        Serial.println(error);
        ERROR_FATAL(error);
    }

    if (xQueueSend(msgQueue, &index, 0) != pdTRUE) {
#ifdef TIGHT_QUEUE        
        ERROR_FATAL("Queue full");
#else        
        my.nMissed[Core0Net] ++;
        xQueueReset(msgQueue);
        Serial.println("Queue full, cleared");
        ERROR_REPORT("Queue full, cleared");
#endif
    }
}

static void readRssi() {
    int rssi = WiFi.RSSI();
    if (rssi != 0) {
        if (rssi > -28) my.rssi = 7;
        else if (rssi <= -91) my.rssi = 0;
        else my.rssi = (rssi + 91) / 9;
    } else my.rssi = 0;
}

// Check connection still working and process message queue

int isConnected() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\nNetwork disconnected. Rebooting");
        writeLine("WiFi lost");
        RESTART(2);
    }

    return 1;
}

static void netWorkOn(int index) {
    unsigned long startWork = micros();

    if (my.isStopped) sendHeartbeat();
    else {
        if (index <= 0 || index >= QUEUE_SIZE) {
            char error[16];
            sprintf(error, "0 %X", index);
            Serial.println(error);
#ifdef STATIC_QUEUE            
            dump(msgQueueItems, QUEUE_SIZE * sizeof(int));
#endif            
            ERROR_FATAL(error);
        }
        if (!sendMessage(msgQueueStore[index]) && !my.dualCore) {
            Serial.println("Requeued message");
            xQueueSendToFront(msgQueue, &index, 0);
        }

#ifndef TIGHT_QUEUE        
        int availableSpaces = uxQueueSpacesAvailable(msgQueue);
        if (availableSpaces < QUEUE_SIZE / 5) {
            my.nMissed[Core0Net] ++;
            xQueueReset(msgQueue);
            Serial.println("Queue more than 80%, cleared");
            my.queueFill = 0.0;
            ERROR_REPORT("Queue filling, cleared");
        } else {
            my.queueFill = 100.0 * (QUEUE_SIZE - availableSpaces) / QUEUE_SIZE;
        }
#endif        

#ifdef LOG_QUEUE
        static time_t timer = 0;
        if (time(NULL) != timer) {
            timer = time(NULL);
            Serial.printf("===== Dump at %d\n", timer);
            dump(msgQueueItems, 32 * sizeof(int));
        }
#endif        
    }
        
    int command = wifiClient.read();
    if (command >= 0) {
        Serial.printf("Remote command 0x%02X\n", command);
        switch(command & REMOTE_COMMAND) {
        case REMOTE_BUTTON:
            Serial.println("Simulated button press");
            manageButton(1);
            break;

        case REMOTE_STOP:
            my.isStopped = true;
            blinkLed(COLOR_WHITE);
            break;
                
        case REMOTE_RESTART:
            Serial.println("Remote restart");
            RESTART(5);
            break;
        }
    }
        
    readRssi();
    logCycleTime(Core0Net, micros_diff(micros(), startWork));
}

static void Core0Loop(void *dummy_to_match_argument_signature) {
    Serial.printf("Core %d started\n", xPortGetCoreID());

    for (;;) {
//        TEST_LOCAL(1);
        int index;
        while (xQueueReceive(msgQueue, &index, 1) != pdTRUE);
        netWorkOn(index);
    }
}

void netWork() {
    int index;
    if (xQueueReceive(msgQueue, &index, 0) == pdTRUE) {
        netWorkOn(index);
    }
}

// Network initializations

static void storeMacAddress() {
    unsigned char mac[8];
    esp_efuse_mac_get_default(mac);
    memcpy(my.mac, mac, 6);
    sprintf(my.macString, "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    Serial.print("MAC address ");
    Serial.println(my.macString);
    writeLine(my.macString);
}

static int scanNetworks() {
    int nScan;
    Serial.print("Scan... ");
    writeLine("Scanning");
    nScan = WiFi.scanNetworks();
    Serial.printf("Done. Found %d APs\n", nScan);

    if (nScan <= 0) {
        Serial.println("\nCan't find AP. Rebooting");
	writeLine("No AP");
        RESTART(2);
    }
    return nScan;
}

typedef struct {
    char ssid[12];
    int serverId;
    int rssi;
    int channel;
} ActisItem;
static ActisItem actisList[10];

static int nActis = 0;
static void findActis(int nScan) {
    int i;
    char ssid[10];
    for (i = 0; i < nScan; i++) {
        strncpy(ssid, WiFi.SSID(i).c_str(), 9);
        ssid[9] = 0;  // always careful
        if (strncmp(ACTISERVER, ssid, 5) == 0) {
            strcpy(actisList[nActis].ssid, ssid);
            sscanf(ssid + 5, "%d", &actisList[nActis].serverId);
            actisList[nActis].rssi = -WiFi.RSSI(i);
            actisList[nActis].channel = WiFi.channel(i);
            nActis ++;
            if (nActis == 10) break;
        }
    }
    if (nActis == 0) {
        Serial.println("\nCan't find server, rebooting");
	writeLine("No Actis");
        RESTART(2);
    }
}

static bool tryConnect(int index) {
    WiFi.disconnect(true, true);
    delay(1000);
    WiFi.mode(WIFI_STA);
    
    int wait = 0;

    char *ssid = actisList[index].ssid;
    my.serverId = actisList[index].serverId;
    
    Serial.print(ssid);
    writeLine(ssid);
    strcpy(my.ssid, ssid);

    char pass[] = "000animalerie";
    memcpy(pass, my.ssid + 5, 3);
    WiFi.begin(my.ssid, pass);

    while (WiFi.status() != WL_CONNECTED) {
        Serial.printf(" %d", WiFi.status());
        blinkLed(COLOR_SWAP);
        wait++;
        if (wait > 10) {
            Serial.println(" Failed!");
            writeLine("Failed");
            return false;
        }
        delay(1000);
    }
    
    Serial.print(" Connected! IP=");
    Serial.print(WiFi.localIP());
    strcpy(my.serverIP, WiFi.gatewayIP().toString().c_str());
    Serial.print(" Server=");
    Serial.println(my.serverIP);

    String ipString = WiFi.localIP().toString();
    char myIPstring[20];
    int trail = 0;
    trail = ipString.indexOf('.');
    trail = ipString.indexOf('.', trail + 1);
    
    strcpy(myIPstring, ipString.substring(trail + 1).c_str());
    writeLine(myIPstring);
    readRssi();
    return true;
}

#define QUERY_LENGTH 1 + 10 * (2 + 1)
static byte assignQuery[QUERY_LENGTH];

static void buildQuery() {
    int i;
    assignQuery[0] = nActis;
    byte *query = &assignQuery[1];
    for (i = 0; i < nActis; i++) {
        query[0] = actisList[i].serverId >> 8;
        query[1] = actisList[i].serverId & 0xFF;
        query[2] = actisList[i].rssi;
        query += 3;
    }
}

static int getAssigned() {
    writeLine("Req assign");
    Serial.printf("Socket to %s:%d\n", my.serverIP, SIDE_PORT);
    int err = wifiClient.connect(my.serverIP, SIDE_PORT);
    Serial.printf("connect() returned %d\n", err);
    if (err == 0) {
        Serial.println("Connection refused");
	writeLine("Can't connect");
        return -1;
    }
    wifiClient.setNoDelay(false);

    Serial.println("Sending assignment request");

    err = wifiClient.write(assignQuery, QUERY_LENGTH);
    if (err < QUERY_LENGTH) {
	Serial.printf("Sent %d bytes != %d\n", err, QUERY_LENGTH);
	writeLine("Query failed");
        return -1;
    }
    
    int assigned;
    time_t timeout = time(NULL);
    do {
        assigned = wifiClient.read();
    } while (assigned < 0 && (time(NULL) - timeout) < 10);
    wifiClient.stop();
    
    if (assigned == -1) {
        Serial.println("No response from Actiserver");
        writeLine("No response");
    } else if (assigned >= 100) {
        Serial.printf("Error relayed %d\n", assigned);
        writeLine("Not assigned");
        assigned = -1;
    } else {
        Serial.printf("Assigned %d: %s\n", assigned, actisList[assigned].ssid);
        writeLine(actisList[assigned].ssid);
    }

    return assigned;
}

static bool connectServer() {
    Serial.printf("Socket to %s:%d\n", my.serverIP, ACTI_PORT);
    int err = wifiClient.connect(my.serverIP, ACTI_PORT);
    Serial.printf("connect() returned %d\n", err);
    if (err == 0) {
        Serial.println("Connection refused");
        writeLine("Refused");
        return false;
    }
    wifiClient.setNoDelay(false);
    writeLine("Connected");
    return true;
}

static time_t getActimIdAndTime() {
    byte initMessage[INIT_LENGTH];
    Serial.print("Getting name and time ");

    memcpy(initMessage, my.boardName, 3);
    memcpy(initMessage + 3, my.mac, 6);
    initMessage[9] = my.sensorBits;
    memcpy(initMessage + 10, VERSION_STR, 3);

    int err = 0;
    err = wifiClient.write(initMessage, INIT_LENGTH);
    if (err < INIT_LENGTH) {
	Serial.printf("\nSent %d bytes != %d\n", err, INIT_LENGTH);
	writeLine("Init failed");
        RESTART(2);
    }
    byte response[RESPONSE_LENGTH];

    err = 0;
    time_t timeout = time(NULL);
    while (err < 6) {
        err += wifiClient.read(response + err, RESPONSE_LENGTH - err);
        if ((time(NULL) - timeout) > 10) {
            Serial.println("No response from Actiserver");
	    writeLine("No response");
            RESTART(2);
        }
    }

    Serial.printf("read: %d bytes, response=%02X%02X, %02X %02X %02X %02X\n",
                  err, response[0], response[1],
                  response[2], response[3], response[4], response[5]);
    
    my.clientId = response[0] * 256 + response[1];
    time_t bootTime = (response[2] << 24) + (response[3] << 16) + (response[4] << 8) + response[5];
    
    sprintf(my.clientName, "Actim%04d", my.clientId);
    Serial.println(my.clientName);

    char message[16];
    sprintf(message, "v%s>%04d  %03d", VERSION_STR, my.clientId, my.serverId);
    displayTitle(message);

    return bootTime;
}

// Huge and ugly but that's the way it is.

void netInit() {
    storeMacAddress();
    
    blinkLed(COLOR_SWAP);
    WiFi.useStaticBuffers(true);
    WiFi.disconnect(true, true);
    delay(100);
    WiFi.mode(WIFI_STA);
    
    blinkLed(COLOR_SWAP);
    int nScan;
    nScan = scanNetworks();
    
    findActis(nScan);
    WiFi.scanDelete();

    buildQuery();

    int i, indexChoice;
    for (i = 0; i < nActis; i++) {
        blinkLed(COLOR_SWAP);
        if (!tryConnect(i)) continue;
        indexChoice = getAssigned();
        if (indexChoice >= 0) break;
    }
    if (i == nActis) {
        Serial.println("Can't get assigned, choose first");
	writeLine("Self-assign");
        indexChoice = 0;
        my.serverId = 0;
    }

    Serial.printf("Selected %s\n", actisList[indexChoice].ssid);

    if (actisList[indexChoice].serverId != my.serverId) {
        WiFi.disconnect(true, true);
        delay(1000);
        Serial.println("Switch AP");
        if (!tryConnect(indexChoice)) {
            Serial.println("Can't connect to chosen server, rebooting");
            writeLine("Can't connect");
            esp_wifi_stop();
            RESTART(2);
        }
    }

    WiFi.setAutoReconnect(true);
    connectServer();

    time_t bootEpoch = getActimIdAndTime();
    initClock(bootEpoch);

#ifdef STATIC_QUEUE    
    msgQueue = xQueueCreateStatic(QUEUE_SIZE, sizeof(int), msgQueueItems, &msgQueueStatic);
#else    
    msgQueue = xQueueCreate(QUEUE_SIZE, sizeof(int));
#endif    
    if (msgQueue == 0) {
        Serial.println("Error creating queue, rebooting");
        writeLine("OS error");
        RESTART(1);
    }
    my.queueFill = 0.0;

    if (my.dualCore)
        setupCore0(Core0Loop);
}

static void _test(int type) {
    switch (type) {
    case 1:
        if (getAbsMicros() > 10000000) {
            ERROR_FATAL("Test FATAL0");
        }
        break;
    }
}
