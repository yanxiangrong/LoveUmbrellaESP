#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ikcp.h>
#include <GDBStub.h>

#define MOTOR_A_PIN D5
#define MOTOR_B_PIN D0
#define BEEP_PIN D7
#define MOTOR_LIM_PIN D2

using namespace std;

//const char *ssid = "IOT_2508"; // key in your own SSID
const char *ssid = "HNIT_Teacher";
const char *password = nullptr;
//const char *password = "wlwsys6688"; // key in your own Wi-Fi access point password
const uint16_t id = 1234;
const char *host = "volunteer.fengxianhub.top";
const uint16_t port = 67;

int reconnectTime = 1;

WiFiUDP Udp;
char incomingPacket[1024];
char sendBuf[1024];
char recvBuf[1024];
ikcpcb *mKCP;
uint16_t localUdpPort;

unsigned long lastSend = 0;
unsigned long lastResponse = 0;
unsigned long checkTime = 0;

unsigned long lastUnlock = 0;
bool lockStatus = false;

void beep() {
    analogWrite(BEEP_PIN, 128);
    delay(100);
    digitalWrite(BEEP_PIN, LOW);
}

void beebee() {
    analogWrite(BEEP_PIN, 128);
    delay(150);
    digitalWrite(BEEP_PIN, LOW);
    delay(100);
    analogWrite(BEEP_PIN, 128);
    delay(150);
    digitalWrite(BEEP_PIN, LOW);
}

void beee() {
    analogWrite(BEEP_PIN, 128);
    delay(1000);
    digitalWrite(BEEP_PIN, LOW);
}

void connectWiFi() {
    // We start by connecting to a Wi-Fi network
    Serial.println();
    Serial.print("Connecting to WiFi: ");
    Serial.println(ssid);

    if (password) {
        Serial.print("By passwd: ");
        Serial.println(password);
        WiFi.begin(ssid, password);
    } else {
        WiFi.begin(ssid);
    }

    int i = 60;
    while (WiFi.status() != WL_CONNECTED and i) {
        delay(500);
        Serial.print(".");

        i--;
    }
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("");
        Serial.println(F("WiFi connect fail"));
        beee();
        return;
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
}

int udpSendOut(const char *pBuf, int lSize, ikcpcb *pKCP, void *pCTX) {
    digitalWrite(LED_BUILTIN, LOW);

    Udp.beginPacket(host, port);
    Udp.write(pBuf, lSize);
    Udp.endPacket();
    Serial.printf("Send %d bytes UDP packet\n", lSize);
    delay(20);

    digitalWrite(LED_BUILTIN, HIGH);
    return 0;
}

void sendData() {
    lastSend = millis();
    ikcp_send(mKCP, sendBuf, (int) strlen(sendBuf));
    Serial.printf("[KCP] Sent: %s\n", sendBuf);
}

void checkNetwork() {
    static int lastReconnected = 0;
    if (!WiFi.isConnected() and ((millis() - lastReconnected) / 1000 >= reconnectTime)) {
        digitalWrite(LED_BUILTIN, LOW);

        connectWiFi();

        lastReconnected = (int) millis();
        if (reconnectTime < 16) {
            reconnectTime *= 2;
        }
        digitalWrite(LED_BUILTIN, HIGH);
    }
}

int lock() {
    if (lockStatus) {
        return 0;
    }

    unsigned long startTime = millis();

    digitalWrite(MOTOR_A_PIN, LOW);
    digitalWrite(MOTOR_B_PIN, HIGH);

    while (millis() - startTime < 2000) {
        if (digitalRead(MOTOR_LIM_PIN) == LOW) {
            digitalWrite(MOTOR_A_PIN, HIGH);
            digitalWrite(MOTOR_B_PIN, HIGH);
            delay(100);
            digitalWrite(MOTOR_A_PIN, LOW);
            digitalWrite(MOTOR_B_PIN, LOW);
            beep();
            lockStatus = true;
            return 0;
        }
        yield();
    }

    digitalWrite(MOTOR_A_PIN, HIGH);
    digitalWrite(MOTOR_B_PIN, HIGH);
    delay(100);
    digitalWrite(MOTOR_A_PIN, LOW);
    digitalWrite(MOTOR_B_PIN, LOW);
    beee();
    lockStatus = true;
    return -1;
}

void unlock() {
    if (not lockStatus) {
        return;
    }

    digitalWrite(MOTOR_A_PIN, HIGH);
    digitalWrite(MOTOR_B_PIN, LOW);
    delay(1000);
    digitalWrite(MOTOR_A_PIN, HIGH);
    digitalWrite(MOTOR_B_PIN, HIGH);
    delay(100);
    digitalWrite(MOTOR_A_PIN, LOW);
    digitalWrite(MOTOR_B_PIN, LOW);

    lastUnlock = millis();
    lockStatus = false;
}

void doCommand(const String &str) {
    if (str == "Hwsping00T") {
        sprintf(sendBuf, "Hwcping00T");
//        sendData();
        return;
    }

    if (str == "HwsLock02onT") {
        beebee();
        unlock();
        sprintf(sendBuf, "HwcUnLock07successT");
        sendData();
        return;
    }

    if (str == "HwsLock03offT") {
        beebee();
        if (lock() == 0) {
            sprintf(sendBuf, "HwcLock07successT");
        } else {
            sprintf(sendBuf, "HwcLock04failT");
        }
        sendData();
        return;
    }
}

void receive() {
    int hr;
    hr = ikcp_recv(mKCP, recvBuf, sizeof(recvBuf));
    // 没有收到包就退出
    if (hr < 0) return;
    recvBuf[hr] = 0;

    Serial.printf("[KCP] Received:  %s\n", recvBuf);

    String str(recvBuf);
    doCommand(str);
}

void readSerial() {
    while (Serial.available()) {
        String str = Serial.readString();
        Serial.printf("[Serial] Received:  %s\n", str.c_str());

        if (str == "LOCK") {
            beebee();
            Serial.printf("[Serial] LOCK...\n");
            if (lock() == 0) {
                Serial.printf("[Serial] OK\n");
            } else {
                Serial.printf("[Serial] FAIL\n");
            }

            return;
        }

        if (str == "UNLOCK") {
            beebee();
            Serial.printf("[Serial] UNLOCK...\n");
            unlock();
            Serial.printf("[Serial] DONE\n");
            return;
        }

        if (str == "RESTART") {
            EspClass::restart();
            Serial.printf("[Serial] OK\n");
            return;
        }

        if (str == "RESET") {
            EspClass::reset();
            Serial.printf("[Serial] OK\n");
            return;
        }

    }
}

void checkLock() {
    if (millis() - lastUnlock > 10e3) {
        if (not lockStatus) {
            if (lock() == 0) {
                sprintf(sendBuf, "HwcLock07successT");
            } else {
                sprintf(sendBuf, "HwcLock04failT");
            }
            sendData();
        }
    }
}

void checkVcc() {
    static unsigned long lastCheck = millis();
    if (millis() - lastCheck > 10e3) {
        lastCheck = millis();
        if (EspClass::getVcc() < 3200) {
            sprintf(sendBuf, "Hwclowpower00T");
            sendData();
        }
    }
}

void setup() {
    uint32_t conv;

    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(MOTOR_A_PIN, OUTPUT);
    pinMode(MOTOR_B_PIN, OUTPUT);
    pinMode(MOTOR_LIM_PIN, INPUT);
    pinMode(BEEP_PIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    beep();
    Serial.begin(115200);
    delay(100);

    Serial.println();
    Serial.println();
    Serial.print("Client ID: ");
    Serial.println(id);
    Serial.print(F("Server host: "));
    Serial.print(host);
    Serial.print(F(" port: "));
    Serial.println(port);
    Serial.print(F("CPU freq: "));
    Serial.print(EspClass::getCpuFreqMHz());
    Serial.println(F("MHz"));
    Serial.print(F("Vcc: "));
    Serial.print(EspClass::getVcc());
    Serial.println(F("mV"));
//    Serial.print(F("GDBStub init"));
//    gdbstub_init();

    connectWiFi();

    localUdpPort = 1024 + EspClass::random() % 3976;
    Serial.print(F("UDP begin, localUdpPort: "));
    Serial.println(localUdpPort);
    Udp.begin(localUdpPort);

    conv = EspClass::random();
    Serial.print(F("KCP create, conv: "));
    Serial.println(conv);
    mKCP = ikcp_create(conv, nullptr);
    ikcp_nodelay(mKCP, 0, 40, 0, 0);
    ikcp_wndsize(mKCP, 16, 16);
    mKCP->rx_minrto = 100;
    mKCP->output = udpSendOut;

    Serial.println(F("Init done"));
    Serial.println();
    Serial.println();
    lastResponse = millis();
    beebee();
    digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
    checkNetwork();

    if (WiFi.isConnected()) {
        int packetSize = Udp.parsePacket();
        if (packetSize) {
            lastResponse = millis();
            Serial.printf("Received %d bytes from %s, port %d\n", packetSize, Udp.remoteIP().toString().c_str(),
                          Udp.remotePort());
            int len = Udp.read(incomingPacket, 255);
            if (len > 0) {
                incomingPacket[len] = '\0';
            }
            ikcp_input(mKCP, incomingPacket, len);
        }

        ikcp_update(mKCP, millis());
    }

    receive();
    readSerial();

    if (ikcp_waitsnd(mKCP) == 0 and (millis() - lastSend) > 2e3) {
        sprintf(sendBuf, "Hwcping00T");
        sendData();
    }

    if (lastSend > lastResponse and millis() - lastResponse > 120e3) {
        Serial.println("KCP Connect timeout, restarting...");
        EspClass::restart();
    }

    if (millis() < checkTime) {
        Serial.println("Time overflow, restarting...");
        EspClass::restart();
    }

    checkTime = millis();

    checkLock();
    checkVcc();
}