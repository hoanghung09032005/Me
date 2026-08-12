#include <Arduino.h>
#include <WiFi.h>

#include "car_bridge.h"

#define STM32_UART_BAUD 115200
#define STM32_UART_RX   21
#define STM32_UART_TX   47

WiFiServer bridgeServer(8080);
WiFiClient bridgeClient;
static bool clientWasConnected = false;

static void sendBridgeInfo(const char *message)
{
    if (bridgeClient && bridgeClient.connected()) {
        bridgeClient.print("[ESP32-S3] ");
        bridgeClient.println(message);
    }
}

static void stopStm32Car()
{
    /* STM32 treats S as a global emergency-stop command. */
    Serial1.print("S\n");
}

void carBridgeBegin()
{
    /* ESP32 RX receives STM32 PA9/TX; ESP32 TX drives STM32 PA10/RX. */
    Serial1.begin(STM32_UART_BAUD, SERIAL_8N1, STM32_UART_RX, STM32_UART_TX);

    bridgeServer.begin();
    bridgeServer.setNoDelay(true);
    clientWasConnected = false;
}

void carBridgeLoop()
{
    bool clientConnected = bridgeClient && bridgeClient.connected();

    /* Keep a single controlling PC. Replacing a disconnected client first
     * stops the car so a lost TCP link cannot leave AUTO mode running. */
    if (bridgeServer.hasClient()) {
        if (!clientConnected) {
            if (clientWasConnected) {
                stopStm32Car();
            }
            if (bridgeClient) {
                bridgeClient.stop();
            }
            bridgeClient = bridgeServer.available();
            clientConnected = bridgeClient && bridgeClient.connected();
            if (clientConnected) {
                sendBridgeInfo("TCP connected; UART bridge ready (RX=21, TX=47, 115200). ");
            }
        } else {
            WiFiClient rejectedClient = bridgeServer.available();
            rejectedClient.stop();
        }
    }

    if (clientWasConnected && !clientConnected) {
        stopStm32Car();
    }
    clientWasConnected = clientConnected;

    /* TCP -> UART: GUI commands to STM32. */
    if (clientConnected) {
        while (bridgeClient.available()) {
            char c = (char)bridgeClient.read();
            Serial1.write((uint8_t)c);
            if (c == '\n') {
                sendBridgeInfo("command forwarded to STM32 UART.");
            }
        }
    }

    /* UART -> TCP: STM32 telemetry to GUI. */
    while (Serial1.available()) {
        char c = (char)Serial1.read();
        if (clientConnected) {
            bridgeClient.write((uint8_t)c);
        }
    }
}

bool carBridgeHasClient()
{
    return bridgeClient && bridgeClient.connected();
}
