#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <NeoPixelBus.h>

// secrets.h should define the following variables
// #define MY_MQTT_SERVER_IP "192.168.0.123"
// #define MY_WIFI_SSID "your wifi ssid goes here"
// #define MY_WIFI_PASS "your wifi password goes here"
#include "secrets.h" 

//
// LEDs must be connected to pin D4 / GPIO2. This is hard coded in NeoPixelBus and is not configurable
//

#define LEDCOUNT   134       // Number of LEDs used for serial
#define BAUDRATE   230400    // Serial port speed
#define SERIAL_TIMEOUT_THRESHOLD 1000 // number of loops before we consider serial to be silent

const char prefix[] = {0x41, 0x64, 0x61, 0x00, 0x85, 0xD0};  // prefix, expect this sequence over serial before each frame

const PROGMEM char* WIFI_SSID = MY_WIFI_SSID;
const PROGMEM char* WIFI_PASS = MY_WIFI_PASS;

// MQTT: ID, server IP, port, username and password
const PROGMEM char* MQTT_CLIENT_ID = "ambilight";       // this is how your NodeMCU will identify itself to the MQTT server. Note: I've had issues when multiple clients connect with the same name.
const PROGMEM char* MQTT_SERVER_IP = MY_MQTT_SERVER_IP; // The IP/hostname of the MQTT server
const PROGMEM uint16_t MQTT_SERVER_PORT = 1883;         // The port where MQTT is running 

// MQTT: topics to subscribe to
const char* MQTT_SWITCH_COMMAND_TOPIC = "ambilight/switch";
const char* MQTT_RGB_COMMAND_TOPIC = "ambilight/rgb";

// MQTT: topic to publish status events
const char* MQTT_STATUS_COMMAND_TOPIC = "ambilight/status";

// No more configurables past this point //

#define MODE_OFF                   0   // no serial data available from serial, MQTT set to black
#define MODE_MQTT                  1   // no serial data available from serial, controlled by MQTT
#define MODE_SERIAL_PREFIX         2   // serial input is driving the lights
#define MODE_SERIAL_DATA           3   // serial input is driving the lights

byte serialBuffer[3];   // buffer for receiving serial data

// Keep track of app state all in one place
struct State {
    byte mode;                      // Current mode: OFF/MQTT/SERIAL
    int serialByteIndex;            // how many serial bytes we've read since the last frame
    int serialTimeoutCounter;       // keep track of how many loops() since we last heard from the serial port
    RgbColor targetColor;           // target (and after the fade, the current) color set by MQTT
    RgbColor prevColor;             // the previous color set by MQTT
    bool isFading;                  // whether or not we are fading between MQTT colors
    float fadeProgress;             // how far we have faded
    float fadeStep;                 // how much the fade progress should change each iteration
} state;

WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);
NeoPixelBus<NeoGrbFeature, NeoEsp8266Uart800KbpsMethod> strip(LEDCOUNT);

void setup() {
    Serial.begin(BAUDRATE);
    strip.Begin();

    state.mode = MODE_OFF;
    state.serialByteIndex = 0;
    state.serialTimeoutCounter = 0;
    
    state.targetColor = RgbColor(0x11, 0, 0xFF);
    state.isFading = false;

    // connect to WiFi
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while (WiFi.status() != WL_CONNECTED) {
        delay(50);
        Serial.print(".");
    }

    Serial.print(" IP address: ");
    Serial.println(WiFi.localIP());

    // setup MQTT
    mqtt.setServer(MQTT_SERVER_IP, MQTT_SERVER_PORT);
    mqtt.setCallback(mqttMessageReceived);

    if (mqtt.connect(MQTT_CLIENT_ID)) {
        mqtt.subscribe(MQTT_SWITCH_COMMAND_TOPIC);
        mqtt.subscribe(MQTT_RGB_COMMAND_TOPIC);

        mqtt.publish(MQTT_STATUS_COMMAND_TOPIC, "Connected");
    }
}

void loop() {
    state.serialTimeoutCounter++;
    switch(state.mode) {
        case MODE_OFF:
        case MODE_MQTT:
            if (Serial.available() > 0) {
                state.mode = MODE_SERIAL_PREFIX;
                state.serialByteIndex = 0;
                state.serialTimeoutCounter = 0;
                mqtt.publish(MQTT_STATUS_COMMAND_TOPIC, "Switch to Serial");
            } else {
                RgbColor color;
                if (state.isFading) {
                    state.fadeProgress += state.fadeStep;
                    
                    if (state.fadeProgress >= 1) {
                        state.isFading = false;
                        color = state.targetColor;
                    } else {
                        color = calculateFade(state.prevColor, state.targetColor, state.fadeProgress);
                    }
                } else {
                    color = state.targetColor;
                }
                
                for (int i = 0; i < LEDCOUNT; i++) {
                    strip.SetPixelColor(i, color);
                }
                strip.Show();
                mqtt.loop();
            }
            break;
        case MODE_SERIAL_PREFIX:
            if (Serial.available() > 0) {
                byte serialByte = Serial.read();
                state.serialTimeoutCounter = 0;

                if (serialByte == prefix[state.serialByteIndex]) {
                    if (state.serialByteIndex == sizeof(prefix) - 1) {
                        state.mode = MODE_SERIAL_DATA;
                        state.serialByteIndex = 0;
                    } else {
                        state.serialByteIndex++;
                    }
                } else {
                    state.serialByteIndex = 0;
                }
            } else {
                handleSerialTimeout();
            }
            break;
        case MODE_SERIAL_DATA:
            if (Serial.available() > 2) {
                Serial.readBytes(serialBuffer, 3);
                state.serialTimeoutCounter = 0;

                RgbColor color = RgbColor(serialBuffer[0], serialBuffer[1], serialBuffer[2]);
                strip.SetPixelColor(state.serialByteIndex++, color);

                if (state.serialByteIndex == LEDCOUNT) {
                    strip.Show();
                    state.mode = MODE_SERIAL_PREFIX;
                    state.serialByteIndex = 0;
                }
            } else {
                handleSerialTimeout();
            }
            break; 
    }
}

void mqttMessageReceived(char* topic, byte* payload, unsigned int payloadLength) {

    // First, convert byte* to char*
    char message[payloadLength+1];
    for (int i = 0; i < payloadLength; i++) {
        message[i] = (char)payload[i];
    }
    message[payloadLength] = '\0';

    // mqtt.publish(MQTT_STATUS_COMMAND_TOPIC, String(String(topic) + ": " + message).c_str());
    if (equal(topic, MQTT_SWITCH_COMMAND_TOPIC)) {
        if (equal(message, "ON")) {
            if (state.mode == MODE_OFF) {
                state.mode = MODE_MQTT;

                fadeTo(state.prevColor, 0.002);
            }
        } else if (equal(message, "OFF")) {
            state.mode = MODE_OFF;
            fadeTo(RgbColor(0, 0, 0), 0.002);
        }
    } else if (equal(topic, MQTT_RGB_COMMAND_TOPIC)) {
        state.mode = MODE_MQTT;
        String r, g, b;

        byte stage = 0;
        for (uint8_t i = 0; i < payloadLength; i++)  {
            char character = (char)message[i];
            if (stage == 0) {
                if (character != ',') {
                    r.concat(character);
                } else {
                    stage = 1;
                }
            } else if (stage == 1) {
                if (character != ',') {
                    g.concat(character);
                } else {
                    stage = 2;
                }
            } else if (stage == 2) {
                b.concat(character);
            }
        }
        fadeTo(RgbColor(r.toInt(), g.toInt(), b.toInt()));
    }
}

bool equal(const char* a, const char* b) {
    return strcmp(a, b) == 0;
}

void fadeTo(RgbColor color) {
    fadeTo(color, 0.005);
}

void fadeTo(RgbColor color, float step) {
    state.prevColor = state.targetColor;
    state.targetColor = color;
    state.isFading = true;
    state.fadeProgress = 0;
    state.fadeStep = step;
}

RgbColor calculateFade(RgbColor fromColor, RgbColor toColor, float progress) {
    return RgbColor(
        fromColor.R + (float)(toColor.R - fromColor.R)*progress,
        fromColor.G + (float)(toColor.G - fromColor.G)*progress,
        fromColor.B + (float)(toColor.B - fromColor.B)*progress
    );
}

void handleSerialTimeout() {
    if (state.serialTimeoutCounter >= SERIAL_TIMEOUT_THRESHOLD) {
        // Reconnect to MQTT because it likely timed out
        if (mqtt.connect("ESP8266Client")) {
            mqtt.subscribe(MQTT_SWITCH_COMMAND_TOPIC);
            mqtt.subscribe(MQTT_RGB_COMMAND_TOPIC);
        }
        if (state.mode == MODE_SERIAL_PREFIX || state.mode == MODE_SERIAL_DATA) {
            state.mode = MODE_OFF;
            state.targetColor = RgbColor(0, 0, 0);
            mqtt.publish(MQTT_STATUS_COMMAND_TOPIC, "Switch to MQTT");
        }
    }
    delay(1);
}
