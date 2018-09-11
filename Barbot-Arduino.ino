/*
Copyright 2018 Thomas A. Bennedum

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
IN THE SOFTWARE.
*/

/*
    https://github.com/adafruit/Adafruit_NeoPixel
    https://github.com/sparkfun/SparkFun_APDS-9960_Sensor_Arduino_Library
    https://github.com/frdfsnlght/NeoPixel-Patterns
*/


#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <SparkFun_APDS9960.h>
#include <NeoPixelController.h>
#include <WipeNeoPixelPattern.h>
#include <MultiWipeNeoPixelPattern.h>
#include <BlinkNeoPixelPattern.h>
#include <RainbowNeoPixelPattern.h>
#include <ChaseNeoPixelPattern.h>
#include <ScanNeoPixelPattern.h>
#include <FadeNeoPixelPattern.h>
#include <FireNeoPixelPattern.h>
#include <Colors.h>

#define PIN_SERIAL_RX           0
#define PIN_SERIAL_TX           1

#define PIN_BUTTON              2
#define PIN_RELAY1              4
#define PIN_RELAY2              5
#define PIN_LIGHTS              6
#define PIN_LED                 13
#define PIN_I2C_DA              A4
#define PIN_I2C_CL              A5

#define NUM_PIXELS              60
#define NUM_SEGMENTS            5
#define NUM_PATTERN_SLOTS       8
#define PATTERN_SLOT_BASE       0

#define INPUT_BUFFER_LENGTH     32

#define LED_TOGGLE_INTERVAL_FAST  250
#define LED_TOGGLE_INTERVAL_SLOW  1000
#define SENSOR_READ_INTERVAL    250
#define PROXIMITY_WINDOW        5
#define BUTTON_PRESS_SHORT      1000
#define BUTTON_PRESS_LONG       8000

#define STATE_OFF               0
#define STATE_OFF_P             1
#define STATE_OFF_LP            2
#define STATE_WAIT_ON           3
#define STATE_WAIT_ON_SP        4
#define STATE_ON                5
#define STATE_ON_P              6
#define STATE_ON_SP             7

#define ERR_OK                  0
#define ERR_NO_PATTERN          1
#define ERR_INVALID_PATTERN     2



typedef struct {
    char data[INPUT_BUFFER_LENGTH];
    uint16_t length;
} inputBuffer_t;
    
inputBuffer_t inputBuffer;
int ch;

// Parameter 1 = number of pixels in strip
// Parameter 2 = number of segments in the strip
// Parameter 3 = pin number
// Parameter 4 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
NeoPixelController lights = NeoPixelController(NUM_PIXELS, NUM_SEGMENTS, PIN_LIGHTS, NEO_GRB + NEO_KHZ800);
uint8_t segments = 0;

// onComplete callback function
//void lightPatternComplete(NeoPatterns * aLedsPtr);

// IMPORTANT: To reduce NeoPixel burnout risk, add 1000 uF capacitor across
// pixel power leads, add 300 - 500 Ohm resistor on first pixel's data input
// and minimize distance between controller and first pixel.  Avoid connecting
// on a live circuit...if you must, connect GND first.


SparkFun_APDS9960 apds = SparkFun_APDS9960();
bool apdsSetup = false;
uint8_t proximityData = 0;
uint32_t lastSensorRead = 0;
uint8_t proximityGain = PGAIN_2X;
uint8_t proximityWindow = PROXIMITY_WINDOW;

bool ledOn = false;
uint32_t lastLEDToggleTime = 0;

int powerDownTime = -1;
uint32_t lastPowerDownTickTime = 0;

uint8_t state = STATE_OFF;
uint32_t buttonPressedTime = 0;


void setup() {
    Serial.begin(115200, SERIAL_8N1);

    pinMode(PIN_BUTTON, INPUT);
    digitalWrite(PIN_BUTTON, HIGH); // enable pullup
    pinMode(PIN_RELAY1, OUTPUT);
    pinMode(PIN_RELAY2, OUTPUT);
    pinMode(PIN_LED, OUTPUT);
    
    lights.setupSegment(0, 0, NUM_PIXELS);
    for (int i = 0; i < NUM_SEGMENTS - 1; i++) {
        lights.setupSegment(i + 1, i * NUM_PIXELS / (NUM_SEGMENTS - 1), NUM_PIXELS / (NUM_SEGMENTS - 1));
    }
    lights.begin();
    lights.setSegmentColor(COLOR_OFF, 0);
    
    randomSeed(analogRead(0));
    
    // Initialize APDS-9960 (configure I2C and initial values)
    apdsSetup = apds.init();
    if (apdsSetup) {
        configureSensors();
    }

    turnOffRelays();
    turnOffLights();
    turnOffLED();
        
    sendMessage(F("Barbot-Arduino ready"));
}

void loop() {
    loopSerial();
    loopLights();
    loopButton();
    loopSensors();
    loopPower();
    loopLED();
}

void loopSerial() {
    while (Serial.available()) {
        ch = Serial.read();
        if ((ch == '\r') || (ch == '\n')) {
            if (inputBuffer.length) {
                processCommand();
            } else {
                sendOK();
            }
            inputBuffer.data[0] = '\0';
            inputBuffer.length = 0;
        } else if (ch == 8) {
            if (inputBuffer.length) {
                inputBuffer.length--;
            }
        } else if (ch == 27) {
            if (inputBuffer.length) {
                inputBuffer.data[0] = '\0';
                inputBuffer.length = 0;
                send(F("CANCELED\n"));
            }
        } else if ((ch >= 32) && (ch <= 126)) {
            if (inputBuffer.length >= (INPUT_BUFFER_LENGTH - 1)) {
                inputBuffer.data[0] = '\0';
                inputBuffer.length = 0;
                sendError(F("overflow"));
                return;
            }
            inputBuffer.data[inputBuffer.length++] = ch;
            inputBuffer.data[inputBuffer.length] = '\0';
        }
    }
}

void loopLights() {
    lights.update();
}

void loopButton() {
    bool pressed = digitalRead(PIN_BUTTON);

    switch (state) {
        case STATE_OFF:
            if (pressed) {
                state = STATE_OFF_P;
                buttonPressedTime = millis();
            }
            break;
        case STATE_OFF_P:
            if (pressed) {
                if ((millis() - buttonPressedTime) >= BUTTON_PRESS_SHORT) {
                    powerUp();
                    state = STATE_WAIT_ON_SP;
                }
            } else {
                state = STATE_OFF;
            }
            break;
        case STATE_OFF_LP:
            if (! pressed) {
                state = STATE_OFF;
            }
            break;
        case STATE_WAIT_ON_SP:
            if (pressed) {
                if ((millis() - buttonPressedTime) >= BUTTON_PRESS_LONG) {
                    powerDown();
                    state = STATE_OFF_LP;
                }
            } else {
                state = STATE_WAIT_ON;
            }
            break;
        case STATE_WAIT_ON:
            if (pressed) {
                state = STATE_WAIT_ON_SP;
                buttonPressedTime = millis();
            }
            break;
            
        case STATE_ON:
            if (pressed) {
                state = STATE_ON_P;
                buttonPressedTime = millis();
            }
            break;
        case STATE_ON_P:
            if (pressed) {
                if ((millis() - buttonPressedTime) >= BUTTON_PRESS_SHORT) {
                    requestPowerDown();
                    state = STATE_ON_SP;
                }
            } else {
                state = STATE_ON;
            }
            break;
        case STATE_ON_SP:
            if (pressed) {
                if ((millis() - buttonPressedTime) >= BUTTON_PRESS_LONG) {
                    powerDown();
                    state = STATE_OFF_LP;
                }
            } else {
                state = STATE_ON;
            }
            break;
        default:
            sendError(F("unknown state"));
            break;
    }
}

void loopSensors() {
    if (apdsSetup && ((millis() - lastSensorRead) >= SENSOR_READ_INTERVAL)) {
        lastSensorRead = millis();
        uint8_t newProximityData;
        if (apds.readProximity(newProximityData)) {
            if (((proximityData > proximityWindow) && (newProximityData < (proximityData - proximityWindow))) ||
                ((proximityData < (255 - proximityWindow)) && (newProximityData > (proximityData + proximityWindow)))
                ) {
                proximityData = newProximityData;
                sendSensorData();
            }
        }
    }
}

void loopPower() {
    if (powerDownTime == -1) return;
    
    if ((millis() - lastPowerDownTickTime) >= 1000) {
        lastPowerDownTickTime = millis();
        powerDownTime--;
        if (powerDownTime == -1) {
            powerDown();
            state = STATE_OFF;
        }
    }
}

void loopLED() {
    switch (state) {
        case STATE_OFF:
        case STATE_OFF_P:
        case STATE_OFF_LP:
            if ((millis() - lastLEDToggleTime) >= LED_TOGGLE_INTERVAL_SLOW) {
                lastLEDToggleTime = millis();
                toggleLED();
            }
            break;
        case STATE_WAIT_ON:
        case STATE_WAIT_ON_SP:
            if ((millis() - lastLEDToggleTime) >= LED_TOGGLE_INTERVAL_FAST) {
                lastLEDToggleTime = millis();
                toggleLED();
            }
            break;
        default:
            if (! ledOn) turnOnLED();
            break;
    }
}

void processCommand() {
    char* cmd = inputBuffer.data;
    switch (cmd[0]) {
        case 'L':
        case 'l':
            processLightCommand(cmd + 1);
            break;
        case 'S':
        case 's':
            processSensorCommand(cmd + 1);
            break;
        case 'R':
        case 'r':
            processPowerCommand(cmd + 1);
            break;
        case 'E':
        case 'e':
            processEEPROMCommand(cmd + 1);
            break;
        default:
            sendError(F("invalid command"));
            break;
    }
}


// =========== Light commands

void processLightCommand(char* cmd) {
    switch (cmd[0]) {
        case 'C':
        case 'c':
            cmdLightColor(cmd + 1);
            break;
        case 'P':
        case 'p':
            cmdLightPlayPattern(cmd + 1);
            break;
        case 'S':
        case 's':
            cmdLightSavePattern(cmd + 1);
            break;
        case 'L':
        case 'l':
            cmdLightLoadPattern(cmd + 1);
            break;
        case '?':
            cmdLightStatus();
            break;
        default:
            sendError(F("invalid light command"));
            break;
    }
}

void cmdLightColor(char* str) {
    segments = readSegments(&str);
    readDelim(&str);
    color_t color = readColor(&str);
    prepareLightSegments();
    for (int i = 0; i < NUM_SEGMENTS; i++) {
        if (segments & (1 << i)) {
            lights.setSegmentColor(color, i);
            if (i == 0) break;
        }
    }
    sendOK();
}

void cmdLightPlayPattern(char* str) {
    switch (playLightPattern(str)) {
        case ERR_INVALID_PATTERN:
            sendError(F("invalid pattern"));
            break;
        case ERR_OK:
            sendOK();
            break;
    }
}
void cmdLightSavePattern(char* str) {
    uint8_t slot = readUInt(&str);
    readDelim(&str);
    
    if (slot >= (NUM_PATTERN_SLOTS - 1)) {
        sendError(F("invalid slot"));
        return;
    }
    for (byte i = 0; ; i++) {
        EEPROM.update(PATTERN_SLOT_BASE + (slot * INPUT_BUFFER_LENGTH) + i, str[i]);
        if (! str[i]) break;
    }
    sendOK();
}

void cmdLightLoadPattern(char* str) {
    uint8_t slot = readUInt(&str);
    if (slot >= (NUM_PATTERN_SLOTS - 1)) {
        sendError(F("invalid slot"));
        return;
    }
    switch (playLightPatternSlot(slot)) {
        case ERR_NO_PATTERN:
            sendError(F("no pattern"));
            break;
        case ERR_INVALID_PATTERN:
            sendError(F("invalid pattern"));
            break;
        case ERR_OK:
            sendOK();
            break;
    }        
}

void cmdLightStatus() {
    for (byte i = 0; i < NUM_SEGMENTS; i++) {
        send(F("segment "));
        sendInt(i);
        send(F(": "));
        send((segments & (1 << i)) ? F("X ") : F("O "));
        sendInt(lights.segmentBasePixel(i));
        sendChar(',');
        sendInt(lights.segmentLength(i));
        send(F(": "));
        if (lights.isSegmentActive(i)) {
            sendChar('A');
        } else {
            sendChar('-');
        }
        
        sendChar('\n');
    }
    for (byte i = 0; i < NUM_PATTERN_SLOTS; i++) {
        send(F("slot "));
        sendInt(i);
        send(F(": "));
        switch (loadLightPatternSlot(i)) {
            case ERR_OK:
                send(inputBuffer.data);
                break;
            case ERR_NO_PATTERN:
                send(F("<empty>"));
                break;
        }
        sendChar('\n');
    }
    sendOK();
}


// =========== Sensor commands

void processSensorCommand(char* cmd) {
    switch (cmd[0]) {
        case 'G':
        case 'g':
            cmdSensorGain(cmd + 1);
            break;
        case 'W':
        case 'w':
            cmdSensorWindow(cmd + 1);
            break;
        case '?':
            cmdSensorStatus();
            break;
        default:
            sendError(F("invalid sensor command"));
            break;
    }
}

void cmdSensorGain(char* str) {
    unsigned gain = readUInt(&str);
    if ((gain < PGAIN_2X) || (gain > PGAIN_8X)) {
        sendError(F("invalid gain"));
        return;
    }
    proximityGain = gain;
    configureSensors();
    sendOK();
}

void cmdSensorWindow(char* str) {
    uint8_t window = (uint8_t)readUInt(&str);
    proximityWindow = window;
    sendOK();
}

void cmdSensorStatus() {
    send(F("gain: "));
    sendInt(proximityGain);
    sendChar('\n');
    
    send(F("window: "));
    sendInt(proximityWindow);
    sendChar('\n');
    
    send(F("data: "));
    sendInt(proximityData);
    sendChar('\n');
    
    sendOK();
}


// =========== Power commands

void processPowerCommand(char* cmd) {
    switch (cmd[0]) {
        case 'O':
        case 'o':
            cmdPowerOn(cmd + 1);
            break;
        case 'T':
        case 't':
            cmdPowerTime(cmd + 1);
            break;
        case 'S':
        case 's':
            cmdPowerStop(cmd + 1);
            break;
        case '?':
            cmdPowerStatus();
            break;
        default:
            sendError(F("invalid power command"));
            break;
    }
}

void cmdPowerOn(char* str) {
    state = STATE_ON;
    turnOffLights();
    sendOK();
}

void cmdPowerTime(char* str) {
    unsigned time = readUInt(&str);
    powerDownTime = time;
    startPowerDown();
    sendOK();
}

void cmdPowerStop(char* str) {
    if (powerDownTime != -1) {
        turnOffLights();
        powerDownTime = -1;
    }
    turnOnRelays();
    sendOK();
}

void cmdPowerStatus() {
    send(F("state: "));
    sendInt(state);
    sendChar('\n');
    
    send(F("timer: "));
    sendInt(powerDownTime);
    sendChar('\n');
    
    sendOK();
}


// =========== EEPROM commands

void processEEPROMCommand(char* cmd) {
    switch (cmd[0]) {
        case 'C':
        case 'c':
            cmdEEPROMClear(cmd + 1);
            break;
        default:
            sendError(F("invalid EEPROM command"));
            break;
    }
}

void cmdEEPROMClear(char* str) {
    for (int i = 0 ; i < EEPROM.length() ; i++) {
        EEPROM.update(i, 0);
    }
    sendOK();
}


// =========== Other stuff

int readInt(char** strPtr) {
    bool neg = false;
    int i = 0;
    char* str = *strPtr;
    if (*str == '-') {
        str++;
        neg = true;
    }
    while ((*str >= '0') && (*str <= '9')) {
        i = (i * 10) + (*str - '0');
        str++;
    }
    *strPtr = str;
    return neg ? -i : i;
}

unsigned readUInt(char** strPtr) {
    unsigned i = 0;
    char* str = *strPtr;
    while ((*str >= '0') && (*str <= '9')) {
        i = (i * 10) + (*str - '0');
        str++;
    }
    *strPtr = str;
    return i;
}

bool readDelim(char** strPtr) {
    return readDelim(strPtr, ',');
}

bool readDelim(char** strPtr, char delim) {
    char* str = *strPtr;
    if (*str == delim) {
        str++;
        *strPtr = str;
        return true;
    } else {
        return false;
    }
}

color_t readColor(char** strPtr) {
    unsigned r, g, b;
    r = g = b = readUInt(strPtr);
    if (readDelim(strPtr, ':')) {
        g = b = readUInt(strPtr);
        if (readDelim(strPtr, ':'))
            b = readUInt(strPtr);
    }
    return COLOR(r, g, b);
}

uint8_t readSegments(char** strPtr) {
    uint8_t segs = 0;
    uint8_t num;
    for (uint8_t i = 0; i < NUM_SEGMENTS; i++) {
        num = readUInt(strPtr);
        segs |= 1 << num;
        if (! readDelim(strPtr, ':')) {
            break;
        }
    }
    return segs;
}

void send(const char* str) {
    Serial.print(str);
}

void send(const __FlashStringHelper *str) {
    Serial.print(str);
}

void sendChar(char ch) {
    Serial.print(ch);
}

void sendInt(int i) {
    Serial.print(i);
}

void sendOK() {
    send(F("OK\n"));
}

void sendError(const char* msg) {
    sendChar('!');
    send(msg);
    sendChar('\n');
}

void sendError(const __FlashStringHelper *msg) {
    sendChar('!');
    send(msg);
    sendChar('\n');
}

void sendMessage(const char* msg) {
    sendChar('#');
    send(msg);
    sendChar('\n');
}

void sendMessage(const __FlashStringHelper *msg) {
    sendChar('#');
    send(msg);
    sendChar('\n');
}

void sendSensorData() {
    send(F("*S"));
    sendInt(proximityData);
    sendChar('\n');
}

void configureSensors() {
    if (! apdsSetup) return;
    apds.disableLightSensor();
    apds.disableGestureSensor();
    
    apds.setProximityGain(proximityGain);
    apds.enableProximitySensor(false);  // no interrupts, polling only
}

void turnOnRelays() {
    digitalWrite(PIN_RELAY1, LOW);
    digitalWrite(PIN_RELAY2, LOW);
}

void turnOffRelays() {
    digitalWrite(PIN_RELAY1, HIGH);
    digitalWrite(PIN_RELAY2, HIGH);
}

void turnOnLED() {
    digitalWrite(PIN_LED, HIGH);
    ledOn = true;
}

void turnOffLED() {
    digitalWrite(PIN_LED, LOW);
    ledOn = false;
}

void toggleLED() {
    if (ledOn) turnOffLED();
    else turnOnLED();
}

void turnOffLights() {
    prepareLightSegments();
    lights.setSegmentColor(COLOR_OFF, 0);
}
    
void powerUp() {
    send(F("*POWER-UP\n"));
    turnOnRelays();
    playLightPatternSlot(0);
}

void powerDown() {
    send(F("*POWER-DOWN\n"));
    turnOffLights();
    turnOffRelays();
}

void requestPowerDown() {
    send(F("*POWER-REQUEST\n"));
}

void startPowerDown() {
    playLightPatternSlot(1);
}

void prepareLightSegments() {
    NeoPixelPattern* pattern = lights.getPattern(0);
    if (pattern) {
        lights.stop(0);
        free(pattern);
    }
    for (int i = 1; i < NUM_SEGMENTS; i++) {
        if ((segments & (1 << i)) || (segments & 1)) {
            pattern = lights.getPattern(i);
            if (pattern) {
                lights.stop(i);
                free(pattern);
            }
        }
    }
}
    
uint8_t playLightPatternSlot(uint8_t slot) {
    uint8_t ret = loadLightPatternSlot(slot);
    if (ret != ERR_OK) return ret;
    return playLightPattern(inputBuffer.data);
}

uint8_t loadLightPatternSlot(uint8_t slot) {
    for (byte i = 0; ; i++) {
        inputBuffer.data[i] = EEPROM.read(PATTERN_SLOT_BASE + (slot * INPUT_BUFFER_LENGTH) + i);
        if ((i == 0) &&
            ((inputBuffer.data[0] == 255) || (inputBuffer.data[0] == 0))) return ERR_NO_PATTERN;
        if (! inputBuffer.data[i]) break;
    }
    return ERR_OK;
}

uint8_t playLightPattern(char* str) {
    segments = readSegments(&str);
    readDelim(&str);
    byte patNum = (byte)readUInt(&str);
    readDelim(&str);
    
    color_t color1, color2;
    unsigned long interval1 = 0, interval2 = 0;
    uint8_t mode = 0;
    uint8_t direction = 0;
    uint16_t steps = 0;
    uint16_t cooling, sparking;

    prepareLightSegments();
    
    switch (patNum) {
        case 0:
            color1 = readColor(&str);
            readDelim(&str);
            interval1 = (unsigned long)readUInt(&str);
            readDelim(&str);
            mode = (uint8_t)readInt(&str);
            for (int i = 0; i < NUM_SEGMENTS; i++) {
                if (segments & (1 << i)) {
                    WipeNeoPixelPattern* pattern = new WipeNeoPixelPattern();
                    pattern->setup(color1, interval1, mode);
                    lights.play(*pattern, i);
                    if (i == 0) break;
                }
            }
            break;
        case 1:
            color1 = readColor(&str);
            readDelim(&str);
            color2 = readColor(&str);
            readDelim(&str);
            interval1 = (unsigned long)readUInt(&str);
            readDelim(&str);
            interval2 = (unsigned long)readUInt(&str);
            for (int i = 0; i < NUM_SEGMENTS; i++) {
                if (segments & (1 << i)) {
                    BlinkNeoPixelPattern* pattern = new BlinkNeoPixelPattern();
                    pattern->setup(color1, color2, interval1, interval2);
                    lights.play(*pattern, i);
                    if (i == 0) break;
                }
            }
            break;
        case 2:
            interval1 = (unsigned long)readUInt(&str);
            readDelim(&str);
            direction = (uint8_t)readInt(&str);
            for (int i = 0; i < NUM_SEGMENTS; i++) {
                if (segments & (1 << i)) {
                    RainbowNeoPixelPattern* pattern = new RainbowNeoPixelPattern();
                    pattern->setup(interval1, direction);
                    lights.play(*pattern, i);
                    if (i == 0) break;
                }
            }
            break;
        case 3:
            color1 = readColor(&str);
            readDelim(&str);
            color2 = readColor(&str);
            readDelim(&str);
            interval1 = (unsigned long)readUInt(&str);
            readDelim(&str);
            direction = (uint8_t)readInt(&str);
            for (int i = 0; i < NUM_SEGMENTS; i++) {
                if (segments & (1 << i)) {
                    ChaseNeoPixelPattern* pattern = new ChaseNeoPixelPattern();
                    pattern->setup(color1, color2, interval1, direction);
                    lights.play(*pattern, i);
                    if (i == 0) break;
                }
            }
            break;
        case 4:
            color1 = readColor(&str);
            readDelim(&str);
            interval1 = (unsigned long)readUInt(&str);
            for (int i = 0; i < NUM_SEGMENTS; i++) {
                if (segments & (1 << i)) {
                    ScanNeoPixelPattern* pattern = new ScanNeoPixelPattern();
                    pattern->setup(color1, interval1);
                    lights.play(*pattern, i);
                    if (i == 0) break;
                }
            }
            break;
        case 5:
            color1 = readColor(&str);
            readDelim(&str);
            color2 = readColor(&str);
            readDelim(&str);
            steps = (uint16_t)readUInt(&str);
            readDelim(&str);
            interval1 = (unsigned long)readUInt(&str);
            readDelim(&str);
            mode = (uint8_t)readInt(&str);
            for (int i = 0; i < NUM_SEGMENTS; i++) {
                if (segments & (1 << i)) {
                    FadeNeoPixelPattern* pattern = new FadeNeoPixelPattern();
                    pattern->setup(color1, color2, steps, interval1, mode);
                    lights.play(*pattern, i);
                    if (i == 0) break;
                }
            }
            break;
        case 6:
            cooling = (uint16_t)readUInt(&str);
            readDelim(&str);
            sparking = (uint16_t)readUInt(&str);
            readDelim(&str);
            interval1 = (unsigned long)readUInt(&str);
            readDelim(&str);
            direction = (uint8_t)readUInt(&str);
            for (int i = 0; i < NUM_SEGMENTS; i++) {
                if (segments & (1 << i)) {
                    FireNeoPixelPattern* pattern = new FireNeoPixelPattern();
                    pattern->setup(cooling, sparking, interval1, direction);
                    lights.play(*pattern, i);
                    if (i == 0) break;
                }
            }
            break;
            
        default:
            return ERR_INVALID_PATTERN;
    }
    return ERR_OK;
}

/*
void startPowerPattern() {
    segments[0] = false;
    for (byte i = 1; i < NUM_SEGMENTS; i++) {
        segments[i] = true;
    }
    prepareLightSegments();
    for (int i = 1; i < NUM_SEGMENTS; i++) {
        MultiWipeNeoPixelPattern* pattern = new MultiWipeNeoPixelPattern(2);
        pattern->setColor(0, COLOR(32, 32, 32));
        pattern->setColor(1, COLOR(32, 0, 0));
        pattern->setup(pattern->CENTER_OUT + pattern->REPEAT);
        lights.play(*pattern, i);
    }
}
*/
