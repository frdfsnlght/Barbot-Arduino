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


// TODO: second relay output
// TODO: flash power up/down patterns
// TODO: combine segments into light pattern/color commands

#include <Arduino.h>
#include <Wire.h>
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
#define PIN_RELAY               4
#define PIN_LIGHTS              5
#define PIN_LED                 13
#define PIN_I2C_DA              A4
#define PIN_I2C_CL              A5

#define NUM_PIXELS              60
#define NUM_SEGMENTS            5

#define INPUT_BUFFER_LENGTH     32

#define LED_TOGGLE_INTERVAL     500
#define SENSOR_READ_INTERVAL    250
#define PROXIMITY_WINDOW        5
#define BUTTON_PRESS_SHORT      1000
#define BUTTON_PRESS_LONG       8000


typedef struct {
    char data[INPUT_BUFFER_LENGTH + 1];
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
bool segments[NUM_SEGMENTS] = {false};

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
uint8_t proximityBoost = LED_BOOST_100;
uint8_t proximityWindow = PROXIMITY_WINDOW;

bool ledOn = false;
uint32_t lastLEDToggle = 0;

bool powerOn = false;
int powerDownTime = -1;
uint32_t lastPowerDownTick = 0;
bool powerDownRequested = false;

bool buttonPressed = false;
uint32_t buttonPressedTime = 0;
bool buttonPressLocked = false;


void setup() {
    Serial.begin(115200, SERIAL_8N1);

    pinMode(PIN_BUTTON, INPUT);
    digitalWrite(PIN_BUTTON, HIGH); // enable pullup
    pinMode(PIN_RELAY, OUTPUT);
    pinMode(PIN_LED, OUTPUT);
    
    turnOffRelays();
    turnOffLED();
    
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
    
    sendMessage("Barbot-Arduino ready");
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
                send("CANCELED\n");
            }
        } else if ((ch >= 32) && (ch <= 126)) {
            if (inputBuffer.length == INPUT_BUFFER_LENGTH) {
                inputBuffer.data[0] = '\0';
                inputBuffer.length = 0;
                sendError("overflow");
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
    
    if (pressed) {
        if (buttonPressLocked) return;
        uint32_t time = millis() - buttonPressedTime;
        if (buttonPressed) {
            if (powerOn && (time >= BUTTON_PRESS_LONG)) {
                powerDown();
            } else if ((! powerDownRequested) && powerOn && (time >= BUTTON_PRESS_SHORT)) {
                requestPowerDown();
            } else if ((! powerOn) && (time >= BUTTON_PRESS_SHORT)) {
                startPowerUp();
            }
        } else {
            buttonPressedTime = millis();
        }
    } else {
        buttonPressed = false;
        buttonPressLocked = false;
        powerDownRequested = false;
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
    
    if ((millis() - lastPowerDownTick) >= 1000) {
        lastPowerDownTick = millis();
        powerDownTime--;
        if (powerDownTime == -1) {
            startPowerDown();
        }
    }
}

void loopLED() {
    if ((millis() - lastLEDToggle) >= LED_TOGGLE_INTERVAL) {
        lastLEDToggle = millis();
        toggleLED();
    }
}

void turnOnRelays() {
    digitalWrite(PIN_RELAY, LOW);
    // TODO: add second relay
}

void turnOffRelays() {
    digitalWrite(PIN_RELAY, HIGH);
    // TODO: add second relay
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
    
void startPowerUp() {
    send("*POWER-UP\n");
    turnOnRelays();
    powerOn = true;
    buttonPressLocked = true;
    // TODO: startPowerPattern();
}

void requestPowerDown() {
    send("*POWER-REQUEST\n");
    powerDownRequested = true;
}

void powerDown() {
    send("*POWER-DOWN\n");
    turnOffLights();
    turnOffRelays();
    powerOn = false;
}

void startPowerDown() {
    // TODO: startPowerPattern();
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
        default:
            sendError("invalid command");
            break;
    }
}


// =========== Light commands

void processLightCommand(char* cmd) {
    switch (cmd[0]) {
        case 'S':
        case 's':
            cmdLightSegments(cmd + 1);
            break;
        case 'C':
        case 'c':
            cmdLightColor(cmd + 1);
            break;
        case 'P':
        case 'p':
            cmdLightPattern(cmd + 1);
            break;
        case '?':
            cmdLightStatus();
            break;
        default:
            sendError("invalid light command");
            break;
    }
}

void cmdLightSegments(char* str) {
    for (int i = 0; i < NUM_SEGMENTS; i++) {
        segments[i] = false;
    }
    for (;;) {
        uint8_t segment = (uint8_t)readUInt(&str);
        if (segment >= NUM_SEGMENTS) {
            sendError("invalid segment");
            return;
        }
        segments[segment] = true;
        if (! readDelim(&str)) break;
    }
    sendOK();
}
    
void cmdLightColor(char* str) {
    color_t color = readColor(&str);
    prepareLightSegments();
    for (int i = 0; i < NUM_SEGMENTS; i++) {
        if (segments[i]) {
            lights.setSegmentColor(color, i);
            if (i == 0) break;
        }
    }
    sendOK();
}

void prepareLightSegments() {
    NeoPixelPattern* pattern = lights.getPattern(0);
    if (pattern) {
        lights.stop(0);
        free(pattern);
    }
    for (int i = 1; i < NUM_SEGMENTS; i++) {
        if (segments[i] || segments[0]) {
            pattern = lights.getPattern(i);
            if (pattern) {
                lights.stop(i);
                free(pattern);
            }
        }
    }
}
    
void cmdLightPattern(char* str) {
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
                if (segments[i]) {
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
                if (segments[i]) {
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
                if (segments[i]) {
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
                if (segments[i]) {
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
                if (segments[i]) {
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
                if (segments[i]) {
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
                if (segments[i]) {
                    FireNeoPixelPattern* pattern = new FireNeoPixelPattern();
                    pattern->setup(cooling, sparking, interval1, direction);
                    lights.play(*pattern, i);
                    if (i == 0) break;
                }
            }
            break;
            
        default:
            sendError("invalid pattern");
            return;
    }
    sendOK();
}

void cmdLightStatus() {
    for (byte i = 0; i < NUM_SEGMENTS; i++) {
        sendChar('S');
        sendInt(i);
        send(": ");
        send(segments[i] ? "X " : "O ");
        sendInt(lights.segmentBasePixel(i));
        sendChar(',');
        sendInt(lights.segmentLength(i));
        send(": ");
        if (lights.isSegmentActive(i)) {
            sendChar('A');
        } else {
            sendChar('-');
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
            sendError("invalid sensor command");
            break;
    }
}

void cmdSensorGain(char* str) {
    unsigned gain = readUInt(&str);
    if ((gain < PGAIN_2X) || (gain > PGAIN_8X)) {
        sendError("invalid gain");
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
    send("gain: ");
    sendInt(proximityGain);
    sendChar('\n');
    
    send("window: ");
    sendInt(proximityWindow);
    sendChar('\n');
    
    send("data: ");
    sendInt(proximityData);
    sendChar('\n');
    
    sendOK();
}


// =========== Power commands

void processPowerCommand(char* cmd) {
    switch (cmd[0]) {
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
            sendError("invalid power command");
            break;
    }
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
    send("power: ");
    send(powerOn ? "on" : "off");
    sendChar('\n');
    
    send("button: ");
    send(buttonPressed ? "pressed" : "open");
    sendChar('\n');
    
    send("timer: ");
    sendInt(powerDownTime);
    sendChar('\n');
    
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

byte readHexDigit(char** strPtr) {
    byte dig;
    char* str = *strPtr;
    if ((*str >= '0') && (*str <= '9'))
        dig = (byte)*str - '0';
    else if ((*str >= 'A') && (*str <= 'F'))
        dig = 10 + (byte)*str - 'A';
    else if ((*str >= 'a') && (*str <= 'f'))
        dig = 10 + (byte)*str - 'a';
    else {
        dig = -1;
        str--;
    }
    str++;
    *strPtr = str;
    return dig;
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
                
void send(const char* str) {
    Serial.print(str);
}

void sendChar(char ch) {
    Serial.print(ch);
}

void sendInt(int i) {
    Serial.print(i);
}

void sendOK() {
    send("OK\n");
}

void sendError(const char* msg) {
    sendChar('!');
    send(msg);
    sendChar('\n');
}

void sendMessage(const char* msg) {
    sendChar('#');
    send(msg);
    sendChar('\n');
}

void sendSensorData() {
    send("*S");
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
