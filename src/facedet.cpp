/********************************************************************************
 * 
 * Face Detection and Recognition Sensor
 * 
 * N.B. The SEN12131 sensor takes significant time to boot and so it must be
 *      accounted for.
 * 
 * Usage
 *  - Boot process
 *     * GPIOs, Serial, and I2C initialized: User LED Blue 
 *     * USPS sensor being initialized: User LED Red
 *     * USPS sensor successfully initialized: User LED Green
 *  - Erase saved faces:
 *    * hold Select button, reboot, release when NeoPix lights up
 *      - Blue (3 sec): success
 *      - Red (3 sec): failure
 *  - Exit setup(): User LED and NeoPix off
 *
 ********************************************************************************/

#include "facedet.h"


//#define TESTING


#ifdef TESTING
#define     tprint(val)      Serial.print(val);
#define     tprintln(val)    Serial.println(val);
#else /* TESTING */
#define     tprint(val)      ;
#define     tprintln(val)    ;
#endif /* TESTING */

#define MAX_NUM_FACES   3


void (* resetFunc)(void) = 0;


const uint8_t VERBOSE = 0;
bool active;
uint8_t detects;
unsigned long activeTime, registerTime;
OprModes mode;
uint8_t faceIdSel;

LEDColors colors[8] = { BLACK, RED, GREEN, YELLOW, BLUE, MAGENTA, CYAN, WHITE };

USPS *usps;

OnBoardLED *userLED;
OnBoardLED *neoPix;

bool npEnable;
uint32_t blinkCnt;
const uint32_t MAX_BLINK_CNT = 1000;

uint8_t registeredFaces;


//// FIXME rewrite this
void blink(OnBoardLED *np, LEDColors c) {
    if (npEnable) {
        np->setColor(c);
        if (blinkCnt++ > MAX_BLINK_CNT) {
            npEnable = false;
            blinkCnt = 0;
            np->off();
        }
    } else {
        np->off();
        if (blinkCnt++ > MAX_BLINK_CNT) {
            npEnable = true;
            blinkCnt = 0;
            np->setColor(c);
        }
    }
};

uint8_t readSwitches() {
    uint8_t val = (((digitalRead(PIN_SW2) << 2) & 0x4) |
                   ((digitalRead(PIN_SW1) << 1) & 0x2) |
                   (digitalRead(PIN_SW0) & 0x1));
    return (~val & 0x7);
};

void setup() {
    Serial.begin(115200);
    while (!Serial) { ; };
    delay(500);

    pinMode(ACTIVATE_PIN, OUTPUT);
    digitalWrite(ACTIVATE_PIN, LOW);

    pinMode(SEL_PIN, INPUT_PULLUP);

    pinMode(PIN_SW0, INPUT_PULLUP);
    pinMode(PIN_SW1, INPUT_PULLUP);
    pinMode(PIN_SW2, INPUT_PULLUP);

    active = false;
    detects = 0;
    mode = DETECT_MODE;
    npEnable = true;
    blinkCnt = 0;
    registeredFaces = 0x00;

    userLED = new OnBoardLED(PIN_LED_R, PIN_LED_G, PIN_LED_B);
    neoPix = new OnBoardLED(NEOPIXEL_POWER, PIN_NEOPIXEL);

    userLED->setColor(BLACK);
    neoPix->setColor(BLACK);

    Wire.begin();

    // have to wait for the sensor to come up
    int8_t n = -1;
    while (n < 0) {
        userLED->setColor(BLUE);
        USPSface_t faces[1];
        delay(500);
        usps = new USPS(5.0, MIN_CONFIDENCE, true, false, true);
        n = usps->getFaces(faces, 1);
    }

    userLED->setColor(GREEN);

    if (digitalRead(SEL_PIN) == 0) {
        if (VERBOSE) {
            Serial.print("INFO: Erasing registered faces...");
        }
        if (usps->eraseRegisteredFaces()) {
            neoPix->setColor(RED);
            if (VERBOSE) {
                Serial.println("Failed");
            }
        } else {
            neoPix->setColor(BLUE);
            if (VERBOSE) {
                Serial.println("Done");
            }
        }
        delay(1000);
    }

    // turn off User LED and NeoPixel on boot completion
    userLED->off();
    neoPix->off();
    tprint("START");
};

void loop() {
    int8_t numFaces;
    USPSface_t faces[MAX_NUM_FACES];

    if (digitalRead(SEL_PIN) == 0) {
        faceIdSel = readSwitches();
        registerTime = millis();
        active = false;
        digitalWrite(ACTIVATE_PIN, LOW);
        userLED->off();
        mode = REGISTER_MODE;
    }

    switch (mode) {
    case REGISTER_MODE:
        active = false;
        blink(neoPix, colors[faceIdSel]);
        if ((millis() - registerTime) < MAX_REGISTER_MS) {
            if (!usps->isFaceRecEnabled()) {
                if (usps->enableFaceRec(true)) {
                    if (VERBOSE) {
                        Serial.printf("ERROR: failed to enable face recognition");
                    }
                    mode = DETECT_MODE;
                    neoPix->off();
                    userLED->setColor(CYAN);
                    break;
                }
            }
            if (usps->registerFace(faceIdSel)) {
                if (VERBOSE) {
                    Serial.print("WARNING: failed to register face ID '");
                    Serial.print(faceIdSel);
                    Serial.println("', retrying...");
                }
                userLED->setColor(YELLOW);
            } else {
                mode = DETECT_MODE;
                neoPix->off();
                tprintln("DONE");
            }
        } else {
            neoPix->off();
            mode = DETECT_MODE;
            tprintln("TIMEDOUT");
        }
        break;
    case DETECT_MODE:
        int8_t numFaces = usps->getFaces(faces, MAX_NUM_FACES);

        if (numFaces < 0) {
            if (VERBOSE) {
                Serial.printf("ERROR: failed to read, resetting...");
            }
            userLED->setColor(RED);  // indicate failure of peripheral read
            neoPix->off();
            setup();  // do a complete reset
            return;
        } else if (numFaces == 0) {
            if ((active == true) && ((millis() - activeTime) > MIN_ACTIVE_MS)) {
                active = false;
                detects = 0;
                digitalWrite(ACTIVATE_PIN, LOW);
                userLED->off();
                neoPix->off();
            }
            return;
        }

        detects++;
        if ((active == false) && (detects >= DETECT_COUNT)) {
            active = true;
            activeTime = millis();
            digitalWrite(ACTIVATE_PIN, HIGH);
            userLED->setColor(WHITE);  // indicate detection active
        }

        for (int i = 0; (i < numFaces); i++) {
            if (faces[i].idConfidence < MIN_ID_CONFIDENCE) {
                if (VERBOSE) {
                    Serial.println("WARNING: face ID below confidence level");
                }
                continue;
            }

            //// TODO decide if a checksum is needed/useful
            Serial.printf("%02x %02x %02x %02x\n",
                          i, faces[i].id, faces[i].idConfidence, faces[i].isFacing);
            neoPix->setColor(colors[faces[i].id]);
            userLED->off();
        }
    }

    delay(LOOP_DELAY);
};
