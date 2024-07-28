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


#define TESTING


#ifdef TESTING
#define     print(val)      Serial.print(val);
#define     println(val)    Serial.println(val);
#define     serialInit()    Serial.begin(115200); while (!Serial) { ; }; delay(500);
#else /* TESTING */
#define     print(val)      ;
#define     println(val)    ;
#define     serialInit()    ;
#endif /* TESTING */

#define MAX_NUM_FACES   3


void (* resetFunc)(void) = 0;


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
const uint32_t MAX_BLINK_CNT = 10;

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
    serialInit();

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

    userLED->setColor(BLUE);
    neoPix->setColor(BLACK);

    Wire.begin();

    // have to wait for the sensor to come up
    int8_t n = -1;
    while (n < 0) {
        userLED->setColor(RED);
        USPSface_t faces[1];
        delay(500);
        usps = new USPS(5.0, MIN_CONFIDENCE, true, false, true);
        n = usps->getFaces(faces, 1);
    }

    userLED->setColor(GREEN);

    if (digitalRead(SEL_PIN) == 0) {
        print("INFO: Erasing registered faces...");
        if (usps->eraseRegisteredFaces()) {
            neoPix->setColor(RED);
            println("Failed");
        } else {
            neoPix->setColor(BLUE);
            println("Done");
        }
        delay(3000);
    }

    // turn off User LED and NeoPixel on boot completion
    userLED->off();
    neoPix->off();
    print("START");
};

void loop() {
    int8_t numFaces;
    USPSface_t faces[MAX_NUM_FACES];

    switch (mode) {
    case REGISTER_MODE:
        println("R");
        active = false;
        blink(neoPix, colors[faceIdSel]);
        if ((millis() - registerTime) < MAX_REGISTER_MS) {
            println("RF");
            if (!usps->isFaceRecEnabled()) {
                if (usps->enableFaceRec(true)) {
                    println("ERROR: failed to enable face recognition")
                    mode = DETECT_MODE;
                    neoPix->off();
                    break;
                }
            }
            if (usps->registerFace(faceIdSel)) {
                print("WARNING: failed to register face ID '");
                print(faceIdSel);
                println("', retrying...");
            } else {
                mode = DETECT_MODE;
                neoPix->off();
                println("DONE");
            }
        } else {
            println("TIMEDOUT");
            neoPix->off();
            mode = DETECT_MODE;
        }
        break;
    case DETECT_MODE:
        int8_t numFaces = usps->getFaces(faces, MAX_NUM_FACES);
        if (numFaces < 0) {
            println("ERROR: failed to read, resetting...");
            // turn User LED red to indicate failure of peripheral read
            userLED->setColor(RED);
            neoPix->off();
            ////usps = new USPS(5.0, DEF_CONFIDENCE, true, false, true);
            //// TODO decide if I should do a system reset here
            setup(); //// FIXME
            return;
        }

        for (int i = 0; (i < numFaces); i++) {
            if (faces[i].idConfidence < MIN_ID_CONFIDENCE) {
                println("WARNING: face ID below confidence level");
            }
            print("! "); print(i); print(", ")
            print(faces[i].id); print(", ");
            print(faces[i].idConfidence); print(", ");
            println(faces[i].isFacing);
            neoPix->setColor(colors[faces[i].id]);
        }

        if (numFaces > 0) {
            detects++;
            if (active == false) {
                if (detects >= DETECT_COUNT) {
                    active = true;
                    activeTime = millis();
                    digitalWrite(ACTIVATE_PIN, HIGH);
                    userLED->on();  // turn User LED White to indicate active
                }
            }
        } else {
            detects = 0;
            if (active == true) {
                if ((millis() - activeTime) > MIN_ACTIVE_MS) {
                    active = false;
                    digitalWrite(ACTIVATE_PIN, LOW);
                    userLED->off();  // turn off User LED to indicate inactive
                }
            }
        }
        print(">> ");println(active);
        break;
    }

    if (digitalRead(SEL_PIN) == 0) {
        faceIdSel = readSwitches();
        registerTime = millis();
        active = false;
        digitalWrite(ACTIVATE_PIN, LOW);
        userLED->off();
        mode = REGISTER_MODE;
    }

    print(".");
    delay(LOOP_DELAY);
};
