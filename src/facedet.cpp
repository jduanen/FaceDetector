/********************************************************************************
 * 
 * Face Detection and Recognition Sensor
 * 
 * N.B. The SEN12131 sensor takes significant time to boot and so it must be
 *      accounted for.
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


bool active = false;
uint8_t detects = 0;
unsigned long activeTime, registerTime;
OprModes mode = DETECT_MODE;
uint8_t faceIdSel;

LEDColors colors[8] = { BLACK, RED, GREEN, YELLOW, BLUE, MAGENTA, CYAN, WHITE };

USPS *usps;

OnBoardLED *userLED;
OnBoardLED *neoPix;

bool npEnable = true;
uint32_t blinkCnt = 0;
const uint32_t MAX_BLINK_CNT = 10;


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

    userLED = new OnBoardLED(PIN_LED_R, PIN_LED_G, PIN_LED_B);
    neoPix = new OnBoardLED(NEOPIXEL_POWER, PIN_NEOPIXEL);

    // start up with User LED Blue and Green NeoPixel
    userLED->setColor(BLUE);
    neoPix->setColor(GREEN);

    Wire.begin();

    // have to wait for the sensor to come up
    int8_t n = -1;
    while (n < 0) {
        USPSface_t faces[1];
        delay(500);
        usps = new USPS();
        n = usps->getFaces(faces, 1);
    }

    // turn off User LED and NeoPixel on boot completion
    userLED->off();
    neoPix->off();
};

void loop() {
    int8_t numFaces;
    USPSface_t faces[MAX_NUM_FACES];

    switch (mode) {
    case REGISTER_MODE:
        if ((millis() - registerTime) < MAX_REGISTER_MS) {
            blink(neoPix, colors[faceIdSel]);
        } else {
            neoPix->off();
            mode = DETECT_MODE;
        }
        break;
    case DETECT_MODE:
        int8_t n = usps->getFaces(faces, MAX_NUM_FACES);
        numFaces = n;
        for (int i = 0; (i < n); i++) {
            if (!faces[i].isFacing || (faces[i].boxConfidence < MIN_CONFIDENCE)) {
                numFaces--;
            }
        }

        if (numFaces < 0) {
            println("ERROR: failed to read, resetting...");
            // turn User LED red to indicate failure of peripheral read
            userLED->setColor(RED);
            neoPix->off();
            usps = new USPS();
            //// TODO decide if I should do a system reset here
            return;
        }
        if (numFaces) {
            detects++;
            if (active == false) {
                if (detects >= DETECT_COUNT) {
                    active = true;
                    activeTime = millis();
                    digitalWrite(ACTIVATE_PIN, HIGH);

                    // turn User LED White to indicate active
                    userLED->on();
                }
            }
        } else {
            detects = 0;
            if (active == true) {
                if ((millis() - activeTime) > MIN_ACTIVE_MS) {
                    active = false;
                    digitalWrite(ACTIVATE_PIN, LOW);

                    // turn off User LED to indicate inactive
                    userLED->off();
                }
            }
        }
        break;
    }

    if (digitalRead(SEL_PIN) == 0) {
        faceIdSel = readSwitches();
        registerTime = millis();
        mode = REGISTER_MODE;
    }

    delay(LOOP_DELAY);
};
