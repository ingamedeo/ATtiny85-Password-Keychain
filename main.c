/* Name: main.c
 * Project: Password Replay
 * Author: Amedeo Arch
 * Creation Date: 07-02-2015
 * Copyright: Amedeo Arch / (c) 2006 by OBJECTIVE DEVELOPMENT Software GmbH
 * License: Open Source Software
 */

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdbool.h>

#include "usbdrv.h"
#include "oddebug.h"

/*
Pin assignment:
PB1 = key input (active low with pull-up)
PB3 = analog input (ADC3)
PB4 = LED output (active high)

PB0, PB2 = USB data lines
*/

#define BIT_LED 4
#define BIT_KEY 1

#define abs(x) ((x) > 0 ? (x) : (-x))
#define arrayLen(x)  (sizeof(x) / sizeof(x[0]))

#define START_FLAG 100
#define WAIT_FLAG 101

#define USB_WRITE_COMMAND 4

#define UTIL_BIN4(x)        (uchar)((0##x & 01000)/64 + (0##x & 0100)/16 + (0##x & 010)/4 + (0##x & 1))
#define UTIL_BIN8(hi, lo)   (uchar)(UTIL_BIN4(hi) * 16 + UTIL_BIN4(lo))

#ifndef NULL
#define NULL    ((void *)0)
#endif

static uchar    reportBuffer[2];    /* buffer for HID reports */
static uchar    idleRate;           /* in 4 ms units */

volatile static uchar LED_state = 0xff; // received from PC

static uchar string[101]; //'A', 0, 'm', 0, 'E', 0, '1', 0, '0', 0, '7', 0
static uchar receivedProcessed = 0, receivedLength = 0; //Used to receive data from PC

const PROGMEM char usbHidReportDescriptor[USB_CFG_HID_REPORT_DESCRIPTOR_LENGTH] = { /* USB report descriptor */
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x06,                    // USAGE (Keyboard)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
    0x19, 0xe0,                    //   USAGE_MINIMUM (Keyboard LeftControl)
    0x29, 0xe7,                    //   USAGE_MAXIMUM (Keyboard Right GUI)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x95, 0x08,                    //   REPORT_COUNT (8)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x25, 0x65,                    //   LOGICAL_MAXIMUM (101)
    0x19, 0x00,                    //   USAGE_MINIMUM (Reserved (no event indicated))
    0x29, 0x65,                    //   USAGE_MAXIMUM (Keyboard Application)
    0x81, 0x00,                    //   INPUT (Data,Ary,Abs)
    0xc0                           // END_COLLECTION
};
/* We use a simplifed keyboard report descriptor which does not support the
 * boot protocol. We don't allow setting status LEDs and we only allow one
 * simultaneous key press (except modifiers). We can therefore use short
 * 2 byte input reports.
 * The report descriptor has been created with usb.org's "HID Descriptor Tool"
 * which can be downloaded from http://www.usb.org/developers/hidpage/.
 * Redundant entries (such as LOGICAL_MINIMUM and USAGE_PAGE) have been omitted
 * for the second INPUT item.
 */

/* Keyboard usage values, see usb.org's HID-usage-tables document, chapter
 * 10 Keyboard/Keypad Page for more codes.
 */
#define MOD_CONTROL_LEFT    (1<<0)
#define MOD_SHIFT_LEFT      (1<<1)
#define MOD_ALT_LEFT        (1<<2)
#define MOD_GUI_LEFT        (1<<3)
#define MOD_CONTROL_RIGHT   (1<<4)
#define MOD_SHIFT_RIGHT     (1<<5)
#define MOD_ALT_RIGHT       (1<<6)
#define MOD_GUI_RIGHT       (1<<7)

#define KEY_1       30
#define KEY_2       31
#define KEY_3       32
#define KEY_4       33
#define KEY_5       34
#define KEY_6       35
#define KEY_7       36
#define KEY_8       37
#define KEY_9       38
#define KEY_0       39
#define KEY_RETURN  40

 
/* ------------------------------------------------------------------------- */

 static void buildReport(uchar key) {

    /* Send Key + Shift if MAIUSC */
    if(key >= 'A' && key <= 'Z') {
        reportBuffer[0] = MOD_SHIFT_LEFT; //Send modifier
    } else {
        reportBuffer[0] = 0; //No modifiers
    }

    if(key >= 'a' && key <= 'z') {
        reportBuffer[1] = 4+(key-'a'); //Simple calculation to get USB Sendcode
    } else if (key >= 'A' && key <= 'Z') { //Pretty much the same here
        reportBuffer[1] = 4+(key-'A');
    } else if (key >= '0' && key <= '9') { //Number Scancodes
        if (key=='0') { //0 is at the end > 39
            reportBuffer[1] = 39;
        } else {
            reportBuffer[1] = 30+(key-'1');
        }
    } else if (key==' ') {
        reportBuffer[1] = 44;
    } else {
        reportBuffer[1] = key; //Default fallback
    }
}

/* ------------------------------------------------------------------------- */
/* ------------------------ interface to USB driver ------------------------ */
/* ------------------------------------------------------------------------- */

uchar	usbFunctionSetup(uchar data[8]) {
    usbRequest_t    *rq = (void *)data;

    if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS){    /* class request type */
    switch(rq->bRequest) { //-> is short-hand for   (*foo).x
        case USBRQ_HID_GET_REPORT:
            usbMsgPtr = reportBuffer;
            reportBuffer[0] = 0;    /* no modifiers */
            reportBuffer[1] = 0;
            return sizeof(reportBuffer);
        case USBRQ_HID_GET_IDLE:
            usbMsgPtr = &idleRate;
            return 1;
        case USBRQ_HID_SET_IDLE:
            idleRate = rq->wValue.bytes[1];
            return 0;
    }
} else if (rq->bRequest == USB_WRITE_COMMAND) {
    receivedLength = (uchar) rq->wLength.word; //Save length of the data we are receiving from a computer
    receivedProcessed = 0;
    return USB_NO_MSG;
}

return 0;
}

// This gets called when data is sent from PC to the device
uchar usbFunctionWrite(uchar *data, uchar len) {
    uchar i;

    for(i = 0; receivedProcessed < receivedLength && i < len; i++, receivedProcessed++)
        string[receivedProcessed] = data[i];

    if (receivedProcessed == receivedLength) {

        uchar k;
        for (k = receivedLength; k < arrayLen(string); k++) {
            string[k] = 0;
        }

        /* Write data from EEPROM */
        uint16_t stringLength = arrayLen(string);
        eeprom_write_word(2, stringLength);

        uint16_t j;
        for (j = 0; j < stringLength; j++) {
            eeprom_write_byte(j+10, string[j]);
        }

        return 1; // 1 if we received it all, 0 if not
    } else {
        return 0;
    }
}

static void calibrateOscillator(void) {
    int frameLength, targetLength = (unsigned)(1499 * (double)F_CPU / 10.5e6 + 0.5);
    int bestDeviation = 9999;
    uchar trialCal, bestCal, step, region;

    // do a binary search in regions 0-127 and 128-255 to get optimum OSCCAL
    for(region = 0; region <= 1; region++) {
        frameLength = 0;
        trialCal = (region == 0) ? 0 : 128;
        
        for(step = 64; step > 0; step >>= 1) { 
            if(frameLength < targetLength) // true for initial iteration
                trialCal += step; // frequency too low
            else
                trialCal -= step; // frequency too high

            OSCCAL = trialCal;
            frameLength = usbMeasureFrameLength();
            
            if(abs(frameLength-targetLength) < bestDeviation) {
                bestCal = trialCal; // new optimum found
                bestDeviation = abs(frameLength -targetLength);
            }
        }
    }

    OSCCAL = bestCal;
}
/*
Calibration runtime replaced for ATtiny85
*/

void    usbEventResetReady(void) {
    /* Disable interrupts during oscillator calibration since
     * usbMeasureFrameLength() counts CPU cycles.
     */
     cli();
     calibrateOscillator();
     sei();
    eeprom_write_byte(0, OSCCAL);   /* store the calibrated value in EEPROM */
 }

/* ------------------------------------------------------------------------- */
/* --------------------------------- main ---------------------------------- */
/* ------------------------------------------------------------------------- */

 void sendKey(uchar key) {
    wdt_reset();
    usbPoll();

    int cycles = 0;

    while (cycles<2) {
        if (usbInterruptIsReady()) {
            if (cycles==0) {
                buildReport(key);
            } else if (cycles==1) {
                buildReport(0);
            }
            usbSetInterrupt(reportBuffer, sizeof(reportBuffer));
            cycles++;
        }
    }
}

int main(void) {
    int i, button_release_counter = 0, status = WAIT_FLAG;
    bool thisState = false, prevState = false;
    uchar   calibrationValue;

    calibrationValue = eeprom_read_byte(0); /* calibration value from last time */
    if(calibrationValue != 0xff){
        OSCCAL = calibrationValue;
    }

    /* Read data from EEPROM */
    uint16_t stringLength = eeprom_read_word(2);
    uint16_t j;
    for (j = 0; (j < stringLength && j<100); j++) {
        string[j] = eeprom_read_byte(j+10);
    }
    
    usbDeviceDisconnect();
    for(i=0;i<20;i++){  /* 300 ms disconnect */
    _delay_ms(15);
}
usbDeviceConnect();

    DDRB |= 1 << BIT_LED;   /* output for LED */
    PORTB |= 1 << BIT_KEY;  /* pull-up on key input */

wdt_enable(WDTO_1S);

usbInit();
    sei(); //enable interrupts (my comment)

    for(;;){    /* main event loop */

    wdt_reset();
    usbPoll();

    if (status==START_FLAG) {
        int count;
        for (count = 0; count < arrayLen(string); count++) {
            sendKey(string[count]);
        }
        status = WAIT_FLAG;
    }

    thisState = !(PINB & (1<<PB1)); //True when pressed, false otherwise - Next! false
    if (thisState && !prevState) { //True and (!False = True) PASSED - Next! False && False
        status = START_FLAG;
        prevState = !prevState;
        button_release_counter = 0;
    } else if (!thisState && prevState) {
        prevState = !prevState;
    }

if(button_release_counter < 255) {
button_release_counter++; // increase release counter
}

}

return 0;
}
