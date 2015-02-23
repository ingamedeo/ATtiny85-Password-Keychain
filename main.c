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

#define LED_PIN (1<<PB4)
#define BUTTON1_PIN (1<<PB1)
#define BUTTON2_PIN (1<<PB3)

#define abs(x) ((x) > 0 ? (x) : (-x))
#define arrayLen(x)  (sizeof(x) / sizeof(x[0]))

#define START_FLAG 100
#define WAIT_FLAG 101

#define SKIP_START 15
#define SIZE_BLOCK_START 1 //Start saving the string sizes at block 1 (2nd block and 2 blocks length) (8 blocks 4*2)
#define FIRST_START_BLOCK 9 //If != 1, this is the device's first boot up

#define BUTTONS_NUM 2

#define USB_WRITE_COMMAND 4
#define USB_DATA_OUT 5

#define UTIL_BIN4(x)        (uchar)((0##x & 01000)/64 + (0##x & 0100)/16 + (0##x & 010)/4 + (0##x & 1))
#define UTIL_BIN8(hi, lo)   (uchar)(UTIL_BIN4(hi) * 16 + UTIL_BIN4(lo))

#ifndef NULL
#define NULL    ((void *)0)
#endif

static uchar    reportBuffer[2];    /* buffer for HID reports */
static uchar    idleRate;           /* in 4 ms units */

volatile static uchar LED_state = 0xff; // received from PC

uint16_t receivedProcessed = 0, receivedLength = 0, buttonNumber, sizeBlock, startOffset; //Used to receive data from PC

static uchar replyBuf[2]; //Buffer to store replies to the PC

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
    receivedLength = (uint16_t) rq->wLength.word; //Save length of the data we are receiving from a computer
    buttonNumber = (uint16_t) rq->wValue.word; //Get button number

    sizeBlock = SIZE_BLOCK_START+(buttonNumber*2); //Select size store block (NOTE! It's a uint16_t - unsigned int - it takes up 2 bytes!)
    startOffset = SKIP_START;

    /* The lengths of this, prev and succ strings */
    uint16_t prevLen = 0;
    uint16_t thisOldLen = eeprom_read_word(sizeBlock);
    uint16_t nextLen = 0;

    if (buttonNumber!=0) {
        prevLen = eeprom_read_word(sizeBlock-2);
        startOffset = startOffset+prevLen; //I start writing from this block
    }

    if (buttonNumber!=(BUTTONS_NUM-1)) {
        nextLen = eeprom_read_word(sizeBlock+2);
    }

    if (nextLen != 0) {
    /* This stuff rewrites the data (starting from the end shifted one position to the right (or left), helps not to override it) */
    int k;
    if (thisOldLen < receivedLength) { //From right to left
        for (k = nextLen-1; k >=0; k--) { //-1? Length is read as < not =<
        uchar currentChar = eeprom_read_byte(k+startOffset+thisOldLen);
        if (currentChar != 0xff) {
          eeprom_update_byte(k+startOffset+receivedLength, currentChar);
        }
    }
    } else { //From left to right
        for (k = 0; k < nextLen; k++) {
       uchar currentChar = eeprom_read_byte(k+startOffset+thisOldLen);
        if (currentChar != 0xff) {
          eeprom_update_byte(k+startOffset+receivedLength, currentChar);
        }
    }
    }
}
    receivedProcessed = 0;
    return USB_NO_MSG;

} else if (rq->bRequest == USB_DATA_OUT) {
    int i;
    uint16_t sizeTot = 0;
    uint16_t currentRewrite = (uint16_t) rq->wValue.word;
    for (i = 0; i < BUTTONS_NUM; i++) {
        if (currentRewrite != i) {
            sizeTot = sizeTot + eeprom_read_word(SIZE_BLOCK_START+(i*2));
        }
    }
    replyBuf[0] = sizeTot & 0xFF;
    replyBuf[1] = sizeTot >> 8;
    usbMsgPtr = replyBuf;
    return sizeof(replyBuf);
}

return 0;
}

// This gets called when data is sent from PC to the device
uchar usbFunctionWrite(uchar *data, uchar len) {
    PORTB |= LED_PIN; //Turn LED On

    uint16_t i;

    /* Write data from EEPROM */
    eeprom_update_word(sizeBlock, receivedLength); //Write total length first, we won't read further
    for (i = 0; receivedProcessed < receivedLength && i < len; i++, receivedProcessed++) {
        eeprom_update_byte(receivedProcessed+startOffset, data[i]);
    }

    if (receivedProcessed == receivedLength) {
        PORTB &= ~LED_PIN; //Turn LED Off at the end
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
    eeprom_update_byte(0, OSCCAL);   /* store the calibrated value in EEPROM */
 }

/* ------------------------------------------------------------------------- */
/* --------------------------------- main ---------------------------------- */
/* ------------------------------------------------------------------------- */

 void sendKey(uchar key) {
    wdt_reset();
    usbPoll();

    uint16_t cycles = 0;

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
    uint16_t i;
    bool prevState = false, thisState;

    bool prevState2 = false, thisState2;

    uchar   calibrationValue;

    calibrationValue = eeprom_read_byte(0); /* calibration value from last time */
    if(calibrationValue != 0xff){
        OSCCAL = calibrationValue;
    }

    /* First Start Check */
    uchar firstStart = eeprom_read_byte(FIRST_START_BLOCK);
    if (firstStart!=1) {
        int s_i;
        for (s_i = 0; s_i < BUTTONS_NUM; s_i++) {
            eeprom_update_word(SIZE_BLOCK_START+(s_i*2), 0);
        }
        eeprom_update_byte(FIRST_START_BLOCK, 1);
    }
    /* First Start Check END */
    
    usbDeviceDisconnect();
    for(i=0;i<20;i++){  /* 300 ms disconnect */
    _delay_ms(15);
}
usbDeviceConnect();

    DDRB |= LED_PIN;   /* output for LED */
    PORTB |= BUTTON1_PIN;  /* pull-up on key input */

wdt_enable(WDTO_1S);

usbInit();
    sei(); //enable interrupts (my comment)

    for(;;){    /* main event loop */

    wdt_reset();
    usbPoll();

    thisState = !(PINB & BUTTON1_PIN); //True when pressed, false otherwise - Next! false
    if (thisState && !prevState) { //True and (!False = True) PASSED - Next! False && False
        
        PORTB |= LED_PIN; //Turn LED On

        /* Read data from EEPROM */
        uint16_t stringLen = eeprom_read_word(SIZE_BLOCK_START);

        uint16_t count;
        for (count = 0; count < stringLen; count++) {
            uchar currentChar = eeprom_read_byte(count+SKIP_START);
            sendKey(currentChar);
        }

        PORTB &= ~LED_PIN; //Turn LED Off at the end

        _delay_ms(50); //Delay just a bit

        prevState = !prevState;
    } else if (!thisState && prevState) {
        prevState = !prevState;
    }

    //Button 2
    thisState2 = !(PINB & BUTTON2_PIN); //True when pressed, false otherwise - Next! false
    if (thisState2 && !prevState2) { //True and (!False = True) PASSED - Next! False && False

        PORTB |= LED_PIN; //Turn LED On

        /* Read data from EEPROM */
        uint16_t stringLen = eeprom_read_word(SIZE_BLOCK_START+2);

        uint16_t startOffset = SKIP_START;
        uint16_t prevLength = eeprom_read_word(SIZE_BLOCK_START);
        if (prevLength != 0xff) {
            startOffset = startOffset+prevLength;
        }

        uint16_t count;
        for (count = 0; count < stringLen; count++) {
            uchar currentChar = eeprom_read_byte(count+startOffset);
            sendKey(currentChar);
        }

        PORTB &= ~LED_PIN; //Turn LED Off at the end

        _delay_ms(50); //Delay just a bit

        prevState2 = !prevState2;
    } else if (!thisState2 && prevState2) {
        prevState2 = !prevState2;
    }

}

return 0;
}
