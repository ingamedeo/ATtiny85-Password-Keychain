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

uint16_t fwVerCode = 1;

/*
Pin assignment:
PB1 = button 1 pin (active low with pull-up)
PB3 = button 2 pin (active low with pull-up)
PB4 = LED pin

PB0, PB2 = USB data lines
*/

#define LED_PIN (1<<PB4)
#define BUTTON1_PIN (1<<PB1)
#define BUTTON2_PIN (1<<PB3)

#define abs(x) ((x) > 0 ? (x) : (-x))
#define arrayLen(x)  (sizeof(x) / sizeof(x[0]))

#define START_FLAG 100
#define WAIT_FLAG 101

#define SKIP_START 9 //Optimized from 15
#define SIZE_BLOCK_START 0 //Start saving the string sizes at block 1 (2nd block and 2 blocks length) (8 blocks 4*2)
#define FIRST_START_BLOCK 8 //If != 1, this is the device's first boot up

#define PW_NUM 4
#define LONG_PRESS_TIMEOUT 200 //In ms/10

#define USB_WRITE_COMMAND 4
#define USB_DATA_OUT 5
#define USB_FWVERDW 6

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

//String.fromCharCode((96 <= key && key <= 105) ? key-48 : key)
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

    receivedLength = receivedLength-1; // (-1 don't save terminator)

    sizeBlock = SIZE_BLOCK_START+(buttonNumber*2); //Select size store block (NOTE! It's a uint16_t - unsigned int - it takes up 2 bytes!)
    startOffset = SKIP_START;

    /* The lengths of this, prev and succ strings */
    uint16_t prevLen = 0;
    uint16_t thisOldLen = eeprom_read_word((uint16_t*)sizeBlock);
    uint16_t nextLen = 0;

    /* If button 1 > (I want to read button 0) -2 > I get button 0 |||| if button 2 > button 0 + button 1 GOOD > PASSED */
    int i;
    for (i = 1; i <= buttonNumber; i++) {
        prevLen = eeprom_read_word((uint16_t*)(SIZE_BLOCK_START+(i*2)-2));
        startOffset = startOffset+prevLen; //I start writing from this block
    }

    for (i = buttonNumber; i < (PW_NUM-1); i++) {
        nextLen = nextLen + eeprom_read_word((uint16_t*)(SIZE_BLOCK_START+(i*2)+2));
    }

    if (nextLen != 0) {
    /* This stuff rewrites the data (starting from the end shifted one position to the right (or left), helps not to override it) */
        int k;
    if (thisOldLen < receivedLength) { //From right to left
        for (k = nextLen-1; k >=0; k--) { //-1? Length is read as < not =<
            uchar currentChar = eeprom_read_byte((uint8_t*)(k+startOffset+thisOldLen));
            if (currentChar != 0xff) {
              eeprom_update_byte((uint8_t*)(k+startOffset+receivedLength), currentChar);
          }
      }
    } else { //From left to right
        for (k = 0; k < nextLen; k++) {
           uchar currentChar = eeprom_read_byte((uint8_t*)(k+startOffset+thisOldLen));
           if (currentChar != 0xff) {
              eeprom_update_byte((uint8_t*)(k+startOffset+receivedLength), currentChar);
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
    for (i = 0; i < PW_NUM; i++) {
        if (currentRewrite != i) {
            sizeTot = sizeTot + eeprom_read_word((uint16_t*)(SIZE_BLOCK_START+(i*2)));
        }
    }
    replyBuf[0] = sizeTot & 0xFF;
    replyBuf[1] = sizeTot >> 8;
    usbMsgPtr = replyBuf;
    return sizeof(replyBuf);
} else if (rq->bRequest == USB_FWVERDW) { /* Send FW Version Info to host */
    replyBuf[0] = fwVerCode & 0xFF;
    replyBuf[1] = fwVerCode >> 8;
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
    eeprom_update_word((uint16_t*)sizeBlock, receivedLength); //Write total length first, we won't read further
    for (i = 0; receivedProcessed < receivedLength && i < len; i++, receivedProcessed++) {
        eeprom_update_byte((uint8_t*)(receivedProcessed+startOffset), data[i]);
    }

    if (receivedProcessed == receivedLength) {
        PORTB &= ~LED_PIN; //Turn LED Off at the end
        return 1; // 1 if we received it all, 0 if not
    } else {
        return 0;
    }
}

void usbEventResetReady(void) {
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

void sendBurstOfData(uint16_t buttonFlag) {
    sizeBlock = SIZE_BLOCK_START+(buttonFlag*2);

        /* Read data from EEPROM */
    uint16_t stringLen = eeprom_read_word((uint16_t*) sizeBlock);

    uint16_t startOffset = SKIP_START;
    uint16_t prevLen = 0;

    int i;
    for (i = 1; i <= buttonFlag; i++) {
        prevLen = eeprom_read_word((uint16_t*)(SIZE_BLOCK_START+(i*2)-2));
        startOffset = startOffset+prevLen; //I start writing from this block
    }
    
    uint16_t count;
    for (count = 0; count < stringLen; count++) {
        uchar currentChar = eeprom_read_byte((uint8_t*)(count+startOffset));
        sendKey(currentChar);
    }

        /* Send terminator manually at the end ;) (We save 1 byte of EEPROM! Yeah) */
    if (stringLen!=0) {
        sendKey('\0');
    }

    for (i = 0; i <= buttonFlag; i++) {
PORTB |= LED_PIN; //Turn LED On
wdt_reset();
_delay_ms(250);
wdt_reset();
PORTB &= ~LED_PIN; //Turn LED Off at the end
_delay_ms(250);
wdt_reset();
    }
    
    }

    int main(void) {
        uint16_t i;
        bool prevState = false, thisState;
        bool prevState2 = false, thisState2;

    /* First Start Check */
        uchar firstStart = eeprom_read_byte((uint8_t*)FIRST_START_BLOCK);
        if (firstStart!=1) {
            int s_i;
            for (s_i = 0; s_i < PW_NUM; s_i++) {
                eeprom_update_word((uint16_t*)(SIZE_BLOCK_START+(s_i*2)), 0);
            }
            eeprom_update_byte((uint8_t*)FIRST_START_BLOCK, 1);
        }
    /* First Start Check END */

        usbDeviceDisconnect();
    for(i=0;i<20;i++){  /* 300 ms disconnect */
        _delay_ms(15);
    }
    usbDeviceConnect();

    DDRB |= LED_PIN;   /* output for LED */
    PORTB |= BUTTON1_PIN;  /* pull-up on key input */
    PORTB |= BUTTON2_PIN; /* UNTESTED */

    wdt_enable(WDTO_1S);

    usbInit();
    sei(); //enable interrupts (my comment)

    for(;;){    /* main event loop */

    wdt_reset();
    usbPoll();

    thisState = !(PINB & BUTTON1_PIN); //True when pressed, false otherwise - Next! false
    if (thisState && !prevState) { //True and (!False = True) PASSED - Next! False && False

        uint16_t inIncrease = 0;
        uint16_t r = 0;
        for (r = 0; r < LONG_PRESS_TIMEOUT; r++) {
            if (!(PINB & BUTTON1_PIN)) {
                inIncrease++;
            } else {
                break;
            }
            _delay_ms(10);
            wdt_reset();
        }

        if (inIncrease>=LONG_PRESS_TIMEOUT) { //We got a long press...
            sendBurstOfData(2);
        } else {
            sendBurstOfData(0);
        }

        prevState = !prevState;
    } else if (!thisState && prevState) {
        prevState = !prevState;
    }

    //Button 2
    thisState2 = !(PINB & BUTTON2_PIN); //True when pressed, false otherwise - Next! false
    if (thisState2 && !prevState2) { //True and (!False = True) PASSED - Next! False && False

        uint16_t inIncrease = 0;
        uint16_t r = 0;
        for (r = 0; r < LONG_PRESS_TIMEOUT; r++) {
            if (!(PINB & BUTTON2_PIN)) {
                inIncrease++;
            } else {
                break;
            }
            _delay_ms(10);
            wdt_reset();
        }

        if (inIncrease>=LONG_PRESS_TIMEOUT) { //We got a long press...
            sendBurstOfData(3);
        } else {
            sendBurstOfData(1);
        }

        prevState2 = !prevState2;
    } else if (!thisState2 && prevState2) {
        prevState2 = !prevState2;
    }

}

return 0;
}