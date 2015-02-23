/**
 * Project: ATtiny85-V-USB-String-Replay
 * License: GNU GPL v3
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

//libusb import
#include <usb.h>

#define USB_DATA_IN 4
#define USB_DATA_OUT 5

 typedef enum { false, true } bool;

// used to get descriptor strings for device identification 
 static int usbGetDescriptorString(usb_dev_handle *dev, int index, int langid, 
 	char *buf, int buflen) {
 	char buffer[256];
 	int rval, i;

	// make standard request GET_DESCRIPTOR, type string and given index 
    // (e.g. dev->iProduct)
 	rval = usb_control_msg(dev, 
 		USB_TYPE_STANDARD | USB_RECIP_DEVICE | USB_ENDPOINT_IN, 
 		USB_REQ_GET_DESCRIPTOR, (USB_DT_STRING << 8) + index, langid, 
 		buffer, sizeof(buffer), 1000);
 	
    if(rval < 0) // error
    	return rval;
    
    // rval should be bytes read, but buffer[0] contains the actual response size
    if((unsigned char)buffer[0] < rval)
		rval = (unsigned char)buffer[0]; // string is shorter than bytes read
	
	if(buffer[1] != USB_DT_STRING) // second byte is the data type
		return 0; // invalid return type
	
	// we're dealing with UTF-16LE here so actual chars is half of rval,
	// and index 0 doesn't count
	rval /= 2;
	
	// lossy conversion to ISO Latin1 
	for(i = 1; i < rval && i < buflen; i++) {
		if(buffer[2 * i + 1] == 0)
			buf[i-1] = buffer[2 * i];
		else
			buf[i-1] = '?'; // outside of ISO Latin1 range
	}
	buf[i-1] = 0;
	
	return i-1;
}

static usb_dev_handle * usbOpenDevice(int vendor, char *vendorName, 
	int product, char *productName) {
	struct usb_bus *bus;
	struct usb_device *dev;
	char devVendor[256], devProduct[256];
	
	usb_dev_handle * handle = NULL;
	
	usb_init();
	usb_find_busses();
	usb_find_devices();
	
	for(bus=usb_get_busses(); bus; bus=bus->next) {
		for(dev=bus->devices; dev; dev=dev->next) {			
			if(dev->descriptor.idVendor != vendor ||
				dev->descriptor.idProduct != product)
				continue;
			
            // we need to open the device in order to query strings 
			if(!(handle = usb_open(dev))) {
				fprintf(stderr, "Warning: Can't open USB device: %s\n",
					usb_strerror());
				continue;
			}
			
            // get vendor name 
			if(usbGetDescriptorString(handle, dev->descriptor.iManufacturer, 0x0409, devVendor, sizeof(devVendor)) < 0) {
				fprintf(stderr, 
					"Warning: Can't get VID for device: %s\n", 
					usb_strerror());
				usb_close(handle);
				continue;
			}
			
            // get product name 
			if(usbGetDescriptorString(handle, dev->descriptor.iProduct, 
				0x0409, devProduct, sizeof(devVendor)) < 0) {
				fprintf(stderr, 
					"Warning: Can't get PID for device: %s\n", 
					usb_strerror());
			usb_close(handle);
			continue;
		}
		
            if(strcmp(devVendor, vendorName) == 0 && strcmp(devProduct, productName) == 0) //Copy ok, return out handle
            	return handle;
            else
            	usb_close(handle);
        }
    }
    
    return NULL;
}

void printHeader() {
	printf("\n");
	printf(" --------------------------------------------\n");
	printf("  ATtiny85 V-USB String Replay Flash Utility\n");
	printf(" --------------------------------------------\n");
	printf("\n");
}

bool check(usb_dev_handle *handle, int* nBytes, int newLen, int buttonNumber) {
	unsigned char buffer[4];
	*nBytes = usb_control_msg(handle, 
		USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, 
		USB_DATA_OUT, buttonNumber, 0, (char*)buffer, sizeof(buffer), 5000);
	uint16_t value = buffer[0] | (buffer[1] << 8);
	printf("Value: %d, newLen: %d\n", value, newLen);
	if ((value + newLen) > 500) {
		return false;
	} else {
		return true;
	}
}

int main(int argc, char **argv) {
	usb_dev_handle *handle = NULL;
	int nBytes = 0;

	if(geteuid() != 0) {
		printHeader();
		printf(" WARNING! You should run this program as root!\n");
		printf("\n");
		exit(1);
	}
	
	if(argc < 2) {
		printHeader();
		printf(" This utility allows you to send data to your ATtiny85 V-USB String Replay device through USB Interface.\n");
		printf("\n");
		printf(" The device should be detected automatically.\n\n > Plug in your device and run:\n");
		printf(" %s write <string>\n", argv[0]);
		printf("\n");
		exit(1);
	}
	
	handle = usbOpenDevice(0x16c0,"ingamedeo.no-ip.org",0x27db,"ATtiny85 USB");
	
	if(handle == NULL) {
		printHeader();
		fprintf(stderr, " Can't connect to your device!\n");
		printf("\n");
		printf(" Double check that it's plugged in or try a different USB port.\n");
		printf("\n");
		exit(1);
	}

	if(strcmp(argv[1], "write") == 0 && argc > 2) {
		printHeader();
		printf(" > Checking... ");
		if (check(handle, &nBytes, strlen(argv[2]), 0)) {
			printf("PASSED!\n");
			printf(" > Writing...\n");
			printf("\n");
			nBytes = usb_control_msg(handle, 
				USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_OUT, 
				USB_DATA_IN, 0, 0, argv[2], strlen(argv[2])+1, 5000);
		} else {
			printf("FAILED! Size exceeded, max size is 500 characters total!\n");
			printf("\n");
			exit(1);
		}
	} else if (strcmp(argv[1], "write2") == 0 && argc > 2) {
		printHeader();
		printf(" > Checking... ");
		if (check(handle, &nBytes, strlen(argv[2]), 1)) {
			printf("PASSED!\n");
			printf(" > Writing...\n");
			printf("\n");
			nBytes = usb_control_msg(handle, 
				USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_OUT, 
				USB_DATA_IN, 1, 0, argv[2], strlen(argv[2])+1, 5000);
		} else {
			printf("FAILED! Size exceeded, max size is 500 characters total!\n");
			printf("\n");
			exit(1);
		}
	}

	if(nBytes < 0) {
		fprintf(stderr, " Error: %s. Please retry\n", usb_strerror());
		printf("\n");
	} else {
		printf(" Enjoy! %lu bytes written.\n", strlen(argv[2]));
		printf("\n");
	}

	usb_close(handle);
	
	return 0;
}

