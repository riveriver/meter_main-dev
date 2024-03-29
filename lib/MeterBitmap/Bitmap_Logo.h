#include <Arduino.h>
#ifndef OLED_BITMAP_LOGO_H
#define OLED_BITMAP_LOGO_H

// 'Logo', 128x64px
const unsigned char Open_Logo [] PROGMEM = {
	0x00, 0x00, 0x00, 0xe0, 0x18, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0xe0, 0x18, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0xe0, 0x18, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0xe4, 0x18, 0x30, 0xff, 0x0c, 0xf3, 0xc7, 0x3f, 0xff, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0xee, 0x18, 0x31, 0xff, 0x1c, 0xf3, 0xcf, 0x3f, 0xff, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0xe7, 0x98, 0x33, 0xc3, 0x3c, 0x33, 0xce, 0x00, 0xc3, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x80, 0xe3, 0xd8, 0x37, 0xc3, 0x7c, 0x33, 0xcc, 0x0f, 0xff, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0xc0, 0xe1, 0xf8, 0x3e, 0xc3, 0xec, 0x33, 0xcc, 0x0f, 0xff, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0xe0, 0xe0, 0x78, 0x3c, 0xc3, 0xcc, 0x33, 0xce, 0x00, 0x3b, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x70, 0xe0, 0x38, 0x38, 0xff, 0x8c, 0xf3, 0xcf, 0x3f, 0x73, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x38, 0xe0, 0x18, 0x30, 0xff, 0x0c, 0xf3, 0xc7, 0x3f, 0xe3, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x1c, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x0e, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x07, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x80, 0x03, 0xe0, 0xf8, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0xc0, 0x01, 0xe0, 0xf8, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0xe0, 0x00, 0xe0, 0x18, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x70, 0x00, 0xe0, 0x18, 0x00, 0xff, 0x0c, 0xf3, 0xcf, 0x3f, 0xff, 0x0c, 0xf3, 0xcf, 0x3f, 
	0x00, 0x38, 0x00, 0xe0, 0x18, 0x00, 0xff, 0x1c, 0xf3, 0xcf, 0x3f, 0xff, 0x0c, 0xf3, 0xcf, 0x3f, 
	0x00, 0x1c, 0x00, 0xe0, 0x18, 0x00, 0xc3, 0x3c, 0x33, 0x00, 0x06, 0xc3, 0x0c, 0x33, 0x0c, 0x06, 
	0x00, 0x0e, 0x00, 0xf8, 0x18, 0x00, 0xc3, 0x7c, 0xf3, 0x0f, 0x06, 0xff, 0x0c, 0x33, 0x00, 0x06, 
	0x00, 0x07, 0x00, 0xf8, 0x18, 0x00, 0xc3, 0xec, 0xf3, 0x0f, 0x06, 0xff, 0x0c, 0x33, 0x00, 0x06, 
	0x80, 0x03, 0x00, 0xf8, 0x18, 0x30, 0xc3, 0xcc, 0x03, 0x0c, 0x06, 0x3b, 0x0c, 0x33, 0x0c, 0x06, 
	0xc0, 0x01, 0x00, 0xf8, 0xf8, 0x3f, 0xff, 0x8c, 0xf3, 0x0f, 0x06, 0x73, 0xfc, 0xf3, 0x0f, 0x06, 
	0xe0, 0x00, 0x00, 0xf8, 0xf8, 0x3f, 0xff, 0x0c, 0xf3, 0x0f, 0x06, 0xe3, 0xfc, 0xf3, 0x0f, 0x06, 
	0x70, 0x00, 0xc0, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x20, 0x00, 0xc0, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0xff, 0xff, 0xff, 0xef, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xef, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xef, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0xff, 0x0f, 0x00, 0x00, 0x00, 
	0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 0x00, 0x00, 0xff, 0x0f, 0x00, 0x00, 0x00, 
	0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xdc, 0x01, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 
	0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8e, 0x03, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 
	0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x07, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x0c, 
	0x66, 0x60, 0xfe, 0x33, 0xc0, 0x0c, 0x8c, 0x63, 0x8e, 0xc1, 0xfc, 0x67, 0xfe, 0xf3, 0x1f, 0x0c, 
	0xe6, 0x60, 0xfe, 0x33, 0xc0, 0x1c, 0xcc, 0xa9, 0x9c, 0xe3, 0xfc, 0x67, 0xfe, 0xf3, 0x1f, 0x0c, 
	0xe6, 0x61, 0x06, 0x33, 0xc0, 0x3c, 0xcc, 0x70, 0x98, 0xf7, 0x0c, 0x60, 0x06, 0x30, 0x98, 0x7f, 
	0xe6, 0x63, 0x06, 0x30, 0xc0, 0x7c, 0xcc, 0xa9, 0x9c, 0xff, 0x0c, 0x60, 0x06, 0x30, 0x98, 0x7f, 
	0x66, 0x67, 0x06, 0x30, 0xc0, 0xec, 0x8c, 0x63, 0x8e, 0xdd, 0xfc, 0x61, 0xfe, 0xf0, 0x1f, 0x0c, 
	0x66, 0x6e, 0x06, 0x30, 0xc0, 0xcc, 0x0d, 0x07, 0x87, 0xc9, 0xfc, 0x61, 0xfe, 0xf0, 0x1f, 0x0c, 
	0x66, 0x7c, 0x06, 0x30, 0xc0, 0x8c, 0x0f, 0x8e, 0x83, 0xc1, 0x0c, 0x60, 0x06, 0xb0, 0x03, 0x0c, 
	0x66, 0x78, 0x06, 0x33, 0xc0, 0x0c, 0x0f, 0xdc, 0x81, 0xc1, 0x0c, 0x60, 0x06, 0x30, 0x07, 0x00, 
	0x66, 0x70, 0xfe, 0xf3, 0xcf, 0x0c, 0x0e, 0xf8, 0x80, 0xc1, 0xfc, 0x67, 0xfe, 0x33, 0x0e, 0x00, 
	0x66, 0x60, 0xfe, 0xf3, 0xcf, 0x0c, 0x0c, 0x70, 0x80, 0xc1, 0xfc, 0x67, 0xfe, 0x33, 0x1c, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8c, 0xc1, 0x00, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8c, 0xe1, 0x00, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8c, 0xf1, 0x00, 0x33, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xdc, 0xc1, 0x00, 0x33, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xd8, 0xc0, 0x00, 0x33, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0xc0, 0x00, 0x33, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0xc0, 0x00, 0x33, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0xf0, 0x33, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0xf0, 0x33, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// Array of all bitmaps for convenience. (Total bytes used to store images in PROGMEM = 1040)
const int Open_allArray_LEN = 1;
const unsigned char* Open_allArray[1] = {
	Open_Logo
};




#endif
