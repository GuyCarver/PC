namespace
{

//Each character uses 5 bytes.
//index using ASCII value * 5.
//Each byte contains a column of pixels.
//The character may be 8 pixels high and 5 wide.

uint8_t	seriffont[] = {6, 8, 31, 127,
	0x20, 0x3E, 0x61, 0x61, 0x3E, 0x20, //Bell icon at 0x31.
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x2F, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x03, 0x00, 0x03, 0x00, 0x00, 0x00,
	0x12, 0x3F, 0x12, 0x3F, 0x12, 0x00,
	0x26, 0x7F, 0x32, 0x00, 0x00, 0x00,
	0x13, 0x0B, 0x34, 0x32, 0x00, 0x00,
	0x1A, 0x25, 0x1A, 0x28, 0x00, 0x00,
	0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x7E, 0x81, 0x00, 0x00, 0x00, 0x00,
	0x81, 0x7E, 0x00, 0x00, 0x00, 0x00,
	0x06, 0x06, 0x00, 0x00, 0x00, 0x00,
	0x08, 0x1C, 0x08, 0x00, 0x00, 0x00,
	0x60, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x08, 0x08, 0x00, 0x00, 0x00, 0x00,
	0x20, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x38, 0x07, 0x00, 0x00, 0x00, 0x00,
	0x1E, 0x21, 0x21, 0x1E, 0x00, 0x00,
	0x02, 0x3F, 0x00, 0x00, 0x00, 0x00,
	0x32, 0x29, 0x29, 0x36, 0x00, 0x00,
	0x12, 0x21, 0x25, 0x1A, 0x00, 0x00,
	0x18, 0x16, 0x3F, 0x10, 0x00, 0x00,
	0x27, 0x25, 0x19, 0x00, 0x00, 0x00,
	0x1E, 0x25, 0x25, 0x18, 0x00, 0x00,
	0x03, 0x39, 0x07, 0x00, 0x00, 0x00,
	0x1A, 0x25, 0x25, 0x1A, 0x00, 0x00,
	0x06, 0x29, 0x29, 0x1E, 0x00, 0x00,
	0x24, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x64, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x08, 0x14, 0x22, 0x00, 0x00, 0x00,
	0x14, 0x14, 0x14, 0x00, 0x00, 0x00,
	0x22, 0x14, 0x08, 0x00, 0x00, 0x00,
	0x02, 0x29, 0x05, 0x02, 0x00, 0x00,
	0x1C, 0x22, 0x49, 0x55, 0x59, 0x12,
	0x30, 0x2C, 0x0B, 0x0B, 0x2C, 0x30,
	0x21, 0x3F, 0x25, 0x25, 0x1A, 0x00,
	0x1E, 0x21, 0x21, 0x21, 0x13, 0x00,
	0x21, 0x3F, 0x21, 0x21, 0x1E, 0x00,
	0x21, 0x3F, 0x25, 0x33, 0x00, 0x00,
	0x21, 0x3F, 0x25, 0x03, 0x00, 0x00,
	0x1E, 0x21, 0x21, 0x29, 0x3B, 0x00,
	0x3F, 0x04, 0x04, 0x3F, 0x00, 0x00,
	0x3F, 0x21, 0x00, 0x00, 0x00, 0x00,
	0x21, 0x1F, 0x01, 0x00, 0x00, 0x00,
	0x21, 0x3F, 0x0C, 0x33, 0x21, 0x00,
	0x3F, 0x21, 0x30, 0x00, 0x00, 0x00,
	0x21, 0x3F, 0x0C, 0x30, 0x0C, 0x3F,
	0x3F, 0x03, 0x0C, 0x3F, 0x01, 0x00,
	0x1E, 0x21, 0x21, 0x21, 0x1E, 0x00,
	0x3F, 0x29, 0x06, 0x00, 0x00, 0x00,
	0x1E, 0x21, 0x21, 0x61, 0x1E, 0x00,
	0x21, 0x3F, 0x09, 0x36, 0x00, 0x00,
	0x32, 0x25, 0x25, 0x1B, 0x00, 0x00,
	0x01, 0x3F, 0x01, 0x00, 0x00, 0x00,
	0x1F, 0x21, 0x20, 0x21, 0x1F, 0x00,
	0x03, 0x0D, 0x30, 0x30, 0x0D, 0x03,
	0x0F, 0x31, 0x0C, 0x0C, 0x31, 0x0F,
	0x21, 0x33, 0x0C, 0x0C, 0x33, 0x21,
	0x03, 0x24, 0x38, 0x24, 0x03, 0x01,
	0x29, 0x25, 0x33, 0x00, 0x00, 0x00,
	0x7F, 0x41, 0x00, 0x00, 0x00, 0x00,
	0x07, 0x38, 0x00, 0x00, 0x00, 0x00,
	0x41, 0x7F, 0x00, 0x00, 0x00, 0x00,
	0x02, 0x01, 0x02, 0x00, 0x00, 0x00,
	0x40, 0x40, 0x40, 0x40, 0x00, 0x00,
	0x00, 0x01, 0x00, 0x00, 0x00, 0x00,
	0x14, 0x24, 0x38, 0x00, 0x00, 0x00,
	0x3F, 0x28, 0x24, 0x18, 0x00, 0x00,
	0x18, 0x24, 0x24, 0x00, 0x00, 0x00,
	0x18, 0x24, 0x25, 0x3F, 0x00, 0x00,
	0x18, 0x24, 0x28, 0x00, 0x00, 0x00,
	0x3E, 0x25, 0x00, 0x00, 0x00, 0x00,
	0x18, 0xA4, 0xA4, 0x7C, 0x00, 0x00,
	0x3F, 0x04, 0x38, 0x00, 0x00, 0x00,
	0x3D, 0x00, 0x00, 0x00, 0x00, 0x00,
	0xFD, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x3F, 0x18, 0x24, 0x00, 0x00, 0x00,
	0x3F, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x3C, 0x04, 0x38, 0x04, 0x38, 0x00,
	0x3C, 0x04, 0x38, 0x00, 0x00, 0x00,
	0x18, 0x24, 0x24, 0x18, 0x00, 0x00,
	0xFC, 0xA4, 0x24, 0x18, 0x00, 0x00,
	0x18, 0x24, 0xA4, 0xFC, 0x00, 0x00,
	0x3C, 0x04, 0x00, 0x00, 0x00, 0x00,
	0x28, 0x24, 0x14, 0x00, 0x00, 0x00,
	0x1E, 0x24, 0x00, 0x00, 0x00, 0x00,
	0x1C, 0x20, 0x3C, 0x00, 0x00, 0x00,
	0x0C, 0x30, 0x0C, 0x00, 0x00, 0x00,
	0x0C, 0x30, 0x0C, 0x30, 0x0C, 0x00,
	0x24, 0x18, 0x24, 0x00, 0x00, 0x00,
	0x9C, 0x60, 0x1C, 0x00, 0x00, 0x00,
	0x34, 0x24, 0x2C, 0x00, 0x00, 0x00,
	0x08, 0x77, 0x00, 0x00, 0x00, 0x00,
	0x7F, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x77, 0x08, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x01, 0x00, 0x00, 0x00, 0x00,
	0x3F, 0x00, 0x00, 0x00, 0x00, 0x00
};
}	//namespace