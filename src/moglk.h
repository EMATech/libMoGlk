/*
 Copyright (C) 2009-2010 Raphaël Doursenaud <rdoursenaud@free.fr>

 This file is part of libmoglk
 a Matrix Orbital Graphical Displays Protocol Library

 libmoglk is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 libmoglk is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with libmoglk. If not, see <http://www.gnu.org/licenses/>.
*/

/*
Source :
Matrix Orbital
GLK19264-7T-1U
Technical Manual
Revision: 1.1
http://www.matrixorbital.ca/manuals/GLK_series/GLK19264-7T-1U/GLK19264-7T-1U.pdf
*/

#ifndef _MOGLK_H
#define _MOGLK_H

/*
  I2C PROTOCOL
	START : Toggle SDA high to low
	Address : 0x50 (write) or 0x51 (read)
	Information : 0x48 0x45 0x4C 0x4C 0x4F
	STOP : Toggle SDA low to high
*/
//TODO:I2C implementation and unavailable commands in this mode

/*
  COMMANDS AND RETURN CODES
	Format :
		Command1/Return1		Code in HEX format
								Parameter1[Range]
								(Comments)
								Parameter2[Range]
								(Comments)
								...
								Parametern[Range]
								(Comments)
		Command1/Return2		Code in HEX format
								...
*/

/*
  GENERAL COMMENTS
	Coordinates are always absolute from the top left corner
	Display is 192x64 so legal values are x[0-191] y[0-63]
*/


// COMMUNICATION
#define CMD_INIT					0xFE //Done
// (Must be issued before any command)
#define CMD_FLOW_CONTROL_ON				0x3A //Done
//										full[0-128]
//										(Number of bytes before almost full
//										 0: full)
//										empty[0-128]
//										(Number of bytes before almost empty
//										 128: empty)

#define RET_ALMOST_FULL					0xFE //Done
#define RET_ALMOST_EMPTY				0xFF //Done

#define CMD_FLOW_CONTROL_OFF			        0x3B //Done

#define CMD_BAUD_RATE					0x39 //Done
//										speed[See table below]

#define BAUD_RATE_9600					0xCF //Done
#define BAUD_RATE_14400					0x8A //Done
#define BAUD_RATE_19200					0x67 //Done
#define BAUD_RATE_28800					0x44 //Done
#define BAUD_RATE_38400					0x33 //Done
#define BAUD_RATE_57600					0x22 //Done
#define BAUD_RATE_76800					0x19 //Done
#define BAUD_RATE_115200					0x10 //Done

#define BAUD_RATE_DEFAULT				BAUD_RATE_19200 //Done

#define CMD_NON_STANDARD_BAUD_RATE		        0xA4 //Done
//										speed[12-2047]
//										(2 bytes,
//										LSB then MSB,
//										Out of bounds value can cause display
//										to stop working
//										Value should be within 3% for the device
//										to communicate)
/*
										Formula :
												 CrystalSpeed (16MHz)
										speed = ---------------------  -  1
												  8 x DesiredBaud
*/

// FONTS
/*
  FONT FILE FORMAT
	Header (4 bytes)
		Nominal Width (1 byte)
		Height (1 byte)
		ASCII Start Value (1 byte)
		ASCII End Value (1 byte)
	Character Table (3 bytes per character)
		High Offset MSB (1 byte)
		Low Offset LSB (1 byte)
		Character Width (1 byte)
	Bitmap Data

Each character is encoded horizontally then padded to form a full last byte
*/
//TODO: implement font file format

#define CMD_UPLOAD_FONT					0x24 //
//										fontid
//										lsbsize
//                                      msbsize
//										fontfile
// (Don't forget to set the new font metrics,
//  see FILESYSTEM file transfer protocol)
#define CMD_USE_FONT					0x31 //Done
//										fontid
#define CMD_FONT_METRICS				0x32 //Done
//										lm
//										(Left Margin: Location in pixels)
//										tm
//										(Top Margin: Location in pixels)
//										csp
//										(Character Spacing in pixels)
//										lsp
//										(Line Spacing in pixels)
//										srow
//										(Scroll Row:
//										Y location of last row in pixels)
#define CMD_BOX_SPACE_MODE				0xAC //Done
//										value[0-1]
//										(0: Off,
//										 1: On)

// TEXT
#define CMD_HOME					0x48 //Done
#define CMD_CURSOR_POSITION				0x47 //Done
//										col
//										row
//										(Derived from current font base size)
#define CMD_CURSOR_COORDINATE			        0x79 //Done
//										x
//										y
#define CMD_AUTO_SCROLL_ON				0x51 //Done
#define CMD_AUTO_SCROLL_OFF				0x52 //Done

// BITMAPS
/*
  BITMAP FILE FORMAT
	The bitmap is encoded into bytes horizontally
*/

#define CMD_UPLOAD_BMP					0x5E //Done
//										bitmapid
//										lsbsize
//                                      msbsize
//										bitmapfile
// (see FILESYSTEM file transfer protocol)
#define CMD_DRAW_MEMORY_BMP				0x62 //Done
//										bitmapid
//										x
//										y
#define CMD_DRAW_BMP					0x64 //Done
//										x
//										y
//										width
//										height
//										data

// BAR GRAPHS & DRAWING
#define CMD_DRAWING_COLOR				0x63 //Done
//										color[0-255]
//										(0: 	White,
//										 1-255: Black)
#define CMD_DRAW_PIXEL					0x70 //Done
//										x
//										y
#define CMD_DRAW_LINE					0x6C //Done
//										x1
//										y1
//										x2
//										y2
// (Lines may interpolate differently Left to Right and Right to Left)
#define CMD_CONTINUE_LINE				0x65 //Done
//										x
//										y
#define CMD_DRAW_RECTANGLE				0x72 //Done
//										color[0-255]
//										(0: 	White,
//										 1-255: Black)
//										x1
//										y1
//										x2
//										y2
#define CMD_DRAW_SOLID_RECTANGLE		        0x78 //Done
//										color[0-255]
//										(0: 	White,
//										 1-255: Black)
//										x1
//										y1
//										x2
//										y2
#define CMD_INITIALIZE_BAR_GRAPH		        0x67 //Done
//										bgid[0-15]
//										type[0-3]
//										(0: Vertical from bottom,
//										 1: Horizontal from left,
//										 2: Vertical from top,
//										 3: Horizontal from right)
//										x1
//										y1
//										x2
//										(x1<x2)
//										y2
//										(y1<y2)
// (Beware of overlapping bar graphs)
#define CMD_DRAW_BAR_GRAPH				0x69 //Done
//										bgid[0-15]
//										value
//										(In pixels)
#define CMD_INITIALIZE_STRIP_CHART		        0x6A //Done
//										scid[0-6]
//										x1
//										y1
//										x2
//										(x1<x2)
//										y2
//										(y1<y2)
#define CMD_SHIFT_STRIP_CHART			        0x6B //Done
//										ref
//										(LSB: scid,
//										 MSB: direction
//										 0: Left,
//										 1: Right)

// GPO
// (Hardwired to 3 tricolor LEDs)
/*
  LEDS LAYOUT
		LED 1 - Top	GPO 2	GPO 1
		LED 2 - Middle	GPO 4	GPO 3
		LED 3 - Bottom	GPO 6	GPO 5
		Yellow		0	0
		Green		0	1
		Red		1	0
		Off		1	1
*/
#define CMD_GPO_OFF					0x56 //Done
//										num[1-6]
#define CMD_GPO_ON					0x57 //Done
//										num[1-6]
#define CMD_STARTUP_GPO_STATE                           0xC3 //Done
//										num[1-6]
//										state[0-1]
//										(0: Off,
//										 1: On)
// (Doesn't affect current state)

// KEYPAD
// Default layout:
#define RET_UP						0x42 //Done
#define RET_DOWN					0x48 //Done
#define RET_LEFT					0x44 //Done
#define RET_RIGHT					0x43 //Done
#define RET_CENTER					0x45 //Done
#define RET_TOP						0x41 //Done
#define RET_BOTTOM					0x47 //Done
#define RET_RELEASE_UP					0x62 //Done
#define RET_RELEASE_DOWN				0x68 //Done
#define RET_RELEASE_LEFT				0x64 //Done
#define RET_RELEASE_RIGHT				0x63 //Done
#define RET_RELEASE_CENTER				0x65 //Done
#define RET_RELEASE_TOP					0x61 //Done
#define RET_RELEASE_BOTTOM				0x67 //Done

#define CMD_AUTO_TRANSMIT_KEY_ON		        0x41 //Done
#define CMD_AUTO_TRANSMIT_KEY_OFF		        0x4F //Done
// (The keypad buffer is reset after 10 key presses)
#define CMD_POLL_KEY					0x26 //Done
// (Returned code MSB flags 'more than one key press in buffer')

#define RET_NO_KEY					0x00 //Done

#define CMD_CLEAR_KEY_BUFFER			        0x45 //Done
#define CMD_DEBOUNCE_TIME				0x55 //Done
//										time[0-255]
//										(6.554ms increments,
//										 Default: 8)
#define CMD_AUTO_REPEAT_MODE			        0x7E //Done
//										mode[0-1]
//										(0: Resend Key,
//										 1: Key Up/Down)
#define CMD_AUTO_REPEAT_OFF				0x60 //Done
#define CMD_CUSTOM_KEYPAD_CODES			        0xD5 //Done
//										kdown[See table bellow]
//										(9 bytes)
//										kup[See table bellow]
//										(9 bytes)
/*
										Byte	Button
										1		Top
										2		Up
										3		Right
										4		Left
										5		Center
										6		N/A
										7		Bottom
										8		Down
										9		N/A
*/
//FIXME: Report it missing from codes list

// DISPLAY
#define CMD_CLEAR_SCREEN				0x58 //Done
#define CMD_DISPLAY_ON					0x42 //Done
//										min[0-90]
#define CMD_DISPLAY_OFF					0x46 //Done
#define CMD_BRIGHTNESS					0x99 //Done
//										brightness[0-255]
//										(Default: 255)
#define CMD_DEFAULT_BRIGHTNESS			        0x98 //Done
//										brightness[0-255]
//										(Default: 255)
#define CMD_CONTRAST					0x50 //Done
//										contrast[0-255]
//										(Default: 128)
#define CMD_DEFAULT_CONTRAST			        0x91 //Done
//										contrast[0-255]
//										(Default: 128)

// FILESYSTEM
/*
  FILE TRANSFER PROTOCOL
	Host									Display
	CMD_INIT
	CMD_UPLOAD(_FS | _FONT | _BMP)
	(parameters)
                                            (RET_CONFIRM | RET_DECLINE)
	CMD_CONFIRM
	-------------------Loop Here----------------------------
	NEXT_BYTE
											NEXT_BYTE
	(CMD_CONFIRM | CMD_DECLINE)
	-------------------End Loop-----------------------------
*/
//TODO: Implement file transfer protocol

#define RET_CONFIRM					0x01    //
#define RET_DECLINE					0x08    //

#define CMD_CONFIRM					0x01    //
#define CMD_DECLINE					0x08    //

#define CMD_WIPE_FILESYSTEM				0x21,0x59,0x21 //Done
// (Be carefull with this one!)
#define CMD_DELETE_FILE					0xAD //Done
//										type[0-1]
//										(0: Font,
//										 1: Bitmap)
//										 fontid or bitmapid
#define CMD_FREE_SPACE					0xAF //Done

/*
  FREE SPACE RETURN FORMAT
	Free space size (4 bytes LSB to MSB)
*/

#define CMD_DIRECTORY					0xB3 //WIP

/*
  DIRECTORY RETURN FORMAT
	Header (1 byte)
		Number of entries (1 byte)1 byte)
		Number of entries (1 byte)
	File Entry (4 bytes)
		Flag (1 byte)
		(0x00: Not used)
		FileID/Type (1 byte)
		(1st bit: File Type
		Next seven bits: FileID)
		File size LSB (1 byte)
		File size MSB (1 byte)
*/

#define CMD_UPLOAD_FS					0xB0 //
//										fsimagefile
//										(Must be 16KB)
#define CMD_DOWNLOAD_FILE				0xB2 //WIP
//										type[0-1]
//										(0: Font,
//										 1: Bitmap)
//										fontid or bitmapid

#define CMD_MOVE_FILE					0xB4 //Done
//										oldtype[0-1]
//										(0: Font,
//										 1: Bitmap)
//										oldid
//										newtype[0-1]
//										(0: Font,
//										 1: Bitmap)
//										newid
#define CMD_DUMP_FS					0x30 //WIP

/*
  DOWNLOAD_FILE AND DUMP_FS RETURN FORMAT
	File size (4 bytes)
	(LSB to MSB)
	File Data
*/

// Undocumented command! Seems to dump the Firmware.
#define CMD_DUMP_FW                                     0xD0 //WIP

// SECURITY
#define CMD_REMEMBER					0x93 //Done
//										value[0-1]
//										(0: Do not remember,
//										 1: Remember)
#define CMD_LOCK_LEVEL					0xCA,0xF5,0xA0 //Done
//										level
//										(Lock bits:
//										 0-2: 	Reserved leave 0,
//										 3: 	Communication speed,
//										 4: 	Settings,
//										 5: 	Filesystem,
//										 6: 	Command,
//										 7: 	Display)
#define CMD_DEFAULT_LOCK_LEVEL			        0xCB,0xF5,0xA0 //Done
//										level
//										(Lock bits:
//										 0-2: 	Reserved leave 0,
//										 3: 	Communication speed,
//										 4: 	Settings,
//										 5: 	Filesystem,
//										 6: 	Command,
//										 7: 	Display)
//FIXME: Report it missing from codes list
#define CMD_WRITE_CUSTOMER_DATA			        0x34 //Done
//										data
//										(16B are accessible)
#define CMD_READ_CUSTOMER_DATA			        0x35 //Done

/*
  READ_CUSTOMER_DATA RETURN FORMAT
	Data (16 bytes)
*/

// MISC
#define CMD_VERSION_NUMBER				0x36 //Done

/*
  VERSION_NUMBER RETURN FORMAT
	Version (1 byte)
	(Represents the version number
	 Hex    Version
	 0x19   1.9
	 0x57   5.7)
*/

#define CMD_MODULE_TYPE					0x37 //Done

/*
  MODULE_TYPE RETURN FORMAT
    Type (1 byte)
	(One of the following return codes)
*/
#define RET_LCD0821					0x01 //Done
#define RET_LCD2021					0x02 //Done
#define RET_LCD2041					0x05 //Done
#define RET_LCD4021					0x06 //Done
#define RET_LCD4041					0x07 //Done
#define RET_LK202_25					0x08 //Done
#define RET_LK204_25					0x09 //Done
#define RET_LK404_55					0x0A //Done
#define RET_VFD2021					0x0B //Done
#define RET_VFD2041					0x0C //Done
#define RET_VFD4021					0x0D //Done
#define RET_VK202_25					0x0E //Done
#define RET_VK204_25					0x0F //Done
#define RET_GLC12232					0x10 //Done
#define RET_GLC24064					0x13 //Done
#define RET_GLK24064_25					0x15 //Done
#define RET_GLK12232_25					0x22 //Done
#define RET_GLK12232_25_SM				0x24 //Done
#define RET_GLK24064_16_1U_USB			        0x25 //Done
#define RET_GLK24064_16_1U				0x26 //Done
#define RET_GLK19264_7T_1U_USB		        	0x27 //Done
#define RET_GLK12236_16					0x28 //Done
#define RET_GLK12232_16_SM				0x29 //Done
#define RET_GLK19264_7T_1U				0x2A //Done
#define RET_LK204_7T_1U					0x2B //Done
#define RET_LK204_7T_1U_USB				0x2C //Done
#define RET_LK404_AT					0x31 //Done
#define RET_MOS_AV_162A					0x32 //Done
#define RET_LK402_12					0x33 //Done
#define RET_LK162_12					0x34 //Done
#define RET_LK204_25PC					0x35 //Done
#define RET_LK202_24_USB				0x36 //Done
#define RET_VK202_24_USB				0x37 //Done
#define RET_LK204_24_USB				0x38 //Done
#define RET_VK204_24_USB				0x39 //Done
#define RET_PK162_12					0x3A //Done
#define RET_VK162_12					0x3B //Done
#define RET_MOS_AP_162A					0x3C //Done
#define RET_PK202_25					0x3D //Done
#define RET_MOS_AL_162A					0x3E //Done
#define RET_MOS_AL_202A					0x3F //Done
#define RET_MOS_AV_202A					0x40 //Done
#define RET_MOS_AP_202A					0x41 //Done
#define RET_PK202_24_USB				0x42 //Done
#define RET_MOS_AL_082					0x43 //Done
#define RET_MOS_AL_204					0x44 //Done
#define RET_MOS_AV_204					0x45 //Done
#define RET_MOS_AL_402					0x46 //Done
#define RET_MOS_AV_402					0x47 //Done
#define RET_LK082_12					0x48 //Done
#define RET_VK402_12					0x49 //Done
#define RET_VK404_55					0x4A //Done
#define RET_LK402_25					0x4B //Done
#define RET_VK402_25					0x4C //Done
#define RET_PK204_25					0x4D //Done
#define RET_MOS						0x4F //Done
#define RET_MOI						0x50 //Done
#define RET_XBOARD_S					0x51 //Done
#define RET_XBOARD_I					0x52 //Done
#define RET_MOU						0x53 //Done
#define RET_XBOARD_U					0x54 //Done
#define RET_LK202_25_USB				0x55 //Done
#define RET_VK202_25_USB				0x56 //Done
#define RET_LK204_25_USB				0x57 //Done
#define RET_VK204_25_USB				0x58 //Done
#define RET_LK162_12_TC					0x5B //Done
#define RET_GLK240128_25				0x72 //Done
#define RET_LK404_25					0x73 //Done
#define RET_VK404_25					0x74 //Done

//Includes
#include <bitset>       // C++ bit structure
//#include <cc++/common.h>//GNU Common C++

//Namespaces
#ifdef	CCXX_NAMESPACES
using namespace std;
using namespace ost;
#endif

// Class definition
class moglk
{
	private:
        bool openPort(char * device_ptr = "/dev/ttyS0");
        void configurePort(void);
        void setPortBaudRate(unsigned long int baud_rate);
        void setPortFlowControl(bool state);

        void transmit(int * data_ptr);
        int receive(void);
        int * receiveFile(int * file_ptr);

        bool upload(char * data_ptr);

	void send(int message[]);

	public:
        moglk(void);
        ~moglk(void);

        bool init(char * device_ptr = "/dev/ttyS0",
                  unsigned long int baud_rate = 0);

        //int autodetect(void);

        //int autodetectPort(void);

        unsigned long int autodetectBaudRate(char * device_ptr = "/dev/ttyS0");

        bool setBaudRate(unsigned long int baud_rate = 19200);

        void setFlowControl(bool state = 0,
                            unsigned char full = 0,
                            unsigned char empty = 0);

        void display(const char text[],
                     unsigned char font = 0,
                     unsigned char x = 255,
                     unsigned char y = 255);

        void goHome(void);

        void setFont(unsigned char id = 1,
                     unsigned char lm = 0,
                     unsigned char tm = 0,
                     unsigned char csp = 0,
                     unsigned char lsp = 1,
                     unsigned char srow = 64);

        void setFontMetrics(unsigned char lm = 0,
                            unsigned char tm = 0,
                            unsigned char csp = 0,
                            unsigned char lsp = 1,
                            unsigned char srow = 64);

        void setBoxSpace(bool mode = 1);

        void setAutoScroll(bool mode = 1);

        void setCursor(unsigned char x,
                       unsigned char y,
                       bool mode = 0);

        void drawMemBmp(unsigned char id = 1,
                        unsigned char x = 0,
                        unsigned char y = 0);

        void setDrawingColor(bool color = 1);

        void drawPixel(unsigned char x,
                       unsigned char y,
                       bool color = 1);

        bool drawLine(unsigned char x1,
                      unsigned char y1,
                      unsigned char x2 = 255,
                      unsigned char y2 = 255,
                      bool color = 1);

        void drawRectangle(unsigned char x1,
                           unsigned char y1,
                           unsigned char x2,
                           unsigned char y2,
                           bool mode = 0,
                           bool color = 1);

        bool initBarGraph(unsigned char x1,
                          unsigned char y1,
                          unsigned char x2,
                          unsigned char y2,
                          unsigned char type = 0,
                          unsigned char id = 0);

        bool drawBarGraph(unsigned char value,
                          unsigned char id = 0);

        bool initStripChart(unsigned char x1,
                            unsigned char y1,
                            unsigned char x2,
                            unsigned char y2,
                            unsigned char id = 0);

        bool shiftStripChart(bool direction = 0,
                             unsigned char id =0);

        bool setGpo(unsigned char id,
                    bool state);

        bool setLed(unsigned char id,
                    unsigned char state);

        bool ledYellow(unsigned char id);

        bool ledGreen(unsigned char id);

        bool ledRed(unsigned char id);

        bool ledOff(unsigned char id);

        bool startupGpo(unsigned char id,
                        bool state);

        void setAutoTxKey(bool state);

        void clearKeyBuffer(void);

        void setDebounce(unsigned char time);

        void setAutoRepeat(bool mode);

        void autoRepeatOff(void);

        void clearScreen(void);

        bool setBacklight(bool state = 1,
                          unsigned char time = 0);

        void setBrightness(unsigned char value = 255);

        void setDefaultBrightness(unsigned char value = 255);

        void setContrast(unsigned char value = 128);

        void setDefaultContrast(unsigned char value = 128);

        void wipeFs(void);

        unsigned short int getFreeSpace(void);

        void setRemember(bool mode = 1);

        void setCustomerData(const char data[15]);

        void getCustomerData(unsigned char data[16]);

        unsigned char getVersion(void);

        unsigned char getModuleType(void);

        void getDirectory(void);

        void deleteFile(bool type,
                        unsigned char id);

        int * downloadFile(bool type,
                           unsigned char id,
                           int * file_ptr);

        int * dumpFs(int * file_ptr);

        int * dumpFw(int * file_ptr);

        void moveFile(bool old_type,
                      unsigned char old_id,
                      bool new_type,
                      unsigned char new_id);

        void setLock(std::bitset<8> level);

        void setDefaultLock(std::bitset<8> level);

        unsigned char pollKey(void);

        void setCustomKeyCodes(unsigned char kup_top,
                               unsigned char kup_up,
                               unsigned char kup_right,
                               unsigned char kup_left,
                               unsigned char kup_center,
                               unsigned char kup_bottom,
                               unsigned char kup_down,
                               unsigned char kdown_top,
                               unsigned char kdown_up,
                               unsigned char kdown_right,
                               unsigned char kdown_left,
                               unsigned char kdown_center,
                               unsigned char kdown_bottom,
                               unsigned char kdown_down);

        void drawBmp(unsigned char x,
                     unsigned char y,
                     unsigned char width,
                     unsigned char height,
                     int * data);

        void uploadFont(unsigned char id,
                        unsigned int size,
                        char * data_ptr);

        void uploadBmp(unsigned char id,
                       unsigned int size,
                       char * data_ptr);

        void uploadFs(unsigned int size,
                      char * data_ptr);

}; /* moglk */

#endif /* #ifndef _MOGLK_H */
