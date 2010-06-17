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

// C headers
#include <termios.h>	// POSIX Terminal IOs
#include <fcntl.h>	// POSIX File Control Operations

// C++ headers
#include <cstring>      // ISO C99 String handling
#include <iostream>     // C++ input output stream
#include <sstream>      // C++ string stream

// libmoglk header
#include "moglk.h"

//#define NDEBUG

//Namespaces
using namespace std;

int serial_port;

moglk::moglk(void)
{
}

moglk::~moglk(void)
{
        close(serial_port);
}

//TODO: Autodetect device
//FIXME: Use a portable serial I/O library
bool moglk::init(char * device_ptr,
                 unsigned long int baud_rate)
{

    if (!baud_rate)
    {
        baud_rate = autodetectBaudRate(device_ptr);
    }

    if(openPort(device_ptr))
    {
        configurePort();
        setPortBaudRate(baud_rate);
        //setFlowControl(1,8,119);

#ifndef NDEBUG
        cout << "DEBUG init(): libmoglk initialized!" << endl;
#endif /* #ifndef NDEBUG */

        return true;
    }
    else
    {
        return false;
    }

} /* init() */

unsigned long int moglk::autodetectBaudRate(char * device_ptr)
{

        unsigned long int detected_baud_rate = 0;
        unsigned char module_type;
        long int baud_rate[] = {9600, 19200, 38400, 115200, EOF};

        for (unsigned char n = 0;baud_rate[n] != EOF;n++)
        {
                init(device_ptr, baud_rate[n]);
                module_type = getModuleType();
                if (module_type)
                {
                        detectedBaudrate = baud_rate[n];
                }
        }

#ifndef NDEBUG
	cout << "DEBUG autodetectBaudRate() : detectedBaudrate = " << dec << detectedBaudrate << endl;
#endif /* #ifndef NDEBUG */

        return detected_baud_rate;

} /* autodetectBaudRate */


bool moglk::openPort(char * device_ptr)
{
        // Open serial port:
	// read/write, no control terminal
	serial_port = open(device_ptr, O_RDWR | O_NOCTTY);

        // Handle errors
	if (serial_port < 0)
	{
		cerr << "ERROR openPort(): can't open serial port on " << device_ptr << endl;
		return false;
	}
        else
        {
#ifndef NDEBUG
        cout << "DEBUG openPort(): sucessfully opened serial port on " << device_ptr << endl;
#endif /* #ifndef NDEBUG */
        }

        return true;

} /* openPort() */

void moglk::configurePort(void)
{
        // Port options
	struct termios options;

	// Get current port options
	tcgetattr(serial_port, &options);

	// Control options:
	// 8 data bits, no parity, 1 stopbit, local line, enable receiver
	options.c_cflag |= (CS8 | CSTOPB | CLOCAL | CREAD);

	// Local options:
	// Raw input
	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

	// Output options:
	// Raw output
	options.c_oflag &= ~OPOST;

        // Character control options:
        // Timeout after X*0.1 sec
        options.c_cc[VTIME] = 1;
        // No minimum number of bytes received
        options.c_cc[VMIN] = 0;

	// Flush line
	tcflush(serial_port, TCIFLUSH);

	// Set port options
	tcsetattr(serial_port,TCSANOW,&options);

#ifndef NDEBUG
        cout << "DEBUG configurePort()" << endl;
#endif /* #ifndef NDEBUG */

} /* configurePort() */

void moglk::setPortBaudRate(unsigned long int baud_rate)
{
        // Port rate flag
	tcflag_t port_rate;

	// baudrate to rate flag
	switch (baud_rate)
	{
		case 9600:
		{
			port_rate = B9600;
			break;
		}

		case 19200:
		{
			port_rate = B19200;
			break;
		}

		case 38400:
		{
			port_rate = B38400;
			break;
		}

		case 115200:
		{
			port_rate = B115200;
			break;
		}

		default:
		{
			port_rate = B19200;
			break;
		}
	} /* switch(baud_rate) */

        // Port options
	struct termios options;

	// Get current port options
	tcgetattr(serial_port, &options);

	// Control options:
	// global baud rate
	options.c_cflag |= (port_rate);

        // Set input baud rate
	cfsetispeed(&options, port_rate);

	// Set output baud rate
	cfsetospeed(&options, port_rate);

	// Flush line
	tcflush(serial_port, TCIFLUSH);

	// Set port options
	tcsetattr(serial_port,TCSANOW,&options);

#ifndef NDEBUG
        cout << "DEBUG setPortBaudRate(): " << dec << baud_rate << " bauds" << endl;
#endif /* #ifndef NDEBUG */

} /* setPortBaudRate() */

void moglk::setPortFlowControl(bool state)
{
        // Port options
	struct termios options;

	// Get current port options
	tcgetattr(serial_port, &options);

	if (state)
	{
        // Input options:
        // enable flow control
        options.c_iflag |= (IXON | IXOFF);

        // Control characters for flow control
        options.c_cc[VSTART] = RET_ALMOST_EMPTY; // XON
        options.c_cc[VSTOP] = RET_ALMOST_FULL; // XOFF

#ifndef NDEBUG
        cout << "DEBUG setPortFlowControl(): enabled" << endl;
#endif /* #ifndef NDEBUG */

	}
	else
	{
		// Input options:
		// disable flow control
                options.c_iflag &= ~(IXON | IXOFF | IXANY);

#ifndef NDEBUG
                cout << "DEBUG setPortFlowControl(): disabled" << endl;
#endif /* #ifndef NDEBUG */

	}

        // Flush line
	tcflush(serial_port, TCIFLUSH);

	// Set port options
	tcsetattr(serial_port,TCSANOW,&options);

} /* setPortFlowControl() */

void moglk::transmit(int * data_ptr)
{
        ssize_t retval = 0;

        // Write data to the serial port
	while (retval <= 0)
	{
	    retval = write(serial_port,
                       data_ptr,
                       1);
	}

#ifndef NDEBUG
        cout << "DEBUG transmit(): transmited 0x" << hex << (int)*data_ptr << endl;
#endif /* #ifndef NDEBUG */

} /* transmit() */

int moglk::receive(void)
{
        unsigned char byte;

        ssize_t retval = -1;

        retval = read(serial_port,
                      &byte,
                      1);


        if (!retval) 
        {
                clog << "WARNING receive(): read timed out" << endl;
        return -1;
        }
	else
        {
#ifndef NDEBUG
                cout << "DEBUG receive(): received 0x" << hex << (int)byte << endl;
#endif /* #ifndef NDEBUG */
        return byte;
        }

    return -1;

} /* receive() */

void moglk::send(int * message_ptr)
{
        unsigned int n = 0;
	int value = *(message_ptr + n);

	while (value != EOF)
	{
                int * data_ptr;
                data_ptr = &value;
                transmit(data_ptr);
                n++;
                value = *(message_ptr + n);
	}

#ifndef NDEBUG
	cout << "DEBUG send(): sent " << dec << n << " byte(s)" << endl;
#endif /* #ifndef NDEBUG */

} /* send() */

int * moglk::receiveFile(int * file_ptr)
{
    unsigned char size[3];

    unsigned int n = 0;
    while (n < 4)
    {
        size[n] = receive();
        n++;
    }

    unsigned int file_size = size[0] + size[1] * 256 + size[2] * 65536 + size[3] * 16777216;

#ifndef NDEBUG
    cout << "DEBUG receiveFile(): file size = " << dec << file_size << "B" << endl;
#endif /* #ifndef NDEBUG */

if (file_size)
{
    int file[file_size + 1];
    file_ptr = &file[0];

    unsigned long int i = 0;
    while (i < (file_size + 1))
    {
#ifndef NDEBUG
        cout << "DEBUG receiveFile(): byte #" << dec << i << " : ";
#endif /* #ifndef NDEBUG */

        file[i] = receive();
        i++;
    }
file[i] = EOF;

#ifndef NDEBUG
    cout << "DEBUG receiveFile(): received file data = ";
    unsigned long int k = 0;
    while (file[k] != EOF)
    {
        cout << hex << (int)file[k];
        k++;
    }
    cout << endl;
    cout << "DEBUG receiveFile(): " << dec << k - 1 << "B" << endl;
#endif /* #ifndef NDEBUG */
}
else
{
        cerr << "ERROR receiveFile(): file doesn't exist" << endl;
}

        return file_ptr;

} /* receiveFile() */

bool moglk::setBaudRate(unsigned long int baud_rate)
{

	unsigned char device_rate;
	unsigned char non_standard_rate_lsb;
	unsigned char non_standard_rate_msb;
	bool standard = 1;

	switch (baud_rate)
	{
                case 0:
                {
                        device_rate = BAUD_RATE_DEFAULT;
                        break;
                }
		case 9600:
		{
			device_rate = BAUD_RATE_9600;
			break;
		}

		case 14400:
		{
			device_rate = BAUD_RATE_14400;
			clog << "WARNING Linux host doesn't support "<< baud_rate << "bauds rate" << endl;
			break;
		}

		case 19200:
		{
			device_rate = BAUD_RATE_19200;
			break;
		}

		case 28800:
		{
			device_rate = BAUD_RATE_28800;
			clog << "WARNING Linux host doesn't support "<< baud_rate << "bauds rate" << endl;
			break;
		}

		case 38400:
		{
			device_rate = BAUD_RATE_38400;
			break;
		}

		case 57600:
		{
			device_rate = BAUD_RATE_57600;
			clog << "WARNING Linux host doesn't support "<< baud_rate << "bauds rate" << endl;
			break;
		}

		case 76800:
		{
			device_rate = BAUD_RATE_76800;
			clog << "WARNING Linux host doesn't support "<< baud_rate << "bauds rate" << endl;
			break;
		}

		case 115200:
		{
			device_rate = BAUD_RATE_115200;
			break;
		}
		default:
		{
                        standard = 0;
                        if (baud_rate >= 977 && baud_rate <= 153800)
                        {
                                unsigned short int non_standard_rate = (16000000 / 8 * baud_rate) - 1;
                                clog << "WARNING Linux host doesn't support custom rates (" << baud_rate << ")" << endl;
                                non_standard_rate_lsb = non_standard_rate;
                                non_standard_rate_msb = non_standard_rate / 256;
                        }
                        else
                        {
                                clog << "WARNING Device doesn't support " << dec << baud_rate << " bauds rate" << endl;
                        }
			break;
		}

	} /* switch(baud_rate) */

        if (!standard)
        {
#ifndef NDEBUG
                cout << "DEBUG setBaudRate(): non-standard "<< dec << baud_rate << " (0x" << hex << (int)device_rate << ")" << endl;
#endif /* #ifndef NDEBUG */
                if ((non_standard_rate_lsb < 12 && non_standard_rate_msb == 0) || (non_standard_rate_lsb == 0xFF && non_standard_rate_msb > 0x07))
                {
                        int message[] = {CMD_INIT,CMD_NON_STANDARD_BAUD_RATE,non_standard_rate_lsb,non_standard_rate_msb,EOF};
                        send(&message[0]);
                }
                else
                {
                        cerr << "ERROR setBaudRate(): command ignored" << endl;
                        return false;
                }
        }
        else
        {
#ifndef NDEBUG
                cout << "DEBUG setBaudRate(): " << dec << baud_rate << " (0x" << hex << (int)device_rate << ")" << endl;
#endif /* #ifndef NDEBUG */

                int message[] = {CMD_INIT,CMD_BAUD_RATE,device_rate,EOF};
                send(&message[0]);
        }

        setPortBaudRate(baud_rate);

        return true;

} /* setBaudRate */

void moglk::setFlowControl(bool state,
						   unsigned char full,
						   unsigned char empty)
{

	if (state)
	{
#ifndef NDEBUG
		cout << "DEBUG setFlowControl(): on, full at "<< dec << (int)full << " byte(s), empty at " << (int)empty << " bytes(s)" << endl;
#endif /* #ifndef NDEBUG */

		int message[] = {CMD_INIT,CMD_FLOW_CONTROL_ON,full,empty,EOF};
		send(&message[0]);
	}
    else
	{
#ifndef NDEBUG
		cout << "DEBUG setFlowControl(): off" << endl;
#endif /* #ifndef NDEBUG */

		int message[] = {CMD_INIT,CMD_FLOW_CONTROL_OFF,EOF};
		send(&message[0]);
	}

    setPortFlowControl(state);

} /* setFlowControl */

void moglk::display(const char text[],
                    unsigned char font,
                    unsigned char x,
                    unsigned char y)
{


    if (font != 0)
    {
        setFont(font);
    }

    if (x != 255 && y != 255)
    {
        setCursor(x,y);
    }

    unsigned int n = 0;
    unsigned int message_length = strlen(text);
    int message[message_length + 1];
    while (n < message_length + 1)
    {
        message[n] = text[n];
        n++;
    }
    message[n] = EOF;

#ifndef NDEBUG
	cout << "DEBUG display(): \"" << text << "\" of " << dec << n - 1 << " characters" << endl;
#endif /* #ifndef NDEBUG */

	send(&message[0]);

} /* display() */

void moglk::clearScreen(void)
{

#ifndef NDEBUG
	cout << "DEBUG clearScreen()" << endl;
#endif /* #ifndef NDEBUG */

	int message[] = {CMD_INIT,CMD_CLEAR_SCREEN,EOF};
	send(&message[0]);

} /* clearScreen() */

void moglk::goHome(void)
{

#ifndef NDEBUG
	cout << "DEBUG goHome()" << endl;
#endif /* #ifndef NDEBUG */

	int message[] = {CMD_INIT,CMD_HOME,EOF};
	send(&message[0]);

} /* goHome() */

void moglk::setFont(unsigned char id, unsigned char lm, unsigned char tm, unsigned char csp, unsigned char lsp, unsigned char srow)
{

#ifndef NDEBUG
	cout << "DEBUG setFont(): font #" << dec << (int)id << endl;
#endif /* #ifndef NDEBUG */

	int message[] = {CMD_INIT,CMD_USE_FONT,id,EOF};
	send(&message[0]);

	//Don't forget to set the font metrics
	setFontMetrics(lm,tm,csp,lsp,srow);

} /* setFont() */

void moglk::setFontMetrics(unsigned char lm,
                           unsigned char tm,
                           unsigned char csp,
                           unsigned char lsp,
                           unsigned char srow)
{

#ifndef NDEBUG
	cout << "DEBUG setFontMetrics(): left margin " << dec << (int)lm << "px,"
		 << " top margin " << (int)tm << "px,"
		 << " character spacing " << (int)csp << "px,"
		 << " line spacing " << (int)lsp << "px,"
		 << " scroll row " << (int)srow << "px" << endl;
#endif /* #ifndef NDEBUG */

	int message[] = {CMD_INIT,CMD_FONT_METRICS,lm,tm,csp,lsp,srow,EOF};
	send(&message[0]);

} /* setFontMetrics() */

void moglk::setBoxSpace(bool mode)
{

#ifndef NDEBUG
    cout << "DEBUG setBoxSpace(): ";
	if (!mode)
	{
		cout << "off";
	}
	else
	{
		cout << "on";
	}
	cout << endl;
#endif /* #ifndef NDEBUG */

	int message[] = {CMD_INIT,CMD_BOX_SPACE_MODE,mode,EOF};
	send(&message[0]);

} /* setBoxSpace() */

void moglk::setAutoScroll(bool mode)
{

	if (!mode)
	{
#ifndef NDEBUG
		cout << "DEBUG setAutoScroll(): off" << endl;
#endif /* #ifndef NDEBUG */

		int message[] = {CMD_INIT,CMD_AUTO_SCROLL_OFF,EOF};
		send(&message[0]);
    }
    else
    {
#ifndef NDEBUG
		cout << "DEBUG setAutoScroll(): on" << endl;
#endif /* #ifndef NDEBUG */

		int message[] = {CMD_INIT,CMD_AUTO_SCROLL_ON,EOF};
		send(&message[0]);
	}

} /* setAutoScroll() */

void moglk::setCursor(unsigned char x,
                      unsigned char y,
		      bool mode)
{

    if (x < 0 || y < 0)
    {
        cerr << "ERROR setCursor(): coodinates should be positive" << endl;
    }

	if (!mode)
	{
#ifndef NDEBUG
		cout << "DEBUG setCursor(): absolute pixel mode, X=" << dec << (int)x << "px, Y=" << (int)y << "px" << endl;
#endif /* #ifndef NDEBUG */

		int message[] = {CMD_INIT,CMD_CURSOR_COORDINATE,x,y,EOF};
		send(&message[0]);
    }
    else
    {
#ifndef NDEBUG
			cout << "DEBUG setCursor(): absolute character size relative mode, X=" << dec << (int)x << ", Y=" << (int)y << endl;
#endif /* #ifndef NDEBUG */

		int message[] = {CMD_INIT,CMD_CURSOR_POSITION,x,y,EOF};
		send(&message[0]);
	}

} /* setCursor() */

unsigned char moglk::getVersion(void)
{
#ifndef NDEBUG
	cout << "DEBUG getVersion()" << endl;
#endif /* #ifndef NDEBUG */

	int message[] = {CMD_INIT,CMD_VERSION_NUMBER,EOF};
	send(&message[0]);

        unsigned char version;
	version = receive();

#ifndef NDEBUG
    stringstream tmp;
    tmp << hex << (int)version;
    unsigned char v[3];
    tmp >> v;
    v[2] = v[1]; // move second digit right
    v[1] = '.'; // add dot between the two digits
    v[3] = 0; // end string
	cout << "DEBUG getVersion(): " << v << endl;
#endif /* #ifndef NDEBUG */

	return version;

} /* getVersion() */

unsigned char moglk::getModuleType(void)
{

#ifndef NDEBUG
    cout << "DEBUG getModuleType()" << endl;
#endif /* #ifndef NDEBUG */

	int message[] = {CMD_INIT,CMD_MODULE_TYPE,EOF};
	send(&message[0]);

        unsigned char type;
	type = receive();

#ifndef NDEBUG
	cout << "DEBUG getModuleType(): ";
    switch (type)
    {
        case RET_LCD0821:
        {
            cout << "LCD0821";
            break;
        }

        case RET_LCD2021:
        {
            cout << "LCD2021";
            break;
        }

        case RET_LCD2041:
        {
            cout << "LCD2041";
            break;
        }

        case RET_LCD4021:
        {
            cout << "LCD4021";
            break;
        }

        case RET_LCD4041:
        {
            cout << "LCD4041";
            break;
        }

        case RET_LK202_25:
        {
            cout << "LK202-25";
            break;
        }


        case RET_LK204_25:
        {
            cout << "LK204-25";
            break;
        }

        case RET_LK404_55:
        {
            cout << "LK404-55";
            break;
        }

        case RET_VFD2021:
        {
            cout << "VFD2021";
            break;
        }

        case RET_VFD2041:
        {
            cout << "VFD2041";
            break;
        }
        case RET_VFD4021:
        {
            cout << "VFD4021";
            break;
        }

        case RET_VK202_25:
        {
            cout << "VK202-25";
            break;
        }

        case RET_VK204_25:
        {
            cout << "VK204-25";
            break;
        }

        case RET_GLC12232:
        {
            cout << "GLC12232";
            break;
        }

        case RET_GLC24064:
        {
            cout << "GLC24064";
            break;
        }

        case RET_GLK24064_25:
        {
            cout << "GLK24064-25";
            break;
        }

        case RET_GLK12232_25:
        {
            cout << "GLK12232-25";
            break;
        }

        case RET_GLK12232_25_SM:
        {
            cout << "GLK12232-25-SM";
            break;
        }


        case RET_GLK24064_16_1U_USB:
        {
            cout << "GLK24064-16-1U-USB";
            break;
        }

        case RET_GLK24064_16_1U:
        {
            cout << "GLK24064-16-1U";
            break;
        }

        case RET_GLK19264_7T_1U_USB:
        {
            cout << "GLK19264-7T-1U-USB";
            break;
        }

        case RET_GLK12236_16:
        {
            cout << "GLK12236-16";
            break;
        }

        case RET_GLK12232_16_SM:
        {
            cout << "GLK12232-16-SM";
            break;
        }

        case RET_GLK19264_7T_1U:
        {
            cout << "GLK19264-7T-1U";
            break;
        }

        case RET_LK204_7T_1U:
        {
            cout << "LK204-7T-1U";
            break;
        }

        case RET_LK204_7T_1U_USB:
        {
            cout << "LK204-7T-1U-USB";
            break;
        }

        case RET_LK404_AT:
        {
            cout << "LK404-AT";
            break;
        }

        case RET_MOS_AV_162A:
        {
            cout << "MOS-AV-162A";
            break;
        }

        case RET_LK402_12:
        {
            cout << "LK402-12";
            break;
        }

        case RET_LK162_12:
        {
            cout << "LK162-12";
            break;
        }

        case RET_LK204_25PC:
        {
            cout << "LK204-25PC";
            break;
        }

        case RET_LK202_24_USB:
        {
            cout << "LK202-24-USB";
            break;
        }

        case RET_VK202_24_USB:
        {
            cout << "VK202-24-USB";
            break;
        }

        case RET_LK204_24_USB:
        {
            cout << "LK204-24-USB";
            break;
        }

        case RET_VK204_24_USB:
        {
            cout << "VK204-24-USB";
            break;
        }

        case RET_PK162_12:
        {
            cout << "PK162-12";
            break;
        }

        case RET_VK162_12:
        {
            cout << "VK162-12";
            break;
        }

        case RET_MOS_AP_162A:
        {
            cout << "MOS-AP-162A";
            break;
        }

        case RET_PK202_25:
        {
            cout << "PK202-25";
            break;
        }

        case RET_MOS_AL_162A:
        {
            cout << "MOS-AL-162A";
            break;
        }

        case RET_MOS_AL_202A:
        {
            cout << "MOS-AL-202A";
            break;
        }

        case RET_MOS_AV_202A:
        {
            cout << "MOS-AV-202A";
            break;
        }

        case RET_MOS_AP_202A:
        {
            cout << "MOS-AP-202A";
            break;
        }

        case RET_PK202_24_USB:
        {
            cout << "PK202-24-USB";
            break;
        }

        case RET_MOS_AL_082:
        {
            cout << "MOS-AL-082";
            break;
        }

        case RET_MOS_AL_204:
        {
            cout << "MOS-AL-204";
            break;
        }

        case RET_MOS_AV_204:
        {
            cout << "MOS-AV-204";
            break;
        }

        case RET_MOS_AL_402:
        {
            cout << "MOS-AL-402";
            break;
        }

        case RET_MOS_AV_402:
        {
            cout << "MOS-AV-402";
            break;
        }

        case RET_LK082_12:
        {
            cout << "LK082-12";
            break;
        }

        case RET_VK402_12:
        {
            cout << "VK402-12";
            break;
        }

        case RET_VK404_55:
        {
            cout << "VK404-55";
            break;
        }

        case RET_LK402_25:
        {
            cout << "LK402-25";
            break;
        }

        case RET_VK402_25:
        {
            cout << "VK402-25";
            break;
        }

        case RET_PK204_25:
        {
            cout << "PK204-25";
            break;
        }

        case RET_MOS:
        {
            cout << "MOS";
            break;
        }

        case RET_MOI:
        {
            cout << "MOI";
            break;
        }

        case RET_XBOARD_S:
        {
            cout << "XBOARD-S";
            break;
        }

        case RET_XBOARD_I:
        {
            cout << "XBOARD-I";
            break;
        }

        case RET_MOU:
        {
            cout << "MOU";
            break;
        }

        case RET_XBOARD_U:
        {
            cout << "XBOARD-U";
            break;
        }

        case RET_LK202_25_USB:
        {
            cout << "LK202-25-USB";
            break;
        }

        case RET_VK202_25_USB:
        {
            cout << "VK202-25-USB";
            break;
        }

        case RET_LK204_25_USB:
        {
            cout << "LK204-25-USB";
            break;
        }

        case RET_VK204_25_USB:
        {
            cout << "VK204-25-USB";
            break;
        }

        case RET_LK162_12_TC:
        {
            cout << "LK162-12-TC";
            break;
        }

        case RET_GLK240128_25:
        {
            cout << "GLK240128-25";
            break;
        }

        case RET_LK404_25:
        {
            cout << "LK404-25";
            break;
        }

        case RET_VK404_25:
        {
            cout << "VK404-25";
            break;
        }

        default:
        {
            cerr << "ERROR getModuleType: unknown module of type 0x" << hex << (int)type << endl;
            break;
        }
    } /* switch(type) */

    cout << " (0x" << hex << (int)type << ")" << endl;
#endif /* #ifndef NDEBUG */

	return type;

} /* getModuleType() */

//FIXME: Use pointers
void moglk::getCustomerData(unsigned char data[16])
{
#ifndef NDEBUG
	cout << "DEBUG getCustomerData()" << endl;
#endif /* #ifndef NDEBUG */

	int message[] = {CMD_INIT,CMD_READ_CUSTOMER_DATA,EOF};
	send(&message[0]);

    unsigned int n = 0;
    while (n < 16)
    {
        data[n] = receive();
        n++;
    }
    data[n] = 0;

#ifndef NDEBUG
	cout << "DEBUG getCustomerData(): " << data << endl;
#endif /* #ifndef NDEBUG */
} /* getConsumerData() */

void moglk::drawMemBmp(unsigned char id,
                       unsigned char x,
                       unsigned char y)
{

#ifndef NDEBUG
	cout << "DEBUG drawMemBmp(): bitmap #" << dec << (int)id << ", X=" << (int)x << "px ,Y=" << (int)y << "px"<< endl;
#endif /* #ifndef NDEBUG */

	int message[] = {CMD_INIT,CMD_DRAW_MEMORY_BMP,id,x,y,EOF};
	send(&message[0]);

} /* drawMemBmp() */

void moglk::setDrawingColor(bool color)
{

#ifndef NDEBUG
    cout << "DEBUG setDrawingColor(): ";
	if (!color)
	{
        cout << "white";
    }
    else
    {
        cout << "black";
	}
	cout << endl;

#endif /* #ifndef NDEBUG */

	int message[] = {CMD_INIT,CMD_DRAWING_COLOR,color,EOF};
	send(&message[0]);

} /* setDrawingColor() */

void moglk::drawPixel(unsigned char x,
                      unsigned char y,
                      bool color)
{

    setDrawingColor(color);

#ifndef NDEBUG
	cout << "DEBUG drawPixel(): X=" << dec << (int)x << "px, Y=" << (int)y << "px"<< endl;
#endif /* #ifndef NDEBUG */

    int message[] = {CMD_INIT,CMD_DRAW_PIXEL,x,y,EOF};
    send(&message[0]);

} /* drawPixel() */

bool moglk::drawLine(unsigned char x1,
                     unsigned char y1,
                     unsigned char x2,
                     unsigned char y2,
                     bool color)
{

    setDrawingColor(color);

    if (x2 < 255 && y2 < 255)
    {

#ifndef NDEBUG
        cout << "DEBUG drawLine(): X1=" << dec << (int)x1 << "px, Y1=" << (int)y1 << "px, X2=" << (int)x2 << "px, Y2=" << (int)y2 << "px" << endl;
#endif /* #ifndef NDEBUG */

        int message[] = {CMD_INIT,CMD_DRAW_LINE,x1,y1,x2,y2,EOF};
        send(&message[0]);
    }
    if (x2 == 255 || y2 == 255)
    {
            cerr << "ERROR drawLine(): command ignored, only one value of X2 or Y2 set!" << endl;
            return false;
    }
    if (x2 == 255 && y2 == 255)
    {

#ifndef NDEBUG
        cout << "DEBUG drawLine(): continue previous line to X=" << dec << (int)x1 << "px, Y=" << (int)y1 << "px" << endl;
#endif /* #ifndef NDEBUG */

        int message[] = {CMD_INIT,CMD_CONTINUE_LINE,x1,y1,EOF};
        send(&message[0]);
    }

    return true;

} /* drawLine() */

void moglk::drawRectangle(unsigned char x1,
                          unsigned char y1,
                          unsigned char x2,
                          unsigned char y2,
                          bool mode,
                          bool color)
{

    if (!mode)
    {
#ifndef NDEBUG
        cout << "DEBUG drawRectangle(): empty, ";
        if (color) cout << "black";
        else cout << "white";
        cout << ", X1=" << dec << (int)x1 << "px, Y1=" << (int)y1 << "px, X2=" << (int)x2 << "px, Y2=" << (int)y2 << "px" << endl;
#endif /* #ifndef NDEBUG */

        int message[] = {CMD_INIT,CMD_DRAW_RECTANGLE,color,x1,y1,x2,y2,EOF};
        send(&message[0]);
    }
    else
    {
#ifndef NDEBUG
        cout << "DEBUG drawRectangle(): solid, ";
        if (color) cout << "black";
        else cout << "white";
        cout << ", X1=" << dec << (int)x1 << "px, Y1=" << (int)y1 << "px, X2=" << (int)x2 << "px, Y2=" << (int)y2 << "px" << endl;
#endif /* #ifndef NDEBUG */

        int message[] = {CMD_INIT,CMD_DRAW_SOLID_RECTANGLE,color,x1,y1,x2,y2,EOF};
        send(&message[0]);
    }

} /* drawRectangle() */

bool moglk::initBarGraph(unsigned char x1,
                        unsigned char y1,
                        unsigned char x2,
                        unsigned char y2,
                        unsigned char type,
                        unsigned char id)
{

    if (id > 15)
    {
        cerr << "ERROR initBarGraph(): id should be within 0-15" << endl;
        return false;
    }

    if (type > 3)
    {
        cerr << "ERROR initBarGraph(): type should be within 0-3" << endl;
        return false;
    }

    if (x1 > x2)
    {
        cerr << "ERROR initBarGraph(): x1 should be less than x2, please use type to choose the direction" << endl;
        return false;
    }

    if (y1 > y2)
    {
        cerr << "ERROR initBarGraph(): y1 should be less than y2, please use type to choose the direction" << endl;
        return false;
    }

#ifndef NDEBUG
    cout << "DEBUG initBarGraph(): bar graph #" << dec << (int)id << ", ";
    switch (type)
    {
        case 0:
        {
            cout << "vertical from bottom";
            break;
        }

        case 1:
        {
            cout << "horizontal from left";
            break;
        }

        case 2:
        {
            cout << "vertical from top";
            break;
        }

        case 3:
        {
            cout << "horizontal from right";
            break;
        }
    } /* switch(type) */
    cout << ", X1=" << dec << (int)x1 << "px, Y1=" << (int)y1 << "px, X2=" << (int)x2 << "px, Y2=" << (int)y2 << "px" << endl;
#endif /* #ifndef NDEBUG */

    int message[] = {CMD_INIT,CMD_INITIALIZE_BAR_GRAPH,id,type,x1,y1,x2,y2,EOF};
    send(&message[0]);

    return true;

} /* initBarGraph() */

bool moglk::drawBarGraph(unsigned char value,
                        unsigned char id)
{
    if (id > 15)
    {
        cerr << "ERROR drawBarGraph(): id should be within 0-15" << endl;
        return false;
    }

#ifndef NDEBUG
    cout << "DEBUG drawBarGraph(): bar graph #" << dec << (int)id << " to " << (int)value << "px" << endl;
#endif /* #ifndef NDEBUG */

    int message[] = {CMD_INIT,CMD_DRAW_BAR_GRAPH,id,value,EOF};
    send(&message[0]);

    return true;

} /* drawBarGraph() */

bool moglk::initStripChart(unsigned char x1,
                          unsigned char y1,
                          unsigned char x2,
                          unsigned char y2,
                          unsigned char id)
{

    if (id > 6)
    {
        cerr << "ERROR initStripChart(): id should be within 0-6" << endl;
        return false;
    }

    if (x1 > x2)
    {
        cerr << "ERROR initStripChart(): x1 should be less than x2, please use type to choose the direction" << endl;
        return false;
    }

    if (y1 > y2)
    {
        cerr << "ERROR initStripChart(): y1 should be less than y2, please use type to choose the direction" << endl;
        return false;
    }

#ifndef NDEBUG
    cout << "DEBUG initStripChart(): strip chart #" << dec << (int)id << ", X1=" << (int)x1 << ", Y1=" << (int)y1 << ", X2=" << (int)x2 << ", Y2=" << (int)y2 << endl;
#endif /* #ifndef NDEBUG */

    int message[] = {CMD_INIT,CMD_INITIALIZE_STRIP_CHART,id,x1,y1,x2,y2,EOF};
    send(&message[0]);

    return true;

} /* initStripChart() */

bool moglk::shiftStripChart(bool direction,
                           unsigned char id)
{

    if (id > 6)
    {
        cerr << "ERROR initStripChart(): id should be within 0-6" << endl;
        return false;
    }


#ifndef NDEBUG
    cout << "DEBUG shiftStripChart(): strip chart #" << dec << (int)id << ", shifted";
    if (!direction)
    {
        cout << "left";
    }
    else
    {
        cout << "right";
    }
    cout << endl;
#endif /* #ifndef NDEBUG */

    unsigned char ref = id + direction * 128;

    int message[] = {CMD_INIT,CMD_SHIFT_STRIP_CHART,ref,EOF};
    send(&message[0]);

    return true;

} /* shiftStripChart() */

bool moglk::setGpo(unsigned char id,
                  bool state)
{

    if (id == 0 || id > 6)
    {
        cerr << "ERROR setGpo(): id should be within 1-6" << endl;
        return false;
    }

    if (!state)
    {
#ifndef NDEBUG
        cout << "DEBUG setGpo(): GPO #" << dec << (int)id << ", off" << endl;
#endif /* #ifndef NDEBUG */

        int message[] = {CMD_INIT,CMD_GPO_OFF,id,EOF};
        send(&message[0]);
    }
    else
    {
#ifndef NDEBUG
        cout << "DEBUG setGpo(): GPO #" << dec << (int)id << ", on" << endl;
#endif /* #ifndef NDEBUG */

        int message[] = {CMD_INIT,CMD_GPO_ON,id,EOF};
        send(&message[0]);
    }

    return true;

} /* setGpo() */

bool moglk::setLed(unsigned char id,
                  unsigned char state)
{

    if (id == 0 || id > 3)
    {
        cerr << "ERROR setLed(): id should be within 1-3" << endl;
        return false;
    };

    if (state > 3)
    {
        cerr << "ERROR setLed(): state should be within 0-3" << endl;
        return false;
    }

#ifndef NDEBUG
    cout << "DEBUG setLed(): ";
    switch (id)
    {
        case 1:
        {
            cout << "top LED";
            break;
        }

        case 2:
        {
            cout << "middle LED";
            break;
        }

        case 3:
        {
            cout << "bottom LED";
            break;
        }
    } /* swicth(id) */
    cout << ", ";
#endif /* #ifndef NDEBUG */

    switch (state)
    {
        case 0:
        {
#ifndef NDEBUG
            cout << "yellow" << endl;
#endif /* #ifndef NDEBUG */

            setGpo(id * 2 - 1,0);
            setGpo(id * 2,0);
            break;
        }

        case 1:
        {
#ifndef NDEBUG
            cout << "green" << endl;
#endif /* #ifndef NDEBUG */

            setGpo(id * 2 - 1,1);
            setGpo(id * 2,0);
            break;
        }

        case 2:
        {
#ifndef NDEBUG
            cout << "red" << endl;
#endif /* #ifndef NDEBUG */

            setGpo(id * 2 - 1,0);
            setGpo(id * 2,1);
            break;
        }

        case 3:
        {
#ifndef NDEBUG
            cout << "off" << endl;
#endif /* #ifndef NDEBUG */

            setGpo(id * 2 - 1,1);
            setGpo(id * 2,1);
            break;
        }
    } /* switch (state)*/

    return true;

} /* setLed() */

bool moglk::ledYellow(unsigned char id)
{
    if (id == 0 || id > 3)
    {
        cerr << "ERROR ledYellow(): id should be within 1-3" << endl;
        return false;
    }

#ifndef NDEBUG
    cout << "DEBUG ledYellow(): ";
    switch (id)
    {
        case 1:
        {
            cout << "top LED";
            break;
        }

        case 2:
        {
            cout << "middle LED";
            break;
        }

        case 3:
        {
            cout << "bottom LED";
            break;
        }
    } /* switch(id) */
    cout << endl;
#endif /* #ifndef NDEBUG */

    setLed(id,0);

    return true;

} /* ledYellow */

bool moglk::ledGreen(unsigned char id)
{
    if (id == 0 || id > 3)
    {
        cerr << "ERROR ledGreen(): id should be within 1-3" << endl;
        return false;
    }

#ifndef NDEBUG
    cout << "DEBUG ledGreen(): ";
    switch (id)
    {
        case 1:
        {
            cout << "top LED";
            break;
        }

        case 2:
        {
            cout << "middle LED";
            break;
        }

        case 3:
        {
            cout << "bottom LED";
            break;
        }
    } /* switch(id) */
    cout << endl;
#endif /* #ifndef NDEBUG */

    setLed(id,1);

    return true;

} /* ledGreen*/

bool moglk::ledRed(unsigned char id)
{
    if (id == 0 || id > 3)
    {
        cerr << "ERROR ledRed(): id should be within 1-3" << endl;
        return false;
    }

#ifndef NDEBUG
    cout << "DEBUG ledRed(): ";
    switch (id)
    {
        case 1:
        {
            cout << "top LED";
            break;
        }

        case 2:
        {
            cout << "middle LED";
            break;
        }

        case 3:
        {
            cout << "bottom LED";
            break;
        }
    } /* switch(id) */
    cout << endl;
#endif /* #ifndef NDEBUG */

    setLed(id,2);

    return true;

} /* ledRed */

bool moglk::ledOff(unsigned char id)
{
    if (id == 0 || id > 3)
    {
        cerr << "ERROR ledOff(): id should be within 1-3" << endl;
        return false;
    }

#ifndef NDEBUG
    cout << "DEBUG ledOff(): ";
    switch (id)
    {
        case 1:
        {
            cout << "top LED";
            break;
        }

        case 2:
        {
            cout << "middle LED";
            break;
        }

        case 3:
        {
            cout << "bottom LED";
            break;
        }
    } /* switch(id) */
    cout << endl;
#endif /* #ifndef NDEBUG */

    setLed(id,3);

    return true;

} /* ledOff */

bool moglk::startupGpo(unsigned char id,
                      bool state)
{

    if (id == 0 || id > 6)
    {
        cerr << "ERROR startupGpo(): id should be within 1-6" << endl;
        return false;
    }

#ifndef NDEBUG
    cout << "DEBUG startupGpo(): GPO #" << dec << (int)id << ", ";
    if (!state)
    {
        cout << "off";
    }
    else
    {
        cout << "on";
    }
    cout << endl;
#endif /* #ifndef NDEBUG */

    int message[] = {CMD_INIT,CMD_STARTUP_GPO_STATE,id,state,EOF};
    send(&message[0]);

    return true;

} /* startupGpo() */

void moglk::setAutoTxKey(bool state)
{

#ifndef NDEBUG
    cout << "DEBUG setAutoTxKey(): ";
#endif /* #ifndef NDEBUG */

    if (!state)
    {
#ifndef NDEBUG
        cout << "off" << endl;
#endif /* #ifndef NDEBUG */

        int message[] = {CMD_INIT,CMD_AUTO_TRANSMIT_KEY_OFF,EOF};
        send(&message[0]);
    }
    else
    {
#ifndef NDEBUG
       cout << "on" << endl;
#endif /* #ifndef NDEBUG */

       int message[] = {CMD_INIT,CMD_AUTO_TRANSMIT_KEY_ON,EOF};
       send(&message[0]);
    }

} /* setAutotxKey() */

bool moglk::setBacklight(bool state,
                        unsigned char time)
{
    if (time > 90)
    {
        cerr << "ERROR setBacklight(): time must be less than 90 minutes" << endl;
        return false;
    }

#ifndef NDEBUG
    cout << "DEBUG setBacklight(): ";
#endif /* #ifndef NDEBUG */

    if (!state)
    {
#ifndef NDEBUG
        cout << "off" << endl;
#endif /* #ifndef NDEBUG */
        if (time)
        {
            clog << "WARNING setBacklight(): unused time value only available for set backlight on" << endl;
        };

        int message[] = {CMD_INIT,CMD_DISPLAY_OFF,EOF};
        send(&message[0]);
    }
    else
    {
#ifndef NDEBUG
        cout << "on in " << dec << (int)time << " minute(s)"<< endl;
#endif /* #ifndef NDEBUG */

        int message[] = {CMD_INIT,CMD_DISPLAY_ON,time,EOF};
        send(&message[0]);
    }

    return true;

} /* setBacklight() */

void moglk::clearKeyBuffer(void)
{

#ifndef NDEBUG
    cout << "DEBUG clearKeyBuffer()" << endl;
#endif /* #ifndef NDEBUG */

    int message[] = {CMD_INIT,CMD_CLEAR_KEY_BUFFER,EOF};
    send(&message[0]);

} /* clearKeyBuffer() */

void moglk::setDebounce(unsigned char time)
{
#ifndef NDEBUG
    cout << "DEBUG setDebounce(): " << dec << (int)time * 8 << "ms" << endl;
#endif /* #ifndef NDEBUG */

    int message[] = {CMD_INIT,CMD_DEBOUNCE_TIME,time,EOF};
    send(&message[0]);


} /* setDebounce() */

void moglk::setAutoRepeat(bool mode)
{
#ifndef NDEBUG
    cout << "DEBUG setAutoRepeat(): ";
    if (!mode)
    {
        cout << "resend key";
    }
    else
    {
        cout << "key up/key down";
    }
    cout << endl;
#endif /* #ifndef NDEBUG */

    int message[] = {CMD_INIT,CMD_AUTO_REPEAT_MODE,mode,EOF};
    send(&message[0]);

} /* setAutoRepeat() */

void moglk::autoRepeatOff(void)
{
#ifndef NDEBUG
    cout << "DEBUG autoRepeatOff()" << endl;
#endif /* #ifndef NDEBUG */

    int message[] = {CMD_INIT,CMD_AUTO_REPEAT_OFF,EOF};
    send(&message[0]);

} /* autoRepeatOff() */

void moglk::setBrightness(unsigned char value)
{
#ifndef NDEBUG
    cout << "DEBUG setBrightness(): " << dec << (int)value << endl;
#endif /* #ifndef NDEBUG */

    int message[] = {CMD_INIT,CMD_BRIGHTNESS,value,EOF};
    send(&message[0]);

} /* setBrightness() */

void moglk::setDefaultBrightness(unsigned char value)
{
#ifndef NDEBUG
    cout << "DEBUG setDefaultBrightness(): " << dec << (int)value << endl;
#endif /* #ifndef NDEBUG */

    int message[] = {CMD_INIT,CMD_DEFAULT_BRIGHTNESS,value,EOF};
    send(&message[0]);

} /* setDefaultBrightness() */

void moglk::setContrast(unsigned char value)
{
#ifndef NDEBUG
    cout << "DEBUG setContrast(): " << dec << (int)value << endl;
#endif /* #ifndef NDEBUG */

    int message[] = {CMD_INIT,CMD_CONTRAST,value,EOF};
    send(&message[0]);

} /* setContrast() */

void moglk::setDefaultContrast(unsigned char value)
{
#ifndef NDEBUG
    cout << "DEBUG setDefaultContrast(): " << dec << (int)value << endl;
#endif /* #ifndef NDEBUG */

    int message[] = {CMD_INIT,CMD_DEFAULT_CONTRAST,value,EOF};
    send(&message[0]);

} /* setDefaultContrast() */

void moglk::wipeFs(void)
{
#ifndef NDEBUG
    cout << "DEBUG wipeFs()" << endl;
#endif /* #ifndef NDEBUG */

    int message[] = {CMD_INIT,CMD_WIPE_FILESYSTEM,EOF};
    send(&message[0]);

} /* wipeFs() */

unsigned short int moglk::getFreeSpace(void)
{
#ifndef NDEBUG
    cout << "DEBUG getFreeSpace()" << endl;
#endif /* #ifndef NDEBUG */

    int message[] = {CMD_INIT,CMD_FREE_SPACE,EOF};
    send(&message[0]);

    unsigned char size[3];

    unsigned int n = 0;
    while (n < 4)
    {
        size[n] = receive();
        n++;
    }

    unsigned short int free_space = size[0] + size[1] * 256 + size[2] * 65536 + size[3] * 16777216;

#ifndef NDEBUG
    cout << "DEBUG getFreeSpace(): " << dec << free_space << "B" << endl;
#endif /* #ifndef NDEBUG */

    return free_space;

} /* getFreeSpace() */

void moglk::setRemember(bool mode)
{
#ifndef NDEBUG
    cout << "DEBUG setRemember(): ";
    if (!mode)
    {
        cout << "off";
    }
    else
    {
        cout << "on";
    }
    cout << endl;
#endif /* #ifndef NDEBUG */

    int message[] = {CMD_INIT,CMD_REMEMBER,mode,EOF};
    send(&message[0]);

} /* setRemember() */

//FIXME: use pointers
void moglk::setCustomerData(const char data[15])
{
#ifndef NDEBUG
    cout << "DEBUG setCustomerData(): " << data << endl;
#endif /* #ifndef NDEBUG */

    int n = 0;
    int message_length = strlen(data);
    int message[message_length+1];
    while (n < message_length+1)
    {
        message[n] = data[n];
        n++;
    }
    message[n] = EOF;
    send(&message[0]);

} /* setCustomerData() */

//FIXME: return a structure with directory informations
void moglk::getDirectory(void)
{
#ifndef NDEBUG
    cout << "DEBUG getDirectory()" << endl;
#endif /* #ifndef NDEBUG */

    int message[] = {CMD_INIT,CMD_DIRECTORY,EOF};
    send(&message[0]);

    unsigned char entriesCount;

    entriesCount = receive();

#ifndef NDEBUG
    cout << "DEBUG getDirectory(): " << dec << (int)entriesCount << " files" << endl;
#endif /* #ifndef NDEBUG */

    for (unsigned int i = 0;i < entriesCount;i++)
    {
        unsigned char flag;
        bitset<8> id;
        unsigned char size_lsb;
        unsigned char size_msb;

        // Get data
        flag = receive();
        id = receive();
        size_lsb = receive();
        size_msb = receive();

        // Extract type from id (first bit)
        bool type = id[7];
        // Strip type from id (first bit)
        id[7] = 0;

#ifndef NDEBUG
        cout << "DEBUG getDirectory(): file #" << dec << i << ", ";
        if (!flag)
        {
            cout << "unused" << endl;
        }
        else
        {

            if (!type)
            {
                cout << "font";
            }
            else
            {
                cout << "bitmap";
            }

            cout << " #" << id.to_ulong() << ", " << dec << (int)size_lsb + (int)size_msb * 256 << "B" << endl;
        }
#endif /* #ifndef NDEBUG */

    }

} /* getDirectory() */

void moglk::deleteFile(bool type,
                       unsigned char id)
{
#ifndef NDEBUG
    cout << "DEBUG deleteFile(): ";
    if (!type)
    {
        cout << "font";
    }
    else
    {
        cout << "bitmap";
    }
    cout << " #" << dec << (int)id << endl;
#endif /* #ifndef NDEBUG */

    int message[] = {CMD_INIT,CMD_DELETE_FILE,type,id,EOF};
    send(&message[0]);

} /* deleteFile() */

int * moglk::downloadFile(bool type,
                         unsigned char id,
                         int * file_ptr)
{
if (id)
{
#ifndef NDEBUG
    cout << "DEBUG downloadFile(): ";
    if (!type)
    {
        cout << "font";
    }
    else
    {
        cout << "bitmap";
    }
    cout << " #" << dec << (int)id << endl;
#endif /* #ifndef NDEBUG */

    int message[] = {CMD_INIT,CMD_DOWNLOAD_FILE,type,id,EOF};
    send(&message[0]);

    receiveFile(file_ptr);
}
else
{
    cerr << "ERROR downloadFile(): ID should not be null";
}

        return(file_ptr);

} /* downloadFile() */

void moglk::moveFile(bool old_type,
                     unsigned char old_id,
                     bool new_type,
                     unsigned char new_id)
{
#ifndef NDEBUG
    cout << "DEBUG moveFile(): ";
    if (!old_type)
    {
        cout << "font";
    }
    else
    {
        cout << "bitmap";
    }
    cout << " #" << dec << (int)old_id << " to ";
    if (!new_type)
    {
        cout << "font";
    }
    else
    {
        cout << "bitmap";
    }
    cout << " #" << dec << (int)new_id << endl;
#endif /* #ifndef NDEBUG */

    int message[] = {CMD_INIT,CMD_MOVE_FILE,old_type,old_id,new_type,new_id,EOF};
    send(&message[0]);

} /* moveFile() */

//TODO: test!
void moglk::setLock(bitset<8> level)
{
    // Bits reserved
    level[7] = 0;
    level[6] = 0;
    level[5] = 0;

#ifndef NDEBUG
    cout << "DEBUG setLock(): ";
    if (level[4])
    {
        cout << "communication locked";
    }
    else
    {
        cout << "communication unlocked";
    }
    cout << ", ";
    if (level[3])
    {
        cout << "settings locked";
    }
    else
    {
        cout << "settings unlocked";
    }
    cout << ", ";
    if (level[2])
    {
        cout << "filesystem locked";
    }
    else
    {
        cout << "filesystem unlocked";
    }
    cout << ", ";
    if (level[1])
    {
        cout << "command locked";
    }
    else
    {
        cout << "command unlocked";
    }
    cout << ", ";
    if (level[0])
    {
        cout << "display locked";
    }
    else
    {
        cout << "display unlocked";
    }
    cout << endl;
#endif /* #ifndef NDEBUG */

    int message[] = {CMD_INIT,CMD_LOCK_LEVEL,level.to_ulong(),EOF};
    send(&message[0]);

} /* setLock() */

//TODO: test!
void moglk::setDefaultLock(bitset<8> level)
{
    // Bits reserved
    level[7] = 0;
    level[6] = 0;
    level[5] = 0;

#ifndef NDEBUG
    cout << "DEBUG setDefaultLock(): ";
    if (level[4])
    {
        cout << "communication locked";
    }
    else
    {
        cout << "communication unlocked";
    }
    cout << ", ";
    if (level[3])
    {
        cout << "settings locked";
    }
    else
    {
        cout << "settings unlocked";
    }
    cout << ", ";
    if (level[2])
    {
        cout << "filesystem locked";
    }
    else
    {
        cout << "filesystem unlocked";
    }
    cout << ", ";
    if (level[1])
    {
        cout << "command locked";
    }
    else
    {
        cout << "command unlocked";
    }
    cout << ", ";
    if (level[0])
    {
        cout << "display locked";
    }
    else
    {
        cout << "display unlocked";
    }
    cout << endl;
#endif /* #ifndef NDEBUG */

    int message[] = {CMD_INIT,CMD_DEFAULT_LOCK_LEVEL,level.to_ulong(),EOF};
    send(&message[0]);

} /* setDefaultLock() */

//FIXME: return an array with all keys
unsigned char moglk::pollKey(void)
{
#ifndef NDEBUG
    cout << "DEBUG pollKey()" << endl;
#endif /* #ifndef NDEBUG */

    int message[] = {CMD_INIT,CMD_POLL_KEY,EOF};
    unsigned char key[10] = {0,0,0,0,0,0,0,0,0,0};

    bitset<8> more = 0x80;
    unsigned int n = 0;
    while (more[7])
    {
        send(&message[0]);
        key[n] = receive();
        //Get more_key_presses_availables flag
        more = key[n];
        // Reset more_key_presses_availables flag to have a pure key descriptor
        if (more[7])
        {
            key[n] = key[n] - 0x80;
        }
        n++;
    }

#ifndef NDEBUG
    unsigned int i = 0;
    while (i < n)
    {
        cout << "DEBUG pollKey(): ";
        switch (key[i])
        {
            case RET_UP:
            {
                cout << "up button pressed" << endl;
                break;
            }

            case RET_DOWN:
            {
                cout << "down button pressed" << endl;
                break;
            }

            case RET_LEFT:
            {
                cout << "left button pressed" << endl;
                break;
            }

            case RET_RIGHT:
            {
                cout << "right button pressed" << endl;
                break;
            }

            case RET_CENTER:
            {
                cout << "center button pressed" << endl;
                break;
            }

            case RET_TOP:
            {
                cout << "top button pressed" << endl;
                break;
            }

            case RET_BOTTOM:
            {
                cout << "bottom button pressed" << endl;
                break;
            }

            case RET_RELEASE_UP:
            {
                cout << "up button released" << endl;
                break;
            }

            case RET_RELEASE_DOWN:
            {
                cout << "down button released" << endl;
                break;
            }

            case RET_RELEASE_LEFT:
            {
                cout << "left button released" << endl;
                break;
            }

            case RET_RELEASE_RIGHT:
            {
                cout << "right button released" << endl;
                break;
            }

            case RET_RELEASE_CENTER:
            {
                cout << "center button released" << endl;
                break;
            }

            case RET_RELEASE_TOP:
            {
                cout << "top button released" << endl;
                break;
            }

            case RET_RELEASE_BOTTOM:
            {
                cout << "bottom button released" << endl;
                break;
            }

            case RET_NO_KEY:
            {
                cout << "no key pressed" << endl;
                break;
            }
        } /* switch(key) */
        i++;
    };
#endif /* #ifndef NDEBUG */

    return key[0];

} /* pollKey() */

void moglk::setCustomKeyCodes(unsigned char kup_top,
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
                              unsigned char kdown_down)
{
#ifndef NDEBUG
    cout << "DEBUG setCustomKeyCodes(): button top key up message " << hex << (int)kup_top << endl;
    cout << "DEBUG setCustomKeyCodes(): button up key up message " << hex << (int)kup_up << endl;
    cout << "DEBUG setCustomKeyCodes(): button right key up message " << hex << (int)kup_right << endl;
    cout << "DEBUG setCustomKeyCodes(): button left key up message " << hex << (int)kup_left << endl;
    cout << "DEBUG setCustomKeyCodes(): button center key up message " << hex << (int)kup_center << endl;
    cout << "DEBUG setCustomKeyCodes(): button bottom key up message " << hex << (int)kup_bottom << endl;
    cout << "DEBUG setCustomKeyCodes(): button down key up message " << hex << (int)kup_down << endl;
    cout << "DEBUG setCustomKeyCodes(): button top key down message " << hex << (int)kdown_top << endl;
    cout << "DEBUG setCustomKeyCodes(): button up key down message " << hex << (int)kdown_up << endl;
    cout << "DEBUG setCustomKeyCodes(): button right key down message " << hex << (int)kdown_right << endl;
    cout << "DEBUG setCustomKeyCodes(): button left key down message " << hex << (int)kdown_left << endl;
    cout << "DEBUG setCustomKeyCodes(): button center key down message " << hex << (int)kdown_center << endl;
    cout << "DEBUG setCustomKeyCodes(): button bottom key down message " << hex << (int)kdown_bottom << endl;
    cout << "DEBUG setCustomKeyCodes(): button down key down message " << hex << (int)kdown_down << endl;
#endif /* #ifndef NDEBUG */

    int message[] = {CMD_INIT,CMD_CUSTOM_KEYPAD_CODES,kup_top,kup_up,kup_right,kup_left,kup_center,0,kup_bottom,kup_down,0,kdown_top,kdown_up,kdown_right,kdown_left,kdown_center,0,kdown_bottom,kdown_down,0,EOF};
    send(&message[0]);

} /* setCustomKeyCodes() */

int * moglk::dumpFs(int * file_ptr)
{
    int message[] = {CMD_INIT,CMD_DUMP_FS,EOF};
    send(&message[0]);

    receiveFile(file_ptr);

    return file_ptr;

} /* dumpFs() */

int * moglk::dumpFw(int * file_ptr)
{
    int message[] = {CMD_INIT,CMD_DUMP_FW,EOF};
    send(&message[0]);

    unsigned int file_size = 466;

    int file[file_size + 1];
    file_ptr = &file[0];

    unsigned long int i = 0;
    while (i < (file_size + 1))
    {
#ifndef NDEBUG
        cout << "DEBUG receiveFile(): byte #" << dec << i << " : ";
#endif /* #ifndef NDEBUG */
        file[i] = receive();
        i++;
    }
    file[i] = EOF;

#ifndef NDEBUG
    cout << "DEBUG receiveFile(): received file data = ";
    unsigned long int j = 0;
    while (file[j] != EOF)
    {
        cout << hex << (int)file[j];
        j++;
    }
    cout << endl;
    cout << "DEBUG receiveFile(): " << dec << j - 1 << "B" << endl;
#endif /* #ifndef NDEBUG */

    return file_ptr;

} /* dumpFW() */

void moglk::drawBmp(unsigned char x, unsigned char y, unsigned char width, unsigned char height, int * data_ptr)
{
#ifndef NDEBUG
    cout << "DEBUG drawBmp(): " << "X=" << dec << (int)x << "px, Y=" << (int)y << "px, " << (int)width << "x" << (int)height << "px" << endl;
#endif /* #ifndef NDEBUG */

    int message[] = {CMD_INIT,CMD_DRAW_BMP,x,y,width,height,EOF};
    send(&message[0]);
    unsigned int n = 0;
    int value = *(data_ptr + n);
    while (value != EOF)
    {
	int data_message[] = {value,EOF};
        send(data_message);
        n++;
        value = *(data_ptr + n);
    }

} /* drawBmp() */

//FIXME: WIP
bool moglk::upload(char * data_ptr)
{
#ifndef NDEBUG
    cout << "DEBUG upload()"<< endl;
#endif /* #ifndef NDEBUG */

  unsigned char reply;
  int c_message[] = {CMD_CONFIRM,EOF};
  int d_message[] = {CMD_DECLINE,EOF};



  return true;

} /* upload() */

//TODO: WIP
void moglk::uploadFont(unsigned char id, unsigned int size, char * data_ptr)
{
    int message[] = {CMD_INIT,CMD_UPLOAD_FONT,id,EOF};
    send(&message[0]);

    upload(data_ptr);

} /* uploadFont */

//TODO: WIP
void moglk::uploadBmp(unsigned char id, unsigned int size, char * data_ptr)
{

    int message[] = {CMD_INIT,CMD_UPLOAD_BMP,id,EOF};
    send(&message[0]);

    upload(data_ptr);

} /* uploadBmp */

//TODO: WIP
void moglk::uploadFs(unsigned int size, char * data_ptr)
{

    int message[] = {CMD_INIT,CMD_UPLOAD_FS,EOF};
    send(&message[0]);

    upload(data_ptr);

} /* uploadFs */
