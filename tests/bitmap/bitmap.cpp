/*
 Copyright (C) 2009 Raphaël Doursenaud <rdoursenaud@free.fr>

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

#include "../../src/moglk.hpp"
#include "ematech.h"
#include <iostream>
#include <bitset>       // C++ bit structure

int main(void)
{
	char port[] = "/dev/ttyUSB0";
	unsigned long int speed = 115200;

	moglk lcd;
	int check = lcd.init(port,speed);
	if (check > 0)
	{
        lcd.clearScreen();

	unsigned char x = 0;
	unsigned char y = 0;
        unsigned int i = 0;
	int data[image_width * image_height / 8];

   	while(i < image_width * image_height)
	{
		std::bitset<8> byte;

		for (unsigned int n = 0;n < 8;n++)
		{
		    byte[n] = image_data[i + 7 - n];
		}
		data[i / 8] = byte.to_ulong();
		i = i + 8;
	}
	data[i /8] = EOF;
	lcd.drawBmp(x,y,image_width,image_height,data);
	}
	else
	{
        	std::cerr << "libmoglk exited with code " << check << std::endl;
	}
}
