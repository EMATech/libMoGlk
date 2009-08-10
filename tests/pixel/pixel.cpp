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

int main(void)
{
	char port[] = "/dev/ttyUSB0";
	unsigned long int speed = 9600;

	moglk lcd;
	int check = lcd.init(port,speed);
	if (check > 0)
	{
        lcd.clearScreen();

        unsigned char x = 0;
        unsigned char y = 0;
        unsigned int i = 0;

        while(i < image_width * image_height)
        {

            if(!image_data[i]) lcd.drawPixel(x,y,!image_data[i]);
            if(x < image_width - 1)
            {
                x++;
            }
            else
            {
                y++;
                x = 0;
            }
            i++;
        };

	}
	else
	{
        std::cerr << "libmoglk exited with code " << check << std::endl;
	}
}
