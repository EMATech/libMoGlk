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

#include "../../src/moglk.h"
#include <iostream>

int main(int argc, char *argv[])
{
	if (argc == 1)
	{
		std::cerr << "Please input a text" << std::endl;
		return -1;
	}
	char port[] = "/dev/ttyUSB0";
	unsigned long int speed = 19200;

	moglk lcd;
	int check = lcd.init(port,speed);
	if (!check)
	{
		lcd.display(argv[1],1,0,0);
		//lcd.display("\n",1);
	}
	else
	{
	        std::cerr << "libmoglk exited with code " << check << std::endl;
		return -1;
	}
	return 0;
}
