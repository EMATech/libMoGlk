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

#include "../src/moglk.h"
#include <iostream>

int main(void)
{
	char port[] = "/dev/ttyUSB0";
	int speed = 19200;

	moglk lcd;
	int check = lcd.init(port,speed);
	if (check > 0)
	{
	lcd.setBaudRate(19200);
	lcd.setFlowControl(0);
	lcd.setLock(0);
	lcd.setRemember(1);
	lcd.setDefaultBrightness();
	lcd.setDefaultContrast(50);
	lcd.startupGpo(1, 1);
	lcd.startupGpo(2, 1);
	lcd.startupGpo(3, 1);
	lcd.startupGpo(4, 1);
	lcd.startupGpo(5, 1);
	lcd.startupGpo(6, 1);
        lcd.display("Reset?");
	}
	else
	{
        std::cerr << "libmoglk exited with code " << check << std::endl;
        return -1;
	}

	return 0;
}
