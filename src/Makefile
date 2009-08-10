#
#    Copyright (C) 2009 Raphaël Doursenaud <rdoursenaud@free.fr>
#
#    This file is part of libmoglk
#    a Matrix Orbital Graphical Displays Protocol Library
#
#    libmoglk is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    libmoglk is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with libmoglk. If not, see <http://www.gnu.org/licenses/>.
.PHONY: test, clean

.SUFFIXE:

CC=g++
CFLAGS=-Wall -fpic -fPIC

all: libmoglk

libmoglk: moglk.cpp
	$(CC) $(CFLAGS) -c -o moglk.o moglk.cpp
	ar rcs libmoglk.a moglk.o
	$(CC) $(CFLAGS) -shared -o libmoglk.so moglk.o

test:
	make -C test/ all

clean:
	rm moglk.o
	rm libmoglk.a
	rm libmoglk.so
