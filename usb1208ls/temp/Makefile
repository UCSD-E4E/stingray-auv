#############################################################################
#	
#	makefile for building:
#
#		libmcchid.so:        Library for USB series
#		test-usb1208LS:   Program to test USB-1208LS module
#		test-usb1024LS:   Program to test USB-1024LS module
#		test-usb1208FS:   Program to test USB-1208FS module
#		test-usb1408FS:   Program to test USB-1408FS module
#		test-usb1608FS:   Program to test USB-1608FS module
#		test-usb1608HS:   Program to test USB-1608HS and 1608HS-2AO module
#		test-usb-dio96H:  Program to test USB-1096HFS, USB-DIO96H, and USB-DIO96H/50
#		test-usb1616FS:   Program to test USB-1616FS module
#               test-usb3100:     Program to test USB-3100 modules
#		test-usb-tc:      Program to test USB-TC module
#		test-usb-tc-ai:   Program to test USB-TC-AI module
#		test-usb-temp:    Program to test USB-TEMP module
#		test-usb-temp-ai: Program to test USB-TEMP-AI module
#               test-usb5201:     Program to test USB-5201 module
#               test-usb5203:     Program to test USB-5203 module
#		test-minilab1008: Program to test miniLAB 1008 module
#               test-usb-ssr:     Program to test USB-SSR24 and SSR08 modules
#               test-usb-pdiso8:  Program to test USB-PDISO8
#               test-usb500:      Program to test USB-501 and USB-502
#               test-usb-erb      Program to test USB-ERB08 and USB-ERB24
#               test-usb4300      Program to test USB-430X
#               test-usb-dio24    Program to test USB-DIO24 and USB-DIO24H
#               test-usb-2416     Proram to test the USB-2416
#               test-usb-1208HS   Proram to test the USB-1208HS and USB-1208HS-4A0
#
#               Copyright (C)  2009
#               Written by:  Warren J. Jasper
#                            North Carolina State Univerisity
#
#
# This program, libmcchid.so, is free software; you can redistribute it
# and/or modify it under the terms of the GNU General Public License as
# published by the Free Software Foundation; either version 2 of the
# License, or (at your option) any later version, provided that this
# copyright notice is preserved on all copies.
#
# ANY RIGHTS GRANTED HEREUNDER ARE GRANTED WITHOUT WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES
# OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE, AND FURTHER,
# THERE SHALL BE NO WARRANTY AS TO CONFORMITY WITH ANY USER MANUALS OR
# OTHER LITERATURE PROVIDED WITH SOFTWARE OR THAM MY BE ISSUED FROM TIME
# TO TIME. IT IS PROVIDED SOLELY "AS IS".
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
###########################################################################

#  Current Version of the driver
VERSION=1.40

SRCS    = pmd.c usb-1208LS.c 
HEADERS = pmd.h usb-1208LS.h 
OBJS = $(SRCS:.c=.o)   # same list as SRCS with extension changed
CC=gcc
CFLAGS= -g -Wall -fPIC -O 
TARGETS=libmcchid.so libmcchid.a test-usb1208LS 
ID=MCCLIBHID
DIST_NAME=$(ID).$(VERSION).tgz
DIST_FILES={README,Makefile,usb-1208LS.h,usb-1208LS.c}
###### RULES
all: $(TARGETS)

%.d: %.c
	set -e; $(CC) -I. -M $(CPPFLAGS) $< \
	| sed 's/\($*\)\.o[ :]*/\1.o $@ : /g' > $@; \
	[ -s $@ ] || rm -f $@
ifneq ($(MAKECMDGOALS),clean)
include $(SRCS:.c=.d)
endif

libmcchid.so: $(OBJS)
	$(CC) -O -shared -Wall $(OBJS) -o $@

libmcchid.a: $(OBJS)
	ar -r libmcchid.a $(OBJS)
	ranlib libmcchid.a

test-usb1208LS:	test-usb1208LS.c libmcchid.a
	$(CC) -g -Wall -I. -o $@  $@.c -lmcchid -L. -lm -L/usr/local/lib -lhid -lusb 

clean:
	rm -rf *.d *.o *~ *.a *.so $(TARGETS)

dist:	
	make clean
	cd ..; tar -zcvf $(DIST_NAME) libhid/$(DIST_FILES);

install:
	-install -d /usr/local/lib
	-install -c --mode=0755 ./libmcchid.a libmcchid.so /usr/local/lib
	-/bin/ln -s /usr/local/lib/libmcchid.so /usr/lib/libmcchid.so
	-/bin/ln -s /usr/local/lib/libmcchid.a /usr/lib/libmcchid.a
	-install -d /usr/local/include/libhid
	-install -c --mode=0644 ${HEADERS} /usr/local/include/libhid/

uninstall:
	-rm -f /usr/local/lib/libmcchid*
	-rm -f /usr/lib/libmcchid*
	-rm -rf /usr/local/include/libhid
