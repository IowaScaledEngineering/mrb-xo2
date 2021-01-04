/*************************************************************************
Title:    XIO I2C Driver Header
Authors:  Nathan D. Holmes <maverick@drgw.net>
File:     xio-driver.c
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2021 Nathan Holmes

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

*************************************************************************/

#ifndef _XIO_DRIVER_H_
#define _XIO_DRIVER_H_

#include <stdlib.h>
#include <stdbool.h>
#include <avr/io.h>
#include <util/delay.h>

#include "avr-i2c-master.h"
#include "xio-hardware-def.h"

#define I2C_XIO0_ADDRESS 0x4E
#define I2C_XIO1_ADDRESS 0x4C
#define I2C_XIO2_ADDRESS 0x4A
#define I2C_XIO3_ADDRESS 0x48
#define I2C_XIO4_ADDRESS 0x46
#define I2C_XIO5_ADDRESS 0x44
#define I2C_XIO6_ADDRESS 0x42
#define I2C_XIO7_ADDRESS 0x40

#define XIO_I2C_ERROR   0x01
#define XIO_INITIALIZED 0x02

#define XIO_PORT_A  0
#define XIO_PORT_B  1
#define XIO_PORT_C  2
#define XIO_PORT_D  3
#define XIO_PORT_E  4

typedef struct
{
	uint8_t clock_A;
	uint8_t clock_B;
	uint8_t debounced_state;
} XIODebounceState;

typedef struct
{
	uint8_t address;
	uint8_t direction[5];
	uint8_t io[5];
	XIODebounceState debounced_in[5];
	uint8_t status;
} XIOControl;

#define xioIsInitialized(xio)  ((xio)->status & XIO_INITIALIZED)
#define xioI2CError(xio)  (((xio)->status & XIO_I2C_ERROR)?0:1)

#define XIO_LOW  false
#define XIO_HIGH true


void xioInputRead(XIOControl *xio);
bool xioGetIO(XIOControl* xio, uint8_t ioNum);
bool xioGetDeferredIO(XIOControl* xio, uint8_t ioNum);
bool xioGetDebouncedIO(XIOControl* xio, uint8_t ioNum);
bool xioGetDebouncedIObyPortBit(XIOControl* xio, uint8_t port, uint8_t bit);

void xioSetIO(XIOControl* xio, uint8_t ioNum, bool state);
void xioSetDeferredIO(XIOControl* xio, uint8_t ioNum, bool state);
void xioSetDeferredIObyPortBit(XIOControl* xio, uint8_t port, uint8_t bit, bool state);
void xioOutputWrite(XIOControl* xio);
void xioInitialize(XIOControl* xio, uint8_t xioAddress, const uint8_t* xioPinDirections);
void xioDirectionSend(XIOControl* xio);
void xioHardwareReset();

#endif

