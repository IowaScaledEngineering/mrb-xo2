/*************************************************************************
Title:    XIO I2C Driver
Authors:  Nathan D. Holmes <maverick@drgw.net>
File:     xio-driver.c
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2014 Nathan Holmes

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

*************************************************************************/

/* 0x00-0x04 - input registers */
/* 0x08-0x0C - output registers */
/* 0x18-0x1C - direction registers - 0 is output, 1 is input */
#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <util/delay.h>

#include "avr-i2c-master.h"
#include "xio-driver.h"

#define PORT_(port) PORT ## port
#define DDR_(port)  DDR  ## port
#define PIN_(port)  PIN  ## port

#define PORT(port) PORT_(port)
#define DDR(port)  DDR_(port)
#define PIN(port)  PIN_(port)


void initDebounceState(XIODebounceState* d, uint8_t initialState)
{
	d->clock_A = d->clock_B = 0;
	d->debounced_state = initialState;
}

uint8_t debounce(XIODebounceState* d, uint8_t raw_inputs)
{
	uint8_t delta = raw_inputs ^ d->debounced_state;   //Find all of the changes
	uint8_t changes;

	d->clock_A ^= d->clock_B;                     //Increment the counters
	d->clock_B  = ~d->clock_B;

	d->clock_A &= delta;                       //Reset the counters if no changes
	d->clock_B &= delta;                       //were detected.

	changes = ~((~delta) | d->clock_A | d->clock_B);
	d->debounced_state ^= changes;
	return(changes & ~(d->debounced_state));
}


void xioDirectionSend(XIOControl* xio)
{
	uint8_t i2cBuf[8], i;
	i2cBuf[0] = xio->address;
	i2cBuf[1] = 0x80 | 0x18;  // 0x80 is auto-increment
	for(i=0; i<5; i++)
		i2cBuf[2+i] = xio->direction[i];
	i2c_transmit(i2cBuf, 7, 1);
	while(i2c_busy());
}

// xioPinDirections is an array of 5 bytes corresponding to IO0_0 (byte 0, bit 0) through IO4_7 (byte 4, bit 7)
// Set xioPinDirections bit to 0 for XIO pin as an output
// Set xioPinDirections bit to 1 for XIO pin as an input

void xioHardwareReset()
{
	PORT(I2C_RESET_PORT) &= ~_BV(I2C_RESET);
	PORT(I2C_OUT_EN_PORT) |= _BV(I2C_OUT_EN);

	DDR(I2C_RESET_PORT) |= _BV(I2C_RESET);
	DDR(I2C_OUT_EN_PORT) |= _BV(I2C_OUT_EN);
	_delay_us(1);

	PORT(I2C_OUT_EN_PORT) &= ~_BV(I2C_OUT_EN);
	PORT(I2C_RESET_PORT) |= _BV(I2C_RESET);
	_delay_us(1);
}

void xioInitialize(XIOControl* xio, uint8_t xioAddress, const uint8_t* xioPinDirections)
{
	uint8_t i;
	memset(xio, 0, sizeof(XIOControl));
		
	xio->address = xioAddress;
	xio->status |= XIO_I2C_ERROR;

	for(i=0; i<5; i++)
	{
		xio->direction[i] = xioPinDirections[i];
		initDebounceState(&xio->debounced_in[i], 0);
	}
	xioDirectionSend(xio);

	if (i2c_transaction_successful())
	{
		xio->status &= ~(XIO_I2C_ERROR);
		xio->status |= XIO_INITIALIZED;
	}
	
	
}

void xioOutputWrite(XIOControl* xio)
{
	uint8_t i2cBuf[8];
	uint8_t i;

	// Reinforce direction
	xioDirectionSend(xio);

	while(i2c_busy());

	if (!i2c_transaction_successful())
		xio->status |= XIO_I2C_ERROR;

	i2cBuf[0] = xio->address;
	i2cBuf[1] = 0x80 | 0x08;  // 0x80 is auto-increment, 0x08 is the base of the output registers
	for(i=0; i<5; i++)
	{
		i2cBuf[2+i] = xio->io[i] & ~xio->direction[i];
	}

	i2c_transmit(i2cBuf, 7, 1);
}

void xioSetDeferredIO(XIOControl* xio, uint8_t ioNum, bool state)
{
	if (ioNum >= 40)
		return;

	// Don't set inputs
	if (xio->direction[ioNum/8] & (1<<(ioNum%8)))
		return;
	
	if (state)
		xio->io[ioNum/8] |= 1<<(ioNum%8);
	else
		xio->io[ioNum/8] &= ~(1<<(ioNum%8));
}

void xioSetIO(XIOControl* xio, uint8_t ioNum, bool state)
{
	xioSetDeferredIO(xio, ioNum, state);
	xioOutputWrite(xio);
}

void xioSetDeferredIObyPortBit(XIOControl* xio, uint8_t port, uint8_t bit, bool state)
{
	uint8_t ioNum = port * 8 + bit;
	xioSetDeferredIO(xio, ioNum, state);
}

bool xioGetDeferredIO(XIOControl* xio, uint8_t ioNum)
{
	if (ioNum >= 40)
		return(0);
	
	return ((xio->io[ioNum/8] & (1<<(ioNum % 8)))?true:false);
}

bool xioGetDebouncedIO(XIOControl* xio, uint8_t ioNum)
{
	if (ioNum >= 40)
		return(0);
	
	return ((xio->debounced_in[ioNum/8].debounced_state & (1<<(ioNum % 8)))?true:false);
}

bool xioGetDebouncedIObyPortBit(XIOControl* xio, uint8_t port, uint8_t bit)
{
	uint8_t ioNum = port * 8 + bit;
	return xioGetDebouncedIO(xio, ioNum);
}

bool xioGetIO(XIOControl* xio, uint8_t ioNum)
{
	xioInputRead(xio);
	return xioGetDeferredIO(xio, ioNum);
}


void xioInputRead(XIOControl *xio)
{
	uint8_t i2cBuf[8];
	uint8_t successful = 0;

	while(i2c_busy());

	if (!i2c_transaction_successful())
		xio->status |= XIO_I2C_ERROR;

	i2cBuf[0] = xio->address;
	i2cBuf[1] = 0x80;  // 0x80 is auto-increment, 0x00 is base of the input registers
	i2c_transmit(i2cBuf, 2, 0);
	i2cBuf[0] = xio->address | 0x01;
	i2c_transmit(i2cBuf, 6, 1);
	while(i2c_busy());
	
	successful = i2c_receive(i2cBuf, 6);

	if (!successful)
		// In the event of a read hose-out, don't put crap in the input buffer
		xio->status |= XIO_I2C_ERROR;
	else
	{
		uint8_t i;
		for(i=0; i<5; i++)
		{
			debounce(&xio->debounced_in[i], (xio->direction[i] & i2cBuf[1+i]));
			// Clear all things marked as inputs, leave outputs alone
			xio->io[i] &= ~xio->direction[i];
			// Only set anything that's high and marked as an input
			xio->io[i] |= (xio->direction[i] & i2cBuf[1+i]);
		}
	}
}



