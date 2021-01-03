/*************************************************************************
Title:    Control Point Input Configuration
Authors:  Nathan D. Holmes <maverick@drgw.net>
File:     config-eeprom.h
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

#ifndef _CONFIG_SIGNALS_H_
#define _CONFIG_SIGNALS_H_

#include "aspects.h"
#include "xio-driver.h"

typedef enum
{
	SIG_MAIN1_E_UPPER,
	SIG_MAIN1_E_LOWER,
	SIG_MAIN2_E_UPPER,
	SIG_MAIN2_E_LOWER,
	SIG_MAIN1_W_UPPER,
	SIG_MAIN1_W_LOWER, 
	SIG_MAIN2_W_UPPER,
	SIG_MAIN2_W_LOWER,
	SIG_END  // Must be last entry
} CPSignalHeadNames_t;

typedef struct
{
	const CPSignalHeadNames_t signalHead;
	const unsigned int xioNum : 3;
	const unsigned int redByte : 3;
	const unsigned int redBit : 3;
	const unsigned int yellowByte : 3;
	const unsigned int yellowBit : 3;
	const unsigned int greenByte : 3;
	const unsigned int greenBit : 3;
	bool isCommonAnode;  // Common anode means output low to turn a signal on
} SignalPinDefinition;

#endif
