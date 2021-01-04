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

#ifndef _CONFIG_INPUTS_H_
#define _CONFIG_INPUTS_H_

// Get our eeprom configuration locations
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <avr/pgmspace.h>
#include "config-eeprom.h"

// Define all input states stored
typedef enum
{
	VOCC_M1E_ADJOIN,
	VOCC_M1E_APPROACH,
	VOCC_M1E_APPROACH2,
	VOCC_M1E_TUMBLE,
	VOCC_M2E_ADJOIN,
	VOCC_M2E_APPROACH, 
	VOCC_M2E_APPROACH2,
	VOCC_M2E_TUMBLE,
	VOCC_M1W_ADJOIN,
	VOCC_M1W_APPROACH,
	VOCC_M1W_APPROACH2,
	VOCC_M1W_TUMBLE,
	VOCC_M2W_ADJOIN,
	VOCC_M2W_APPROACH,
	VOCC_M2W_APPROACH2,
	VOCC_M2W_TUMBLE,
	VOCC_M1_OS,
	VOCC_M2_OS,
	OCC_M1_OS,
	OCC_M2_OS,
	E_XOVER_ACTUAL_POS,
	W_XOVER_ACTUAL_POS,
	E_XOVER_MANUAL_POS,
	W_XOVER_MANUAL_POS,
	TIMELOCK_SW_POS,
	VINPUT_END  // Must be last entry
} CPInputNames_t;

#define vInputConfigRecSize     4
#define xioInputConfigRecSize   4

#endif
