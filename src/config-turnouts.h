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

#ifndef _CONFIG_TURNOUTS_H_
#define _CONFIG_TURNOUTS_H_


typedef enum
{
	TURNOUT_E_XOVER,
	TURNOUT_W_XOVER,
	TURNOUT_END  // Must be last entry
} CPTurnoutNames_t;

typedef enum
{
	MAIN_TIMELOCK,
	TIMELOCK_END  // Must be last entry
} CPTimelockNames_t;

#endif
