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

#ifndef _CONFIG_ROUTE_H_
#define _CONFIG_ROUTE_H_

#define MAX_ROUTES 2

typedef enum
{
	ROUTE_NONE,
	ROUTE_MAIN1_EASTBOUND,
	ROUTE_MAIN1_WESTBOUND,
	ROUTE_MAIN2_EASTBOUND,
	ROUTE_MAIN2_WESTBOUND,
	ROUTE_MAIN2_VIA_MAIN1_EASTBOUND,
	ROUTE_MAIN2_VIA_MAIN1_WESTBOUND,
	ROUTE_MAIN1_TO_MAIN2_EASTBOUND,
	ROUTE_MAIN1_TO_MAIN2_WESTBOUND,
	ROUTE_MAIN2_TO_MAIN1_EASTBOUND,
	ROUTE_MAIN2_TO_MAIN1_WESTBOUND
} CPRoute_t;

typedef enum
{
	ROUTE_ENTR_NONE,
	ROUTE_ENTR_M1_EASTBOUND,
	ROUTE_ENTR_M1_WESTBOUND,
	ROUTE_ENTR_M2_EASTBOUND,
	ROUTE_ENTR_M2_WESTBOUND
} CPRouteEntrance_t;

#endif
