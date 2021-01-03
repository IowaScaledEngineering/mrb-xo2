/*************************************************************************
Title:    Signal Head Aspect Definitions
Authors:  Nathan D. Holmes <maverick@drgw.net>
File:     aspects.h
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

#ifndef _SIG_ASPECTS_H_
#define _SIG_ASPECTS_H_

typedef enum
{
	ASPECT_OFF       = 0,
	ASPECT_GREEN     = 1,
	ASPECT_YELLOW    = 2,
	ASPECT_FL_YELLOW = 3,
	ASPECT_RED       = 4,
	ASPECT_FL_GREEN  = 5,
	ASPECT_FL_RED    = 6,
	ASPECT_LUNAR     = 7
} SignalHeadAspect_t;

#endif
