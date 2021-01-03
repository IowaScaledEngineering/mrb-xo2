/*************************************************************************
Title:    XIO I2C Driver Hardware Configuration
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

#ifndef _XIO_HARDWARE_DEF_H_
#define _XIO_HARDWARE_DEF_H_

#include <avr/io.h>

// Change these to reflect where the XIO control lines are hooked up

#define I2C_RESET         0
#define I2C_RESET_PORT    B
#define I2C_OUT_EN        1
#define I2C_OUT_EN_PORT   B
#define I2C_IRQ           2
#define I2C_IRQ_PORT      B

#endif

