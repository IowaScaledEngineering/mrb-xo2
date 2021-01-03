/*************************************************************************
Title:    Control Point Configuration EEPROM Locations
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

#ifndef _CONFIG_EEPROM_H_
#define _CONFIG_EEPROM_H_

// EEPROM Location Definitions
#define EE_HEADS_COM_ANODE    0x07
#define EE_OPTIONS            0x08

#define EE_UNLOCK_TIME        0x09
// Unlock time in decisecs

#define EE_M1E_APRCH_ADDR       0x10
#define EE_M1E_APRCH2_ADDR      0x11
#define EE_M1E_ADJ_ADDR         0x12
#define EE_M2E_APRCH_ADDR       0x13
#define EE_M2E_APRCH2_ADDR      0x14
#define EE_M2E_ADJ_ADDR         0x15
#define EE_M1W_APRCH_ADDR       0x16
#define EE_M1W_APRCH2_ADDR      0x17
#define EE_M1W_ADJ_ADDR         0x18
#define EE_M2W_APRCH_ADDR       0x19
#define EE_M2W_APRCH2_ADDR      0x1A
#define EE_M2W_ADJ_ADDR         0x1B
#define EE_M1E_TUMBLE_ADDR      0x1C
#define EE_M2E_TUMBLE_ADDR      0x1D
#define EE_M1W_TUMBLE_ADDR      0x1E
#define EE_M2W_TUMBLE_ADDR      0x1F
#define EE_M1_OS_ADDR           0x20
#define EE_M2_OS_ADDR           0x21

#define EE_M1E_APRCH_PKT        0x30
#define EE_M1E_APRCH2_PKT       0x31
#define EE_M1E_ADJ_PKT          0x32
#define EE_M2E_APRCH_PKT        0x33
#define EE_M2E_APRCH2_PKT       0x34
#define EE_M2E_ADJ_PKT          0x35
#define EE_M1W_APRCH_PKT        0x36
#define EE_M1W_APRCH2_PKT       0x37
#define EE_M1W_ADJ_PKT          0x38
#define EE_M2W_APRCH_PKT        0x39
#define EE_M2W_APRCH2_PKT       0x3A
#define EE_M2W_ADJ_PKT          0x3B
#define EE_M1E_TUMBLE_PKT       0x3C
#define EE_M2E_TUMBLE_PKT       0x3D
#define EE_M1W_TUMBLE_PKT       0x3E
#define EE_M2W_TUMBLE_PKT       0x3F
#define EE_M1_OS_PKT            0x40
#define EE_M2_OS_PKT            0x41

#define EE_M1E_APRCH_BITBYTE    0x50
#define EE_M1E_APRCH2_BITBYTE   0x51
#define EE_M1E_ADJ_BITBYTE      0x52
#define EE_M2E_APRCH_BITBYTE    0x53
#define EE_M2E_APRCH2_BITBYTE   0x54
#define EE_M2E_ADJ_BITBYTE      0x55
#define EE_M1W_APRCH_BITBYTE    0x56
#define EE_M1W_APRCH2_BITBYTE   0x57
#define EE_M1W_ADJ_BITBYTE      0x58
#define EE_M2W_APRCH_BITBYTE    0x59
#define EE_M2W_APRCH2_BITBYTE   0x5A
#define EE_M2W_ADJ_BITBYTE      0x5B
#define EE_M1E_TUMBLE_BITBYTE   0x5C
#define EE_M2E_TUMBLE_BITBYTE   0x5D
#define EE_M1W_TUMBLE_BITBYTE   0x5E
#define EE_M2W_TUMBLE_BITBYTE   0x5F
#define EE_M1_OS_BITBYTE        0x60
#define EE_M2_OS_BITBYTE        0x61

#endif
