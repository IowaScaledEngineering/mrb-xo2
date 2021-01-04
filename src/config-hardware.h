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

#ifndef _CONFIG_SIGNAL_HW_H_
#define _CONFIG_SIGNAL_HW_H_

#include "config-signals.h"
#include "config-inputs.h"

typedef struct
{
	const CPTurnoutNames_t turnoutID;
	const unsigned int xioNum : 3;
	const unsigned int controlByte : 3;
	const unsigned int controlBit : 3;
	bool isNormalLow;  // Means the turnout is normal/main when control is low
} TurnoutPinDefinition;

const TurnoutPinDefinition const cpTurnoutPinDefs[TURNOUT_END] = 
{
	{TURNOUT_E_XOVER, 0, XIO_PORT_D, 2, true},
	{TURNOUT_W_XOVER, 0, XIO_PORT_D, 3, false}
};

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

const SignalPinDefinition const cpSignalPinDefs[SIG_END] = 
{
	{SIG_MAIN1_E_UPPER, 0, XIO_PORT_A, 0, XIO_PORT_A, 1, XIO_PORT_A, 2, false},
	{SIG_MAIN1_E_LOWER, 0, XIO_PORT_A, 3, XIO_PORT_A, 4, XIO_PORT_A, 5, false},
	{SIG_MAIN2_E_UPPER, 0, XIO_PORT_A, 6, XIO_PORT_A, 7, XIO_PORT_B, 0, false},
	{SIG_MAIN2_E_LOWER, 0, XIO_PORT_B, 1, XIO_PORT_B, 2, XIO_PORT_B, 3, false},
	{SIG_MAIN1_W_UPPER, 0, XIO_PORT_B, 4, XIO_PORT_B, 5, XIO_PORT_B, 6, false},
	{SIG_MAIN1_W_LOWER, 0, XIO_PORT_B, 7, XIO_PORT_C, 0, XIO_PORT_C, 1, false},
	{SIG_MAIN2_W_UPPER, 0, XIO_PORT_C, 2, XIO_PORT_C, 3, XIO_PORT_C, 4, false},
	{SIG_MAIN2_W_LOWER, 0, XIO_PORT_C, 5, XIO_PORT_C, 6, XIO_PORT_C, 7, false}
};

const uint8_t vInputConfigArray[] PROGMEM = 
{
/* These go in 4-byte increments - vinput ID, srcAddr, pktType, bitbyte
 *  Virtual Input ID (from CPInputNames_t)
 *  |                    Source MRBus Addr
 *  |                    |                       Source MRBus Pkt Type
 *  |                    |                       |
 *  |                    |                       |                     Source MRBus bit/byte
 *  v                    v                       v                     v
*/
	VOCC_M1E_ADJOIN,     EE_M1E_ADJ_ADDR,        EE_M1E_ADJ_PKT,       EE_M1E_ADJ_BITBYTE,
	VOCC_M1E_APPROACH,   EE_M1E_APRCH_ADDR,      EE_M1E_APRCH_PKT,     EE_M1E_APRCH_BITBYTE,
	VOCC_M1E_APPROACH2,  EE_M1E_APRCH2_ADDR,     EE_M1E_APRCH2_PKT,    EE_M1E_APRCH2_BITBYTE,
	VOCC_M1E_TUMBLE,     EE_M1E_TUMBLE_ADDR,     EE_M1E_TUMBLE_PKT,    EE_M1E_TUMBLE_BITBYTE,
	
	VOCC_M1W_ADJOIN,     EE_M1W_ADJ_ADDR,        EE_M1W_ADJ_PKT,       EE_M1W_ADJ_BITBYTE,
	VOCC_M1W_APPROACH,   EE_M1W_APRCH_ADDR,      EE_M1W_APRCH_PKT,     EE_M1W_APRCH_BITBYTE,
	VOCC_M1W_APPROACH2,  EE_M1W_APRCH2_ADDR,     EE_M1W_APRCH2_PKT,    EE_M1W_APRCH2_BITBYTE,
	VOCC_M1W_TUMBLE,     EE_M1W_TUMBLE_ADDR,     EE_M1W_TUMBLE_PKT,    EE_M1W_TUMBLE_BITBYTE,
	
	VOCC_M2E_ADJOIN,     EE_M2E_ADJ_ADDR,        EE_M2E_ADJ_PKT,       EE_M2E_ADJ_BITBYTE,
	VOCC_M2E_APPROACH,   EE_M2E_APRCH_ADDR,      EE_M2E_APRCH_PKT,     EE_M2E_APRCH_BITBYTE,
	VOCC_M2E_APPROACH2,  EE_M2E_APRCH2_ADDR,     EE_M2E_APRCH2_PKT,    EE_M2E_APRCH2_BITBYTE,
	VOCC_M2E_TUMBLE,     EE_M2E_TUMBLE_ADDR,     EE_M2E_TUMBLE_PKT,    EE_M2E_TUMBLE_BITBYTE,
	
	VOCC_M2W_ADJOIN,     EE_M2W_ADJ_ADDR,        EE_M2W_ADJ_PKT,       EE_M2W_ADJ_BITBYTE,
	VOCC_M2W_APPROACH,   EE_M2W_APRCH_ADDR,      EE_M2W_APRCH_PKT,     EE_M2W_APRCH_BITBYTE,
	VOCC_M2W_APPROACH2,  EE_M2W_APRCH2_ADDR,     EE_M2W_APRCH2_PKT,    EE_M2W_APRCH2_BITBYTE,
	VOCC_M2W_TUMBLE,     EE_M2W_TUMBLE_ADDR,     EE_M2W_TUMBLE_PKT,    EE_M2W_TUMBLE_BITBYTE,
	
	VOCC_M1_OS,          EE_M1_OS_ADDR,          EE_M1_OS_PKT,         EE_M1_OS_BITBYTE,
	VOCC_M2_OS,          EE_M2_OS_ADDR,          EE_M2_OS_PKT,         EE_M2_OS_BITBYTE
};

const uint8_t xioInputConfigArray[] PROGMEM = 
{
/* These go in 4-byte increments - vinput ID, srcAddr, pktType, bitbyte
 *  Virtual Input ID (from CPInputNames_t)
 *  |                    XIO #
 *  |                    |    XIO Port (A-E)
 *  |                    |    |            XIO Port Pin
 *  |                    |    |            |
 *  v                    v    v            v
*/
	E_XOVER_ACTUAL_POS,  0,   XIO_PORT_D,  6,
	W_XOVER_ACTUAL_POS,  0,   XIO_PORT_D,  7,
	E_XOVER_MANUAL_POS,  0,   XIO_PORT_D,  4,
	W_XOVER_MANUAL_POS,  0,   XIO_PORT_D,  5,
	TIMELOCK_SW_POS,     0,   XIO_PORT_D,  1
};

#endif
