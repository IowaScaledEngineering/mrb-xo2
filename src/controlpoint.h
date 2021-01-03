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

#ifndef _CONTROLPOINT_H_
#define _CONTROLPOINT_H_

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "mrbus.h"
#include "config-turnouts.h"
#include "config-signals.h"
#include "config-inputs.h"

#define BITBYTE_BYTENUM(a)   ((a) & 0x1F)
#define BITBYTE_BITNUM(a)    ((a)>>5)
#define BITBYTE_BITMASK(a)   (1<<BITBYTE_BITNUM(a))

typedef enum
{
	STATE_LOCKED = 0,
	STATE_TIMERUN = 1,
	STATE_UNLOCKED = 2,
	STATE_RELOCKING = 3
} CPTurnoutState_t;

typedef struct
{
	bool isNormal;
	bool isRequestedNormal;
	bool isLocked;
	bool isManual;
	CPTurnoutState_t turnoutState;
} CPTurnout_t;

typedef struct
{
	CPInputNames_t inputID;
	bool isSet;
	bool isVirtual;
	uint8_t pktSrc;
	uint8_t pktType;
	uint8_t pktBitByte;
} CPInput_t;


typedef struct 
{
	SignalHeadAspect_t signalHeads[SIG_END];
	CPTurnout_t turnouts[TURNOUT_END];
	CPInput_t inputs[VINPUT_END];
} CPState_t;

void CPInitialize(CPState_t* state);
void CPInitializeTurnout(CPTurnout_t *turnout);
void CPInitializeSignalHead(SignalHeadAspect_t *sig);
bool CPInputStateGet(CPState_t* state, CPInputNames_t inputID);
bool CPInputStateSet(CPState_t* state, CPInputNames_t inputID, bool isSet);
void CPSignalHeadSetAspect(CPState_t *cpState, CPSignalHeadNames_t signalID, SignalHeadAspect_t aspect);
void CPSignalHeadAllSetAspect(CPState_t *cpState, SignalHeadAspect_t aspect);
void CPMRBusVirtInputFilter(CPState_t* state, const uint8_t const *mrbRxBuffer);
SignalHeadAspect_t CPSignalHeadGetAspect(CPState_t *cpState, CPSignalHeadNames_t signalID);

// Control point to physical hardware functions
void CPSignalsToOutputs(CPState_t *cpState, XIOControl* xio, bool blinkerOn);

#endif
