/*************************************************************************
Title:    Control Point Core Functionality
Authors:  Nathan D. Holmes <maverick@drgw.net>
File:     controlpoint.c
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

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "mrbus.h"
#include "controlpoint.h"
#include "config-hardware.h"

void CPMRBusVirtInputFilter(CPState_t* state, const uint8_t const *mrbRxBuffer)
{
	uint8_t i;
	for (i=0; i<sizeof(state->inputs) / sizeof(CPInput_t); i++)
	{
		if (!state->inputs[i].isVirtual)
			continue;

		uint8_t byteNum = BITBYTE_BYTENUM(state->inputs[i].pktBitByte);
		uint8_t bitMask = BITBYTE_BITMASK(state->inputs[i].pktBitByte);

		if (mrbRxBuffer[MRBUS_PKT_SRC] != state->inputs[i].pktSrc 
			|| mrbRxBuffer[MRBUS_PKT_TYPE] != state->inputs[i].pktType
			|| byteNum > mrbRxBuffer[MRBUS_PKT_LEN])
			continue;

		state->inputs[i].isSet = (mrbRxBuffer[byteNum] & bitMask)?true:false;
	}
}

void CPXIOInputFilter(CPState_t* state, XIOControl* xio)
{
	for (uint8_t i=0; i<VINPUT_END; i++)
	{
		if (state->inputs[i].isVirtual)
			continue;

		state->inputs[i].isSet = xioGetDebouncedIObyPortBit(&xio[state->inputs[i].pktSrc], state->inputs[i].pktType, state->inputs[i].pktBitByte);
	}
}

void CPInitializeTurnout(CPTurnout_t *turnout)
{
	turnout->isNormal = true;
	turnout->isRequestedNormal = true;
	turnout->isLocked = false;
	turnout->isManual = false;
}

void CPInitializeTimelock(CPTimelock_t *timelock)
{
	timelock->state = STATE_LOCKED;
	timelock->secs = 0;
}

void CPTurnoutRequestedDirectionSet(CPState_t *state, CPTurnoutNames_t turnoutID, bool setNormal)
{
	if(turnoutID < TURNOUT_END)
		state->turnouts[turnoutID].isRequestedNormal = setNormal;
}

void CPTurnoutManualOperationsSet(CPState_t *state, CPTurnoutNames_t turnoutID, bool setManual)
{
	if(turnoutID < TURNOUT_END)
		state->turnouts[turnoutID].isManual = setManual;
}



void CPInitializeSignalHead(SignalHeadAspect_t *sig)
{
	*sig = ASPECT_RED;
}

bool CPInputStateGet(CPState_t* state, CPInputNames_t inputID)
{
	if (inputID < VINPUT_END)
		return state->inputs[inputID].isSet;

	return false;
}

bool CPInputStateSet(CPState_t* state, CPInputNames_t inputID, bool isSet)
{
	if (inputID < VINPUT_END)
	{
		state->inputs[inputID].isSet = isSet;
		return true;
	}
	return false;
}

void CPSignalHeadSetAspect(CPState_t *cpState, CPSignalHeadNames_t signalID, SignalHeadAspect_t aspect)
{
	if(signalID < SIG_END)
		cpState->signalHeads[signalID] = aspect;
}

void CPSignalHeadAllSetAspect(CPState_t *cpState, SignalHeadAspect_t aspect)
{
	for(uint8_t i=0; i<SIG_END; i++)
		cpState->signalHeads[i] = aspect;
}

SignalHeadAspect_t CPSignalHeadGetAspect(CPState_t *cpState, CPSignalHeadNames_t signalID)
{
	return cpState->signalHeads[signalID];
}


CPTimelockState_t CPTimelockStateGet(CPState_t *cpState, CPTimelockNames_t timelockID)
{
	if(timelockID < TIMELOCK_END)
		return cpState->timelocks[timelockID].state;
	return STATE_UNKNOWN;
}

void CPTimelockStateSet(CPState_t *cpState, CPTimelockNames_t timelockID, CPTimelockState_t state)
{
	if(timelockID < TIMELOCK_END)
		cpState->timelocks[timelockID].state = state;
}

void CPTimelockTimeSet(CPState_t *cpState, CPTimelockNames_t timelockID, uint8_t seconds)
{
	if(timelockID < TIMELOCK_END)
		cpState->timelocks[timelockID].secs = seconds;
}

uint8_t CPTimelockTimeGet(CPState_t *cpState, CPTimelockNames_t timelockID)
{
	if(timelockID < TIMELOCK_END)
		return cpState->timelocks[timelockID].secs;
	return 0;
}

void CPTurnoutsToOutputs(CPState_t *cpState, XIOControl* xio)
{
	for (uint8_t tid=0; tid<TURNOUT_END; tid++)
	{
		for (uint8_t j=0; j<sizeof(cpTurnoutPinDefs)/sizeof(TurnoutPinDefinition); j++)
		{
			if(cpTurnoutPinDefs[j].turnoutID == tid)
			{
				bool isRequestedNormal = cpState->turnouts[tid].isRequestedNormal;
				
				xioSetDeferredIObyPortBit(&xio[cpTurnoutPinDefs[j].xioNum], 
					cpTurnoutPinDefs[j].controlByte, cpTurnoutPinDefs[j].controlBit,
					(cpTurnoutPinDefs[j].isNormalLow)?!isRequestedNormal:isRequestedNormal);
				break; // process next turnout
			}
		}
	}
}


void CPSignalsToOutputs(CPState_t *cpState, XIOControl* xio, bool blinkerOn)
{
	uint8_t sigDefIdx;
	for(sigDefIdx=0; sigDefIdx<SIG_END; sigDefIdx++)
	{
		uint8_t xioNum = cpSignalPinDefs[sigDefIdx].xioNum;
		uint8_t redByte = cpSignalPinDefs[sigDefIdx].redByte;
		uint8_t redBit = cpSignalPinDefs[sigDefIdx].redBit;
		uint8_t yellowByte = cpSignalPinDefs[sigDefIdx].yellowByte;
		uint8_t yellowBit = cpSignalPinDefs[sigDefIdx].yellowBit;
		uint8_t greenByte = cpSignalPinDefs[sigDefIdx].greenByte;
		uint8_t greenBit = cpSignalPinDefs[sigDefIdx].greenBit;
		bool inactiveState = (cpSignalPinDefs[sigDefIdx].isCommonAnode)?XIO_HIGH:XIO_LOW;
		bool activeState = !inactiveState;

		xioSetDeferredIObyPortBit(&xio[xioNum], redByte, redBit, inactiveState);
		xioSetDeferredIObyPortBit(&xio[xioNum], yellowByte, yellowBit, inactiveState);
		xioSetDeferredIObyPortBit(&xio[xioNum], greenByte, greenBit, inactiveState);

		switch(cpState->signalHeads[cpSignalPinDefs[sigDefIdx].signalHead])
		{
			case ASPECT_OFF:
				break;
		
			case ASPECT_GREEN:
				xioSetDeferredIObyPortBit(&xio[xioNum], greenByte, greenBit, activeState);
				break;
		
			case ASPECT_FL_GREEN:
				if (blinkerOn)
					xioSetDeferredIObyPortBit(&xio[xioNum], greenByte, greenBit, activeState);
				break;

			case ASPECT_YELLOW:
				xioSetDeferredIObyPortBit(&xio[xioNum], yellowByte, yellowBit, activeState);
				break;
		
			case ASPECT_FL_YELLOW:
				if (blinkerOn)
					xioSetDeferredIObyPortBit(&xio[xioNum], yellowByte, yellowBit, activeState);
				break;

			case ASPECT_RED:
			case ASPECT_LUNAR: // Can't display, so make like red
			default:
				xioSetDeferredIObyPortBit(&xio[xioNum], redByte, redBit, activeState);
				break;

			case ASPECT_FL_RED:
				if (blinkerOn)
					xioSetDeferredIObyPortBit(&xio[xioNum], redByte, redBit, activeState);
				break;
		}
	}
}


void CPInitializeInput(CPInput_t *input)
{
	uint16_t i;
	uint8_t vInputConfigRec[vInputConfigRecSize];
	uint8_t xioInputConfigRec[xioInputConfigRecSize];
	input->isVirtual = false;
	input->isSet = false;
	input->pktSrc = 0x00;
	input->pktType = 0x00;
	input->pktBitByte = 0x00;

	for (i=0; i<sizeof(vInputConfigArray); i+=vInputConfigRecSize)
	{
		memcpy_P(vInputConfigRec, &vInputConfigArray[i], vInputConfigRecSize);
		if (vInputConfigRec[0] == input->inputID)
		{
			input->isVirtual = true;
			input->pktSrc = vInputConfigRec[1];
			input->pktType = vInputConfigRec[2];
			input->pktBitByte = vInputConfigRec[3];
			return;
		}
	}
	
	for (i=0; i<sizeof(xioInputConfigArray); i+=xioInputConfigRecSize)
	{
		memcpy_P(xioInputConfigRec, &xioInputConfigArray[i], xioInputConfigRecSize);
		if (xioInputConfigRec[0] == input->inputID)
		{
			input->pktSrc = xioInputConfigRec[1];
			input->pktType = xioInputConfigRec[2];
			input->pktBitByte = xioInputConfigRec[3];
			return;
		}
	}
}

void CPTimelockApply1HzTick(CPState_t* state)
{
	for (uint8_t i=0; i<sizeof(state->timelocks) / sizeof(CPTimelockState_t); i++)
	{
		if (state->timelocks[i].secs > 0)
			state->timelocks[i].secs--;
	}
}

void CPInitialize(CPState_t* state)
{
	uint8_t i;

	for (i=0; i<sizeof(state->signalHeads) / sizeof(SignalHeadAspect_t); i++)
		CPInitializeSignalHead(&state->signalHeads[i]);

	for (i=0; i<sizeof(state->turnouts) / sizeof(CPTurnout_t); i++)
		CPInitializeTurnout(&state->turnouts[i]);

	for (i=0; i<VINPUT_END; i++)
	{
		state->inputs[i].inputID = (CPInputNames_t)i;
		CPInitializeInput(&state->inputs[i]);
	}

	for (i=0; i<sizeof(state->timelocks) / sizeof(CPTimelock_t); i++)
		CPInitializeTimelock(&state->timelocks[i]);
}


