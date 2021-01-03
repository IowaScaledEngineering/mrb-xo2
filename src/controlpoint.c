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

void CPInitializeTurnout(CPTurnout_t *turnout)
{
	turnout->isNormal = true;
	turnout->isRequestedNormal = true;
	turnout->isLocked = false;
	turnout->isManual = false;
}

void CPInitializeSignalHead(SignalHeadAspect_t *sig)
{
	*sig = ASPECT_RED;
}

bool CPInputStateGet(CPState_t* state, CPInputNames_t inputID)
{
	if (inputID < sizeof(state->inputs) / sizeof(CPInput_t))
		return state->inputs[inputID].isSet;

	return false;
}

bool CPInputStateSet(CPState_t* state, CPInputNames_t inputID, bool isSet)
{
	if (inputID < sizeof(state->inputs) / sizeof(CPInput_t))
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

	input->isVirtual = false;
	input->isSet = false;
	input->pktSrc = 0x00;
	input->pktType = 0x00;
	input->pktBitByte = 0x00;

	for (i=0; !input->isVirtual && i<sizeof(vInputConfigArray); i+=vInputConfigRecSize)
	{
		memcpy_P(vInputConfigRec, &vInputConfigArray[i], vInputConfigRecSize);
		if (vInputConfigRec[0] == input->inputID)
		{
			input->isVirtual = true;
			input->pktSrc = vInputConfigRec[1];
			input->pktType = vInputConfigRec[2];
			input->pktBitByte = vInputConfigRec[3];
			break;
		}
	}
}

void CPInitialize(CPState_t* state)
{
	uint8_t i;

	for (i=0; i<sizeof(state->signalHeads) / sizeof(SignalHeadAspect_t); i++)
		CPInitializeSignalHead(&state->signalHeads[i]);

	for (i=0; i<sizeof(state->turnouts) / sizeof(CPTurnout_t); i++)
		CPInitializeTurnout(&state->turnouts[i]);

	for (i=0; i<sizeof(state->inputs) / sizeof(CPInput_t); i++)
	{
		state->inputs[i].inputID = (CPInputNames_t)i;
		CPInitializeInput(&state->inputs[i]);
	}

}


