/*************************************************************************
Title:    MRBus Simple 3-Way CTC Control Point Node
Authors:  Nathan D. Holmes <maverick@drgw.net>
File:     $Id: $
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2021 Nathan Holmes <maverick@drgw.net>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

*************************************************************************/

#include <stdlib.h>
#include <stdbool.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <string.h>
#include <util/delay.h>

#include "mrbus.h"
#include "avr-i2c-master.h"
#include "busvoltage.h"
#include "controlpoint.h"

void PktHandler(CPState_t *cpState);

#define txBuffer_DEPTH 4
#define rxBuffer_DEPTH 8

MRBusPacket mrbusTxPktBufferArray[txBuffer_DEPTH];
MRBusPacket mrbusRxPktBufferArray[rxBuffer_DEPTH];

uint8_t mrbus_dev_addr = 0;
volatile uint8_t events = 0;

#define EVENT_READ_INPUTS    0x01
#define EVENT_WRITE_OUTPUTS  0x02
#define EVENT_1HZ            0x04
#define EVENT_I2C_ERROR      0x40
#define EVENT_BLINKY         0x80

#define POINTS_NORMAL_SAFE    'M'
#define POINTS_REVERSE_SAFE   'D'
#define POINTS_NORMAL_FORCE   'm'
#define POINTS_REVERSE_FORCE  'd'
#define POINTS_UNAFFECTED     'X'

// Used for status occupancy byte 0
#define OCC_M1_OS_SECT          0x01
#define OCC_M2_OS_SECT          0x02
#define OCC_VIRT_M1E_ADJOIN     0x04
#define OCC_VIRT_M1E_APPROACH   0x08
#define OCC_VIRT_M2E_ADJOIN     0x10
#define OCC_VIRT_M2E_APPROACH   0x20
#define OCC_VIRT_M1W_ADJOIN     0x40
#define OCC_VIRT_M1W_APPROACH   0x80

// Used for status occupancy byte 1
#define OCC_VIRT_M2W_ADJOIN     0x01
#define OCC_VIRT_M2W_APPROACH   0x02

uint8_t debounced_inputs[2], old_debounced_inputs[2];
uint8_t clearance, old_clearance;
uint8_t clock_a[2] = {0,0}, clock_b[2] = {0,0};

// ******** Start 100 Hz Timer, 0.16% error version (Timer 0)
// If you can live with a slightly less accurate timer, this one only uses Timer 0, leaving Timer 1 open
// for more advanced things that actually need a 16 bit timer/counter

// Initialize a 100Hz timer for use in triggering events.
// If you need the timer resources back, you can remove this, but I find it
// rather handy in triggering things like periodic status transmissions.
// If you do remove it, be sure to yank the interrupt handler and ticks/secs as well
// and the call to this function in the main function

volatile uint8_t decisecs=0;
volatile uint8_t buttonLockout=5;

uint8_t updateInterval=10;
uint8_t i2cResetCounter = 0;

void initialize100HzTimer(void)
{
	// Set up timer 1 for 100Hz interrupts
	TCNT0 = 0;
	OCR0A = 0xC2;
	decisecs = 0;
	TCCR0A = _BV(WGM01);
	TCCR0B = _BV(CS02) | _BV(CS00);
	TIMSK0 |= _BV(OCIE0A);
}

ISR(TIMER0_COMPA_vect)
{
	static uint8_t ticks = 0;
	static uint8_t blinkyCounter = 0;

	if (ticks & 0x01)
		events |= EVENT_READ_INPUTS;

	if (++ticks >= 10)
	{
		ticks = 0;
		decisecs++;

		if (++blinkyCounter > 5)
		{
			events ^= EVENT_BLINKY;
			blinkyCounter = 0;
			if (0 == (events & EVENT_BLINKY))
				events |= EVENT_1HZ;
		}

		if (buttonLockout != 0)
			buttonLockout--;

		events |= EVENT_WRITE_OUTPUTS;
	}
}

// End of 100Hz timer

void cpLockAllTurnouts(CPState_t* state)
{
	CPTurnoutLockSet(state, TURNOUT_E_XOVER, true);
	CPTurnoutLockSet(state, TURNOUT_W_XOVER, true);
}

void cpUnlockAllTurnouts(CPState_t* state)
{
	CPTurnoutLockSet(state, TURNOUT_E_XOVER, false);
	CPTurnoutLockSet(state, TURNOUT_W_XOVER, false);
}


void init(void)
{
	// Clear watchdog
	MCUSR = 0;
	// If you don't want the watchdog to do system reset, remove this chunk of code
	wdt_reset();
	wdt_enable(WDTO_1S);
	wdt_reset();

	// Initialize MRBus address from EEPROM address 0
	mrbus_dev_addr = eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR);
	// Bogus addresses, fix to default address
	if (0xFF == mrbus_dev_addr || 0x00 == mrbus_dev_addr)
	{
		mrbus_dev_addr = 0x03;
		eeprom_write_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR, mrbus_dev_addr);
	}

	uint16_t tmp_updateInterval = (uint16_t)eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_L) 
		| (((uint16_t)eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_H)) << 8);

	// Don't update more than once per second and max out at 25.5s
	updateInterval = max(10, min(255L, tmp_updateInterval));

	// Setup ADC for bus voltage monitoring
	busVoltageMonitorInit();
}


void CodeCTCRoute(uint8_t controlPoint, uint8_t newPointsE, uint8_t newPointsW, uint8_t newClear)
{
	/*SetTurnout(TURNOUT_E_XOVER, newPointsE);
	SetTurnout(TURNOUT_W_XOVER, newPointsW);
	SetClearance(controlPoint, newClear);*/
}



static inline void vitalLogic(CPState_t *cpState)
{
	bool occupancyMain1 = CPInputStateGet(cpState, VOCC_M1_OS);
	bool occupancyMain2 = CPInputStateGet(cpState, VOCC_M2_OS);

	// First, if we have occupancy, drop any routes affected
	if (occupancyMain1 || occupancyMain2)
	{
		CPRouteClear(cpState, ROUTE_MAIN2_VIA_MAIN1_EASTBOUND);
		CPRouteClear(cpState, ROUTE_MAIN2_VIA_MAIN1_WESTBOUND);
		CPRouteClear(cpState, ROUTE_MAIN1_TO_MAIN2_EASTBOUND);
		CPRouteClear(cpState, ROUTE_MAIN1_TO_MAIN2_WESTBOUND);
		CPRouteClear(cpState, ROUTE_MAIN2_TO_MAIN1_EASTBOUND);
		CPRouteClear(cpState, ROUTE_MAIN2_TO_MAIN1_WESTBOUND);
	}
	
	if (occupancyMain1)
	{
		CPRouteClear(cpState, ROUTE_MAIN1_EASTBOUND);
		CPRouteClear(cpState, ROUTE_MAIN1_WESTBOUND);
	}
	
	if (occupancyMain2)
	{
		CPRouteClear(cpState, ROUTE_MAIN2_EASTBOUND);
		CPRouteClear(cpState, ROUTE_MAIN2_WESTBOUND);
	}
	

	// Now, unlock appropriate turnouts if no routes are set and there's no occupancy
	if (!(occupancyMain1 || occupancyMain2) && CPRouteNoneSet(cpState))
	{
		cpUnlockAllTurnouts(cpState);
	}
	
	/*
	 * 	ROUTE_NONE,
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
	* 
	* 	SIG_MAIN1_E_UPPER,
	SIG_MAIN1_E_LOWER,
	SIG_MAIN2_E_UPPER,
	SIG_MAIN2_E_LOWER,
	SIG_MAIN1_W_UPPER,
	SIG_MAIN1_W_LOWER, 
	SIG_MAIN2_W_UPPER,
	SIG_MAIN2_W_LOWER,
	* 
	* 	ASPECT_OFF       = 0,
	ASPECT_GREEN     = 1,
	ASPECT_YELLOW    = 2,
	ASPECT_FL_YELLOW = 3,
	ASPECT_RED       = 4,
	ASPECT_FL_GREEN  = 5,
	ASPECT_FL_RED    = 6,
	ASPECT_LUNAR     = 7
	* 
	* 
	* */

	bool eastCrossover = CPTurnoutActualDirectionGet(cpState, TURNOUT_E_XOVER);
	bool westCrossover = CPTurnoutActualDirectionGet(cpState, TURNOUT_W_XOVER);

	// Start out with a safe default - everybody red
	CPSignalHeadAllSetAspect(cpState, ASPECT_RED);

	if (CPTurnoutRequestedDirectionGet(cpState, TURNOUT_E_XOVER) != CPTurnoutActualDirectionGet(cpState, TURNOUT_E_XOVER)
		|| CPTurnoutRequestedDirectionGet(cpState, TURNOUT_W_XOVER) != CPTurnoutActualDirectionGet(cpState, TURNOUT_W_XOVER))
	{
		// Turnouts are in motion - RED!
	}
	else if (STATE_LOCKED != CPTimelockStateGet(cpState, MAIN_TIMELOCK))
	{
		// Timelock isn't locked - RED!
		if (STATE_UNLOCKED == CPTimelockStateGet(cpState, MAIN_TIMELOCK))
		{
			// If we're actually unlocked, put up restricting indications where appropriate
			if (eastCrossover && westCrossover) // Both normal
			{
				CPSignalHeadSetAspect(cpState, SIG_MAIN1_W_UPPER, ASPECT_FL_RED);
				CPSignalHeadSetAspect(cpState, SIG_MAIN1_E_UPPER, ASPECT_FL_RED);
				CPSignalHeadSetAspect(cpState, SIG_MAIN2_W_UPPER, ASPECT_FL_RED);
				CPSignalHeadSetAspect(cpState, SIG_MAIN2_E_UPPER, ASPECT_FL_RED);
			}
			else if (eastCrossover && !westCrossover)
			{
				CPSignalHeadSetAspect(cpState, SIG_MAIN1_E_LOWER, ASPECT_FL_RED);
				CPSignalHeadSetAspect(cpState, SIG_MAIN2_W_LOWER, ASPECT_FL_RED);
			}
			else if (!eastCrossover && westCrossover)
			{
				CPSignalHeadSetAspect(cpState, SIG_MAIN1_W_LOWER, ASPECT_FL_RED);
				CPSignalHeadSetAspect(cpState, SIG_MAIN2_E_LOWER, ASPECT_FL_RED);
			} else {
				CPSignalHeadSetAspect(cpState, SIG_MAIN2_W_LOWER, ASPECT_FL_RED);
				CPSignalHeadSetAspect(cpState, SIG_MAIN2_E_LOWER, ASPECT_FL_RED);
			}

		}
	}
	else
	{
		// Work through all routes set, setting signals appropriate to state
		if (CPRouteTest(cpState, ROUTE_MAIN1_EASTBOUND))
		{
			CPSignalHeadSetAspect(cpState, SIG_MAIN1_W_LOWER, ASPECT_RED);
			if (CPInputStateGet(cpState, VOCC_M1E_ADJOIN))
			{
				CPSignalHeadSetAspect(cpState, SIG_MAIN1_W_UPPER, ASPECT_RED);
			}
			else if (CPInputStateGet(cpState, VOCC_M1E_APPROACH))
			{
				CPSignalHeadSetAspect(cpState, SIG_MAIN1_W_UPPER, ASPECT_YELLOW);
			}
			else if (CPInputStateGet(cpState, VOCC_M1E_APPROACH2))
			{
				CPSignalHeadSetAspect(cpState, SIG_MAIN1_W_UPPER, ASPECT_FL_YELLOW);
			} else {
				CPSignalHeadSetAspect(cpState, SIG_MAIN1_W_UPPER, ASPECT_GREEN);
			}
		}
		else if (CPRouteTest(cpState, ROUTE_MAIN1_WESTBOUND))
		{
			CPSignalHeadSetAspect(cpState, SIG_MAIN1_E_LOWER, ASPECT_RED);
			if (CPInputStateGet(cpState, VOCC_M1W_ADJOIN))
			{
				CPSignalHeadSetAspect(cpState, SIG_MAIN1_E_UPPER, ASPECT_RED);
			}
			else if (CPInputStateGet(cpState, VOCC_M1W_APPROACH))
			{
				CPSignalHeadSetAspect(cpState, SIG_MAIN1_E_UPPER, ASPECT_YELLOW);
			}
			else if (CPInputStateGet(cpState, VOCC_M1W_APPROACH2))
			{
				CPSignalHeadSetAspect(cpState, SIG_MAIN1_E_UPPER, ASPECT_FL_YELLOW);
			} else {
				CPSignalHeadSetAspect(cpState, SIG_MAIN1_E_UPPER, ASPECT_GREEN);
			}
		}

		if (CPRouteTest(cpState, ROUTE_MAIN2_EASTBOUND))
		{
			CPSignalHeadSetAspect(cpState, SIG_MAIN2_W_LOWER, ASPECT_RED);
			if (CPInputStateGet(cpState, VOCC_M2E_ADJOIN))
			{
				CPSignalHeadSetAspect(cpState, SIG_MAIN2_W_UPPER, ASPECT_RED);
			}
			else if (CPInputStateGet(cpState, VOCC_M2E_APPROACH))
			{
				CPSignalHeadSetAspect(cpState, SIG_MAIN2_W_UPPER, ASPECT_YELLOW);
			}
			else if (CPInputStateGet(cpState, VOCC_M2E_APPROACH2))
			{
				CPSignalHeadSetAspect(cpState, SIG_MAIN2_W_UPPER, ASPECT_FL_YELLOW);
			} else {
				CPSignalHeadSetAspect(cpState, SIG_MAIN2_W_UPPER, ASPECT_GREEN);
			}
		} 
		else if (CPRouteTest(cpState, ROUTE_MAIN2_WESTBOUND))
		{
			CPSignalHeadSetAspect(cpState, SIG_MAIN2_E_LOWER, ASPECT_RED);
			if (CPInputStateGet(cpState, VOCC_M2W_ADJOIN))
			{
				CPSignalHeadSetAspect(cpState, SIG_MAIN2_E_UPPER, ASPECT_RED);
			}
			else if (CPInputStateGet(cpState, VOCC_M2W_APPROACH))
			{
				CPSignalHeadSetAspect(cpState, SIG_MAIN2_E_UPPER, ASPECT_YELLOW);
			}
			else if (CPInputStateGet(cpState, VOCC_M2W_APPROACH2))
			{
				CPSignalHeadSetAspect(cpState, SIG_MAIN2_E_UPPER, ASPECT_FL_YELLOW);
			} else {
				CPSignalHeadSetAspect(cpState, SIG_MAIN2_E_UPPER, ASPECT_GREEN);
			}
		}
		else if (CPRouteTest(cpState, ROUTE_MAIN2_VIA_MAIN1_EASTBOUND))
		{
			CPSignalHeadSetAspect(cpState, SIG_MAIN2_W_UPPER, ASPECT_RED);
			if (CPInputStateGet(cpState, VOCC_M2E_ADJOIN))
			{
				CPSignalHeadSetAspect(cpState, SIG_MAIN2_W_LOWER, ASPECT_RED);
			}
			else if (CPInputStateGet(cpState, VOCC_M2E_APPROACH))
			{
				CPSignalHeadSetAspect(cpState, SIG_MAIN2_W_LOWER, ASPECT_YELLOW);
			}
			else if (CPInputStateGet(cpState, VOCC_M2E_APPROACH2))
			{
				CPSignalHeadSetAspect(cpState, SIG_MAIN2_W_LOWER, ASPECT_FL_YELLOW);
			} else {
				CPSignalHeadSetAspect(cpState, SIG_MAIN2_W_LOWER, ASPECT_GREEN);
			}
		}
		else if (CPRouteTest(cpState, ROUTE_MAIN2_VIA_MAIN1_WESTBOUND))
		{
			CPSignalHeadSetAspect(cpState, SIG_MAIN2_E_UPPER, ASPECT_RED);
			if (CPInputStateGet(cpState, VOCC_M2W_ADJOIN))
			{
				CPSignalHeadSetAspect(cpState, SIG_MAIN2_E_LOWER, ASPECT_RED);
			}
			else if (CPInputStateGet(cpState, VOCC_M2W_APPROACH))
			{
				CPSignalHeadSetAspect(cpState, SIG_MAIN2_E_LOWER, ASPECT_YELLOW);
			}
			else if (CPInputStateGet(cpState, VOCC_M2W_APPROACH2))
			{
				CPSignalHeadSetAspect(cpState, SIG_MAIN2_E_LOWER, ASPECT_FL_YELLOW);
			} else {
				CPSignalHeadSetAspect(cpState, SIG_MAIN2_E_LOWER, ASPECT_GREEN);
			}
		}
		
		// Work through all routes set, setting signals appropriate to state
		if (CPRouteTest(cpState, ROUTE_MAIN1_TO_MAIN2_EASTBOUND))
		{
			CPSignalHeadSetAspect(cpState, SIG_MAIN1_W_UPPER, ASPECT_RED);
			if (CPInputStateGet(cpState, VOCC_M2E_ADJOIN))
			{
				CPSignalHeadSetAspect(cpState, SIG_MAIN1_W_LOWER, ASPECT_RED);
			}
			else if (CPInputStateGet(cpState, VOCC_M2E_APPROACH))
			{
				CPSignalHeadSetAspect(cpState, SIG_MAIN1_W_LOWER, ASPECT_YELLOW);
			}
			else if (CPInputStateGet(cpState, VOCC_M2E_APPROACH2))
			{
				CPSignalHeadSetAspect(cpState, SIG_MAIN1_W_LOWER, ASPECT_FL_YELLOW);
			} else {
				CPSignalHeadSetAspect(cpState, SIG_MAIN1_W_LOWER, ASPECT_GREEN);
			}
		}
		else if (CPRouteTest(cpState, ROUTE_MAIN1_TO_MAIN2_WESTBOUND))
		{
			CPSignalHeadSetAspect(cpState, SIG_MAIN1_E_UPPER, ASPECT_RED);
			if (CPInputStateGet(cpState, VOCC_M2W_ADJOIN))
			{
				CPSignalHeadSetAspect(cpState, SIG_MAIN1_E_LOWER, ASPECT_RED);
			}
			else if (CPInputStateGet(cpState, VOCC_M2W_APPROACH))
			{
				CPSignalHeadSetAspect(cpState, SIG_MAIN1_E_LOWER, ASPECT_YELLOW);
			}
			else if (CPInputStateGet(cpState, VOCC_M2W_APPROACH2))
			{
				CPSignalHeadSetAspect(cpState, SIG_MAIN1_E_LOWER, ASPECT_FL_YELLOW);
			} else {
				CPSignalHeadSetAspect(cpState, SIG_MAIN1_E_LOWER, ASPECT_GREEN);
			}
		}
		else if (CPRouteTest(cpState, ROUTE_MAIN2_TO_MAIN1_EASTBOUND))
		{
			CPSignalHeadSetAspect(cpState, SIG_MAIN2_W_UPPER, ASPECT_RED);
			if (CPInputStateGet(cpState, VOCC_M1E_ADJOIN))
			{
				CPSignalHeadSetAspect(cpState, SIG_MAIN2_W_LOWER, ASPECT_RED);
			}
			else if (CPInputStateGet(cpState, VOCC_M1E_APPROACH))
			{
				CPSignalHeadSetAspect(cpState, SIG_MAIN2_W_LOWER, ASPECT_YELLOW);
			}
			else if (CPInputStateGet(cpState, VOCC_M1E_APPROACH2))
			{
				CPSignalHeadSetAspect(cpState, SIG_MAIN2_W_LOWER, ASPECT_FL_YELLOW);
			} else {
				CPSignalHeadSetAspect(cpState, SIG_MAIN2_W_LOWER, ASPECT_GREEN);
			}
		}
		else if (CPRouteTest(cpState, ROUTE_MAIN2_TO_MAIN1_WESTBOUND))
		{
			CPSignalHeadSetAspect(cpState, SIG_MAIN2_E_UPPER, ASPECT_RED);
			if (CPInputStateGet(cpState, VOCC_M1W_ADJOIN))
			{
				CPSignalHeadSetAspect(cpState, SIG_MAIN2_E_LOWER, ASPECT_RED);
			}
			else if (CPInputStateGet(cpState, VOCC_M1W_APPROACH))
			{
				CPSignalHeadSetAspect(cpState, SIG_MAIN2_E_LOWER, ASPECT_YELLOW);
			}
			else if (CPInputStateGet(cpState, VOCC_M1W_APPROACH2))
			{
				CPSignalHeadSetAspect(cpState, SIG_MAIN2_E_LOWER, ASPECT_FL_YELLOW);
			} else {
				CPSignalHeadSetAspect(cpState, SIG_MAIN2_E_LOWER, ASPECT_GREEN);
			}
		}
	}




}

// For the XIO pins, 0 is output, 1 is input
const uint8_t const xio1PinDirection[5] = { 0x00, 0x00, 0x00, 0xF2, 0x00 };

void setTimelockLED(XIOControl* xio, bool state)
{
	xioSetDeferredIObyPortBit(xio, XIO_PORT_D, 0, state);
}


/*
 West M2 -------------------  M2 East
             \        /
      M1 -------------------  M1
*/


#define MRB_STATUS6_MAIN1_OS_OCC        0x01
#define MRB_STATUS6_MAIN2_OS_OCC        0x02
#define MRB_STATUS6_M1E_ENTR_CLEARED    0x04
#define MRB_STATUS6_M1W_ENTR_CLEARED    0x08
#define MRB_STATUS6_M2E_ENTR_CLEARED    0x10
#define MRB_STATUS6_M2W_ENTR_CLEARED    0x20


#define MRB_STATUS7_E_XOVER_NORMAL      0x01
#define MRB_STATUS7_E_XOVER_REVERSE     0x02
#define MRB_STATUS7_E_XOVER_MANUAL      0x04
#define MRB_STATUS7_E_XOVER_LOCK        0x08
#define MRB_STATUS7_W_XOVER_NORMAL      0x10
#define MRB_STATUS7_W_XOVER_REVERSE     0x20
#define MRB_STATUS7_W_XOVER_MANUAL      0x40
#define MRB_STATUS7_W_XOVER_LOCK        0x80

#define MRB_STATUS8_M1E_VIRT_ADJ        0x01
#define MRB_STATUS8_M1E_VIRT_APPR       0x02
#define MRB_STATUS8_M1E_VIRT_APPR2      0x04
#define MRB_STATUS8_M1E_VIRT_TUMBLE     0x08
#define MRB_STATUS8_M1W_VIRT_ADJ        0x10
#define MRB_STATUS8_M1W_VIRT_APPR       0x20
#define MRB_STATUS8_M1W_VIRT_APPR2      0x40
#define MRB_STATUS8_M1W_VIRT_TUMBLE     0x80

#define MRB_STATUS9_M2E_VIRT_ADJ        0x01
#define MRB_STATUS9_M2E_VIRT_APPR       0x02
#define MRB_STATUS9_M2E_VIRT_APPR2      0x04
#define MRB_STATUS9_M2E_VIRT_TUMBLE     0x08
#define MRB_STATUS9_M2W_VIRT_ADJ        0x10
#define MRB_STATUS9_M2W_VIRT_APPR       0x20
#define MRB_STATUS9_M2W_VIRT_APPR2      0x40
#define MRB_STATUS9_M2W_VIRT_TUMBLE     0x80

#define OCC_VIRT_ADJ        0x01
#define OCC_VIRT_APPR       0x02
#define OCC_VIRT_APPR2      0x04

uint8_t SignalHeadsToVirtOcc(SignalHeadAspect_t upperHead, SignalHeadAspect_t lowerHead)
{
	uint8_t mask = 0;
	
	if ( (upperHead == ASPECT_RED || upperHead == ASPECT_FL_RED || upperHead == ASPECT_LUNAR)
		&& (lowerHead == ASPECT_RED || lowerHead == ASPECT_FL_RED || lowerHead == ASPECT_LUNAR))
		mask |= (OCC_VIRT_ADJ | OCC_VIRT_APPR | OCC_VIRT_APPR2);
	else if (upperHead == ASPECT_YELLOW || lowerHead == ASPECT_YELLOW)
		mask |= (OCC_VIRT_APPR | OCC_VIRT_APPR2);
	else if (upperHead == ASPECT_FL_YELLOW || lowerHead == ASPECT_FL_YELLOW)
		mask |= (OCC_VIRT_APPR2);

	return mask;
}


uint8_t cpStateToStatusPacket(CPState_t* cpState, uint8_t *mrbTxBuffer, uint8_t mrbTxBufferSz)
{
	memset(mrbTxBuffer, 0, mrbTxBufferSz);
	
	mrbTxBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
	mrbTxBuffer[MRBUS_PKT_DEST] = 0xFF;
	mrbTxBuffer[MRBUS_PKT_LEN] = 12;
	mrbTxBuffer[5] = 'S';
	
	// Byte 6 - Occupancy & Entrance Signals
	if (CPInputStateGet(cpState, VOCC_M1_OS))
		mrbTxBuffer[6] |= MRB_STATUS6_MAIN1_OS_OCC;

	if (CPInputStateGet(cpState, VOCC_M2_OS))
		mrbTxBuffer[6] |= MRB_STATUS6_MAIN2_OS_OCC;


	if (CPRouteTest(cpState, ROUTE_MAIN1_WESTBOUND)
		|| CPRouteTest(cpState, ROUTE_MAIN1_TO_MAIN2_WESTBOUND))
		mrbTxBuffer[6] |= MRB_STATUS6_M1E_ENTR_CLEARED;

	if (CPRouteTest(cpState, ROUTE_MAIN1_EASTBOUND)
		|| CPRouteTest(cpState, ROUTE_MAIN1_TO_MAIN2_EASTBOUND))
		mrbTxBuffer[6] |= MRB_STATUS6_M1W_ENTR_CLEARED;

	if (CPRouteTest(cpState, ROUTE_MAIN2_WESTBOUND)
		|| CPRouteTest(cpState, ROUTE_MAIN2_TO_MAIN1_WESTBOUND)
		|| CPRouteTest(cpState, ROUTE_MAIN2_VIA_MAIN1_WESTBOUND))
		mrbTxBuffer[6] |= MRB_STATUS6_M2E_ENTR_CLEARED;

	if (CPRouteTest(cpState, ROUTE_MAIN2_EASTBOUND)
		|| CPRouteTest(cpState, ROUTE_MAIN2_TO_MAIN1_EASTBOUND)
		|| CPRouteTest(cpState, ROUTE_MAIN2_VIA_MAIN1_EASTBOUND))
		mrbTxBuffer[6] |= MRB_STATUS6_M2W_ENTR_CLEARED;

	// Byte 7 - turnout states
	if (STATE_LOCKED != CPTimelockStateGet(cpState, MAIN_TIMELOCK))
		mrbTxBuffer[7] |= MRB_STATUS7_E_XOVER_MANUAL | MRB_STATUS7_W_XOVER_MANUAL;

	if (CPTurnoutActualDirectionGet(cpState, TURNOUT_E_XOVER))
		mrbTxBuffer[7] |= MRB_STATUS7_E_XOVER_NORMAL;
	else
		mrbTxBuffer[7] |= MRB_STATUS7_E_XOVER_REVERSE;

	if (CPTurnoutLockGet(cpState, TURNOUT_E_XOVER))
		mrbTxBuffer[7] |= MRB_STATUS7_E_XOVER_LOCK;

	if (CPTurnoutActualDirectionGet(cpState, TURNOUT_W_XOVER))
		mrbTxBuffer[7] |= MRB_STATUS7_W_XOVER_NORMAL;
	else
		mrbTxBuffer[7] |= MRB_STATUS7_W_XOVER_REVERSE;

	if (CPTurnoutLockGet(cpState, TURNOUT_W_XOVER))
		mrbTxBuffer[7] |= MRB_STATUS7_W_XOVER_LOCK;

	// Compute virtual occupancy
	mrbTxBuffer[8] = SignalHeadsToVirtOcc(CPSignalHeadGetAspect(cpState, SIG_MAIN1_E_UPPER), CPSignalHeadGetAspect(cpState, SIG_MAIN1_E_LOWER))
		| (SignalHeadsToVirtOcc(CPSignalHeadGetAspect(cpState, SIG_MAIN1_W_UPPER), CPSignalHeadGetAspect(cpState, SIG_MAIN1_W_LOWER))<<4);

	if (CPRouteTest(cpState, ROUTE_MAIN1_WESTBOUND)
		|| CPRouteTest(cpState, ROUTE_MAIN2_TO_MAIN1_WESTBOUND))
		mrbTxBuffer[8] |= MRB_STATUS8_M1W_VIRT_TUMBLE;

	if (CPRouteTest(cpState, ROUTE_MAIN1_EASTBOUND)
		|| CPRouteTest(cpState, ROUTE_MAIN2_TO_MAIN1_EASTBOUND))
		mrbTxBuffer[8] |= MRB_STATUS8_M1E_VIRT_TUMBLE;

	mrbTxBuffer[9] = SignalHeadsToVirtOcc(CPSignalHeadGetAspect(cpState, SIG_MAIN2_E_UPPER), CPSignalHeadGetAspect(cpState, SIG_MAIN2_E_LOWER))
		| (SignalHeadsToVirtOcc(CPSignalHeadGetAspect(cpState, SIG_MAIN2_W_UPPER), CPSignalHeadGetAspect(cpState, SIG_MAIN2_W_LOWER))<<4);

	if (CPRouteTest(cpState, ROUTE_MAIN2_WESTBOUND)
		|| CPRouteTest(cpState, ROUTE_MAIN2_VIA_MAIN1_WESTBOUND)
		|| CPRouteTest(cpState, ROUTE_MAIN1_TO_MAIN2_WESTBOUND))
		mrbTxBuffer[9] |= MRB_STATUS9_M2W_VIRT_TUMBLE;

	if (CPRouteTest(cpState, ROUTE_MAIN2_EASTBOUND)
		|| CPRouteTest(cpState, ROUTE_MAIN2_VIA_MAIN1_EASTBOUND)
		|| CPRouteTest(cpState, ROUTE_MAIN1_TO_MAIN2_EASTBOUND))
		mrbTxBuffer[9] |= MRB_STATUS9_M2E_VIRT_TUMBLE;

	return mrbTxBuffer[MRBUS_PKT_LEN];

}

bool cpSetTurnout(CPState_t* cpState, CPTurnoutNames_t turnout, bool setNormal)
{
	if (STATE_LOCKED != CPTimelockStateGet(cpState, MAIN_TIMELOCK))
		return false; // Can't set any turnout when the timelock is open

	if (CPInputStateGet(cpState, VOCC_M1_OS) || CPInputStateGet(cpState, VOCC_M2_OS))
		return false; // Can't set any turnout when the CP tracks are occupied
		
	if (CPTurnoutLockGet(cpState, turnout))
		return false; // Turnout locked, cannot change
		
	CPTurnoutRequestedDirectionSet(cpState, turnout, setNormal);
	return true;
}

bool cpCodeRoute(CPState_t* cpState, CPRouteEntrance_t entrance, bool setRoute)
{
	if (STATE_LOCKED != CPTimelockStateGet(cpState, MAIN_TIMELOCK))
		return false; // Can't set any route when the timelock is open

	bool eastCrossover = CPTurnoutRequestedDirectionGet(cpState, TURNOUT_E_XOVER);
	bool westCrossover = CPTurnoutRequestedDirectionGet(cpState, TURNOUT_W_XOVER);
	
	// Remember, with turnouts normal = true

	switch(entrance)
	{
		case ROUTE_ENTR_M1_EASTBOUND:
			if (!westCrossover) // Turnout set against us
				return false;

			if (eastCrossover)
			{
				// Both crossovers normal, straight through route
				// Is there already a conflicting route set?
				if (CPRouteTest(cpState, ROUTE_MAIN1_WESTBOUND))
					return false;

				// Lock turnouts
				cpLockAllTurnouts(cpState);
				// Set route
				CPRouteSet(cpState, ROUTE_MAIN1_EASTBOUND);
			} else {
				// West crossover reversed, M1->M2
				if (CPRouteTest(cpState, ROUTE_MAIN2_TO_MAIN1_WESTBOUND))
					return false;

				CPRouteSet(cpState, ROUTE_MAIN1_TO_MAIN2_EASTBOUND);
			}
			break;
			
			
		case ROUTE_ENTR_M1_WESTBOUND:
			if (!eastCrossover) // Turnout set against us
				return false;
				
			if (westCrossover)
			{
				// Both crossovers normal, straight through route
				// Is there already a conflicting route set?
				if (CPRouteTest(cpState, ROUTE_MAIN1_EASTBOUND))
					return false;

				// Lock turnouts
				cpLockAllTurnouts(cpState);
				// Set route
				CPRouteSet(cpState, ROUTE_MAIN1_WESTBOUND);
			} else {
				// West crossover reversed, M1->M2
				if (CPRouteTest(cpState, ROUTE_MAIN2_TO_MAIN1_EASTBOUND))
					return false;

				CPRouteSet(cpState, ROUTE_MAIN1_TO_MAIN2_WESTBOUND);
			}
			break;
		
		case ROUTE_ENTR_M2_EASTBOUND:
			if (!eastCrossover && !westCrossover)
			{
				// Main 2 -> Main 2 via Main 1 - icky
				if (CPRouteTest(cpState, ROUTE_MAIN2_VIA_MAIN1_WESTBOUND))
					return false;

				cpLockAllTurnouts(cpState);
				CPRouteSet(cpState, ROUTE_MAIN2_VIA_MAIN1_EASTBOUND);
				
			} else if (eastCrossover && westCrossover) {
				// Both crossovers normal, straight through route
				// Is there already a conflicting route set?
				if (CPRouteTest(cpState, ROUTE_MAIN2_WESTBOUND))
					return false;

				// Set route
				cpLockAllTurnouts(cpState);
				CPRouteSet(cpState, ROUTE_MAIN2_EASTBOUND);
				return true;
			} else if (!westCrossover && eastCrossover) {
				// West crossover reversed, M2->M1
				if (CPRouteTest(cpState, ROUTE_MAIN1_TO_MAIN2_WESTBOUND))
					return false;

				cpLockAllTurnouts(cpState);
				CPRouteSet(cpState, ROUTE_MAIN2_TO_MAIN1_EASTBOUND);
				return true;
			} else {
				return false;
			}
			break;
			
			
		case ROUTE_ENTR_M2_WESTBOUND:
			if (!eastCrossover && !westCrossover)
			{
				// Main 2 -> Main 2 via Main 1 - icky
				if (CPRouteTest(cpState, ROUTE_MAIN2_VIA_MAIN1_EASTBOUND))
					return false;
					
				cpLockAllTurnouts(cpState);
				CPRouteSet(cpState, ROUTE_MAIN2_VIA_MAIN1_WESTBOUND);
				return true;
			} else if (eastCrossover && westCrossover) {
				// Both crossovers normal, straight through route
				// Is there already a conflicting route set?
				if (CPRouteTest(cpState, ROUTE_MAIN2_EASTBOUND))
					return false;

				// Set route
				cpLockAllTurnouts(cpState);
				CPRouteSet(cpState, ROUTE_MAIN2_WESTBOUND);
				return true;
			} else if (!westCrossover && eastCrossover) {
				// West crossover reversed, M2->M1
				if (CPRouteTest(cpState, ROUTE_MAIN1_TO_MAIN2_EASTBOUND))
					return false;

				cpLockAllTurnouts(cpState);
				CPRouteSet(cpState, ROUTE_MAIN2_TO_MAIN1_WESTBOUND);
				return true;
			} else {
				return false;
			}
			break;
			
		default:
			break;
	}
	
	return false;
}



void cpHandleTurnouts(CPState_t* state, XIOControl* xio)
{
	// Copy over the actual states of each turnout
	CPTurnoutActualDirectionSet(state, TURNOUT_E_XOVER, CPInputStateGet(state, E_XOVER_ACTUAL_POS));
	CPTurnoutActualDirectionSet(state, TURNOUT_W_XOVER, CPInputStateGet(state, W_XOVER_ACTUAL_POS));

	// First, deal with the timelock
	bool manualUnlockSwitchOn = !CPInputStateGet(state, TIMELOCK_SW_POS);//getTimelockSwitchState(xio);
	
	switch(CPTimelockStateGet(state, MAIN_TIMELOCK))
	{
		case STATE_LOCKED:
			if (manualUnlockSwitchOn)
			{
				CPTimelockTimeSet(state, MAIN_TIMELOCK, eeprom_read_byte((uint8_t*)EE_UNLOCK_TIME)/10);
				CPTimelockStateSet(state, MAIN_TIMELOCK, STATE_TIMERUN);
				setTimelockLED(xio, events & EVENT_BLINKY);
				// FIXME: Drop Clearance
			} else {
				setTimelockLED(xio, false);
				CPTurnoutManualOperationsSet(state, TURNOUT_E_XOVER, false);
				CPTurnoutManualOperationsSet(state, TURNOUT_W_XOVER, false);
			}
			break;

		case STATE_TIMERUN:
			if (!manualUnlockSwitchOn)
				CPTimelockStateSet(state, MAIN_TIMELOCK, STATE_LOCKED);
			else if (0 == CPTimelockTimeGet(state, MAIN_TIMELOCK))
				CPTimelockStateSet(state, MAIN_TIMELOCK, STATE_UNLOCKED);
			else
			{
				CPRouteAllClear(state);
				CPTurnoutManualOperationsSet(state, TURNOUT_E_XOVER, true);
				CPTurnoutManualOperationsSet(state, TURNOUT_W_XOVER, true);
				setTimelockLED(xio, events & EVENT_BLINKY);
			}
			break;
					
		case STATE_UNLOCKED:
			setTimelockLED(xio, true);
			CPTurnoutLockSet(state, TURNOUT_E_XOVER, false);
			CPTurnoutLockSet(state, TURNOUT_W_XOVER, false);
			// Set requested directions here based on manual inputs
			if (!manualUnlockSwitchOn)
				CPTimelockStateSet(state, MAIN_TIMELOCK, STATE_RELOCKING);
			else
			{
				CPRouteAllClear(state);
				CPTurnoutManualOperationsSet(state, TURNOUT_E_XOVER, true);
				CPTurnoutManualOperationsSet(state, TURNOUT_W_XOVER, true);

				bool reqPos = CPInputStateGet(state, E_XOVER_MANUAL_POS);
				CPTurnoutRequestedDirectionSet(state, TURNOUT_E_XOVER, reqPos);
				
				reqPos = CPInputStateGet(state, W_XOVER_MANUAL_POS);
				CPTurnoutRequestedDirectionSet(state, TURNOUT_W_XOVER, reqPos);
			}

			break;

		case STATE_RELOCKING:
			// Put anything here that needs to be true before the CP can relock, such as 
			// required switch positions
			if (!manualUnlockSwitchOn)
				CPTimelockStateSet(state, MAIN_TIMELOCK, STATE_LOCKED);
			break;
				
		default: // No idea why we'd get here, but just in case...
			CPTimelockStateSet(state, MAIN_TIMELOCK, STATE_RELOCKING);
			break;
	} 
}



int main(void)
{
	CPState_t cpState;
	XIOControl xio1;
	bool changed = false;
	uint8_t update_decisecs = 20;
	uint8_t lastStatusPacket[MRBUS_BUFFER_SIZE];
	uint8_t mrbTxBuffer[MRBUS_BUFFER_SIZE];
	// Application initialization
	init();

	CPInitialize(&cpState);

	// Initialize a 100 Hz timer. 
	initialize100HzTimer();

	// Initialize MRBus core
	mrbusPktQueueInitialize(&mrbusTxQueue, mrbusTxPktBufferArray, txBuffer_DEPTH);
	mrbusPktQueueInitialize(&mrbusRxQueue, mrbusRxPktBufferArray, rxBuffer_DEPTH);
	mrbusInit();

	sei();

	// Initialize I2C and XIOs for output - needs to have interrupts on for I2C to work
	i2c_master_init();
	xioHardwareReset();
	xioInitialize(&xio1, I2C_XIO0_ADDRESS, xio1PinDirection);

	while (1)
	{
		wdt_reset();

		// Handle any packets that may have come in
		if (mrbusPktQueueDepth(&mrbusRxQueue))
			PktHandler(&cpState);

		// The EVENT_I2C_ERROR flag gets set if a read or write fails for some reason
		// I'm going to assume it's because the I2C bus went heywire, and we need to do
		// a very solid reset on things.  No I2C stuff will happen until this gets cleared
		
		if (events & EVENT_I2C_ERROR)
		{
			i2cResetCounter++;
			xioHardwareReset();
			xioInitialize(&xio1, I2C_XIO0_ADDRESS, xio1PinDirection);
			if (xioIsInitialized(&xio1))
				events &= ~(EVENT_I2C_ERROR); // If we initialized successfully, clear error
		}

		if (events & EVENT_1HZ)
		{
			events &= ~(EVENT_1HZ);
			CPTimelockApply1HzTick(&cpState);
		}

		if(events & (EVENT_READ_INPUTS))
		{
			// Read local  and hardware inputs
			events &= ~(EVENT_READ_INPUTS);
			xioInputRead(&xio1);
			CPXIOInputFilter(&cpState, &xio1);
		}

		// Vital Logic
		cpHandleTurnouts(&cpState, &xio1);
		vitalLogic(&cpState);

		// Send output
		if (events & EVENT_WRITE_OUTPUTS)
		{
			CPSignalsToOutputs(&cpState, &xio1, events & EVENT_BLINKY);
			CPTurnoutsToOutputs(&cpState, &xio1);
			xioOutputWrite(&xio1);
			events &= ~(EVENT_WRITE_OUTPUTS);
		}
		
		uint8_t statusLen = cpStateToStatusPacket(&cpState, mrbTxBuffer, sizeof(mrbTxBuffer));
		
		if (0 != memcmp(mrbTxBuffer, lastStatusPacket, statusLen))
		{
			memset(lastStatusPacket, 0, sizeof(lastStatusPacket));
			memcpy(lastStatusPacket, mrbTxBuffer, statusLen);
			changed = true;
		}
		if(decisecs >= update_decisecs)
			changed = true;

		if (changed)
		{
			mrbusPktQueuePush(&mrbusTxQueue, mrbTxBuffer, statusLen);
			decisecs = 0;
		}

		// If we have a packet to be transmitted, try to send it here
		if (mrbusPktQueueDepth(&mrbusTxQueue))
		{
			uint8_t fail = mrbusTransmit();

			// If we're here, we failed to start transmission due to somebody else transmitting
			// Given that our transmit buffer is full, priority one should be getting that data onto
			// the bus so we can start using our tx buffer again.  So we stay in the while loop, trying
			// to get bus time.

			// We want to wait 20ms before we try a retransmit to avoid hammering the bus
			// Because MRBus has a minimum packet size of 6 bytes @ 57.6kbps,
			// need to check roughly every millisecond to see if we have a new packet
			// so that we don't miss things we're receiving while waiting to transmit
			if (fail)
			{
				uint8_t bus_countdown = 40;
				while (bus_countdown-- > 0 && !mrbusIsBusIdle())
				{
					wdt_reset();
					_delay_ms(1);
					if (mrbusPktQueueDepth(&mrbusRxQueue))
						PktHandler(&cpState);
				}
			}
		}
	}
}

void PktHandler(CPState_t *cpState)
{
	uint16_t crc = 0;
	uint8_t i;
	uint8_t rxBuffer[MRBUS_BUFFER_SIZE];
	uint8_t txBuffer[MRBUS_BUFFER_SIZE];

	if (0 == mrbusPktQueuePop(&mrbusRxQueue, rxBuffer, sizeof(rxBuffer)))
		return;

	//*************** PACKET FILTER ***************
	// Loopback Test - did we send it?  If so, we probably want to ignore it
	if (rxBuffer[MRBUS_PKT_SRC] == mrbus_dev_addr) 
		goto PktIgnore;

	// Destination Test - is this for us or broadcast?  If not, ignore
	if (0xFF != rxBuffer[MRBUS_PKT_DEST] && mrbus_dev_addr != rxBuffer[MRBUS_PKT_DEST]) 
		goto PktIgnore;
	
	// CRC16 Test - is the packet intact?
	for(i=0; i<rxBuffer[MRBUS_PKT_LEN]; i++)
	{
		if ((i != MRBUS_PKT_CRC_H) && (i != MRBUS_PKT_CRC_L)) 
			crc = mrbusCRC16Update(crc, rxBuffer[i]);
	}
	if ((UINT16_HIGH_BYTE(crc) != rxBuffer[MRBUS_PKT_CRC_H]) || (UINT16_LOW_BYTE(crc) != rxBuffer[MRBUS_PKT_CRC_L]))
		goto PktIgnore;
		
	//*************** END PACKET FILTER ***************


	//*************** PACKET HANDLER - PROCESS HERE ***************

	// **** Handle commands addressed to us
	switch(rxBuffer[MRBUS_PKT_TYPE])
	{
		case 'A':
			// PING packet
			txBuffer[MRBUS_PKT_DEST] = rxBuffer[MRBUS_PKT_SRC];
			txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
			txBuffer[MRBUS_PKT_LEN] = 6;
			txBuffer[MRBUS_PKT_TYPE] = 'a';
			mrbusPktQueuePush(&mrbusTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
			goto PktIgnore;

		case 'C':
			// CTC Command
			// Structure of command:
			//  byte 6:
			//    'G' - Set/Clear route from entrance signal (byte 7 signal number, byte 8 'S'/'C' for set/clear)
			//    'T' - Set turnout (byte 7) normal or diverging ('M'/'D' - byte 8)
			if (rxBuffer[MRBUS_PKT_LEN] >= 9 && 'G' == rxBuffer[6] && ('S' == rxBuffer[8] || 'C' == rxBuffer[8]))
				cpCodeRoute(cpState, rxBuffer[7], ('S' == rxBuffer[8])?true:false);

			if (rxBuffer[MRBUS_PKT_LEN] >= 9 && 'T' == rxBuffer[6] && ('M' == rxBuffer[8] || 'D' == rxBuffer[8]))
				cpSetTurnout(cpState, rxBuffer[7], ('M'==rxBuffer[8])?true:false);

			goto PktIgnore;

		case 'W':
			// EEPROM WRITE Packet

			// EEPROM Write packets must be directed at us and us only
			if (rxBuffer[MRBUS_PKT_DEST] != mrbus_dev_addr)
				goto PktIgnore;
			
			txBuffer[MRBUS_PKT_DEST] = rxBuffer[MRBUS_PKT_SRC];
			txBuffer[MRBUS_PKT_LEN] = 8;			
			txBuffer[MRBUS_PKT_TYPE] = 'w';
			eeprom_write_byte((uint8_t*)(uint16_t)rxBuffer[6], rxBuffer[7]);
			txBuffer[6] = rxBuffer[6];
			txBuffer[7] = rxBuffer[7];
			if (MRBUS_EE_DEVICE_ADDR == rxBuffer[6])
				mrbus_dev_addr = eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR);
			txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
			mrbusPktQueuePush(&mrbusTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
			goto PktIgnore;	

		case 'R':
			// EEPROM READ Packet
			txBuffer[MRBUS_PKT_DEST] = rxBuffer[MRBUS_PKT_SRC];
			txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
			txBuffer[MRBUS_PKT_LEN] = 8;
			txBuffer[MRBUS_PKT_TYPE] = 'r';
			txBuffer[6] = rxBuffer[6];
			txBuffer[7] = eeprom_read_byte((uint8_t*)(uint16_t)rxBuffer[6]);
			mrbusPktQueuePush(&mrbusTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
			goto PktIgnore;

		case 'V':
			// Version
			txBuffer[MRBUS_PKT_DEST] = rxBuffer[MRBUS_PKT_SRC];
			txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
			txBuffer[MRBUS_PKT_LEN] = 16;
			txBuffer[MRBUS_PKT_TYPE] = 'v';
			txBuffer[6]  = MRBUS_VERSION_WIRED;
			txBuffer[7]  = 0; // Software Revision
			txBuffer[8]  = 0; // Software Revision
			txBuffer[9]  = 0; // Software Revision
			txBuffer[10]  = 0; // Hardware Major Revision
			txBuffer[11]  = 0; // Hardware Minor Revision
			txBuffer[12] = 'C';
			txBuffer[13] = 'P';
			txBuffer[14] = '3';
			txBuffer[15] = ' ';
			mrbusPktQueuePush(&mrbusTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
			goto PktIgnore;

		case 'X':
			// Reset
			cli();
			wdt_reset();
			MCUSR &= ~(_BV(WDRF));
			WDTCSR |= _BV(WDE) | _BV(WDCE);
			WDTCSR = _BV(WDE);
			while(1);  // Force a watchdog reset, hopefully
			sei();
			break;
	}


	/*************** NOT A PACKET WE EXPLICITLY UNDERSTAND, TRY BIT/BYTE RULES ***************/
	CPMRBusVirtInputFilter(cpState, rxBuffer);

	//*************** END PACKET HANDLER  ***************

	
	//*************** RECEIVE CLEANUP ***************
PktIgnore:
	// Yes, I hate gotos as well, but sometimes they're a really handy and efficient
	// way to jump to a common block of cleanup code at the end of a function 

	// This section resets anything that needs to be reset in order to allow us to receive
	// another packet.  Typically, that's just clearing the MRBUS_RX_PKT_READY flag to 
	// indicate to the core library that the rxBuffer is clear.
	return;	
}


