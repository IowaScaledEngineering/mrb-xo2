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
		}

		if (buttonLockout != 0)
			buttonLockout--;

		events |= EVENT_WRITE_OUTPUTS;
	}
}

// End of 100Hz timer


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

void SetTurnout(uint8_t turnout, uint8_t points)
{
	/*uint8_t options = eeprom_read_byte((uint8_t*)EE_OPTIONS);

	if (POINTS_UNAFFECTED == points)
		return;

	switch(turnout)
	{
		case TURNOUT_E_XOVER:
			if (POINTS_REVERSE_FORCE == points || (POINTS_REVERSE_SAFE == points && !(occupancy[0] & OCC_OS_SECT)))
			{
				turnouts |= PNTS_EX_CNTL;
				// Implementation-specific behaviour - do whatever needs to happen to physically move the turnout here
				if (options & 0x01)
					xio1Outputs[2] |= PNTS_EX_CNTL;
				else
					xio1Outputs[2] &= ~(PNTS_EX_CNTL); 
			}
			else if (POINTS_NORMAL_FORCE == points || (POINTS_NORMAL_SAFE == points && !(occupancy[0] & OCC_OS_SECT)))
			{
				turnouts &= ~(PNTS_EX_CNTL);
				// Implementation-specific behaviour - do whatever needs to happen to physically move the turnout here
				if (options & 0x01)
					xio1Outputs[2] &= ~(PNTS_EX_CNTL); 
				else
					xio1Outputs[2] |= PNTS_EX_CNTL;
			}
			break;

		case TURNOUT_W_XOVER:
			if (POINTS_REVERSE_FORCE == points || (POINTS_REVERSE_SAFE == points && !(occupancy[0] & OCC_OS_SECT)))
			{
				turnouts |= PNTS_WX_CNTL;
				// Implementation-specific behaviour - do whatever needs to happen to physically move the turnout here
				if (options & 0x02)
					xio1Outputs[2] |= PNTS_WX_CNTL;
				else
					xio1Outputs[2] &= ~(PNTS_WX_CNTL); 
			}
			else if (POINTS_NORMAL_FORCE == points || (POINTS_NORMAL_SAFE == points && !(occupancy[0] & OCC_OS_SECT)))
			{
				turnouts &= ~(PNTS_WX_CNTL);
				// Implementation-specific behaviour - do whatever needs to happen to physically move the turnout here
				if (options & 0x02)
					xio1Outputs[2] &= ~(PNTS_WX_CNTL); 
				else
					xio1Outputs[2] |= PNTS_WX_CNTL;
			}
			break;

		default:
			break;
	}*/
}

uint8_t GetTurnout(uint8_t turnout)
{
/*	switch(turnout)
	{
		case TURNOUT_E_XOVER:
			return ((turnouts & PNTS_EX_CNTL)?1:0);
		case TURNOUT_W_XOVER:
			return ((turnouts & PNTS_WX_CNTL)?1:0);
	}

	return(0);*/
	return 0;
}

uint8_t GetClearance(uint8_t controlPoint)
{
/*	switch(controlPoint)
	{
		case CONTROLPOINT_1:
			return(clearance & 0x0F);

	}
	return(CLEARANCE_NONE); */
	return 0;
}

void SetClearance(uint8_t controlPoint, uint8_t newClear)
{
/*	if (CLEARANCE_NONE != newClear 
		&& CLEARANCE_EAST  != newClear 
		&& CLEARANCE_WEST != newClear)
		return;

	switch(controlPoint)
	{
		case CONTROLPOINT_1:
			if (CLEARANCE_NONE != newClear)
			{
				if (OCC_OS_SECT & occupancy[0])
					break;
			}
			clearance &= 0xF0;
			clearance |= newClear;
			break;

		default:
			break;
	}*/
}

void CodeCTCRoute(uint8_t controlPoint, uint8_t newPointsE, uint8_t newPointsW, uint8_t newClear)
{
	/*SetTurnout(TURNOUT_E_XOVER, newPointsE);
	SetTurnout(TURNOUT_W_XOVER, newPointsW);
	SetClearance(controlPoint, newClear);*/
}



static inline void vitalLogic(CPState_t *cpState)
{
//	uint8_t turnoutLocked = (!(((turnouts & (PNTS_EX_STATUS | PNTS_WX_STATUS))?1:0) ^ ((turnouts & (PNTS_EX_STATUS | PNTS_WX_STATUS))?1:0)));
//	uint8_t cleared = CLEARANCE_NONE;

	// Start out with a safe default - everybody red
	CPSignalHeadAllSetAspect(cpState, ASPECT_RED);

/*	// Drop clearance if we see occupancy
	if (occupancy[0] & OCC_OS_SECT)
		SetClearance(CONTROLPOINT_1, CLEARANCE_NONE);

	cleared = GetClearance(CONTROLPOINT_1);
	
	if (STATE_UNLOCKED == turnoutState || STATE_RELOCKING == turnoutState)
	{

		if(turnouts & (PNTS_EX_STATUS))
		{
			signalHeads[SIG_PNTS_LOWER] = ASPECT_FL_RED;
			signalHeads[SIG_MAIN_A] = ASPECT_FL_RED;
		}
		else if (!(turnouts & (PNTS_EX_STATUS)) && (turnouts & (PNTS_WX_STATUS)))
		{
			signalHeads[SIG_PNTS_LOWER] = ASPECT_FL_RED;
			signalHeads[SIG_MAIN_C] = ASPECT_FL_RED;
		}
		else if (!(turnouts & (PNTS_EX_STATUS)) && !(turnouts & (PNTS_WX_STATUS)))
		{
			signalHeads[SIG_PNTS_UPPER] = ASPECT_FL_RED;
			signalHeads[SIG_MAIN_B] = ASPECT_FL_RED;
		}

	} 
	else if (turnoutLocked && CLEARANCE_EAST == cleared)
	{
		// Eastbound clearance at the east control point means frog->points movement direction
		uint8_t head = 0;
		
		if(turnouts & (PNTS_EX_STATUS))
		{
			head = SIG_MAIN_A;
		}
		else if (!(turnouts & (PNTS_EX_STATUS)) && (turnouts & (PNTS_WX_STATUS)))
		{
			head = SIG_MAIN_C;
		}
		else if (!(turnouts & (PNTS_EX_STATUS)) && !(turnouts & (PNTS_WX_STATUS)))
		{
			head = SIG_MAIN_B;
		}
		
		if (getExtInput(ext_occupancy, sizeof(ext_occupancy), XOCC_P_ADJOIN) || (OCC_OS_SECT & occupancy[0]))
			signalHeads[head] = ASPECT_RED;
		else if (getExtInput(ext_occupancy, sizeof(ext_occupancy), XOCC_P_APPROACH))
			signalHeads[head] = ASPECT_YELLOW;
		else if (getExtInput(ext_occupancy, sizeof(ext_occupancy), XOCC_P_APPROACH2))
			signalHeads[head] = ASPECT_FL_YELLOW;
		else
			signalHeads[head] = ASPECT_GREEN;
	}
	else if (turnoutLocked && CLEARANCE_WEST == cleared)
	{
		// Westbound clearance at the east control point means points->frog movement direction	
		if(turnouts & (PNTS_EX_STATUS))
		{
			// Lined to siding
			if ((OCC_OS_SECT & occupancy[0]) || getExtInput(ext_occupancy, sizeof(ext_occupancy), XOCC_MA_ADJOIN))
				signalHeads[SIG_PNTS_LOWER] = ASPECT_RED;
			else if (getExtInput(ext_occupancy, sizeof(ext_occupancy), XOCC_MA_APPROACH))
				signalHeads[SIG_PNTS_LOWER] = ASPECT_YELLOW;
			else if (getExtInput(ext_occupancy, sizeof(ext_occupancy), XOCC_MA_APPROACH2))
				signalHeads[SIG_PNTS_LOWER] = ASPECT_FL_YELLOW;
			else
				signalHeads[SIG_PNTS_LOWER] = ASPECT_GREEN;
		}
		else if (!(turnouts & (PNTS_EX_STATUS)) && (turnouts & (PNTS_WX_STATUS)))
		{
			if ((OCC_OS_SECT & occupancy[0]) || getExtInput(ext_occupancy, sizeof(ext_occupancy), XOCC_MC_ADJOIN))
				signalHeads[SIG_PNTS_LOWER] = ASPECT_RED;
			else if (getExtInput(ext_occupancy, sizeof(ext_occupancy), XOCC_MC_APPROACH))
				signalHeads[SIG_PNTS_LOWER] = ASPECT_YELLOW;
			else if (getExtInput(ext_occupancy, sizeof(ext_occupancy), XOCC_MC_APPROACH2))
				signalHeads[SIG_PNTS_LOWER] = ASPECT_FL_YELLOW;
			else
				signalHeads[SIG_PNTS_LOWER] = ASPECT_GREEN;
		}
		else if (!(turnouts & (PNTS_EX_STATUS)) && !(turnouts & (PNTS_WX_STATUS)))
		{
			if ((OCC_OS_SECT & occupancy[0]) || getExtInput(ext_occupancy, sizeof(ext_occupancy), XOCC_MB_ADJOIN))
				signalHeads[SIG_PNTS_UPPER] = ASPECT_RED;
			else if (getExtInput(ext_occupancy, sizeof(ext_occupancy), XOCC_MB_APPROACH))
				signalHeads[SIG_PNTS_UPPER] = ASPECT_YELLOW;
			else if (getExtInput(ext_occupancy, sizeof(ext_occupancy), XOCC_MB_APPROACH2))
				signalHeads[SIG_PNTS_UPPER] = ASPECT_FL_YELLOW;
			else
				signalHeads[SIG_PNTS_UPPER] = ASPECT_GREEN;
		}
	}
	// The else case is that the turnout isn't locked up or we're not cleared
	// Good news - the signals are already defaulted to red

	// Clear virtual occupancies
	occupancy[0] &= ~(OCC_VIRT_P_APPROACH | OCC_VIRT_P_ADJOIN | OCC_VIRT_MA_APPROACH | OCC_VIRT_MA_ADJOIN | OCC_VIRT_MB_APPROACH | OCC_VIRT_MB_ADJOIN);
	occupancy[1] &= ~(OCC_VIRT_MC_APPROACH | OCC_VIRT_MC_ADJOIN);

	// Calculate east CP virtual occupancies
	if(turnoutLocked)
	{
		if(ASPECT_FL_RED == signalHeads[SIG_MAIN_A] || ASPECT_RED == signalHeads[SIG_MAIN_A])
			occupancy[0] |= OCC_VIRT_MA_ADJOIN | OCC_VIRT_MA_APPROACH;
		else if (ASPECT_YELLOW == signalHeads[SIG_MAIN_A])
			occupancy[0] |= OCC_VIRT_MA_APPROACH;

		if(ASPECT_FL_RED == signalHeads[SIG_MAIN_B] || ASPECT_RED == signalHeads[SIG_MAIN_B])
			occupancy[0] |= OCC_VIRT_MB_ADJOIN | OCC_VIRT_MB_APPROACH;
		else if (ASPECT_YELLOW == signalHeads[SIG_MAIN_B])
			occupancy[0] |= OCC_VIRT_MB_APPROACH;

		if(ASPECT_FL_RED == signalHeads[SIG_MAIN_C] || ASPECT_RED == signalHeads[SIG_MAIN_C])
			occupancy[1] |= OCC_VIRT_MC_ADJOIN | OCC_VIRT_MC_APPROACH;
		else if (ASPECT_YELLOW == signalHeads[SIG_MAIN_C])
			occupancy[1] |= OCC_VIRT_MC_APPROACH;

		
		// Turnout is properly lined one way or the other
		if ((ASPECT_FL_RED == signalHeads[SIG_PNTS_LOWER] || ASPECT_RED == signalHeads[SIG_PNTS_LOWER]) && (ASPECT_RED == signalHeads[SIG_PNTS_UPPER] || ASPECT_FL_RED == signalHeads[SIG_PNTS_UPPER]))
			occupancy[0] |= OCC_VIRT_P_ADJOIN | OCC_VIRT_P_APPROACH;
		else if (ASPECT_YELLOW == signalHeads[SIG_PNTS_LOWER] || ASPECT_FL_YELLOW == signalHeads[SIG_PNTS_LOWER] || ASPECT_YELLOW == signalHeads[SIG_PNTS_UPPER] || ASPECT_FL_YELLOW == signalHeads[SIG_PNTS_UPPER])
			occupancy[0] |= OCC_VIRT_P_APPROACH;
	
	} else {
		//  Control Point improperly lined, trip virtual occupancy
		occupancy[0] |= OCC_VIRT_P_APPROACH | OCC_VIRT_P_ADJOIN | OCC_VIRT_MA_APPROACH | OCC_VIRT_MA_ADJOIN | OCC_VIRT_MB_APPROACH | OCC_VIRT_MB_ADJOIN;
		occupancy[1] |= OCC_VIRT_MC_APPROACH | OCC_VIRT_MC_ADJOIN;
	}*/

}

bool pointsUnlockedSwitch()
{
/*	bool timelockSwitchInverted = (OPTIONS_INVERT_TIMELOCK & eeprom_read_byte((uint8_t*)EE_OPTIONS));
	bool retval = (debounced_inputs[0] & PNTS_UNLOCK)?false:true;
	if (timelockSwitchInverted)
		retval = !retval;
	return retval;*/
	return false;
}

// For the XIO pins, 0 is output, 1 is input
const uint8_t const xio1PinDirection[5] = { 0x00, 0x00, 0x00, 0x00, 0x00 };

int main(void)
{
	CPState_t cpState;
	XIOControl xio1;
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

		if(events & (EVENT_READ_INPUTS))
		{
			// Read local  and hardware inputs
			events &= ~(EVENT_READ_INPUTS);
			xioInputRead(&xio1);
		}

		// Vital Logic
		vitalLogic(&cpState);

		// Send output
		if (events & EVENT_WRITE_OUTPUTS)
		{
			//uint8_t comAnodeHeads = eeprom_read_byte((uint8_t*)EE_HEADS_COM_ANODE);
//			SignalsToOutputs(comAnodeHeads);
			xioOutputWrite(&xio1);
			events &= ~(EVENT_WRITE_OUTPUTS);
		}
	}
/*
		if (CPStateChange(cpOldState, cpState))
		{
			
			
		}

		// Test if something changed from the last time
		// around the loop - we need to send an update 
		//   packet if it did 
	
		if (memcmp(signalHeads, old_signalHeads, sizeof(signalHeads))
			|| old_turnouts != turnouts
			|| old_clearance != clearance
			|| old_occupancy[0] != occupancy[0]
			|| old_occupancy[1] != occupancy[1])
		{
			// Something Changed - time to update
			for(i=0; i<sizeof(signalHeads); i++)
				old_signalHeads[i] = signalHeads[i];
			for(i=0; i<sizeof(ext_occupancy); i++)
				old_ext_occupancy[i] = ext_occupancy[i];
			for(i=0; i<sizeof(occupancy); i++)
				old_occupancy[i] = occupancy[i];

			old_turnouts = turnouts;
			old_clearance = clearance;

			// Set changed such that a packet gets sent
			changed = 1;
		}
		else if (decisecs >= update_decisecs)
			changed = 1;

		if (changed)
		{
			ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
			{
				if (decisecs > update_decisecs)
					decisecs -= update_decisecs;
				else
					decisecs = 0;
			}
		}
		// If we need to send a packet and we're not already busy...
#define MRB_STATUS_CP_CLEARED_EAST      0x01
#define MRB_STATUS_CP_CLEARED_WEST      0x02
#define MRB_STATUS_CP_CLEARED_NONE      0x04
#define MRB_STATUS_CP_MANUAL_UNLOCK     0x08
#define MRB_STATUS_CP_SWITCH_EX_NORMAL  0x10
#define MRB_STATUS_CP_SWITCH_EX_REVERSE 0x20
#define MRB_STATUS_CP_SWITCH_WX_NORMAL  0x40
#define MRB_STATUS_CP_SWITCH_WX_REVERSE 0x80

		if (changed && !mrbusPktQueueFull(&mrbusTxQueue))
		{
			uint8_t txBuffer[MRBUS_BUFFER_SIZE];
			txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
			txBuffer[MRBUS_PKT_DEST] = 0xFF;
			txBuffer[MRBUS_PKT_LEN] = 12;
			txBuffer[5] = 'S';
			txBuffer[6] = ((signalHeads[SIG_MAIN_B]<<4) & 0xF0) | (signalHeads[SIG_MAIN_A] & 0x0F);
			txBuffer[7] = (signalHeads[SIG_MAIN_C] & 0x0F) | ((occupancy[1]<<4) & 0xF0);
			txBuffer[8] = ((signalHeads[SIG_PNTS_UPPER]<<4) & 0xF0) | (signalHeads[SIG_PNTS_LOWER] & 0x0F);
			
			txBuffer[9] = occupancy[0];
			
			switch(GetClearance(CONTROLPOINT_1))
			{
				case CLEARANCE_EAST:
					txBuffer[10] = MRB_STATUS_CP_CLEARED_EAST;
					break;
				case CLEARANCE_WEST:
					txBuffer[10] = MRB_STATUS_CP_CLEARED_WEST;
					break;
				case CLEARANCE_NONE:
				default:
					txBuffer[10] = MRB_STATUS_CP_CLEARED_NONE;
					break;
			}

			if (STATE_LOCKED != turnoutState)
				txBuffer[10] |= MRB_STATUS_CP_MANUAL_UNLOCK;

			if (turnouts & PNTS_EX_STATUS)  // Low is normal, high is reverse
				txBuffer[10] |= MRB_STATUS_CP_SWITCH_EX_REVERSE;
			else
				txBuffer[10] |= MRB_STATUS_CP_SWITCH_EX_NORMAL;

			if (turnouts & PNTS_WX_STATUS)  // Low is normal, high is reverse
				txBuffer[10] |= MRB_STATUS_CP_SWITCH_WX_REVERSE;
			else
				txBuffer[10] |= MRB_STATUS_CP_SWITCH_WX_NORMAL;


			txBuffer[11] = i2cResetCounter;
			mrbusPktQueuePush(&mrbusTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
			changed = 0;
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
				uint8_t bus_countdown = 20;
				while (bus_countdown-- > 0 && !mrbusIsBusIdle())
				{
					wdt_reset();
					_delay_ms(1);
					if (mrbusPktQueueDepth(&mrbusRxQueue))
						PktHandler();
				}
			}
		}*/
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

/*			if (rxBuffer[MRBUS_PKT_LEN] >= 9 && 'G' == rxBuffer[6] && ('S' == rxBuffer[8] || 'C' == rxBuffer[8]))
				CodeCTCRoute(rxBuffer[7], rxBuffer[8]);

			if (rxBuffer[MRBUS_PKT_LEN] >= 9 && 'T' == rxBuffer[6] && ('S' == rxBuffer[8] || 'C' == rxBuffer[8]))
				SetTurnout(rxBuffer[7], rxBuffer[8]);*/

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


