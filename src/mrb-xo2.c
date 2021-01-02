/*************************************************************************
Title:    MRBus Simple CTC Control Point Node
Authors:  Nathan D. Holmes <maverick@drgw.net>
File:     $Id: $
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2020 Nathan Holmes

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


typedef enum
{
	STATE_LOCKED = 0,
	STATE_TIMERUN = 1,
	STATE_UNLOCKED = 2,
	STATE_RELOCKING = 3
} turnoutState_t;

turnoutState_t turnoutState = STATE_LOCKED;

void PktHandler(void);

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

#define CONTROLPOINT_1       0x01

#define CLEARANCE_NONE		0x00
#define CLEARANCE_EAST		0x01
#define CLEARANCE_WEST		0x02

#define POINTS_NORMAL_SAFE    'M'
#define POINTS_REVERSE_SAFE   'D'
#define POINTS_NORMAL_FORCE   'm'
#define POINTS_REVERSE_FORCE  'd'
#define POINTS_UNAFFECTED     'X'

#define OCC_OS_SECT           0x01
#define OCC_VIRT_P_ADJOIN     0x02
#define OCC_VIRT_P_APPROACH   0x04
#define OCC_VIRT_M_ADJOIN     0x10
#define OCC_VIRT_M_APPROACH   0x20
#define OCC_VIRT_S_ADJOIN     0x40
#define OCC_VIRT_S_APPROACH   0x80

#define XOCC_P_ADJOIN         0x01
#define XOCC_P_APPROACH       0x02
#define XOCC_P_APPROACH2      0x04
#define XOCC_P_TUMBLE         0x08
#define XOCC_M_ADJOIN         0x10
#define XOCC_M_APPROACH       0x20
#define XOCC_M_APPROACH2      0x40
#define XOCC_M_TUMBLE         0x80

#define XOCC2_S_ADJOIN        0x01
#define XOCC2_S_APPROACH      0x02
#define XOCC2_S_APPROACH2     0x04
#define XOCC2_S_TUMBLE        0x08

#define PNTS_TIMELOCK_LED     0x01

#define PNTS_LOCAL_DIR        0x01
#define PNTS_UNLOCK           0x04
#define PNTS_CNTL             0x10
#define PNTS_STATUS           0x40

// EEPROM Location Definitions
#define EE_HEADS_COM_ANODE    0x07
#define EE_OPTIONS            0x08

#define EE_UNLOCK_TIME        0x09
// Unlock time in decisecs

#define EE_P_APRCH_ADDR       0x10
#define EE_P_APRCH2_ADDR      0x11
#define EE_P_ADJ_ADDR         0x12
#define EE_M_APRCH_ADDR       0x13
#define EE_M_APRCH2_ADDR      0x14
#define EE_M_ADJ_ADDR         0x15
#define EE_S_APRCH_ADDR       0x16
#define EE_S_APRCH2_ADDR      0x17
#define EE_S_ADJ_ADDR         0x18
#define EE_P_TUMBLE_ADDR      0x19
#define EE_M_TUMBLE_ADDR      0x1A
#define EE_S_TUMBLE_ADDR      0x1B
#define EE_OS_ADDR            0x1C

#define EE_P_APRCH_PKT        0x20
#define EE_P_APRCH2_PKT       0x21
#define EE_P_ADJ_PKT          0x22
#define EE_M_APRCH_PKT        0x23
#define EE_M_APRCH2_PKT       0x24
#define EE_M_ADJ_PKT          0x25
#define EE_S_APRCH_PKT        0x26
#define EE_S_APRCH2_PKT       0x27
#define EE_S_ADJ_PKT          0x28
#define EE_P_TUMBLE_PKT       0x29
#define EE_M_TUMBLE_PKT       0x2A
#define EE_S_TUMBLE_PKT       0x2B
#define EE_OS_PKT             0x2C

#define EE_P_APRCH_BITBYTE    0x30
#define EE_P_APRCH2_BITBYTE   0x31
#define EE_P_ADJ_BITBYTE      0x32
#define EE_M_APRCH_BITBYTE    0x33
#define EE_M_APRCH2_BITBYTE   0x34
#define EE_M_ADJ_BITBYTE      0x35
#define EE_S_APRCH_BITBYTE    0x36
#define EE_S_APRCH2_BITBYTE   0x37
#define EE_S_ADJ_BITBYTE      0x38
#define EE_P_TUMBLE_BITBYTE   0x39
#define EE_M_TUMBLE_BITBYTE   0x3A
#define EE_S_TUMBLE_BITBYTE   0x3B
#define EE_OS_BITBYTE         0x3C

#define EE_P_APRCH_SUBTYPE    0x40
#define EE_P_APRCH2_SUBTYPE   0x41
#define EE_P_ADJ_SUBTYPE      0x42
#define EE_M_APRCH_SUBTYPE    0x43
#define EE_M_APRCH2_SUBTYPE   0x44
#define EE_M_ADJ_SUBTYPE      0x45
#define EE_S_APRCH_SUBTYPE    0x46
#define EE_S_APRCH2_SUBTYPE   0x47
#define EE_S_ADJ_SUBTYPE      0x48
#define EE_P_TUMBLE_SUBTYPE   0x49
#define EE_M_TUMBLE_SUBTYPE   0x4A
#define EE_S_TUMBLE_SUBTYPE   0x4B
#define EE_OS_SUBTYPE         0x4C



uint8_t debounced_inputs[2], old_debounced_inputs[2];
uint8_t clearance, old_clearance;
uint8_t occupancy, old_occupancy;
uint8_t ext_occupancy, old_ext_occupancy;
uint8_t ext_occupancy2, old_ext_occupancy2;
uint8_t turnouts, old_turnouts;
uint8_t clock_a[2] = {0,0}, clock_b[2] = {0,0};
uint8_t signalHeads[8], old_signalHeads[8];

uint8_t PktDirToClearance(uint8_t pktDir)
{
	switch(pktDir)
	{
		case 'E':
			return(CLEARANCE_EAST);
		case 'W':
			return(CLEARANCE_WEST);
		default:
			return(CLEARANCE_NONE);
	}

}

// Aspect Definitions

#define ASPECT_LUNAR     0x07
#define ASPECT_FL_RED    0x06
#define ASPECT_FL_GREEN  0x05
#define ASPECT_RED       0x04
#define ASPECT_FL_YELLOW 0x03
#define ASPECT_YELLOW    0x02
#define ASPECT_GREEN     0x01
#define ASPECT_OFF       0x00

// Signal Head Definitions

#define SIG_PNTS_UPPER 0
#define SIG_PNTS_LOWER 1
#define SIG_MAIN       2
#define SIG_SIDING     3

// ******** Start 100 Hz Timer, 0.16% error version (Timer 0)
// If you can live with a slightly less accurate timer, this one only uses Timer 0, leaving Timer 1 open
// for more advanced things that actually need a 16 bit timer/counter

// Initialize a 100Hz timer for use in triggering events.
// If you need the timer resources back, you can remove this, but I find it
// rather handy in triggering things like periodic status transmissions.
// If you do remove it, be sure to yank the interrupt handler and ticks/secs as well
// and the call to this function in the main function


volatile uint8_t ticks = 0;
volatile uint16_t decisecs=0;
uint16_t update_decisecs=10;
volatile uint8_t blinkyCounter = 0;
volatile uint8_t buttonLockout=5;

uint8_t i2cResetCounter = 0;
volatile uint8_t timeCounter = 0;

void initialize100HzTimer(void)
{
	// Set up timer 1 for 100Hz interrupts
	TCNT0 = 0;
	OCR0A = 0xC2;
	ticks = 0;
	decisecs = 0;
	TCCR0A = _BV(WGM01);
	TCCR0B = _BV(CS02) | _BV(CS00);
	TIMSK0 |= _BV(OCIE0A);
}

ISR(TIMER0_COMPA_vect)
{
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

		if (0 != timeCounter)
			timeCounter--;

		events |= EVENT_WRITE_OUTPUTS;
	}
}

// End of 100Hz timer


void init(void)
{
	uint8_t i;
	// Clear watchdog
	MCUSR = 0;
	// If you don't want the watchdog to do system reset, remove this chunk of code
	wdt_reset();
	wdt_enable(WDTO_1S);
	wdt_reset();

	// Initialize MRBus address from EEPROM address 1
	mrbus_dev_addr = eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR);
	// Bogus addresses, fix to default address
	if (0xFF == mrbus_dev_addr || 0x00 == mrbus_dev_addr)
	{
		mrbus_dev_addr = 0x03;
		eeprom_write_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR, mrbus_dev_addr);
	}

	update_decisecs = (uint16_t)eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_L) 
		| (((uint16_t)eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_H)) << 8);

	update_decisecs = max(1, update_decisecs);

	// Setup ADC for bus voltage monitoring
	busVoltageMonitorInit();

	// Initialize signals and such
	for(i=0; i<sizeof(debounced_inputs); i++)
		debounced_inputs[i] = old_debounced_inputs[i] = 0;
	
	for(i=0; i<sizeof(signalHeads); i++)
		signalHeads[i] = old_signalHeads[i] = ASPECT_RED;

	clearance = old_clearance = 0;
	occupancy = old_occupancy = 0;
	ext_occupancy = old_ext_occupancy = 0;
	ext_occupancy2 = old_ext_occupancy2 = 0;
	turnouts = old_turnouts = 0;
}

uint8_t xio1Outputs[4];
uint8_t xio1Inputs[2];

/* 0x00-0x04 - input registers */
/* 0x08-0x0C - output registers */
/* 0x18-0x1C - direction registers - 0 is output, 1 is input */

#define I2C_RESET         0
#define I2C_OUTPUT_ENABLE 1
#define I2C_IRQ           2
#define I2C_XIO1_ADDRESS 0x4E

void xioDirectionSet()
{
	uint8_t i2cBuf[8];

	i2cBuf[0] = I2C_XIO1_ADDRESS;
	i2cBuf[1] = 0x80 | 0x18;  // 0x80 is auto-increment
	i2cBuf[2] = 0;
	i2cBuf[3] = 0;
	i2cBuf[4] = 0;
	i2cBuf[5] = 0x40;
	i2cBuf[6] = 0x05;
	i2c_transmit(i2cBuf, 7, 1);
	while(i2c_busy());
}

void xioInitialize()
{
	events |= EVENT_I2C_ERROR;

	PORTB &= ~(_BV(I2C_RESET) | _BV(I2C_OUTPUT_ENABLE));
	DDRB |= _BV(I2C_RESET) | _BV(I2C_OUTPUT_ENABLE);
	_delay_us(1);
	PORTB &= ~(_BV(I2C_OUTPUT_ENABLE));
	PORTB |= _BV(I2C_RESET);
	_delay_us(1);

	xioDirectionSet();
	
	if (i2c_transaction_successful())
	{
		buttonLockout = 5;
		events &= ~(EVENT_I2C_ERROR);
	}
}

void xioOutputWrite()
{
	uint8_t i2cBuf[8];
	uint8_t i;

	// Reset the direction, in case noise killed us somehow
	xioDirectionSet();

	if (!i2c_transaction_successful())
		events |= EVENT_I2C_ERROR;

	i2cBuf[0] = I2C_XIO1_ADDRESS;
	i2cBuf[1] = 0x80 | 0x08;  // 0x80 is auto-increment
	for(i=0; i<sizeof(xio1Outputs); i++)
		i2cBuf[2+i] = xio1Outputs[i];

	i2c_transmit(i2cBuf, 2+sizeof(xio1Outputs), 1);
}

void xioInputRead()
{
	uint8_t i2cBuf[4];
	uint8_t successful = 0;

	if (events & EVENT_I2C_ERROR)
		return;

	while(i2c_busy());

	if (!i2c_transaction_successful())
		events |= EVENT_I2C_ERROR;

	i2cBuf[0] = I2C_XIO1_ADDRESS;
	i2cBuf[1] = 0x80 | 0x03;  // 0x80 is auto-increment, 0x03 is the first register with inputs
	i2c_transmit(i2cBuf, 2, 0);
	i2cBuf[0] = I2C_XIO1_ADDRESS | 0x01;
	i2c_transmit(i2cBuf, 3, 1);
	while(i2c_busy());
	successful = i2c_receive(i2cBuf, 3);

	if (!successful)
		// In the event of a read hose-out, don't put crap in the input buffer
		events |= EVENT_I2C_ERROR;
	else
	{
		xio1Inputs[0] = i2cBuf[1];
		xio1Inputs[1] = i2cBuf[2];	
	}
}

void SetTurnout(uint8_t controlPoint, uint8_t points)
{
	uint8_t options = eeprom_read_byte((uint8_t*)EE_OPTIONS);

	if (POINTS_UNAFFECTED == points)
		return;

	switch(controlPoint)
	{
		case CONTROLPOINT_1:
			if (POINTS_REVERSE_FORCE == points || (POINTS_REVERSE_SAFE == points && !(occupancy & OCC_OS_SECT)))
			{
				turnouts |= PNTS_CNTL;
				// Implementation-specific behaviour - do whatever needs to happen to physically move the turnout here
				if (options & 0x01)
					xio1Outputs[3] |= PNTS_CNTL;
				else
					xio1Outputs[3] &= ~(PNTS_CNTL); 
			}
			else if (POINTS_NORMAL_FORCE == points || (POINTS_NORMAL_SAFE == points && !(occupancy & OCC_OS_SECT)))
			{
				turnouts &= ~(PNTS_CNTL);
				// Implementation-specific behaviour - do whatever needs to happen to physically move the turnout here
				if (options & 0x01)
					xio1Outputs[3] &= ~(PNTS_CNTL); 
				else
					xio1Outputs[3] |= PNTS_CNTL;

			}
			break;
		default:
			break;
	}
}

uint8_t GetTurnout(uint8_t controlPoint)
{
	switch(controlPoint)
	{
		case CONTROLPOINT_1:
			return ((turnouts & PNTS_CNTL)?1:0);
	}

	return(CLEARANCE_NONE);
}

uint8_t GetClearance(uint8_t controlPoint)
{
	switch(controlPoint)
	{
		case CONTROLPOINT_1:
			return(clearance & 0x0F);

	}
	return(CLEARANCE_NONE);
}

void SetClearance(uint8_t controlPoint, uint8_t newClear)
{
	if (CLEARANCE_NONE != newClear 
		&& CLEARANCE_EAST  != newClear 
		&& CLEARANCE_WEST != newClear)
		return;

	switch(controlPoint)
	{
		case CONTROLPOINT_1:
			if (CLEARANCE_NONE != newClear)
			{
				if (XOCC_P_TUMBLE & ext_occupancy)
					break;
				if (OCC_OS_SECT & occupancy)
					break;
				// FIXME: logic to prevent the CTC ends lining into each other
			}
			clearance &= 0xF0;
			clearance |= newClear;
			break;

		default:
			break;
	}
}

void CodeCTCRoute(uint8_t controlPoint, uint8_t newPoints, uint8_t newClear)
{
	SetClearance(controlPoint, newClear);
	SetTurnout(controlPoint, newPoints);
}


// SignalsToOutputs is responsible for converting the signal head aspects in the
// signalHeads[] array to the physical wires on the XIO.
// Thus, it's hardware configuration dependent.

/* ASPECT_LUNAR     0x07
 ASPECT_FL_RED    0x06
 ASPECT_FL_GREEN  0x05
 ASPECT_RED       0x04
 ASPECT_FL_YELLOW 0x03
 ASPECT_YELLOW    0x02
 ASPECT_GREEN     0x01
 ASPECT_OFF       0x00 */

typedef struct
{
	const uint8_t signalHead;
	const uint8_t redByte;
	const uint8_t redMask;
	const uint8_t yellowByte;
	const uint8_t yellowMask;
	const uint8_t greenByte;
	const uint8_t greenMask;
} SignalPinDefinition;

#define XIO_PORT_A  0
#define XIO_PORT_B  1
#define XIO_PORT_C  2
#define XIO_PORT_D  3
#define XIO_PORT_E  4

const SignalPinDefinition sigPinDefs[4] = 
{
	{SIG_PNTS_UPPER, XIO_PORT_B, _BV(4), XIO_PORT_B, _BV(5), XIO_PORT_B, _BV(6)},
	{SIG_PNTS_LOWER, XIO_PORT_B, _BV(7), XIO_PORT_C, _BV(0), XIO_PORT_C, _BV(1)},
	{SIG_MAIN      , XIO_PORT_C, _BV(2), XIO_PORT_C, _BV(3), XIO_PORT_C, _BV(4)},
	{SIG_SIDING    , XIO_PORT_C, _BV(5), XIO_PORT_C, _BV(6), XIO_PORT_C, _BV(7)}
};



void SignalsToOutputs(uint8_t invertSignalOutputs)
{
	uint8_t sigDefIdx;
	for(sigDefIdx=0; sigDefIdx<sizeof(sigPinDefs)/sizeof(SignalPinDefinition); sigDefIdx++)
	{
		uint8_t redByte = sigPinDefs[sigDefIdx].redByte;
		uint8_t redMask = sigPinDefs[sigDefIdx].redMask;
		uint8_t yellowByte = sigPinDefs[sigDefIdx].yellowByte;
		uint8_t yellowMask = sigPinDefs[sigDefIdx].yellowMask;
		uint8_t greenByte = sigPinDefs[sigDefIdx].greenByte;
		uint8_t greenMask = sigPinDefs[sigDefIdx].greenMask;

		if (invertSignalOutputs & (1<<sigDefIdx))
		{
			// For active high signals

			xio1Outputs[redByte] &= ~redMask;
			xio1Outputs[yellowByte] &= ~yellowMask;
			xio1Outputs[greenByte] &= ~greenMask;

			switch(signalHeads[sigPinDefs[sigDefIdx].signalHead])
			{
				case ASPECT_OFF:
					break;
			
				case ASPECT_GREEN:
					xio1Outputs[greenByte] |= greenMask;
					break;
			
				case ASPECT_FL_GREEN:
					if (events & EVENT_BLINKY)
						xio1Outputs[greenByte] |= greenMask;
					break;

				case ASPECT_YELLOW:
					xio1Outputs[yellowByte] |= yellowMask;
					break;
			
				case ASPECT_FL_YELLOW:
					if (events & EVENT_BLINKY)
						xio1Outputs[yellowByte] |= yellowMask;
					break;
			
			
				case ASPECT_RED:
				case ASPECT_LUNAR: // Can't display, so make like red
				default:
					xio1Outputs[redByte] |= redMask;
					break;

				case ASPECT_FL_RED:
					if (events & EVENT_BLINKY)
						xio1Outputs[redByte] |= redMask;
					break;
			}


		} else {
			// For active high signals

			xio1Outputs[redByte] |= redMask;
			xio1Outputs[yellowByte] |= yellowMask;
			xio1Outputs[greenByte] |= greenMask;

			switch(signalHeads[sigPinDefs[sigDefIdx].signalHead])
			{
				case ASPECT_OFF:
					break;
			
				case ASPECT_GREEN:
					xio1Outputs[greenByte] &= ~greenMask;
					break;
			
				case ASPECT_FL_GREEN:
					if (events & EVENT_BLINKY)
						xio1Outputs[greenByte] &= ~greenMask;
					break;

				case ASPECT_YELLOW:
					xio1Outputs[yellowByte] &= ~yellowMask;
					break;
			
				case ASPECT_FL_YELLOW:
					if (events & EVENT_BLINKY)
						xio1Outputs[yellowByte] &= ~yellowMask;
					break;
			
			
				case ASPECT_RED:
				case ASPECT_LUNAR: // Can't display, so make like red
				default:
					xio1Outputs[redByte] &= ~redMask;
					break;

				case ASPECT_FL_RED:
					if (events & EVENT_BLINKY)
						xio1Outputs[redByte] &= ~redMask;
					break;
			}
		}
	}
}

static inline void vitalLogic()
{
	uint8_t turnoutLocked = (!(((turnouts & (PNTS_STATUS))?1:0) ^ ((turnouts & (PNTS_CNTL))?1:0)));
	uint8_t cleared = CLEARANCE_NONE;

	// Start out with a safe default - everybody red
	signalHeads[SIG_PNTS_UPPER] = ASPECT_RED;
	signalHeads[SIG_PNTS_LOWER] = ASPECT_RED;
	signalHeads[SIG_MAIN] = ASPECT_RED;
	signalHeads[SIG_SIDING] = ASPECT_RED;

	// Drop clearance if we see occupancy
	if (occupancy & OCC_OS_SECT)
		SetClearance(CONTROLPOINT_1, CLEARANCE_NONE);

	cleared = GetClearance(CONTROLPOINT_1);
	
	if (STATE_UNLOCKED == turnoutState || STATE_RELOCKING == turnoutState)
	{

		if(turnouts & (PNTS_STATUS))
		{
			signalHeads[SIG_PNTS_LOWER] = ASPECT_FL_RED;
			signalHeads[SIG_SIDING] = ASPECT_FL_RED;
		}
		else
		{
			signalHeads[SIG_PNTS_UPPER] = ASPECT_FL_RED;
			signalHeads[SIG_MAIN] = ASPECT_FL_RED;
		}
	} 
	else if (turnoutLocked && CLEARANCE_EAST == cleared)
	{
		// Eastbound clearance at the east control point means frog->points movement direction
		uint8_t head = (turnouts & (PNTS_STATUS))?SIG_SIDING:SIG_MAIN;
		
		if (XOCC_P_ADJOIN & ext_occupancy || OCC_OS_SECT & occupancy)
			signalHeads[head] = ASPECT_RED;
		else if (XOCC_P_APPROACH & ext_occupancy)
			signalHeads[head] = ASPECT_YELLOW;
		else if (XOCC_P_APPROACH2 & ext_occupancy)
			signalHeads[head] = ASPECT_FL_YELLOW;
		else
			signalHeads[head] = ASPECT_GREEN;
	}
	else if (turnoutLocked && CLEARANCE_WEST == cleared)
	{
		// Westbound clearance at the east control point means points->frog movement direction	
		if(turnouts & (PNTS_STATUS))
		{
			// Lined to siding
			if ((OCC_OS_SECT & occupancy) || (XOCC2_S_ADJOIN & ext_occupancy2))
				signalHeads[SIG_PNTS_LOWER] = ASPECT_RED;
			else if (XOCC2_S_APPROACH & ext_occupancy2)
				signalHeads[SIG_PNTS_LOWER] = ASPECT_YELLOW;
			else if (XOCC2_S_APPROACH2 & ext_occupancy2)
				signalHeads[SIG_PNTS_LOWER] = ASPECT_FL_YELLOW;
			else
				signalHeads[SIG_PNTS_LOWER] = ASPECT_GREEN;
		} else {
			// Lined to mainline
			if ( ((OCC_OS_SECT) & occupancy) || (XOCC_M_ADJOIN & ext_occupancy))
				signalHeads[SIG_PNTS_UPPER] = ASPECT_RED;
			else if (XOCC_M_APPROACH & ext_occupancy)
				signalHeads[SIG_PNTS_UPPER] = ASPECT_YELLOW;
			else if (XOCC_M_APPROACH2 & ext_occupancy)
				signalHeads[SIG_PNTS_UPPER] = ASPECT_FL_YELLOW;
			else
				signalHeads[SIG_PNTS_UPPER] = ASPECT_GREEN;
		}
	}
	// The else case is that the turnout isn't locked up or we're not cleared
	// Good news - the signals are already defaulted to red

	// Clear virtual occupancies
	occupancy &= ~(OCC_VIRT_P_APPROACH | OCC_VIRT_P_ADJOIN | OCC_VIRT_M_APPROACH | OCC_VIRT_M_ADJOIN | OCC_VIRT_S_APPROACH | OCC_VIRT_S_ADJOIN);

	// Calculate east CP virtual occupancies
	if(!(((turnouts & (PNTS_STATUS))?1:0) ^ ((turnouts & (PNTS_CNTL))?1:0)))
	{
		if(ASPECT_FL_RED == signalHeads[SIG_MAIN] || ASPECT_RED == signalHeads[SIG_MAIN])
			occupancy |= OCC_VIRT_M_ADJOIN;
		else if (ASPECT_YELLOW == signalHeads[SIG_MAIN])
			occupancy |= OCC_VIRT_M_APPROACH;

		if(ASPECT_FL_RED == signalHeads[SIG_SIDING] || ASPECT_RED == signalHeads[SIG_SIDING])
			occupancy |= OCC_VIRT_S_ADJOIN;
		else if (ASPECT_YELLOW == signalHeads[SIG_SIDING])
			occupancy |= OCC_VIRT_S_APPROACH;

		
		// Turnout is properly lined one way or the other
		if ((turnouts & PNTS_STATUS) && ( ASPECT_YELLOW == signalHeads[SIG_PNTS_LOWER] ))
			occupancy |= OCC_VIRT_P_APPROACH;
		else if ((!(turnouts & PNTS_STATUS)) && ( ASPECT_YELLOW == signalHeads[SIG_PNTS_UPPER] ))
			occupancy |= OCC_VIRT_P_APPROACH;
	
		if ((ASPECT_FL_RED == signalHeads[SIG_PNTS_LOWER] || ASPECT_RED == signalHeads[SIG_PNTS_LOWER]) && (ASPECT_RED == signalHeads[SIG_PNTS_UPPER] || ASPECT_FL_RED == signalHeads[SIG_PNTS_UPPER]))
			occupancy |= OCC_VIRT_P_ADJOIN;
	} else {
		// East Control Point improperly lined, trip virtual occupancy
			occupancy |= OCC_VIRT_P_ADJOIN | OCC_VIRT_M_ADJOIN | OCC_VIRT_S_ADJOIN;
	}

}

#define pointsUnlockedSwitch()  ((debounced_inputs[1] & PNTS_UNLOCK)?false:true)

int main(void)
{
	uint8_t changed = 0;
	uint8_t i;

	// Application initialization
	init();

	// Initialize a 100 Hz timer. 
	initialize100HzTimer();

	// Initialize MRBus core
	mrbusPktQueueInitialize(&mrbusTxQueue, mrbusTxPktBufferArray, txBuffer_DEPTH);
	mrbusPktQueueInitialize(&mrbusRxQueue, mrbusRxPktBufferArray, rxBuffer_DEPTH);
	mrbusInit();

	sei();	
	i2c_master_init();
	xioInitialize();

	CodeCTCRoute(CONTROLPOINT_1, POINTS_NORMAL_FORCE, CLEARANCE_NONE);

	while (1)
	{
		wdt_reset();

		// Handle any packets that may have come in
		if (mrbusPktQueueDepth(&mrbusRxQueue))
			PktHandler();

		// The EVENT_I2C_ERROR flag gets set if a read or write fails for some reason
		// I'm going to assume it's because the I2C bus went heywire, and we need to do
		// a very solid reset on things.  No I2C stuff will happen until this gets cleared
		
		if (events & EVENT_I2C_ERROR)
		{
			i2cResetCounter++;
			xioInitialize();
		}

		if(events & (EVENT_READ_INPUTS))
		{
			uint8_t delta;
			xioInputRead();
			for(i=0; i<2; i++)
			{
				// Vertical counter debounce courtesy 
				delta = xio1Inputs[i] ^ debounced_inputs[i];
				clock_a[i] ^= clock_b[i];
				clock_b[i]  = ~(clock_b[i]);
				clock_a[i] &= delta;
				clock_b[i] &= delta;
				debounced_inputs[i] ^= ~(~delta | clock_a[i] | clock_b[i]);
			}

			switch(turnoutState)
			{
				case STATE_LOCKED:
					if (pointsUnlockedSwitch())
					{
						timeCounter = eeprom_read_byte((uint8_t*)EE_UNLOCK_TIME);
						turnoutState = STATE_TIMERUN;
						CodeCTCRoute(CONTROLPOINT_1, POINTS_UNAFFECTED, CLEARANCE_NONE);
					} else {
						xio1Outputs[3] &= ~(PNTS_TIMELOCK_LED);
					}
					break;
					
				case STATE_TIMERUN:
					if (events & EVENT_BLINKY)
						xio1Outputs[3] &= ~(PNTS_TIMELOCK_LED);
					else
						xio1Outputs[3] |= PNTS_TIMELOCK_LED;

					if (!pointsUnlockedSwitch())
						turnoutState = STATE_LOCKED;

					if (0 == timeCounter)
						turnoutState = STATE_UNLOCKED;
					break;
					
				case STATE_UNLOCKED:
					// FIXME: Set turnout position here
					xio1Outputs[3] |= PNTS_TIMELOCK_LED;
					if (xio1Inputs[1] & PNTS_LOCAL_DIR)
						CodeCTCRoute(CONTROLPOINT_1, POINTS_NORMAL_FORCE, CLEARANCE_NONE);
					else
						CodeCTCRoute(CONTROLPOINT_1, POINTS_REVERSE_FORCE, CLEARANCE_NONE);

					if (!pointsUnlockedSwitch())
						turnoutState = STATE_RELOCKING;
					break;

				case STATE_RELOCKING:
					if (xio1Inputs[1] & PNTS_LOCAL_DIR)
						CodeCTCRoute(CONTROLPOINT_1, POINTS_NORMAL_FORCE, CLEARANCE_NONE);
					else
						CodeCTCRoute(CONTROLPOINT_1, POINTS_REVERSE_FORCE, CLEARANCE_NONE);

					if (!pointsUnlockedSwitch() && !(turnouts & (PNTS_STATUS)))
						turnoutState = STATE_LOCKED;

					break;
				
				default: // No idea why we'd get here, but just in case...
					turnoutState = STATE_RELOCKING;
					break;
			} 

			turnouts &= ~(PNTS_STATUS);
			turnouts |= debounced_inputs[0] & (PNTS_STATUS);

			old_debounced_inputs[0] = debounced_inputs[0];
			old_debounced_inputs[1] = debounced_inputs[1];

			events &= ~(EVENT_READ_INPUTS);
		}

		// Vital Logic
		vitalLogic();

		// Send output
		if (events & EVENT_WRITE_OUTPUTS)
		{
			uint8_t comAnodeHeads = eeprom_read_byte((uint8_t*)EE_HEADS_COM_ANODE);
			SignalsToOutputs(comAnodeHeads);
			xioOutputWrite();
			events &= ~(EVENT_WRITE_OUTPUTS);
		}

		// Test if something changed from the last time
		// around the loop - we need to send an update 
		//   packet if it did 
	
		if (memcmp(signalHeads, old_signalHeads, sizeof(signalHeads))
			|| old_turnouts != turnouts
			|| old_clearance != clearance
			|| old_occupancy != occupancy)
		{
			// Something Changed - time to update
			for(i=0; i<sizeof(signalHeads); i++)
				old_signalHeads[i] = signalHeads[i];

			old_turnouts = turnouts;
			old_clearance = clearance;
			old_occupancy = occupancy;
			old_ext_occupancy = ext_occupancy;
			old_ext_occupancy2 = ext_occupancy2;
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
		/* If we need to send a packet and we're not already busy... */

/*
Byte
6 - East Frog Signals
 4:7 - Siding Signal
 0:3 - Main Signal
7 - East Points Signal
 4:7 - E Points Signal Upper
 0:3 - E Points Signal Lower
8 - West Frog Signals
 4:7 - Siding Signal
 0:3 - Main Signal
9 - West Points Signal
 4:7 - W Points Signal Upper
 0:3 - W Points Signal Lower

10 - Occupancy 1
 7 - E Adjoining
 6 - E Adjacent
 5 - W Adjoining
 4 - W Adjacent
 3 - Main
 2 - Siding
 1 - East OS Section
 0 - West OS Section

11: East OS Status
 7 - E OS Adjacent Virtual Occ
 6 - E OS Virtual Occ
 5 - E points lined Siding
 4 - E points lined Mainline
 3 - Undefined
 2 - E OS not cleared
 1 - E OS cleared Westbound
 0 - E OS cleared Eastbound


 
 */

#define MRB_STATUS_CP_CLEARED_EAST   0x01
#define MRB_STATUS_CP_CLEARED_WEST   0x02
#define MRB_STATUS_CP_CLEARED_NONE   0x04
#define MRB_STATUS_CP_MANUAL_UNLOCK  0x08
#define MRB_STATUS_CP_SWITCH_NORMAL  0x10
#define MRB_STATUS_CP_SWITCH_REVERSE 0x20
#define MRB_STATUS_CP_VOCC_ADJOIN    0x40
#define MRB_STATUS_CP_VOCC_APPROACH  0x80

		if (changed && !mrbusPktQueueFull(&mrbusTxQueue))
		{
			uint8_t txBuffer[MRBUS_BUFFER_SIZE];
			txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
			txBuffer[MRBUS_PKT_DEST] = 0xFF;
			txBuffer[MRBUS_PKT_LEN] = 11;
			txBuffer[5] = 'S';
			txBuffer[6] = ((signalHeads[SIG_SIDING]<<4) & 0xF0) | (signalHeads[SIG_MAIN] & 0x0F);
			txBuffer[7] = ((signalHeads[SIG_PNTS_UPPER]<<4) & 0xF0) | (signalHeads[SIG_PNTS_LOWER] & 0x0F);
			txBuffer[8] = occupancy;
			
			switch(GetClearance(CONTROLPOINT_1))
			{
				case CLEARANCE_EAST:
					txBuffer[9] = MRB_STATUS_CP_CLEARED_EAST;
					break;
				case CLEARANCE_WEST:
					txBuffer[9] = MRB_STATUS_CP_CLEARED_WEST;
					break;
				case CLEARANCE_NONE:
				default:
					txBuffer[9] = MRB_STATUS_CP_CLEARED_NONE;
					break;
			}

			if (STATE_LOCKED != turnoutState)
				txBuffer[9] |= MRB_STATUS_CP_MANUAL_UNLOCK;

			if (turnouts & PNTS_STATUS)  // Low is normal, high is reverse
				txBuffer[9] |= MRB_STATUS_CP_SWITCH_REVERSE;
			else
				txBuffer[9] |= MRB_STATUS_CP_SWITCH_NORMAL;

			txBuffer[10] = i2cResetCounter;
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
		}
	}
}

void PktHandler(void)
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
		goto	PktIgnore;

	// Destination Test - is this for us or broadcast?  If not, ignore
	if (0xFF != rxBuffer[MRBUS_PKT_DEST] && mrbus_dev_addr != rxBuffer[MRBUS_PKT_DEST]) 
		goto	PktIgnore;
	
	// CRC16 Test - is the packet intact?
	for(i=0; i<rxBuffer[MRBUS_PKT_LEN]; i++)
	{
		if ((i != MRBUS_PKT_CRC_H) && (i != MRBUS_PKT_CRC_L)) 
			crc = mrbusCRC16Update(crc, rxBuffer[i]);
	}
	if ((UINT16_HIGH_BYTE(crc) != rxBuffer[MRBUS_PKT_CRC_H]) || (UINT16_LOW_BYTE(crc) != rxBuffer[MRBUS_PKT_CRC_L]))
		goto	PktIgnore;
		
	//*************** END PACKET FILTER ***************


	//*************** PACKET HANDLER - PROCESS HERE ***************

	// Just smash the transmit buffer if we happen to see a packet directed to us
	// that requires an immediate response
	//
	// If we're in here, then either we're transmitting, then we can't be 
	// receiving from someone else, or we failed to transmit whatever we were sending
	// and we're waiting to try again.  Either way, we're not going to corrupt an
	// in-progress transmission.
	//
	// All other non-immediate transmissions (such as scheduled status updates)
	// should be sent out of the main loop so that they don't step on things in
	// the transmit buffer
	
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
			// The data elements are:
			//  6 - Control point being manipulated
			//  7 - Turnout normal/reverse
			//  8 - Clear eastbound or westbound
			if (CONTROLPOINT_1 == rxBuffer[6] && STATE_LOCKED == turnoutState)
				CodeCTCRoute(CONTROLPOINT_1, rxBuffer[7], PktDirToClearance(rxBuffer[8]));
			goto PktIgnore;

		case 'T':
			if (1 == rxBuffer[6] && STATE_LOCKED == turnoutState)
				CodeCTCRoute(CONTROLPOINT_1, rxBuffer[6], CLEARANCE_NONE);
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
			txBuffer[12] = 'T';
			txBuffer[13] = 'M';
			txBuffer[14] = 'P';
			txBuffer[15] = 'L';
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
	for (i=0; i<13; i++)
	{
		uint8_t byte, bitset=0;
		uint8_t srcAddr = eeprom_read_byte((uint8_t*)(i+EE_P_APRCH_ADDR));
		if (rxBuffer[MRBUS_PKT_SRC] != srcAddr || 0x00 == srcAddr)
			continue;

		if (rxBuffer[MRBUS_PKT_TYPE] != eeprom_read_byte((uint8_t*)(i+EE_P_APRCH_PKT)))
			continue;
		
		byte = eeprom_read_byte((uint8_t*)(i+EE_P_APRCH_SUBTYPE));
		if ((0xFF != byte) && (rxBuffer[MRBUS_PKT_SUBTYPE] != byte))
			continue;

		/* BITBYTE is computed as follows:
			x = bit = 0-7
			y = byte = byte in data stream (6 is first data byte)
			xxxyyyy
		*/
		byte = eeprom_read_byte((uint8_t*)(i+EE_P_APRCH_BITBYTE));
		bitset = rxBuffer[(byte & 0x1F)] & (1<<((byte>>5) & 0x07));

		switch(i + EE_P_APRCH_ADDR)
		{
			case EE_M_APRCH_ADDR:
				if (bitset)
					ext_occupancy |= XOCC_M_APPROACH;
				else
					ext_occupancy &= ~(XOCC_M_APPROACH);
				break;

			case EE_M_APRCH2_ADDR:
				if (bitset)
					ext_occupancy |= XOCC_M_APPROACH2;
				else
					ext_occupancy &= ~(XOCC_M_APPROACH2);
				break;
				
			case EE_M_ADJ_ADDR:
				if (bitset)
					ext_occupancy |= XOCC_M_ADJOIN;
				else
					ext_occupancy &= ~(XOCC_M_ADJOIN);
				break;
			
			case EE_M_TUMBLE_ADDR:
				if (bitset)
					ext_occupancy |= XOCC_M_TUMBLE;
				else
					ext_occupancy &= ~(XOCC_M_TUMBLE);
				break;
				
				
				
			case EE_P_APRCH_ADDR:
				if (bitset)
					ext_occupancy |= XOCC_P_APPROACH;
				else
					ext_occupancy &= ~(XOCC_P_APPROACH);
				break;

			case EE_P_APRCH2_ADDR:
				if (bitset)
					ext_occupancy |= XOCC_P_APPROACH2;
				else
					ext_occupancy &= ~(XOCC_P_APPROACH2);
				break;
				
			case EE_P_ADJ_ADDR:
				if (bitset)
					ext_occupancy |= XOCC_P_ADJOIN;
				else
					ext_occupancy &= ~(XOCC_P_ADJOIN);
				break;
			
			case EE_P_TUMBLE_ADDR:
				if (bitset)
					ext_occupancy |= XOCC_P_TUMBLE;
				else
					ext_occupancy &= ~(XOCC_P_TUMBLE);
				break;


			case EE_S_APRCH_ADDR:
				if (bitset)
					ext_occupancy2 |= XOCC2_S_APPROACH;
				else
					ext_occupancy2 &= ~(XOCC2_S_APPROACH);
				break;

			case EE_S_APRCH2_ADDR:
				if (bitset)
					ext_occupancy2 |= XOCC2_S_APPROACH2;
				else
					ext_occupancy2 &= ~(XOCC2_S_APPROACH2);
				break;
				
			case EE_S_ADJ_ADDR:
				if (bitset)
					ext_occupancy2 |= XOCC2_S_ADJOIN;
				else
					ext_occupancy2 &= ~(XOCC2_S_ADJOIN);
				break;
			
			case EE_S_TUMBLE_ADDR:
				if (bitset)
					ext_occupancy2 |= XOCC2_S_TUMBLE;
				else
					ext_occupancy2 &= ~(XOCC2_S_TUMBLE);
				break;


			case EE_OS_ADDR:
				if (bitset)
					occupancy |= OCC_OS_SECT;
				else
					occupancy &= ~(OCC_OS_SECT);
				break;


		}
	}
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


