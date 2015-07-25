/*
             LUFA Library
     Copyright (C) Dean Camera, 2014.

  dean [at] fourwalledcubicle [dot] com
           www.lufa-lib.org
*/

/*
  Copyright 2014  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaims all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

/** \file
 *
 *  Target-related functions for the PDI Protocol decoder.
 */

#define  INCLUDE_FROM_XPROGTARGET_C
#include "XPROGTarget.h"

#if defined(ENABLE_XPROG_PROTOCOL) || defined(__DOXYGEN__)

/** Flag to indicate if the USART is currently in Tx or Rx mode. */
bool IsSending;

/** Enables the target's PDI interface, holding the target in reset until PDI mode is exited. */
void XPROGTarget_EnableTargetPDI(void)
{
	IsSending = false;
	
	/* Set Tx and XCK as outputs, Rx as input */
	//DDRD |=  (1 << 5) | (1 << 3);
	PDI_PORT.DIRSET = PDI_TX | PDI_RESET;
	//DDRD &= ~(1 << 2);
	PDI_PORT.DIRCLR = PDI_RX;
	
	/* Set DATA line high for at least 90ns to disable /RESET functionality */
	//PORTD |= (1 << 3);
	PDI_PORT.OUTSET = PDI_TX;
	_delay_us(100);

// TODO CHECK
	
	PDI_PORT.PIN1CTRL = PORT_INVEN_bm;
	/* Set up the synchronous USART for XMEGA communications - 8 data bits, even parity, 2 stop bits */
	//UBRR1  = ((F_CPU / 2 / XPROG_HARDWARE_SPEED) - 1);
	//PDI_USART.BAUDCTRLB = (uint8_t)(XPROG_HARDWARE_SPEED >> 8);
	//PDI_USART.BAUDCTRLA = (uint8_t)(XPROG_HARDWARE_SPEED & 0xFF);
	PDI_USART.BAUDCTRLA = 1;
	PDI_USART.BAUDCTRLB = 0;
	//UCSR1B = (1 << TXEN1);
	PDI_USART.CTRLB = USART_TXEN_bm;
	//UCSR1C = (1 << UMSEL10) | (1 << UPM11) | (1 << USBS1) | (1 << UCSZ11) | (1 << UCSZ10) | (1 << UCPOL1);
	PDI_USART.CTRLC = USART_CMODE_SYNCHRONOUS_gc | USART_CHSIZE_8BIT_gc | USART_PMODE_EVEN_gc | USART_SBMODE_bm;

	/* Send two IDLEs of 12 bits each to enable PDI interface (need at least 16 idle bits) */
	XPROGTarget_SendIdle();
	XPROGTarget_SendIdle();
}

/** Enables the target's TPI interface, holding the target in reset until TPI mode is exited. */
void XPROGTarget_EnableTargetTPI(void)
{
	IsSending = false;
	
//TODO
#if 0
	/* Set /RESET line low for at least 400ns to enable TPI functionality */
	AUX_LINE_DDR  |=  AUX_LINE_MASK;
	AUX_LINE_PORT &= ~AUX_LINE_MASK;
	_delay_us(100);

	/* Set Tx and XCK as outputs, Rx as input */
	DDRD |=  (1 << 5) | (1 << 3);
	DDRD &= ~(1 << 2);

	/* Set up the synchronous USART for TPI communications - 8 data bits, even parity, 2 stop bits */
	UBRR1  = ((F_CPU / 2 / XPROG_HARDWARE_SPEED) - 1);
	UCSR1B = (1 << TXEN1);
	UCSR1C = (1 << UMSEL10) | (1 << UPM11) | (1 << USBS1) | (1 << UCSZ11) | (1 << UCSZ10) | (1 << UCPOL1);
#endif

	/* Send two IDLEs of 12 bits each to enable TPI interface (need at least 16 idle bits) */
	XPROGTarget_SendIdle();
	XPROGTarget_SendIdle();
}

/** Disables the target's PDI interface, exits programming mode and starts the target's application. */
void XPROGTarget_DisableTargetPDI(void)
{
	/* Switch to Rx mode to ensure that all pending transmissions are complete */
	if (IsSending)
	  XPROGTarget_SetRxMode();

//TODO CHECK
	/* Turn off receiver and transmitter of the USART, clear settings */
	//UCSR1A  = ((1 << TXC1) | (1 << RXC1));
	PDI_USART.STATUS = USART_RXCIF_bm | USART_TXCIF_bm;
	//UCSR1B  = 0;
	PDI_USART.CTRLB = 0;
	//UCSR1C  = 0;
	PDI_USART.CTRLC = 0;

	/* Tristate all pins */
	//DDRD  &= ~((1 << 5) | (1 << 3));
	PDI_PORT.DIRCLR = PDI_RESET | PDI_TX;
	//PORTD &= ~((1 << 5) | (1 << 3) | (1 << 2));
	PDI_PORT.OUTCLR = PDI_RESET | PDI_TX | PDI_RX;
}

/** Disables the target's TPI interface, exits programming mode and starts the target's application. */
void XPROGTarget_DisableTargetTPI(void)
{
	/* Switch to Rx mode to ensure that all pending transmissions are complete */
	if (IsSending)
	  XPROGTarget_SetRxMode();

//TODO
#if 0
	/* Turn off receiver and transmitter of the USART, clear settings */
	UCSR1A |= (1 << TXC1) | (1 << RXC1);
	UCSR1B  = 0;
	UCSR1C  = 0;

	/* Set all USART lines as inputs, tristate */
	DDRD  &= ~((1 << 5) | (1 << 3));
	PORTD &= ~((1 << 5) | (1 << 3) | (1 << 2));

	/* Tristate target /RESET line */
	AUX_LINE_DDR  &= ~AUX_LINE_MASK;
	AUX_LINE_PORT &= ~AUX_LINE_MASK;
#endif
}

/** Sends a byte via the USART.
 *
 *  \param[in] Byte  Byte to send through the USART
 */
void XPROGTarget_SendByte(const uint8_t Byte)
{
	/* Switch to Tx mode if currently in Rx mode */
	if (!(IsSending))
	  XPROGTarget_SetTxMode();

//TODO
#if 0
	/* Wait until there is space in the hardware Tx buffer before writing */
	while (!(UCSR1A & (1 << UDRE1)));
	UCSR1A |= (1 << TXC1);
	UDR1    = Byte;
#endif

	while (!(PDI_USART.STATUS & USART_DREIF_bm));
	PDI_USART.STATUS |= USART_TXCIF_bm;
	PDI_USART.DATA = Byte;
}

/** Receives a byte via the hardware USART, blocking until data is received or timeout expired.
 *
 *  \return Received byte from the USART
 */
uint8_t XPROGTarget_ReceiveByte(void)
{
	uint8_t data;
	/* Switch to Rx mode if currently in Tx mode */
	if (IsSending)
	  XPROGTarget_SetRxMode();

//TODO CHECK
#if 0
	/* Wait until a byte has been received before reading */
	while (!(UCSR1A & (1 << RXC1)) && TimeoutTicksRemaining);

	return UDR1;
#endif

	while (!(PDI_USART.STATUS & USART_RXCIF_bm) && TimeoutTicksRemaining);
	data = PDI_USART.DATA;
	return data;
}

/** Sends an IDLE via the USART to the attached target, consisting of a full frame of idle bits. */
void XPROGTarget_SendIdle(void)
{
	/* Switch to Tx mode if currently in Rx mode */
	if (!(IsSending))
	  XPROGTarget_SetTxMode();

//TODO CHECK THIS
	/* Need to do nothing for a full frame to send an IDLE */
	for (uint8_t i = 0; i < BITS_IN_USART_FRAME; i++)
	{
#if 1
		/* Wait for a full cycle of the clock */
		//while (PIND & (1 << 5));
		while (PDI_PORT.IN & PDI_RESET);
		//while (!(PIND & (1 << 5)));
		while (!(PDI_PORT.IN & PDI_RESET));
		//while (PIND & (1 << 5));
		while (PDI_PORT.IN & PDI_RESET);
#endif
	}
}

static void XPROGTarget_SetTxMode(void)
{
//TODO CHECK THIS

	/* Wait for a full cycle of the clock */
	//while (PIND & (1 << 5));
	while (PDI_PORT.IN & PDI_RESET);
	//while (!(PIND & (1 << 5)));
	while (!(PDI_PORT.IN & PDI_RESET));
	//while (PIND & (1 << 5));
	while (PDI_PORT.IN & PDI_RESET);

	//PORTD  |=  (1 << 3);
	PDI_PORT.OUTSET = PDI_TX;
	//DDRD   |=  (1 << 3);
	PDI_PORT.DIRSET = PDI_TX;

	//UCSR1B &= ~(1 << RXEN1);
	PDI_USART.CTRLB &= ~USART_RXEN_bm;
	//UCSR1B |=  (1 << TXEN1);
	PDI_USART.CTRLB |= USART_TXEN_bm;

	IsSending = true;
}

static void XPROGTarget_SetRxMode(void)
{
//TODO
#if 0
	while (!(UCSR1A & (1 << TXC1)));
	UCSR1A |=  (1 << TXC1);

	UCSR1B &= ~(1 << TXEN1);
	UCSR1B |=  (1 << RXEN1);
#endif

	while (!(PDI_USART.STATUS & USART_TXCIF_bm));
	PDI_USART.STATUS |= USART_TXCIF_bm;
	
	PDI_USART.CTRLB &= ~USART_TXEN_bm;
	PDI_USART.CTRLB |= USART_RXEN_bm;

	//DDRD   &= ~(1 << 3);
	PDI_PORT.DIRCLR = PDI_TX;
	//PORTD  &= ~(1 << 3);
	PDI_PORT.OUTCLR = PDI_TX;

	IsSending = false;
}

#endif

