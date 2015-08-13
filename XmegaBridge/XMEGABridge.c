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
 *  Main source file for the XPLAINBridge project. This file contains the main tasks of
 *  the project and is responsible for the initial application hardware configuration.
 */

#include "XMEGABridge.h"

/* Function Prototypes */
static void Sleep_Init(void);
static inline void Sleep_Check(void);
static inline void Sleep_Now(void);
static void Button_Init(void);
static bool GetStartupFirmwareMode(void);
static void SetupButtonInterrupt(void);
static void BTimer_Init(void);
static inline void BTimer_Start(void);
static inline void BTimer_Stop(void);
//static void JumpToBootloader(void);

/** Current firmware mode, making the device behave as either a programmer or a USART bridge */
bool CurrentFirmwareMode = MODE_USART_BRIDGE;

static volatile bool GoSleep = false;
static volatile bool JumpToBoot = false;
static volatile uint8_t TimeCount = 0;

/** LUFA CDC Class driver interface configuration and state information. This structure is
 *  passed to all CDC Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface =
	{
		.Config =
			{
				.ControlInterfaceNumber         = INTERFACE_ID_CDC_CCI,
				.DataINEndpoint                 =
					{
						.Address                = CDC_TX_EPADDR,
						.Size                   = CDC_TXRX_EPSIZE,
						.Banks                  = 1,
					},
				.DataOUTEndpoint                =
					{
						.Address                = CDC_RX_EPADDR,
						.Size                   = CDC_TXRX_EPSIZE,
						.Banks                  = 1,
					},
				.NotificationEndpoint           =
					{
						.Address                = CDC_NOTIFICATION_EPADDR,
						.Size                   = CDC_NOTIFICATION_EPSIZE,
						.Banks                  = 1,
					},
			},
	};

/** Circular buffer to hold data from the host before it is sent to the device via the serial port. */
RingBuffer_t   USBtoUART_Buffer;

/** Underlying data buffer for \ref USBtoUART_Buffer, where the stored bytes are located. */
static uint8_t USBtoUART_Buffer_Data[128];

/** Circular buffer to hold data from the serial port before it is sent to the host. */
RingBuffer_t   UARTtoUSB_Buffer;

/** Underlying data buffer for \ref UARTtoUSB_Buffer, where the stored bytes are located. */
static uint8_t UARTtoUSB_Buffer_Data[128];


/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
int main(void)
{
	SetupHardware();

	LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
	GlobalInterruptEnable();
	CurrentFirmwareMode = GetStartupFirmwareMode();
	
	SetupButtonInterrupt();

	for (;;)
	{
		if (CurrentFirmwareMode == MODE_USART_BRIDGE)
		  UARTBridge_Task();
		else
		  AVRISP_Task();

		USB_USBTask();	
		Sleep_Check();
		
		//if (JumpToBoot == true)
		//	JumpToBootloader();
	}
}

void AVRISP_Task(void)
{
	/* Must be in the configured state for the AVRISP code to process data */
	if (USB_DeviceState != DEVICE_STATE_Configured)
	  return;

	V2Params_UpdateParamValues();
	//TODO
	Endpoint_SelectEndpoint(AVRISP_DATA_OUT_EPADDR);

	/* Check to see if a V2 Protocol command has been received */
	if (Endpoint_IsOUTReceived())
	{
		LEDs_SetAllLEDs(LEDMASK_BUSY);

		/* Pass off processing of the V2 Protocol command to the V2 Protocol handler */
		V2Protocol_ProcessCommand();

		LEDs_SetAllLEDs(LEDMASK_USB_READY);
	}
}

void UARTBridge_Task(void)
{
	/* Must be in the configured state for the USART Bridge code to process data */
	if (USB_DeviceState != DEVICE_STATE_Configured)
	  return;

	/* Only try to read in bytes from the CDC interface if the transmit buffer is not full */
	if (!(RingBuffer_IsFull(&USBtoUART_Buffer)))
	{
		int16_t ReceivedByte = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);

		/* Read bytes from the USB OUT endpoint into the UART transmit buffer */
		if (!(ReceivedByte < 0)) {
		  RingBuffer_Insert(&USBtoUART_Buffer, ReceivedByte);
		  LEDs_ToggleLEDs(LEDS_LED1);
		}
	}
	
	/* Check if the UART receive buffer flush timer has expired or buffer is nearly full */
	uint16_t BufferCount = RingBuffer_GetCount(&UARTtoUSB_Buffer);
	//TODO
	//if ((TIFR0 & (1 << TOV0)) || (BufferCount > 200))
	if (BufferCount > 0)
	{
		/* Clear flush timer expiry flag */
		//TIFR0 |= (1 << TOV0);

		/* Read bytes from the USART receive buffer into the USB IN endpoint */
		while (BufferCount--)
		{
			/* Try to send the next byte of data to the host, abort if there is an error without dequeuing */
			if (CDC_Device_SendByte(&VirtualSerial_CDC_Interface,
									RingBuffer_Peek(&UARTtoUSB_Buffer)) != ENDPOINT_READYWAIT_NoError)
			{
				break;
			}

			/* Dequeue the already sent byte from the buffer now we have confirmed that no transmission error occurred */
			RingBuffer_Remove(&UARTtoUSB_Buffer);	
			LEDs_ToggleLEDs(LEDS_LED1);
		}
	}

	if (Serial_IsSendReady(&USARTX) && !(RingBuffer_IsEmpty(&USBtoUART_Buffer)))
		Serial_SendByte(&USARTX, RingBuffer_Remove(&USBtoUART_Buffer));
		
	LEDs_TurnOffLEDs(LEDS_LED1);
	CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
}

static int Stdout_putchar(char c, FILE *stream) {
	Serial_SendByte(&USARTX, c);
	return 0;
}

FILE usartx = FDEV_SETUP_STREAM(Stdout_putchar, NULL, _FDEV_SETUP_WRITE);

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void)
{
	/* Start the PLL to multiply the 2MHz RC oscillator to 32MHz and switch the CPU core to run from it */
	XMEGACLK_StartPLL(CLOCK_SRC_INT_RC2MHZ, 2000000, F_CPU);
	XMEGACLK_SetCPUClockSource(CLOCK_SRC_PLL);

	/* Start the 32MHz internal RC oscillator and start the DFLL to increase it to 48MHz using the USB SOF as a reference */
	XMEGACLK_StartInternalOscillator(CLOCK_SRC_INT_RC32MHZ);
	XMEGACLK_StartDFLL(CLOCK_SRC_INT_RC32MHZ, DFLL_REF_INT_USBSOF, F_USB);

	PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;

	//CurrentFirmwareMode = MODE_USART_BRIDGE; //TODO Make this usefull

	/* Hardware Initialization */
	USARTX_PORT.DIRSET = USARTX_TX_PIN;
	Serial_Init(&USARTX, 9600, false);
	
	USARTX.CTRLA |= USART_RXCINTLVL_LO_gc;
	stdout = &usartx;
		
	LEDs_Init();
	Sleep_Init();
	Button_Init();
	BTimer_Init();
	
	#if defined(RESET_TOGGLES_LIBUSB_COMPAT)
	UpdateCurrentCompatibilityMode();
	#endif

	/* USB Stack Initialization */
	USB_Init();
	
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;
	
	/* Configure the device endpoints according to the selected mode */
	if (CurrentFirmwareMode == MODE_USART_BRIDGE)
	{
		ConfigSuccess &= CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface);

		/* Configure the UART flush timer - run at Fcpu/1024 for maximum interval before overflow */
		//TCCR0B = ((1 << CS02) | (1 << CS00));
		//TODO

		/* Initialize ring buffers used to hold serial data between USB and software UART interfaces */
		RingBuffer_InitBuffer(&USBtoUART_Buffer, USBtoUART_Buffer_Data, sizeof(USBtoUART_Buffer_Data));
		RingBuffer_InitBuffer(&UARTtoUSB_Buffer, UARTtoUSB_Buffer_Data, sizeof(UARTtoUSB_Buffer_Data));

		/* Start the software USART */
		Serial_Init(&USARTX, 9600, false);
	}
	else
	{
		ConfigSuccess &= Endpoint_ConfigureEndpoint(AVRISP_DATA_OUT_EPADDR, EP_TYPE_BULK, AVRISP_DATA_EPSIZE, 1);
		ConfigSuccess &= Endpoint_ConfigureEndpoint(AVRISP_DATA_IN_EPADDR, EP_TYPE_BULK, AVRISP_DATA_EPSIZE, 1);

		/* Configure the V2 protocol packet handler */
		V2Protocol_Init();
		//if (ConfigSuccess)
		//	Serial_SendString(&USARTX, "New Config OK\r\n");
	}

	LEDs_SetAllLEDs(ConfigSuccess ? LEDMASK_USB_READY : LEDMASK_USB_ERROR);
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
	if (CurrentFirmwareMode == MODE_USART_BRIDGE)
	  CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_Interface);
}

/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void)
{
	LEDs_SetAllLEDs(LEDMASK_USB_ENUMERATING);
}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{
	LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
}

/** Event handler for the CDC Class driver Line Encoding Changed event.
 *
 *  \param[in] CDCInterfaceInfo  Pointer to the CDC class interface configuration structure being referenced
 */
void EVENT_CDC_Device_LineEncodingChanged(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo)
{
	/* Change the software UART's baud rate to match the new baud rate */
	Serial_Init(&USARTX, CDCInterfaceInfo->State.LineEncoding.BaudRateBPS, false);
}

/** This function is called by the library when in device mode, and must be overridden (see library "USB Descriptors"
 *  documentation) by the application code so that the address and size of a requested descriptor can be given
 *  to the USB library. When the device receives a Get Descriptor request on the control endpoint, this function
 *  is called so that the descriptor details can be passed back and the appropriate descriptor sent back to the
 *  USB host.
 *
 *  \param[in]  wValue                 Descriptor type and index to retrieve
 *  \param[in]  wIndex                 Sub-index to retrieve (such as a localized string language)
 *  \param[out] DescriptorAddress      Address of the retrieved descriptor
 *  \param[out] DescriptorMemorySpace  Memory space that the descriptor is stored in
 *
 *  \return Length of the retrieved descriptor in bytes, or NO_DESCRIPTOR if the descriptor was not found
 */
uint16_t CALLBACK_USB_GetDescriptor(const uint16_t wValue,
                                    const uint8_t wIndex,
                                    const void** const DescriptorAddress,
		                            uint8_t* const DescriptorMemorySpace)
{
	/* Return the correct descriptors based on the selected mode */
	if (CurrentFirmwareMode == MODE_USART_BRIDGE)
	  return USART_GetDescriptor(wValue, wIndex, DescriptorAddress, DescriptorMemorySpace);
	else
	  return AVRISP_GetDescriptor(wValue, wIndex, DescriptorAddress, DescriptorMemorySpace);
}


static void Sleep_Init(void)
{
	WAKEUP_PORT.DIRCLR = WAKEUP_PIN;
	WAKEUP_PORT.INTCTRL = WAKEUP_ILVL;
	WAKEUP_PORT.WAKEUP_IMASK = WAKEUP_PIN;
	WAKEUP_PORT.WAKEUP_PINCTRL = PORT_ISC_BOTHEDGES_gc;
	
	sleep_set_mode(SLEEP_SMODE_PDOWN_gc);
}

static inline void Sleep_Check(void)
{
	if (GoSleep == true)
	{
		GoSleep = false;
		Sleep_Now();
	}
}

static inline void Sleep_Now(void)
{
	LEDs_ToggleLEDs(LEDS_LED1);
	sleep_enable();
	sleep_enter(); 
	/* Zzzz */
	sleep_disable();
	LEDs_ToggleLEDs(LEDS_LED1);
}

static void Button_Init(void)
{
	BUTTON_PORT.BUTTON_PIN_CTRL = PORT_OPC_PULLUP_gc;
	BUTTON_PORT.DIRCLR = BUTTON_PIN;
	BUTTON_PORT.BUTTON_IMASK = BUTTON_PIN;
	BUTTON_PORT.WAKEUP_PINCTRL = PORT_ISC_BOTHEDGES_gc;
	/* Dont use interrupt for now */
	BUTTON_PORT.INTCTRL = PORT_INT1LVL_OFF_gc;
}

static bool GetStartupFirmwareMode(void)
{
	bool mode = MODE_USART_BRIDGE;
	
	Delay_MS(BUTTON_STARTUP_DELAY_MS);
	if (~BUTTON_PORT.IN & BUTTON_PIN)
		Delay_MS(50);
		if (~BUTTON_PORT.IN & BUTTON_PIN)
			mode = MODE_PDI_PROGRAMMER;
	
	return mode;
}

static void SetupButtonInterrupt(void)
{
	BUTTON_PORT.INTCTRL = BUTTON_ILVL;
}

static void BTimer_Init(void)
{
	BTIMER.PER = BTIMER_PER;
	BTIMER.CTRLA = BTIMER_PRESC;
	BTIMER.INTCTRLA = TC_OVFINTLVL_OFF_gc;
}

static inline void BTimer_Start(void)
{
	BTIMER.CNT = 0;
	BTIMER.INTCTRLA = BTIMER_ILVL;
}

static inline void BTimer_Stop(void)
{
	BTIMER.INTCTRLA = TC_OVFINTLVL_OFF_gc;
}



/*! \brief Receive complete interrupt service routine.
*/
ISR(USARTX_RXC_vect)
{
	uint8_t ReceivedByte = USARTX.DATA;

	if ((USB_DeviceState == DEVICE_STATE_Configured) && !(RingBuffer_IsFull(&UARTtoUSB_Buffer)))
		RingBuffer_Insert(&UARTtoUSB_Buffer, ReceivedByte);
}

ISR(WAKEUP_vect)
{
	if (WAKEUP_PORT.IN & WAKEUP_PIN)
	{
		// Wakeup!!
	}
	else
	{
		GoSleep = true;
	}
}

ISR(BUTTON_vect)
{
	// Start timer to determine what the user wants.
	if (~BUTTON_PORT.IN & BUTTON_PIN)
	{
		// Button pressed.
		if (CurrentFirmwareMode == MODE_USART_BRIDGE)
			CurrentFirmwareMode = MODE_PDI_PROGRAMMER;
		else
			CurrentFirmwareMode = MODE_USART_BRIDGE;
			
	}
	else
	{
		// Button released.
	}
}

ISR(BTIMER_vect)
{
	TimeCount++;
}

#if 0
static void JumpToBootloader(void)
{
	USB_Disable();
	Delay_MS(3000);
	cli();
	EIND = BOOT_SECTION_START>>17;
	((void (*)(void))BOOTLOADER_START_ADDRESS)();
}
#endif