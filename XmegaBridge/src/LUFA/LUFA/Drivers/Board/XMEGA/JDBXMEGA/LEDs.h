/*
 * LEDs.h
 *
 * Created: 6-8-2015 22:02:28
 *  Author: Vito
 */ 


#ifndef LEDS_H_
#define LEDS_H_


/* Includes: */
#include <avr/io.h>

/* Enable C linkage for C++ Compilers: */
#if defined(__cplusplus)
extern "C" {
	#endif

	/* Preprocessor Checks: */
	#if !defined(__INCLUDE_FROM_LEDS_H)
	#error Do not include this file directly. Include LUFA/Drivers/Board/LEDS.h instead.
	#endif
	
	#define LEDS_PORT		PORTA

	/* Public Interface - May be used in end-application: */
	/* Macros: */
	/** LED mask for the first LED on the board. */
	#define LEDS_LED1        PIN6_bm

	/** LED mask for all the LEDs on the board. */
	#define LEDS_ALL_LEDS    LEDS_LED1

	/** LED mask for the none of the board LEDs. */
	#define LEDS_NO_LEDS     0

	/* Inline Functions: */
	#if !defined(__DOXYGEN__)
	static inline void LEDs_Init(void)
	{
		LEDS_PORT.DIRSET  |= LEDS_ALL_LEDS;
		LEDS_PORT.OUTCLR |= LEDS_ALL_LEDS;
	}
	
	static inline void LEDs_TurnOnLEDs(const uint8_t LEDMask)
	{
		LEDS_PORT.OUTCLR = LEDMask;
	}

	static inline void LEDs_TurnOffLEDs(const uint8_t LEDMask)
	{
		LEDS_PORT.OUTSET = LEDMask;
	}

	static inline void LEDs_SetAllLEDs(const uint8_t LEDMask)
	{
		LEDS_PORT.OUTSET = (LEDS_ALL_LEDS & LEDMask);
	}
	
	static inline void LEDs_ChangeLEDs(const uint8_t LEDMask,
	const uint8_t ActiveMask)
	{
		LEDS_PORT.OUT = ((LEDS_PORT.OUT | (LEDMask & LEDS_ALL_LEDS)) & (~ActiveMask & LEDS_ALL_LEDS));
	}
	
	static inline void LEDs_ToggleLEDs(const uint8_t LEDMask)
	{
		LEDS_PORT.OUTTGL = (LEDMask & LEDS_ALL_LEDS);
	}
	
	static inline uint8_t LEDs_GetLEDs(void) ATTR_WARN_UNUSED_RESULT;
	static inline uint8_t LEDs_GetLEDs(void)
	{
		return (LEDS_PORT.OUT & LEDS_ALL_LEDS);
	}
	#endif
	
	/* Disable C linkage for C++ Compilers: */
	#if defined(__cplusplus)
}
#endif

#endif /* LEDS_H_ */