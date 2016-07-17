/*
 * main.c
 */

#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"

#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"

#include "nrf24l01.h"

struct nrf24l01p nrf;
uint8_t addresses[][6] = {"1Node","2Node"};

int main(void) {
	//
	// Enable lazy stacking for interrupt handlers.  This allows floating-point
	// instructions to be used within interrupt handlers, but at the expense of
	// extra stack usage.
	//
	FPULazyStackingEnable();
	FPUEnable();

	//
	// Set the clocking to run directly from the crystal.
	// Clock to 80MHZ
	//
	SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
					   SYSCTL_OSC_MAIN);

	IntMasterEnable();

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_6);

	//set up the radio
	if( !nrf24l01p_setup(&nrf, GPIO_PORTC_BASE, GPIO_PIN_4, SSI0_BASE) ){
		//we couldn't communicate with the radio
		return 1;
	}

	nrf24l01p_set_PA_level(&nrf, RF24_PA_LOW);

	nrf24l01p_open_writing_pipe(&nrf, addresses[0]);
	nrf24l01p_open_reading_pipe(&nrf, 1,addresses[1]);

	nrf24l01p_start_listening(&nrf);

	while(1){
		if (nrf24l01p_available(&nrf)) {
			uint32_t got_time;

			// Variable for the received timestamp
			while (nrf24l01p_available(&nrf)) {                 // While there is data ready
				nrf24l01p_read(&nrf, &got_time, sizeof(uint32_t));     // Get the payload
			}

			nrf24l01p_stop_listening(&nrf);        // First, stop listening so we can talk
			nrf24l01p_write(&nrf, &got_time, 1); // Send the final one back.
			nrf24l01p_start_listening(&nrf); // Now, resume listening so we catch the next packets.

		}

		SysCtlDelay((SysCtlClockGet() >> 12) * 50);
	}

	return 0;
}
