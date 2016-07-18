/*
    Copyright (C) 2016 Dean Miller

    Based on RF24 arduino Library example.
    This will talk to an NRF24L01 on another board and ping back whatever is sent.

    Permission is hereby granted, free of charge, to any person
    obtaining a copy of this software and associated documentation
    files (the "Software"), to deal in the Software without
    restriction, including without limitation the rights to use, copy,
    modify, merge, publish, distribute, sublicense, and/or sell copies
    of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:
    The above copyright notice and this permission notice shall be
    included in all copies or substantial portions of the Software.
    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
    MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
    DEALINGS IN THE SOFTWARE.
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
			nrf24l01p_write(&nrf, &got_time, sizeof(uint32_t)); // Send the final one back.
			nrf24l01p_start_listening(&nrf); // Now, resume listening so we catch the next packets.

		}

		//give it a little space
		SysCtlDelay((SysCtlClockGet() >> 12) * 5);
	}

	return 0;
}
