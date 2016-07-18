/*
    Copyright (C) 2016 Dean Miller
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
#include <math.h>

#include "inc/hw_memmap.h"
#include "inc/hw_ssi.h"

#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/ssi.h"

#include "nrf24l01.h"

static const uint8_t child_pipe_enable[] =
{
  ERX_P0, ERX_P1, ERX_P2, ERX_P3, ERX_P4, ERX_P5
};
static const uint8_t child_pipe[]  =
{
  RX_ADDR_P0, RX_ADDR_P1, RX_ADDR_P2, RX_ADDR_P3, RX_ADDR_P4, RX_ADDR_P5
};
static const uint8_t child_payload_size[]  =
{
  RX_PW_P0, RX_PW_P1, RX_PW_P2, RX_PW_P3, RX_PW_P4, RX_PW_P5
};

//*********** INTERNAL ****************//

void nrf24l01p_begin_transaction(struct nrf24l01p *nrf){
	if(nrf->SSI_BASE == SSI0_BASE){
		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0x00);
	}
	//TODO: other bases
}

void nrf24l01p_end_transaction(struct nrf24l01p *nrf){

	while(SSIBusy(nrf->SSI_BASE));
	if(nrf->SSI_BASE == SSI0_BASE){
		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
	}
	//TODO: other bases
}

uint32_t nrf24l01p_flush_rx(struct nrf24l01p *nrf){
	uint32_t result;

	nrf24l01p_begin_transaction(nrf);
	SSIDataPut(nrf->SSI_BASE, FLUSH_RX);
	SSIDataPut(nrf->SSI_BASE, 0xff);

	SSIDataGet(nrf->SSI_BASE, &result);
	nrf24l01p_end_transaction(nrf);

	return result;
}

uint32_t nrf24l01p_flush_tx(struct nrf24l01p *nrf){
	uint32_t result;

	nrf24l01p_begin_transaction(nrf);
	SSIDataPut(nrf->SSI_BASE, FLUSH_TX);
	SSIDataPut(nrf->SSI_BASE, 0xff);

	SSIDataGet(nrf->SSI_BASE, &result);
	nrf24l01p_end_transaction(nrf);

	return result;
}

uint32_t nrf24l01p_write_payload(struct nrf24l01p *nrf, const void* buf, uint8_t data_len, const uint8_t writeType)
{
  uint32_t status, dontCare;
  const uint8_t* current = (const uint8_t *)(buf);

   data_len = min(data_len, nrf->payload_size);
   uint8_t blank_len = nrf->dynamic_payloads_enabled ? 0 : nrf->payload_size - data_len;

   //clear the rx buffer
   while(SSIDataGetNonBlocking(nrf->SSI_BASE, &dontCare));

  nrf24l01p_begin_transaction(nrf);
  SSIDataPut(nrf->SSI_BASE, writeType );
  SSIDataGet(nrf->SSI_BASE, &status);
  while ( data_len-- ) {
    SSIDataPut(nrf->SSI_BASE, *current++);
  }
  while ( blank_len-- ) {
	  SSIDataPut(nrf->SSI_BASE, 0x00);
  }
  nrf24l01p_end_transaction(nrf);

  return status;
}

int nrf24l01p_start_fast_write(struct nrf24l01p *nrf, const void* buf, uint8_t len, const bool multicast){
	nrf24l01p_write_payload(nrf, buf, len, multicast ? W_TX_PAYLOAD_NO_ACK : W_TX_PAYLOAD ) ;
	nrf24l01p_ce(nrf, 1);
	return 1;
}

uint32_t nrf24l01p_read_payload(struct nrf24l01p *nrf, void* buf, uint8_t data_len)
{
  uint32_t status, dontCare;
  uint8_t i = 0;
  uint8_t *current = (uint8_t *)(buf);

  if(data_len > nrf->payload_size) data_len = nrf->payload_size;
  uint8_t blank_len = nrf->dynamic_payloads_enabled ? 0 : nrf->payload_size - data_len;

  //clear the rx buffer
  while(SSIDataGetNonBlocking(nrf->SSI_BASE, &dontCare));

  nrf24l01p_begin_transaction(nrf);
  SSIDataPut(nrf->SSI_BASE, R_RX_PAYLOAD );
  SSIDataGet(nrf->SSI_BASE, &status);
  while ( i < data_len ) {
	SSIDataPut(nrf->SSI_BASE, 0xff );
    SSIDataGet(nrf->SSI_BASE, (uint32_t *)(current++));
    i++;
  }
  while ( blank_len-- ) {
	  SSIDataPut(nrf->SSI_BASE, 0xff );
  }
  nrf24l01p_end_transaction(nrf);

  return status;
}

int nrf24l01p_power_up(struct nrf24l01p *nrf){
	uint32_t cfg = nrf24l01p_read_register(nrf, NRF_CONFIG);

   // if not powered up then power up and wait for the radio to initialize
   if (!(cfg & _BV(PWR_UP))){
	  nrf24l01p_write_register(nrf, NRF_CONFIG, cfg | _BV(PWR_UP));

	  // For nRF24L01+ to go from power down mode to TX or RX mode it must first pass through stand-by mode.
	  // There must be a delay of Tpd2stby (see Table 16.) after the nRF24L01+ leaves power down mode before
	  // the CEis set high. - Tpd2stby can be up to 5ms per the 1.0 datasheet
	  SysCtlDelay((SysCtlClockGet() >> 12) * 5);
   }

   return 1;
}

void nrf24l01p_ce(struct nrf24l01p *nrf, bool level){
	if(level)
		GPIOPinWrite(nrf->CE_PIN_BASE, nrf->CE_PIN, nrf->CE_PIN);
	else
		GPIOPinWrite(nrf->CE_PIN_BASE, nrf->CE_PIN, 0x00);
}

uint32_t nrf24l01p_get_status(struct nrf24l01p *nrf){
	return nrf24l01p_read_register(nrf, NRF_STATUS);
}

//************ PUBLIC *************//

//initialize the radio
int nrf24l01p_setup(struct nrf24l01p *nrf,
		uint32_t CE_PIN_BASE,
		uint32_t CE_PIN,
		uint32_t SSI_BASE){

	uint8_t setup = 0;

	nrf->SSI_BASE = SSI_BASE;
	nrf->CE_PIN_BASE = CE_PIN_BASE;
	nrf->CE_PIN = CE_PIN;
	nrf->addr_width = 5; //default addr_width
	nrf->payload_size = 32;
	nrf->pipe0_reading_address[0]=0;

	if(SSI_BASE == SSI0_BASE){
		SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

		GPIOPinConfigure(GPIO_PA2_SSI0CLK);
		GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_3);
		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
		GPIOPinConfigure(GPIO_PA4_SSI0RX);
		GPIOPinConfigure(GPIO_PA5_SSI0TX);
		GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5|GPIO_PIN_4|GPIO_PIN_2);
	}
	//TODO: other bases

	SSIConfigSetExpClk(SSI_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,SSI_MODE_MASTER,DATARATE,8);

	if(CE_PIN_BASE == GPIO_PORTC_BASE){
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	}
	//TODO: other bases
	GPIOPinTypeGPIOOutput(CE_PIN_BASE, CE_PIN);

	SSIEnable(SSI_BASE);

	SysCtlDelay((SysCtlClockGet() >> 12) * 5);

	// Reset NRF_CONFIG and enable 16-bit CRC.
	nrf24l01p_write_register(nrf, NRF_CONFIG, 0b00001100 );

	// Set 1500uS (minimum for 32B payload in ESB@250KBPS) timeouts, to make testing a little easier
	// WARNING: If this is ever lowered, either 250KBS mode with AA is broken or maximum packet
	// sizes must never be used. See documentation for a more complete explanation.
	nrf24l01p_set_retries(nrf,5,15);

	// Then set the data rate to the slowest (and most reliable) speed supported by all
	// hardware.
	nrf24l01p_set_data_rate(nrf, RF24_1MBPS );


	// Disable dynamic payloads, to match dynamic_payloads_enabled setting - Reset value is 0
	nrf24l01p_toggle_features(nrf);
	nrf24l01p_write_register(nrf, FEATURE,0 );
	nrf24l01p_write_register(nrf, DYNPD,0);

	// Reset current status
	// Notice reset and flush is the last thing we do
	nrf24l01p_write_register(nrf, NRF_STATUS,_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );

	// Set up default configuration.  Callers can always change it later.
	// This channel should be universally safe and not bleed over into adjacent
	// spectrum.
	nrf24l01p_set_channel(nrf, 76);

	// Flush buffers
	nrf24l01p_flush_rx(nrf);
	nrf24l01p_flush_tx(nrf);

	nrf24l01p_power_up(nrf); //Power up by default when begin() is called

	// Enable PTX, do not write CE high so radio will remain in standby I mode ( 130us max to transition to RX or TX instead of 1500us from powerUp )
	// PTX should use only 22uA of power
	nrf24l01p_write_register(nrf, NRF_CONFIG, ( nrf24l01p_read_register(nrf, NRF_CONFIG) ) & ~_BV(PRIM_RX) );

	setup = nrf24l01p_read_register(nrf, RF_SETUP);

	// if setup is 0 or ff then there was no response from module
	return ( setup != 0 && setup != 0xff );
}

uint32_t nrf24l01p_write_register(struct nrf24l01p *nrf, uint8_t reg, uint8_t value){
	uint32_t reg_masked, val, status;

	reg_masked = (W_REGISTER | ( REGISTER_MASK & reg ));
	val = value;

	nrf24l01p_begin_transaction(nrf);
	SSIDataPut(nrf->SSI_BASE, reg_masked);
	SSIDataPut(nrf->SSI_BASE, val);
	nrf24l01p_end_transaction(nrf);

	SSIDataGet(nrf->SSI_BASE, &status);

	return status;
}

uint32_t nrf24l01p_write_buffer(struct nrf24l01p *nrf, uint8_t reg, const uint8_t* buf, uint8_t len){
	uint8_t ix = 0;
	uint32_t reg_masked;

	reg_masked = (W_REGISTER | ( REGISTER_MASK & reg ));

	nrf24l01p_begin_transaction(nrf);
	SSIDataPut(nrf->SSI_BASE, reg_masked);

	while(ix < len){
		SSIDataPut(nrf->SSI_BASE, buf[ix]);
		ix++;
	}
	nrf24l01p_end_transaction(nrf);

	return 1;
}

uint32_t nrf24l01p_read_register(struct nrf24l01p *nrf, uint8_t reg) {
	uint32_t result, reg_masked;

	reg_masked = ( R_REGISTER | ( REGISTER_MASK & reg ) );

	//read out anything currently in the buffer
	while(SSIDataGetNonBlocking(nrf->SSI_BASE, &result));

	nrf24l01p_begin_transaction(nrf);
	SSIDataPut(nrf->SSI_BASE, reg_masked);
	SSIDataPut(nrf->SSI_BASE, 0xff);

	SSIDataGet(nrf->SSI_BASE, &result); //flush out the status reg
	SSIDataGet(nrf->SSI_BASE, &result);
	nrf24l01p_end_transaction(nrf);

	return result;
}

int nrf24l01p_read(struct nrf24l01p *nrf, void *buf, uint8_t len) {
	// Fetch the payload
	if(nrf24l01p_read_payload(nrf, buf, len)){

		//Clear the two possible interrupt flags with one command
		nrf24l01p_write_register(nrf, NRF_STATUS, _BV(RX_DR) | _BV(MAX_RT) | _BV(TX_DS));
		return 1;
	}
	else return -1;
}

//Similar to the previous write, clears the interrupt flags
bool nrf24l01p_write(struct nrf24l01p *nrf, const void* buf, uint8_t len)
{
	//Start Writing
	nrf24l01p_start_fast_write(nrf, buf, len, 0);

	while( ! ( nrf24l01p_get_status(nrf)  & ( _BV(TX_DS) | _BV(MAX_RT) )));

	nrf24l01p_ce(nrf, 0);

	uint8_t status = nrf24l01p_write_register(nrf, NRF_STATUS,_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );

  //Max retries exceeded
  if( status & _BV(MAX_RT)){
  	nrf24l01p_flush_tx(nrf); //Only going to be 1 packet int the FIFO at a time using this method, so just flush
  	return 0;
  }
	//TX OK 1 or 0
  return 1;
}

int nrf24l01p_open_writing_pipe(struct nrf24l01p *nrf, const uint8_t *address)
{
  nrf24l01p_write_buffer(nrf, RX_ADDR_P0,address, nrf->addr_width);
  nrf24l01p_write_buffer(nrf, TX_ADDR, address, nrf->addr_width);

  //const uint8_t max_payload_size = 32;
  //nrf24l01p_write_register(nrf, RX_PW_P0,rf24_min(payload_size,max_payload_size));
  nrf24l01p_write_register(nrf,RX_PW_P0,nrf->payload_size);

  return 1;
}

int nrf24l01p_open_reading_pipe(struct nrf24l01p *nrf, uint8_t child, const uint8_t *address){
	// If this is pipe 0, cache the address.  This is needed because
	// nrf24l01p_open_writing_pipe() will overwrite the pipe 0 address, so
	// nrf24l01p_start_listening() will have to restore it.
	if (child == 0){
	memcpy(nrf->pipe0_reading_address,&address,nrf->addr_width);
	}

	if (child <= 6)
	{
	// For pipes 2-5, only write the LSB
	if ( child < 2 )
	  nrf24l01p_write_buffer(nrf,child_pipe[child], address, nrf->addr_width);
	else
	  nrf24l01p_write_buffer(nrf,child_pipe[child], address, 1);

	nrf24l01p_write_register(nrf,child_payload_size[child],nrf->payload_size);

	// Note it would be more efficient to set all of the bits for all open
	// pipes at once.  However, I thought it would make the calling code
	// more simple to do it this way.
	nrf24l01p_write_register(nrf,EN_RXADDR,nrf24l01p_read_register(nrf,EN_RXADDR) | _BV(child_pipe_enable[child]));
	}

	return 1;
}

int nrf24l01p_close_reading_pipe(struct nrf24l01p *nrf, uint8_t pipe )
{
  nrf24l01p_write_register(nrf, EN_RXADDR,nrf24l01p_read_register(nrf, EN_RXADDR) & ~_BV(child_pipe_enable[pipe]));
  return 1;
}

int nrf24l01p_start_listening(struct nrf24l01p *nrf){
	nrf24l01p_write_register(nrf, NRF_CONFIG, nrf24l01p_read_register(nrf,NRF_CONFIG) | _BV(PRIM_RX));
	nrf24l01p_write_register(nrf, NRF_STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT) );
	nrf24l01p_ce(nrf, 1);
	// Restore the pipe0 adddress, if exists
	if (nrf->pipe0_reading_address[0] > 0){
		nrf24l01p_write_buffer(nrf, RX_ADDR_P0, nrf->pipe0_reading_address, nrf->addr_width);
	}else{
		nrf24l01p_close_reading_pipe(nrf, 0);
	}

	// Flush buffers
	//nrf24l01p_flush_rx(nrf);
	if(nrf24l01p_read_register(nrf, FEATURE) & _BV(EN_ACK_PAY)){
		nrf24l01p_flush_tx(nrf);
	}

	return 1;
}

int nrf24l01p_stop_listening(struct nrf24l01p *nrf){
	  nrf24l01p_ce(nrf, 0);

	  SysCtlDelay( (SysCtlClockGet() >> 21) * nrf->txRxDelay);

	  if(nrf24l01p_read_register(nrf, FEATURE) & _BV(EN_ACK_PAY)){
		  SysCtlDelay( (SysCtlClockGet() >> 21) * nrf->txRxDelay);
		nrf24l01p_flush_tx(nrf);
	  }
	  //nrf24l01p_flush_rx(nrf);
	  nrf24l01p_write_register(nrf, NRF_CONFIG, ( nrf24l01p_read_register(nrf, NRF_CONFIG) ) & ~_BV(PRIM_RX) );

	  nrf24l01p_write_register(nrf, EN_RXADDR,nrf24l01p_read_register(nrf, EN_RXADDR) | _BV(child_pipe_enable[0])); // Enable RX on pipe0

	  return 1;
}

bool nrf24l01p_available(struct nrf24l01p *nrf){
	if (!(nrf24l01p_read_register(nrf, FIFO_STATUS) & _BV(RX_EMPTY)))
		return 1;

	return 0;
}



int nrf24l01p_set_PA_level(struct nrf24l01p *nrf, uint8_t level)
{

  uint8_t setup = nrf24l01p_read_register(nrf, RF_SETUP) & 0b11111000;

  if(level > 3){  						// If invalid level, go to max PA
	  level = (RF24_PA_MAX << 1) + 1;		// +1 to support the SI24R1 chip extra bit
  }else{
	  level = (level << 1) + 1;	 		// Else set level as requested
  }

  if( nrf24l01p_write_register(nrf, RF_SETUP, setup |= level )){
	  return 1;
  }
  else return -1;
}

int nrf24l01p_set_retries(struct nrf24l01p *nrf, uint8_t delay, uint8_t count){
	return nrf24l01p_write_register(nrf, SETUP_RETR,(delay&0xf)<<ARD | (count&0xf)<<ARC);
}

int nrf24l01p_set_channel(struct nrf24l01p *nrf, uint8_t channel){
	const uint8_t max_channel = 125;
	if(nrf24l01p_write_register(nrf, RF_CH, min(channel,max_channel))){
		nrf->channel = channel;
		return 1;
	}
	else{
		return -1;
	}
}

int nrf24l01p_set_data_rate(struct nrf24l01p *nrf, rf24_datarate_e speed) {
	bool result = false;
	uint8_t setup = nrf24l01p_read_register(nrf, RF_SETUP);

	// HIGH and LOW '00' is 1Mbs - our default
	setup &= ~(_BV(RF_DR_LOW)| _BV(RF_DR_HIGH));
	nrf->txRxDelay=250;

	if( speed == RF24_250KBPS )
	{
		// Must set the RF_DR_LOW to 1; RF_DR_HIGH (used to be RF_DR) is already 0
		// Making it '10'.
		setup |= _BV(RF_DR_LOW);
		nrf->txRxDelay=450;

	}
	else
	{
		// Set 2Mbs, RF_DR (RF_DR_HIGH) is set 1
		// Making it '01'
		if ( speed == RF24_2MBPS )
		{
			setup |= _BV(RF_DR_HIGH);
			nrf->txRxDelay=190;
		}
	}

	nrf24l01p_write_register(nrf, RF_SETUP,setup);

	// Verify our result
	if ( nrf24l01p_read_register(nrf, RF_SETUP) == setup )
	{
		result = true;
	}
	return result;
}

int nrf24l01p_toggle_features(struct nrf24l01p *nrf)
{
	nrf24l01p_begin_transaction(nrf);
	SSIDataPut(nrf->SSI_BASE, ACTIVATE);
	SSIDataPut(nrf->SSI_BASE, 0x73);
	nrf24l01p_end_transaction(nrf);
	return 1;
}

