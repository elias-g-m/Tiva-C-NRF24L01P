#ifndef NRF24L01_H_
#define NRF24L01_H_

/*
    Copyright (c) 2007 Stefan Engelke <mbox@stefanengelke.de>
	Portions Copyright (C) 2011 Greg Copeland
	Portions Copyright (C) 2016 Dean Miller
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

/* Memory Map */
#define NRF_CONFIG      0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define NRF_STATUS  0x07
#define OBSERVE_TX  0x08
#define CD          0x09
#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16
#define FIFO_STATUS 0x17
#define DYNPD	    0x1C
#define FEATURE	    0x1D

/* Bit Mnemonics */
#define MASK_RX_DR  6
#define MASK_TX_DS  5
#define MASK_MAX_RT 4
#define EN_CRC      3
#define CRCO        2
#define PWR_UP      1
#define PRIM_RX     0
#define ENAA_P5     5
#define ENAA_P4     4
#define ENAA_P3     3
#define ENAA_P2     2
#define ENAA_P1     1
#define ENAA_P0     0
#define ERX_P5      5
#define ERX_P4      4
#define ERX_P3      3
#define ERX_P2      2
#define ERX_P1      1
#define ERX_P0      0
#define AW          0
#define ARD         4
#define ARC         0
#define PLL_LOCK    4
#define RF_DR       3
#define RF_PWR      6
#define RX_DR       6
#define TX_DS       5
#define MAX_RT      4
#define RX_P_NO     1
#define TX_FULL     0
#define PLOS_CNT    4
#define ARC_CNT     0
#define TX_REUSE    6
#define FIFO_FULL   5
#define TX_EMPTY    4
#define RX_FULL     1
#define RX_EMPTY    0
#define DPL_P5	    5
#define DPL_P4	    4
#define DPL_P3	    3
#define DPL_P2	    2
#define DPL_P1	    1
#define DPL_P0	    0
#define EN_DPL	    2
#define EN_ACK_PAY  1
#define EN_DYN_ACK  0

/* Instruction Mnemonics */
#define R_REGISTER    0x00
#define W_REGISTER    0x20
#define REGISTER_MASK 0x1F
#define ACTIVATE      0x50
#define R_RX_PL_WID   0x60
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define W_ACK_PAYLOAD 0xA8
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define REUSE_TX_PL   0xE3
#define NOP           0xFF

/* Non-P omissions */
#define LNA_HCURR   0

/* P model memory Map */
#define RPD         0x09
#define W_TX_PAYLOAD_NO_ACK  0xB0

/* P model bit Mnemonics */
#define RF_DR_LOW   5
#define RF_DR_HIGH  3
#define RF_PWR_LOW  1
#define RF_PWR_HIGH 2

#define DATARATE 4000000

#ifndef _BV
#define _BV(bit) (1 << (bit))
#endif

 #define min(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })

/**
 * Power Amplifier level.
 *
 * For use with setPALevel()
 */
typedef enum { RF24_PA_MIN = 0,RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX, RF24_PA_ERROR } rf24_pa_dbm_e ;

/**
 * Data rate.  How fast data moves through the air.
 *
 * For use with setDataRate()
 */
typedef enum { RF24_1MBPS = 0, RF24_2MBPS, RF24_250KBPS } rf24_datarate_e;

/**
 * CRC Length.  How big (if any) of a CRC is included.
 *
 * For use with setCRCLength()
 */
typedef enum { RF24_CRC_DISABLED = 0, RF24_CRC_8, RF24_CRC_16 } rf24_crclength_e;

struct nrf24l01p {
	uint32_t	CE_PIN_BASE;
	uint32_t	CE_PIN;
	uint32_t	SSI_BASE;
	uint8_t payload_size; /**< Fixed size of payloads */
	bool dynamic_payloads_enabled; /**< Whether dynamic payloads are enabled. */
	uint8_t pipe0_reading_address[5]; /**< Last address set on pipe 0 for reading. */
	uint8_t addr_width; /**< The address width to use - 3,4 or 5 bytes. */
	uint32_t txRxDelay; /**< Var for adjusting delays depending on datarate */
	rf24_datarate_e datarate;
	uint8_t channel;
};

//******* Internal functions (Leaving everything public for now) ********//

//SPI transaction functions. These manually write CS.
//Necessary because I think Tiva hardware SSI module only does fixed width transfers.
void nrf24l01p_begin_transaction(struct nrf24l01p *nrf);
void nrf24l01p_end_transaction(struct nrf24l01p *nrf);

uint32_t nrf24l01p_flush_rx(struct nrf24l01p *nrf);
uint32_t nrf24l01p_flush_tx(struct nrf24l01p *nrf);

//internal writing functions
uint32_t nrf24l01p_write_payload(struct nrf24l01p *nrf, const void* buf, uint8_t data_len, const uint8_t writeType);
int nrf24l01p_start_fast_write(struct nrf24l01p *nrf, const void* buf, uint8_t len, const bool multicast);

//internal reading
uint32_t nrf24l01p_read_payload(struct nrf24l01p *nrf, void* buf, uint8_t data_len);

int nrf24l01p_power_up(struct nrf24l01p *nrf);

//set receive transmit mode
void nrf24l01p_ce(struct nrf24l01p *nrf, bool level);

//read the status register (as if it doesn't tell us enough times already)
uint32_t nrf24l01p_get_status(struct nrf24l01p *nrf);

//******** Public API stuff ***************//

//initialize the radio
int nrf24l01p_setup(struct nrf24l01p *nrf, uint32_t CE_PIN_BASE,
		uint32_t CE_PIN,
		uint32_t SSI_BASE);

//Single register functions
uint32_t nrf24l01p_write_register(struct nrf24l01p *nrf, uint8_t reg, uint8_t value);
uint32_t nrf24l01p_write_buffer(struct nrf24l01p *nrf, uint8_t reg, const uint8_t* buf, uint8_t len);
uint32_t nrf24l01p_read_register(struct nrf24l01p *nrf, uint8_t reg);

//reading and writing
int nrf24l01p_read(struct nrf24l01p *nrf, void *buf, uint8_t len);

bool nrf24l01p_write(struct nrf24l01p *nrf, const void* buf, uint8_t len);

//Pipe opening and closing
int nrf24l01p_open_writing_pipe(struct nrf24l01p *nrf, const uint8_t *address);

int nrf24l01p_open_reading_pipe(struct nrf24l01p *nrf, uint8_t number, const uint8_t *address);
int nrf24l01p_close_reading_pipe(struct nrf24l01p *nrf, uint8_t pipe );

//start listening on pipes opened for reading
int nrf24l01p_start_listening(struct nrf24l01p *nrf);

//stop listening and switch to transmit mode
int nrf24l01p_stop_listening(struct nrf24l01p *nrf);

//check whether there are bytes available to be read
bool nrf24l01p_available(struct nrf24l01p *nrf);

//settings
int nrf24l01p_set_PA_level(struct nrf24l01p *nrf, uint8_t level);
int nrf24l01p_set_retries(struct nrf24l01p *nrf, uint8_t delay, uint8_t count);
int nrf24l01p_set_channel(struct nrf24l01p *nrf, uint8_t channel);
int nrf24l01p_set_data_rate(struct nrf24l01p *nrf, rf24_datarate_e speed);
int nrf24l01p_toggle_features(struct nrf24l01p *nrf);

#endif /* NRF24L01_H_ */
