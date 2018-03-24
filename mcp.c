
/* $Id$ */

#include <ctype.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <util/twi.h>
#include <avr/eeprom.h>

#include <mcp.h>

#define regbit_set_up(reg, bit)    (reg) |= (1 << (bit))
#define regbit_set_down(reg, bit)  (reg) &= ~(1 << (bit))
#define regbit_is_set(reg, bit)    ((reg) & (1 << (bit)))
#define reg_set_value(reg, value)  ((reg) = (value))

#define TX_BUF_COUNT 3
#define RX_BUF_COUNT 2

#define PIN_SCK   PB5
#define PIN_MISO  PB4
#define PIN_MOSI  PB3
#define PIN_SS    PB2

void spi_init(void) {
    /* Set pins as input */
    regbit_set_down(DDRB, PIN_MISO);

    /* Set pins as output */
    regbit_set_up(DDRB, PIN_SS);
    regbit_set_up(DDRB, PIN_MOSI);
    regbit_set_up(DDRB, PIN_SCK);

    /* Enable SPI interrupt(s) */
    //regbit_set_up(SPCR, SIPE);

    /* Set SPI as master */
    regbit_set_up(SPCR, MSTR);

    /* Set clock prescaler 1/16 */
    regbit_set_up(SPCR, SPR0);
    regbit_set_down(SPCR, SPR1);

    /* Set clock prescaler 1/4 */
    //regbit_set_down(SPCR, SPR0);
    //regbit_set_down(SPCR, SPR1);


    /* Reset status register */
    reg_set_value(SPSR, 0);
    /* Enable SPI */
    regbit_set_up(SPCR, SPE);
}

inline void spi_write_byte(uint8_t data) {
    SPDR = data;
    while (!(SPSR & (1 << SPIF)));
}

inline uint8_t spi_read_byte(void) {
    SPDR = 0xFF;
    while (!(SPSR & (1 << SPIF)));
    return SPDR;
}

#define spi_select_mcp()   regbit_set_down(PORTB, PIN_SS)
#define spi_unselect_mcp() regbit_set_up(PORTB, PIN_SS)

inline void _spi_select_mcp(void) {
    regbit_set_down(PORTB, PIN_SS);
}

inline void _spi_unselect_mcp(void) {
    regbit_set_up(PORTB, PIN_SS);
}

void mcp_write_reg(uint8_t address, uint8_t data) {
    spi_select_mcp();

    spi_write_byte(MCP_CMD_WRITE);
    spi_write_byte(address);
    spi_write_byte(data);

    spi_unselect_mcp();
}

uint8_t mcp_read_reg(uint8_t address) {
    uint8_t data;
    spi_select_mcp();

    spi_write_byte(MCP_CMD_READ);
    spi_write_byte(address);
    data = spi_read_byte();

    spi_unselect_mcp();
    return data;
}

void mcp_modify_reg(uint8_t address, uint8_t mask, uint8_t data) {
    spi_select_mcp();

    spi_write_byte(MCP_CMD_BIT_MODIFY);
    spi_write_byte(address);
    spi_write_byte(mask);
    spi_write_byte(data);

    spi_unselect_mcp();
}

void mcp_reg_bits_up(uint8_t address, uint8_t bits) {
    uint8_t reg = mcp_read_reg(address);
    reg |= bits;
    mcp_write_reg(address, reg);
}

void mcp_reg_bits_down(uint8_t address, uint8_t bits) {
    uint8_t reg = mcp_read_reg(address);
    reg &= ~(bits);
    mcp_write_reg(address, reg);
}


void mcp_reset(void) {
    spi_select_mcp();
    spi_write_byte(MCP_CMD_RESET);
    _delay_ms(10);
    spi_unselect_mcp();
    _delay_ms(10);
}

void mcp_read_rx(mcp_buffer_t *buffer, uint8_t rx) {

    uint8_t rx_cmd_read[] = { MCP_CMD_READ_RX0, MCP_CMD_READ_RX1 };

    spi_select_mcp();

    spi_write_byte(rx_cmd_read[rx]);
    buffer->sidh = spi_read_byte();
    buffer->sidl = spi_read_byte();
    buffer->eid8 = spi_read_byte();
    buffer->eid0 = spi_read_byte();
    buffer->dlc = spi_read_byte();

    buffer->d[0]  = spi_read_byte();
    buffer->d[1]  = spi_read_byte();
    buffer->d[2]  = spi_read_byte();
    buffer->d[3]  = spi_read_byte();
    buffer->d[4]  = spi_read_byte();
    buffer->d[5]  = spi_read_byte();
    buffer->d[6]  = spi_read_byte();
    buffer->d[7]  = spi_read_byte();

    spi_unselect_mcp();
}

void mcp_load_tx(mcp_buffer_t *buffer, uint8_t tx) {

    uint8_t tx_write_cmd[TX_BUF_COUNT] = { MCP_CMD_WRITE_TX0, MCP_CMD_WRITE_TX1, MCP_CMD_WRITE_TX2 };
    spi_select_mcp();

    spi_write_byte(tx_write_cmd[tx]);

    spi_write_byte(buffer->sidh);
    spi_write_byte(buffer->sidl);
    spi_write_byte(buffer->eid8);
    spi_write_byte(buffer->eid0);
    spi_write_byte(buffer->dlc);
    spi_write_byte(buffer->d[0]);
    spi_write_byte(buffer->d[1]);
    spi_write_byte(buffer->d[2]);
    spi_write_byte(buffer->d[3]);
    spi_write_byte(buffer->d[4]);
    spi_write_byte(buffer->d[5]);
    spi_write_byte(buffer->d[6]);
    spi_write_byte(buffer->d[7]);

    spi_unselect_mcp();
}

uint8_t mcp_read_status(void) {
    spi_select_mcp();
    spi_write_byte(MCP_CMD_GET_READ_STATUS);
    uint8_t data = spi_read_byte();
    spi_unselect_mcp();
    return data;
}

uint8_t mcp_read_rx_status(void) {
    spi_select_mcp();
    spi_write_byte(MCP_CMD_GET_RX_STATUS);
    uint8_t data = spi_read_byte();
    spi_unselect_mcp();
    return data;
}

//#define MCP_LOOPBACK_MODE   (1 << REQOP1)
#define MCP_NORMAL_MODE     0x00    //000 = Set Normal Operation mode
#define MCP_SLEEP_MODE      0x20    //001 = Set Sleep mode
#define MCP_LOOPBACK_MODE   0x40    //010 = Set Loopback mode
#define MCP_LISTEN_MODE     0x60    //011 = Set Listen-only mode
#define MCP_CONF_MODE       0x80    //100 = Set Configuration mode

void mcp_set_mode(uint8_t mode) {
    uint8_t reg = mcp_read_reg(CANCTRL);
    /* Mask mode bits */
    reg &= 0x1F;
    /* Set mode bits */
    reg |= mode;
    mcp_write_reg(CANCTRL, reg);
}

/* ID allocation into MCP2515:
 * ID 10:3  SIDH: SID10 .............................  SID3
 * ID  2:0  SIDL: SID2  SID1 SID0 SRR  IDE  XXX  EID17 EID16
 * ID  8:19 EID8: EID15 .............................  EID8
 * ID 18:11 EID8: EID7  .............................  EID0
 */
void mcp_pack_msg(can_msg_t *msg, mcp_buffer_t *buffer) {

    if (msg->exid) {
        buffer->sidh = (uint8_t)(msg->id >> 3);
        buffer->sidl = ((uint8_t)(msg->id << 5));
        buffer->sidl |= ((uint8_t)(msg->id >> 27) & 0x03);
        buffer->sidl |= (1 << EXIDE);

        buffer->eid8 = (uint8_t)(msg->id >> 19);
        buffer->eid0 = (uint8_t)(msg->id >> 11);
    } else {
        buffer->sidh = (uint8_t)(msg->id >> 3);
        buffer->sidl = ((uint8_t)(msg->id << 5));
        buffer->eid8 = 0x00;
        buffer->eid0 = 0x00;
    }
    for (uint8_t i = 0; i < msg->length; i++) 
        buffer->d[i] = msg->data[i];

    buffer->dlc = (uint8_t)(msg->length) & 0x0F;
}

void mcp_unpack_msg(mcp_buffer_t *buffer, can_msg_t *msg) {

    if (buffer->sidl & (1 << EXIDE)) {
        msg->id =  (uint32_t)((buffer->sidh) << 3);
        msg->id |= (uint32_t)(buffer->sidl) >> 5;

        msg->id |= ((uint32_t)(buffer->sidl) & 0x03) << 27;
        msg->id |= (uint32_t)(buffer->eid8) << 19;
        msg->id |= (uint32_t)(buffer->eid0) << 11;
        msg->exid = true;

    } else {
        msg->id =  (uint32_t)(buffer->sidh) << 3;
        msg->id |= (uint32_t)(buffer->sidl) >> 5;
        msg->exid = false;
    }

    for (uint8_t i = 0; i < msg->length; i++) 
        msg->data[i] = buffer->d[i];

    msg->length = (buffer->dlc) & 0x0F;
}

int8_t mcp_find_free_tx(void) {
    uint8_t txnctrl[TX_BUF_COUNT] = { TXB0CTRL, TXB1CTRL, TXB2CTRL };

    uint8_t i = 0;
    while(i < TX_BUF_COUNT) {
        uint8_t reg = mcp_read_reg(txnctrl[i]);
        if (!(reg & (1 << TXREQ)))
            return i;
        i++;
    }
    return -1;
}

void mcp_set_tx(can_msg_t *msg, int8_t tx) {
    uint8_t txnctrl[TX_BUF_COUNT] = { TXB0CTRL, TXB1CTRL, TXB2CTRL };
    mcp_write_reg(txnctrl[tx], (uint8_t)(msg->priority));
}

void mcp_rts(int8_t tx) {
    uint8_t tx_rts_cmd[TX_BUF_COUNT] = { MCP_CMD_RTS_TX0, MCP_CMD_RTS_TX1, MCP_CMD_RTS_TX2 };
    spi_select_mcp();
    spi_write_byte(tx_rts_cmd[tx]);
    spi_unselect_mcp();
}

bool mcp_send_msg(can_msg_t *msg) {
    mcp_buffer_t buffer;

    int8_t tx = mcp_find_free_tx();
    if(tx < 0)
        return false;

    mcp_set_tx(msg, tx);
    mcp_pack_msg(msg, &buffer);
    mcp_load_tx(&buffer, tx);
    mcp_rts(tx);

    return true;
}

#define MCP_INTERRUPTS  ((1 << RX1IE) | (1 << RX0IE))

#if 0
    // 10 kbps
    {0x04, 0xb6, 0xe7},
    // 20 kbps
    {0x04, 0xb6, 0xd3},
    // 50 kbps
    {0x04, 0xb6, 0xc7},
    // 100 kbps
    {0x04, 0xb6, 0xc3}, 
    // 125 kbps
    {(1 << PHSEG21), (1 << BTLMODE) | (1 << PHSEG11), (1 << BRP2) | (1 << BRP1) | (1 << BRP0)},
    // 250 kbps
    {0x03, 0xac, 0x81},
    // 500 kbps
    {0x03, 0xac, 0x80},
    // 1 Mbps
    {(1 << PHSEG21),  (1 << BTLMODE) | (1 << PHSEG11), 0}

#endif


void mcp_init(void) {
    mcp_reset();
    mcp_write_reg(TXRTSCTRL, 0);
    mcp_write_reg(CANINTE, MCP_INTERRUPTS);

    mcp_write_reg(CNF1, ((1 << BRP2) | (1 << BRP1) | (1 << BRP0)));
    mcp_write_reg(CNF2, ((1 << BTLMODE) | (1 << PHSEG11)));
    mcp_write_reg(CNF3, (1 << PHSEG21));

    //mcp_set_mode(MCP_LOOPBACK_MODE);
    mcp_set_mode(MCP_NORMAL_MODE);
}

#if MCP_DEBUG

void mcp_print_buffer(mcp_buffer_t *buffer) {
    printf("\r\nSIDH = 0x%02X\r\n", buffer->sidh);
    printf("SIDL = 0x%02X\r\n", buffer->sidl);
    printf("EID8 = 0x%02X\r\n", buffer->eid8);
    printf("EID0 = 0x%02X\r\n", buffer->eid0);
    printf("DLC  = 0x%02X\r\n", buffer->dlc);
    printf("D0   = 0x%02X\r\n", buffer->d[0]);
    printf("D1   = 0x%02X\r\n", buffer->d[1]);
    printf("D2   = 0x%02X\r\n", buffer->d[2]);
    printf("D3   = 0x%02X\r\n", buffer->d[3]);
    printf("D4   = 0x%02X\r\n", buffer->d[4]);
    printf("D5   = 0x%02X\r\n", buffer->d[5]);
    printf("D6   = 0x%02X\r\n", buffer->d[6]);
    printf("D7   = 0x%02X\r\n", buffer->d[7]);
}

void mcp_print_reg(uint8_t address) {
    uint8_t data = mcp_read_reg(address);
    printf("REG 0x%02X = 0x%02X\r\n", address, data);
}

void mcp_print_rx (uint8_t rx) {
    mcp_buffer_t buffer;
    mcp_read_rx(&buffer, rx);
    mcp_print_buffer(&buffer);
}

void mcp_print_bit(uint8_t *descr, uint8_t byte, uint8_t bit) {
    uint8_t tmpl[] = "BIT %d = %d\r\n";
    printf(descr);
    if (byte & (1 << bit))
        printf(tmpl, bit, 1);
    else
        printf(tmpl, bit, 0);
}

void mcp_print_status (void) {
    uint8_t data = mcp_read_status();

    mcp_print_bit("CANINTF.RX0IF   ", data, 0);
    mcp_print_bit("CANINTFL.RX1IF  ", data, 1);
    mcp_print_bit("TXB0CNTRL.TXREQ ", data, 2);
    mcp_print_bit("CANINTF.TX0IF   ", data, 3);
    mcp_print_bit("TXB1CNTRL.TXREQ ", data, 4);
    mcp_print_bit("CANINTF.TX1IF   ", data, 5);
    mcp_print_bit("TXB2CNTRL.TXREQ ", data, 6);
    mcp_print_bit("CANINTF.TX2IF   ", data, 7);
}

void mcp_print_rx_status (void) {
    uint8_t data = mcp_read_rx_status();
    mcp_print_bit("Rem frame ", data, 3);
    mcp_print_bit("Ext frame ", data, 4);
    mcp_print_bit("RXB0 stat ", data, 6);
    mcp_print_bit("RXB1 stat ", data, 7);
}

#endif

/* EOF */
