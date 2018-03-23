
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

#define BAUD 19200
#include <util/setbaud.h>

#include <uart.h>
#include <fifo.h>
#include <tools.h>
#include <shell.h>

#include <mcp.h>

#define regbit_set_up(reg, bit)    (reg) |= (1 << (bit))
#define regbit_set_down(reg, bit)  (reg) &= ~(1 << (bit))
#define regbit_is_set(reg, bit)    ((reg) & (1 << (bit)))
#define reg_set_value(reg, value)  ((reg) = (value))


/* Shell */
act_t shell_act[] = { };



ISR(USART_RX_vect) {
    volatile uint8_t ichar = UDR0;

    if (ichar == '\r') {
        fifo_putc(&fifo_in, '\n');
        fifo_putc(&fifo_out, '\n');
    }

    fifo_putc(&fifo_in, ichar);
    fifo_putc(&fifo_out, ichar);
}


/* Timer0 */
void timer0_init(void) {

    /* Set clock to 1/64 */
    regbit_set_up(TCCR0B, CS00);
    regbit_set_up(TCCR0B, CS01);

    /* Enable interrupt */
    regbit_set_up(TIMSK0,TOIE0);

    /* Disable comparators */
    regbit_set_down(TCCR0A, COM0A0);
    regbit_set_down(TCCR0A, COM0A1);

    regbit_set_down(TCCR0A, COM0B1); 
    regbit_set_down(TCCR0A, COM0B0);
}

/* Timer 0 */
ISR(TIMER0_OVF_vect) {
    volatile uint8_t c;

    while ((c = fifo_getc(&fifo_out)) > 0) {
        while(!regbit_is_set(UCSR0A, UDRE0));
        UDR0 = c;
    }
}

#define MAX_CMD_LEN 164

/* --- BEGIN MCP ---------------------------------------------------- */


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
    /* Set clock prescaler 1/61 */
    regbit_set_up(SPCR, SPR0);

    /* Reset status register */
    reg_set_value(SPSR, 0);
    /* Enable SPI */
    regbit_set_up(SPCR, SPE);
}

void spi_write_byte(uint8_t data) {
    SPDR = data;
    while (!(SPSR & (1 << SPIF)));
}

uint8_t spi_read_byte(void) {
    SPDR = 0xFF;
    while (!(SPSR & (1 << SPIF)));
    return SPDR;
}

#define spi_select_mcp()   regbit_set_down(PORTB, PIN_SS)
#define spi_unselect_mcp() regbit_set_up(PORTB, PIN_SS)

typedef struct mcp_buffer {
    uint8_t ctrl;
    uint8_t sidh;
    uint8_t sidl;
    uint8_t eid8;
    uint8_t eid0;
    uint8_t dlc;
    uint8_t d[8];
} mcp_buffer_t;


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

void _mcp_rts(uint8_t command) {
    spi_select_mcp();
    spi_write_byte(command);
    spi_unselect_mcp();
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


/* CAN 2.0B Extended Message, MCP2515 Register Assignments 
 * +--------------------------------+-----------------------------------------------------+
 * | CAN Standard ID                | CAN Extended ID                                     |
 * +-----------------------+--------+-----+-----------------------+-----------------------+
 * | SIDH                  | SIDL   | SIDL| EID8                  | EID0                  |
 * +-----------------------+--------+-----+-----------------------+-----------------------+
 */

typedef struct can_msg {
    uint32_t id;
    uint32_t priority;
    uint8_t length;
    uint8_t data[8];
} can_msg_t;

void mcp_pack_msg(can_msg_t *msg, mcp_buffer_t *buffer) {
    #define EXT_MSG 1
    #if EXT_MSG
    buffer->sidh = (uint8_t)(msg->id >> 3);
    buffer->sidl = ((uint8_t)(msg->id << 5)) | (1 << EXIDE) | ((uint8_t)(msg->id >> 27) & 0x03);
    buffer->eid8 = (uint8_t)(msg->id >> 19);
    buffer->eid0 = (uint8_t)(msg->id >> 11);
    #else
    buffer->sidh = (uint8_t)(msg->id >> 3);
    buffer->sidl = ((uint8_t)(msg->id << 5));
    buffer->eid8 = 0x00;
    buffer->eid0 = 0x00;
    #endif

    for (uint8_t i = 0; i < msg->length; i++) 
        buffer->d[i] = msg->data[i];

    buffer->dlc = (uint8_t)(msg->length) & 0x0F;

}

void mcp_unpack_msg(mcp_buffer_t *buffer, can_msg_t *msg) {
    #define EXT_MSG 1
    #if EXT_MSG
    msg->id =  (uint32_t)(buffer->sidh) << 3;
    msg->id |= (uint32_t)(buffer->sidl) >> 5;
    msg->id |= (uint32_t)(buffer->eid8) << 19;
    msg->id |= (uint32_t)(buffer->eid0) << 11;
    #else
    msg->id =  (uint32_t)(buffer->sidh) << 3;
    msg->id |= (uint32_t)(buffer->sidl) >> 5;
    #endif

    for (uint8_t i = 0; i < msg->length; i++) 
        msg->data[i] = buffer->d[i];

    msg->length = (buffer->dlc) & 0x0F;
}



#define TX_BUF_COUNT 3

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

void mcp_load_tx(mcp_buffer_t *buffer, int8_t tx) {

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

void mcp_print_buffer(mcp_buffer_t *buffer);


bool mcp_send_msg(can_msg_t *msg) {
    mcp_buffer_t buffer;

    int8_t tx = mcp_find_free_tx();
    if(tx < 0)
        return false;

    mcp_set_tx(msg, tx);
    mcp_pack_msg(msg, &buffer);
    mcp_print_buffer(&buffer);
    mcp_load_tx(&buffer, tx);
    mcp_rts(tx);

    return true;
}

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


void int0_init(void) {
    /* Interrupt by raise signal */
    //regbit_set_up(EICRA, ISC00);
    //regbit_set_up(EICRA, ISC01);

    /* Interrupt by drop signal */
    regbit_set_down(EICRA, ISC00);
    regbit_set_up(EICRA, ISC01);

    /* Set PD2 pin to input */
    regbit_set_down(DDRD, PD2);
    /* Enable external interrupt INT0 */
    regbit_set_up(EIMSK, INT0);
}

ISR(INT0_vect) {

    uint8_t reg = mcp_read_reg(CANINTF);
    if (reg && (1 << MERRF));
    if (reg & (1 << WAKIF));
    if (reg & (1 << ERRIF));
    if (reg & (1 << TX2IF));
    if (reg & (1 << TX1IF));
    if (reg & (1 << TX0IF));
    if (reg & (1 << RX1IF));

    if (reg & (1 << RX0IF)) {
        mcp_buffer_t buffer;
        can_msg_t msg;
        mcp_read_rx(&buffer, 0);
        mcp_unpack_msg(&buffer, &msg);
        printf("len %3u: ", msg.length); 
        for (uint8_t i = 0; i < 8; i++) 
            printf("%3u ", msg.data[i]); 
        printf("err count: 0x%02X", mcp_read_reg(REC));
        printf("\r\n");
    }
}

#define MCP_INTERRUPTS  ((1 << RX1IE) | (1 << RX0IE))

bool mcp_init(void) {

    mcp_reset();
    mcp_write_reg(TXRTSCTRL, 0);
    mcp_write_reg(CANINTE, MCP_INTERRUPTS);

    #if 0
    spi_select_mcp();
    spi_write_byte(MCP_CMD_WRITE);
    spi_write_byte(CNF3);
    spi_write_byte(mcp_cnf[bitrate][0]);
    spi_write_byte(mcp_cnf[bitrate][1]);
    spi_write_byte(mcp_cnf[bitrate][2]);
    spi_unselect_mcp();
    #endif

    mcp_write_reg(CNF3, (1 << PHSEG21));
    mcp_write_reg(CNF2, ((1 << BTLMODE) | (1 << PHSEG11)));
    mcp_write_reg(CNF1, ((1 << BRP2) | (1 << BRP1) | (1 << BRP0)));

    //mcp_set_mode(MCP_LOOPBACK_MODE);
    mcp_set_mode(MCP_NORMAL_MODE);

}

/* --- END MCP ---------------------------------------------------- */

int main() {
    io_hook();
    uart_init();
    _delay_ms(100);
    timer0_init();
    int0_init();

    _delay_ms(100);

    uint8_t str[MAX_CMD_LEN];
    uint8_t prompt[] = "READY>";
    printf(prompt);

    spi_init();
    mcp_init();

    sei();

    can_msg_t _msg = {
        .id = 0x55,
        .priority = 0x01,
        .length = 8,
        .data = { 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17 } 
    };

    mcp_buffer_t buffer;
    uint8_t i = 0;

    while (1) {

        can_msg_t msg = {
            .id = i,
            .priority = 0x01,
            .length = 8,
            .data = { i, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17 } 
        };

        i++;



        //printf("Free TX is %d\r\n", mcp_find_free_tx());
        //mcp_print_buffer(&buffer);
        //mcp_pack_msg(&msg, &buffer);
        //mcp_print_buffer(&buffer);
        //mcp_send_msg(&msg);
        //mcp_write_reg(RXB0SIDH, 0x44);

        //mcp_print_reg(RXB0SIDL);
        //mcp_print_rx(0);

        //mcp_print_reg(RXB1SIDL);
        //mcp_print_rx(1);

        //mcp_print_reg(RXB1D2);
        //mcp_print_rx(1);

        //printf("---BUFF:\r\n");

        ///mcp_print_buffer(&buffer);

        //mcp_send_msg(&msg);
        //_delay_ms(10);
        //mcp_print_rx_status();

        //printf("---RX0:\r\n");
        //mcp_print_rx(0);
        //printf("---RX1:\r\n");
        //mcp_print_rx(1);

        while (fifo_get_token(&fifo_in, str, MAX_CMD_LEN, '\r') > 0) {
            int8_t ret_code = shell(str, shell_act, sizeof(shell_act) / sizeof(shell_act[0]));
            if (ret_code == SH_CMD_NOTFND)
                printf("COMMAND NOT FOUND\r\n");
            printf(prompt);
        }
        _delay_ms(500);
    }
}
/* EOF */
