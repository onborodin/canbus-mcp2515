
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

void int0_init(void) {
    /* Interrupt by raise signal */
    //regbit_set_up(EICRA, ISC00);
    //regbit_set_up(EICRA, ISC01);

    /* Interrupt by drop signal */
    regbit_set_down(EICRA, ISC00);
    regbit_set_up(EICRA, ISC01);

    /* Set PD2 pin to input */
    //regbit_set_down(DDRD, PD2);
    regbit_set_down(DDRD, PD3);
    /* Enable external interrupt INT0 */
    regbit_set_up(EIMSK, INT0);
}

uint32_t rx_count0 = 0;
uint32_t rx_count1 = 0;

ISR(INT0_vect) {
    uint8_t reg = mcp_read_reg(CANINTF);
    if (reg && (1 << MERRF));
    if (reg & (1 << WAKIF));
    if (reg & (1 << ERRIF));
    if (reg & (1 << TX2IF));
    if (reg & (1 << TX1IF));
    if (reg & (1 << TX0IF));

    if (reg & (1 << RX1IF)) {
        mcp_buffer_t buffer;
        mcp_read_rx(&buffer, 0);
        rx_count1++;
    }

    if (reg & (1 << RX0IF)) {
        mcp_buffer_t buffer;
        mcp_read_rx(&buffer, 0);
        rx_count0++;
        //for (uint8_t i = 0; i < 8; i++)
        //        fifo_putc(&fifo_out, buffer.d[i]);
        //fifo_putc(&fifo_out, '\r');
        //fifo_putc(&fifo_out, '\n');
    }
}

#define MAX_CMD_LEN 164

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

    mcp_init(MCP_NORMAL_MODE, MCP_BR_125KB_CNF);

    sei();

    mcp_buffer_t buffer;
    uint8_t i = 0;

    can_msg_t msg = {
        .id = 0x123,
        .priority = 0x01,
        .length = 8,
        .data = { 'Q', 'W', 'E', 'R', 'T', 'Y', '1', '2' } 
    };

    while (1) {
        printf("%8lX %8lX\r\n", rx_count0, rx_count1);
        for (uint8_t i = 0; i < 20; i++) {
            mcp_send_msg(&msg);
            _delay_ms(50);
        }
        while (fifo_get_token(&fifo_in, str, MAX_CMD_LEN, '\r') > 0) {
            int8_t ret_code = shell(str, shell_act, sizeof(shell_act) / sizeof(shell_act[0]));
            if (ret_code == SH_CMD_NOTFND)
                printf("COMMAND NOT FOUND\r\n");
            printf(prompt);
        }
        _delay_ms(100);
    }
}
/* EOF */
