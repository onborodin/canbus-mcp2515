
/* $Id$ */

#ifndef UART_H_ITU
#define UART_H_ITU

#include <fifo.h>

extern fifo_t fifo_in, fifo_out;
extern FILE uart_str;

int uart_putchar(char c, FILE * stream);
int uart_getchar(FILE * stream);
void io_hook(void);
void uart_init(void);

#endif
/* EOF */
