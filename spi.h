#ifndef	SPI_H
#define	SPI_H

#include <avr/io.h>

inline void spi_start(uint8_t data) {
    SPDR = data;
}

inline uint8_t spi_wait(void) {
    while (!(SPSR & (1 << SPIF)));
    return SPDR;
}

void spi_init(void);
uint8_t spi_putc(uint8_t data);

#endif

