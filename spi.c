

#include <avr/io.h>

#define PIN_SCK   PB5
#define PIN_MISO  PB4
#define PIN_MOSI  PB3
#define PIN_SS    PB2

#define regbit_set_up(reg, bite)    (reg) |= (1 << (bite))
#define regbit_set_down(reg, bite)  (reg) &= ~(1 << (bite))

void spi_init(void) {

    /* Set PIN_MISO as input */
    regbit_set_down(DDRB, PIN_MISO);

    /* Set PIN_SS as output */
    regbit_set_up(DDRB, PIN_SS);
    regbit_set_up(DDRB, PIN_MOSI);
    regbit_set_up(DDRB, PIN_SCK);

    SPCR = (1 << SPE) | (1 << MSTR) | (1<<SPR0);
    SPSR = 0;
}

uint8_t spi_putc(uint8_t data) {
    SPDR = data;
    while (!(SPSR & (1 << SPIF)));
    return SPDR;
}


