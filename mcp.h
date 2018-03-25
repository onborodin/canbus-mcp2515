
/*
 * Copyright 2018 Oleg Borodin <onborodin@gmail.com>
 */

#ifndef MCP_H_ITU
#define MCP_H_ITU

/* MCP commands */
#define MCP_CMD_RESET         0xC0
#define MCP_CMD_READ          0x03
#define MCP_CMD_WRITE         0x02

#define MCP_CMD_RTS_BASE      0x80
#define MCP_CMD_RTS_TX0       0x81
#define MCP_CMD_RTS_TX1       0x82
#define MCP_CMD_RTS_TX2       0x84

#define MCP_CMD_BIT_MODIFY    0x05

#define MCP_CMD_READ_RX_BASE  0x90
#define MCP_CMD_READ_RX0      0x90
#define MCP_CMD_READ_RX1      0x94

#define MCP_CMD_WRITE_TX0     0x40
#define MCP_CMD_WRITE_TX1     0x42
#define MCP_CMD_WRITE_TX2     0x44

#define MCP_CMD_GET_READ_STATUS   0xA0
#define MCP_CMD_GET_RX_STATUS     0xB0

/* MCP2515 register adresses  */
#define RXF0SIDH    0x00
#define RXF0SIDL    0x01
#define RXF0EID8    0x02
#define RXF0EID0    0x03
#define RXF1SIDH    0x04
#define RXF1SIDL    0x05
#define RXF1EID8    0x06
#define RXF1EID0    0x07
#define RXF2SIDH    0x08
#define RXF2SIDL    0x09
#define RXF2EID8    0x0A
#define RXF2EID0    0x0B
#define BFPCTRL     0x0C
#define TXRTSCTRL   0x0D
#define CANSTAT     0x0E
#define CANCTRL     0x0F

#define RXF3SIDH    0x10
#define RXF3SIDL    0x11
#define RXF3EID8    0x12
#define RXF3EID0    0x13
#define RXF4SIDH    0x14
#define RXF4SIDL    0x15
#define RXF4EID8    0x16
#define RXF4EID0    0x17
#define RXF5SIDH    0x18
#define RXF5SIDL    0x19
#define RXF5EID8    0x1A
#define RXF5EID0    0x1B
#define TEC         0x1C
#define REC         0x1D

#define RXM0SIDH    0x20
#define RXM0SIDL    0x21
#define RXM0EID8    0x22
#define RXM0EID0    0x23
#define RXM1SIDH    0x24
#define RXM1SIDL    0x25
#define RXM1EID8    0x26
#define RXM1EID0    0x27
#define CNF3        0x28
#define CNF2        0x29
#define CNF1        0x2A
#define CANINTE     0x2B
#define CANINTF     0x2C
#define EFLG        0x2D

#define TXB0CTRL    0x30
#define TXB0SIDH    0x31
#define TXB0SIDL    0x32
#define TXB0EID8    0x33
#define TXB0EID0    0x34
#define TXB0DLC     0x35
#define TXB0D0      0x36
#define TXB0D1      0x37
#define TXB0D2      0x38
#define TXB0D3      0x39
#define TXB0D4      0x3A
#define TXB0D5      0x3B
#define TXB0D6      0x3C
#define TXB0D7      0x3D

#define TXB1CTRL    0x40
#define TXB1SIDH    0x41
#define TXB1SIDL    0x42
#define TXB1EID8    0x43
#define TXB1EID0    0x44
#define TXB1DLC     0x45
#define TXB1D0      0x46
#define TXB1D1      0x47
#define TXB1D2      0x48
#define TXB1D3      0x49
#define TXB1D4      0x4A
#define TXB1D5      0x4B
#define TXB1D6      0x4C
#define TXB1D7      0x4D

#define TXB2CTRL    0x50
#define TXB2SIDH    0x51
#define TXB2SIDL    0x52
#define TXB2EID8    0x53
#define TXB2EID0    0x54
#define TXB2DLC     0x55
#define TXB2D0      0x56
#define TXB2D1      0x57
#define TXB2D2      0x58
#define TXB2D3      0x59
#define TXB2D4      0x5A
#define TXB2D5      0x5B
#define TXB2D6      0x5C
#define TXB2D7      0x5D

#define RXB0CTRL    0x60
#define RXB0SIDH    0x61
#define RXB0SIDL    0x62
#define RXB0EID8    0x63
#define RXB0EID0    0x64
#define RXB0DLC     0x65
#define RXB0D0      0x66
#define RXB0D1      0x67
#define RXB0D2      0x68
#define RXB0D3      0x69
#define RXB0D4      0x6A
#define RXB0D5      0x6B
#define RXB0D6      0x6C
#define RXB0D7      0x6D

#define RXB1CTRL    0x70
#define RXB1SIDH    0x71
#define RXB1SIDL    0x72
#define RXB1EID8    0x73
#define RXB1EID0    0x74
#define RXB1DLC     0x75
#define RXB1D0      0x76
#define RXB1D1      0x77
#define RXB1D2      0x78
#define RXB1D3      0x79
#define RXB1D4      0x7A
#define RXB1D5      0x7B
#define RXB1D6      0x7C
#define RXB1D7      0x7D


/* Register bit definition */

/* Bit definition of BFPCTRL */
#define B1BFS       5
#define B0BFS       4
#define B1BFE       3
#define B0BFE       2
#define B1BFM       1
#define B0BFM       0

/* Bit definition of TXRTSCTRL */
#define B2RTS       5
#define B1RTS       4
#define B0RTS       3
#define B2RTSM      2
#define B1RTSM      1
#define B0RTSM      0

/* Bit definition of CANSTAT */
#define OPMOD2      7
#define OPMOD1      6
#define OPMOD0      5
#define ICOD2       3
#define ICOD1       2
#define ICOD0       1

/* Bit definition of CANCTRL */
#define REQOP2      7
#define REQOP1      6
#define REQOP0      5
#define ABAT        4
#define CLKEN       2
#define CLKPRE1     1
#define CLKPRE0     0

/* Bit definition of CNF3 */
#define WAKFIL      6
#define PHSEG22     2
#define PHSEG21     1
#define PHSEG20     0

/* Bit definition of CNF2 */
#define BTLMODE     7
#define SAM         6
#define PHSEG12     5
#define PHSEG11     4
#define PHSEG10     3
#define PHSEG2      2
#define PHSEG1      1
#define PHSEG0      0

/* Bit definition of CNF1 */
#define SJW1        7
#define SJW0        6
#define BRP5        5
#define BRP4        4
#define BRP3        3
#define BRP2        2
#define BRP1        1
#define BRP0        0

/* Bit definition of CANINTE */
#define MERRE       7
#define WAKIE       6
#define ERRIE       5
#define TX2IE       4
#define TX1IE       3
#define TX0IE       2
#define RX1IE       1
#define RX0IE       0

/* Bit definition of CANINTF */
#define MERRF       7
#define WAKIF       6
#define ERRIF       5
#define TX2IF       4
#define TX1IF       3
#define TX0IF       2
#define RX1IF       1
#define RX0IF       0

/* Bit definition of EFLG */
#define RX1OVR      7
#define RX0OVR      6
#define TXB0        5
#define TXEP        4
#define RXEP        3
#define TXWAR       2
#define RXWAR       1
#define EWARN       0

/* Bit definition of TXBnCTRL (n = 0, 1, 2) */
#define ABTF        6
#define MLOA        5
#define TXERR       4
#define TXREQ       3
#define TXP1        1
#define TXP0        0

/* Bit definition of RXB0CTRL */
#define RXM1        6
#define RXM0        5
#define RXRTR       3
#define BUKT        2
#define BUKT1       1
#define FILHIT0     0

/* Bit definition of TXBnSIDL (n = 0, 1) */
#define EXIDE       3

/*
 *  Bit definition of RXB1CTRL
 *  RXM1, RXM0, RXRTR und FILHIT0 sind schon fuer RXB0CTRL definiert
 */
#define FILHIT2     2
#define FILHIT1     1

/* Bit definition of RXBnSIDL (n = 0, 1) */
#define SRR         4
#define IDE         3

/*
 * Bit definition of RXBnDLC (n = 0, 1)
 * TXBnDLC
 */
#define RTR         6
#define DLC3        3
#define DLC2        2
#define DLC1        1
#define DLC0        0

typedef struct can_msg {
    uint32_t id;
    bool exid;
    uint32_t priority;
    uint8_t length;
    union {
        uint8_t data[8];
        uint16_t word[4];
        uint32_t dword[2];
    };
} can_msg_t;

typedef struct mcp_buffer {
    uint8_t ctrl;
    uint8_t sidh;
    uint8_t sidl;
    uint8_t eid8;
    uint8_t eid0;
    uint8_t dlc;
    uint8_t d[8];

} mcp_buffer_t;


#define MCP_BR_5KB_CNF     (1 << SJW1) | (1 << BRP4) | (1 << BRP3) | (1 << BRP2) | (1 << BRP1) | (1 << BRP0)
#define MCP_BR_10KB_CNF    (1 << SJW0) | (1 << BRP3) | (1 << BRP2) | (1 << BRP1) | (1 << BRP0)
#define MCP_BR_20KB_CNF    (1 << SJW1) | (1 << BRP5) | (1 << BRP2) | (1 << BRP1) | (1 << BRP0)
#define MCP_BR_50KB_CNF    (1 << BRP3) | (1 << BRP2) | (1 << BRP1) | (1 << BRP0)
#define MCP_BR_125KB_CNF   (1 << SJW1) | (1 << BRP4) | (1 << BRP3) | (1 << BRP2) | (1 << BRP1) | (1 << BRP0)
#define MCP_BR_250KB_CNF   (1 << SJW1) | (1 << SJW0) | (1 << BRP3) | (1 << BRP2) | (1 << BRP1) | (1 << BRP0)
#define MCP_BR_500KB_CNF   (1 << SJW1) | (1 << SJW0) | (1 << BRP5) | (1 << BRP2) | (1 << BRP1) | (1 << BRP0)
#define MCP_BR_1000KB_CNF  (1 << SJW1) | (1 << SJW0) | (1 << BRP5) | (1 << BRP4) | (1 << BRP1) | (1 << BRP0)

#define MCP_NORMAL_MODE     0x00    /* 000 = Set Normal Operation mode */
#define MCP_SLEEP_MODE      0x20    /* 001 = Set Sleep mode */
#define MCP_LOOPBACK_MODE   0x40    /* 010 = Set Loopback mode */
#define MCP_LISTEN_MODE     0x60    /* 011 = Set Listen-only mode */
#define MCP_CONF_MODE       0x80    /* 100 = Set Configuration mode */


void spi_init(void);
void spi_write_byte(uint8_t data);
uint8_t spi_read_byte(void);

void mcp_write_reg(uint8_t address, uint8_t data);
uint8_t mcp_read_reg(uint8_t address);
void mcp_modify_reg(uint8_t address, uint8_t mask, uint8_t data);
void mcp_reg_bits_up(uint8_t address, uint8_t bits);
void mcp_reg_bits_down(uint8_t address, uint8_t bits);

void mcp_reset(void);
void mcp_read_rx(mcp_buffer_t *buffer, uint8_t rx);
void mcp_load_tx(mcp_buffer_t *buffer, uint8_t tx);

uint8_t mcp_read_status(void);
uint8_t mcp_read_rx_status(void);

void mcp_set_mode(uint8_t mode);

void mcp_pack_msg(can_msg_t *msg, mcp_buffer_t *buffer);
void mcp_unpack_msg(mcp_buffer_t *buffer, can_msg_t *msg);

int8_t mcp_find_free_tx(void);

void mcp_set_tx(can_msg_t *msg, int8_t tx);
void mcp_rts(int8_t tx);

bool mcp_send_msg(can_msg_t *msg);

void mcp_init(uint8_t mode, uint8_t rate_cnf);

#if MCP_DEBUG
void mcp_print_buffer(mcp_buffer_t *buffer);
void mcp_print_reg(uint8_t address);
void mcp_print_rx(uint8_t rx);
void mcp_print_bit(uint8_t *descr, uint8_t byte, uint8_t bit);
void mcp_print_status(void);
void mcp_print_rx_status(void);
#endif

#endif

/* EOF */
