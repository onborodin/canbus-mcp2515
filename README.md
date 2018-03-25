

# Sample CAN bus MCP2515 driver

I wrote this code from scratch, based on documentation and some recomendation.

Central object is abstract CAN bus message

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

An auxiliary object is a structure that maps structure of the chip registers

    typedef struct mcp_buffer {
        uint8_t sidh;
        uint8_t sidl;
        uint8_t eid8;
        uint8_t eid0;
        uint8_t dlc;
        uint8_t d[8];
    } mcp_buffer_t;

Of course, is possible to write directly to registers for speed,
but the introduction mcp_buffer_t object makes debugging
more easier.

## Usage

### Initialize

    #include <mcp.h>

    int main (void) {
        ...
        spi_init();
        mcp_init(MCP_NORMAL_MODE, MCP_BR_125KB_CNF);
        ...
    }

You can use other mode, loopback or listen-only. See mcp.h

### Send message

    can_msg_t msg;

    msg.id = 0x1234;
    msg.exid = true;
    msg.data[0] = 0x12;
    msg.data[1] = 0x34;
    msg.length = 2;
    msg.priority = 0;

    mcp_send_msg(&msg);

### Read message with interrupt 

    ISR(INT0_vect) {
        uint8_t reg = mcp_read_reg(CANINTF);

        if (reg & (1 << RX1IF)) {
            mcp_buffer_t buffer;
            mcp_read_rx(&buffer, 0);
            ...
        }

        if (reg & (1 << RX0IF)) {
            mcp_buffer_t buffer;
            mcp_read_rx(&buffer, 0);
            ...
        }
    }

Or

    volatile can_msg_t msg;

    ISR(INT0_vect) {

        if (reg & (1 << RX1IF) || reg & (1 << RX0IF)) {
            mcp_read_msg(&msg);
            ...
        }
    }


### Read message with polling

    volatile can_msg_t msg;

    while (mcp_read_msg(&msg)) {
        ...
    }


### No guarantees. This code is given only as an example

I have not written management of ID filters & masks.
I'll write it if necessary.



