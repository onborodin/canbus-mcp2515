

# Sample CAN bus MCP2515 driver


I wrote this code from scratch, based on documentation.


## Usage

### Send message

    can_msg_t msg;

    msg.id = 0x12345,
    msg.exid = true,
    msg.data[0] = 0x12;
    msg.data[1] = 0x34;

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





