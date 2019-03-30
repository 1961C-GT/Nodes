
char large_buf[240];
char med_buf[120];

typedef struct various_msg{unsigned char x[5];} various_msg;


typedef struct com_stack
{
    uint8_t node_id : 4;     // ID of the node sending this com_stack
    uint8_t number_msgs : 4; // Number of messages contained inside this com_stack
    uint8_t msg_id_0 : 2;    // Message ID Type for msg 0 inside this com_stack
    uint8_t msg_id_1 : 2;    // Message ID Type for msg 1 inside this com_stack
    uint8_t msg_id_2 : 2;    // Message ID Type for msg 2 inside this com_stack
    uint8_t msg_id_3 : 2;    // Message ID Type for msg 3 inside this com_stack
    various_msg msg0;
    various_msg msg1;
    various_msg msg2;
    various_msg msg3;
} com_stack;

// #pragma pack(push, 1)
typedef struct range_msg {    // 5 Bytes
    uint8_t from : 4;         // Node ID of the originating node
    uint8_t to : 4;           // Node ID of the other node in the ranging
    uint8_t msg_seq : 8;      // Unique message number
    uint8_t hops : 3;         // Total allowed remaining hops for this message
    uint16_t range : 12;      // Raw range value
    uint16_t heading : 9;     // Raw heading value
} range_msg;
// #pragma pack(pop)

typedef struct stats_msg {    // 5 Bytes
    uint8_t from : 4;         // Node ID of the originating node
    uint8_t msg_seq : 8;      // Unique message number
    uint8_t hops : 3;         // Total allowed remaining hops for this message
    uint16_t bat : 8;         // Raw battery voltage value
    int pad : 17;             // Extra padding to make 5 byte aligned
} stats_msg;

typedef struct cmd_msg {      // 5 Bytes
    uint8_t msg_seq : 8;      // Unique message number
    uint8_t cmd_id : 5;       // Command ID
    uint8_t hops : 3;         // Total allowed remaining hops for this message
    uint32_t data : 24;       // Command data
} cmd_msg;

void hexDump(char *desc, void *addr, int len)
{
    int i;
    unsigned char buff[17];
    unsigned char *pc = (unsigned char *)addr;

    // Output description if given.
    if (desc != NULL){
        sprintf(med_buf, "%s:\n\r", desc);
        Serial.print(med_buf);
    }
    
    if (len == 0)
    {
        Serial.print("  ZERO LENGTH\n\r");
        return;
    }
    if (len < 0)
    {
        sprintf(med_buf, "  NEGATIVE LENGTH: %i\n\r", len);
        Serial.print(med_buf);
        return;
    }

    // Process every byte in the data.
    for (i = 0; i < len; i++)
    {
        // Multiple of 16 means new line (with line offset).

        if ((i % 16) == 0)
        {
            // Just don't print ASCII for the zeroth line.
            if (i != 0){
                sprintf(med_buf, "  %s\n\r", buff);
                Serial.print(med_buf);
            }

            // Output the offset.
            sprintf(med_buf, "  %04x ", i);
            Serial.print(med_buf);
        }

        // Now the hex code for the specific character.
        sprintf(med_buf, " %02x", pc[i]);
        Serial.print(med_buf);

        // And store a printable ASCII character for later.
        if ((pc[i] < 0x20) || (pc[i] > 0x7e))
            buff[i % 16] = '.';
        else
            buff[i % 16] = pc[i];
        buff[(i % 16) + 1] = '\0';
    }

    // Pad out last line if not exactly 16 characters.
    while ((i % 16) != 0)
    {
        Serial.print("   ");
        i++;
    }

    // And print the final ASCII bit.
    sprintf(med_buf, "  %s\n\r", buff);
    Serial.print(med_buf);
}


void setup()
{
    Serial.begin(9600);
}

void loop(){
    range_msg a0 = {.from = 1, .to = 2, .msg_seq = 12, .hops = 2, .range = 1201, .heading = 331};
    range_msg a1 = {.from = 1, .to = 3, .msg_seq = 13, .hops = 1, .range = 1405, .heading = 111};
    range_msg a2 = {.from = 1, .to = 4, .msg_seq = 14, .hops = 3, .range = 201, .heading = 12};
    range_msg a3 = {.from = 1, .to = 0, .msg_seq = 15, .hops = 0, .range = 445, .heading = 71};

    various_msg v0 = rangeMessageToVariousMessage(&a0);
    various_msg v1 = rangeMessageToVariousMessage(&a1);
    various_msg v2 = rangeMessageToVariousMessage(&a2);
    various_msg v3 = rangeMessageToVariousMessage(&a3);

    com_stack com = {.node_id = 4, .number_msgs = 4, .msg_id_0 = 0, .msg_id_1 = 0, .msg_id_2 = 0, .msg_id_3 = 0, .msg0 = v0, .msg1 = v1, .msg2 = v2, .msg3 = v3};
    uint8_t com_stack_arr[sizeof(com_stack)];
    // comStackToByteArray(com_stack_arr, &com);
    memcpy(&com_stack_arr, &com, sizeof(com_stack));
    // Serial.print("Com_Stack:\n\r0x");
    // for(int i = 0; i < sizeof(com_stack); i++){
    //     sprintf(med_buf, "%01x", com_stack_arr[i]);
    //     Serial.print(med_buf);
    // }
    // Serial.println("\n\r");

    hexDump("Com_Stack", &com_stack_arr, sizeof(com_stack));
    com_stack *com2 = (com_stack *)com_stack_arr;
    various_msg vm1 = com2->msg1;
    range_msg rm1;
    memcpy(&rm1, &vm1, sizeof(range_msg));
    printRangeFrame(&rm1);

    // sprintf(large_buf, "0x%010X", a); Serial.println(large_buf);
    // hexDump("Range Message", &a0, sizeof(a0));
    // printRangeFrame(&a0);
    // Serial.println("Converted...");
    // uint8_t arr[sizeof(a0)];
    // memcpy(arr, &a0, sizeof(a0));
    // Serial.print("0x");
    // for (int i = 0; i < sizeof(a0); i++)
    // {
    //     sprintf(med_buf, "%01x", arr[i]);
    //     Serial.print(med_buf);
    // }
    // range_msg* b = (range_msg*)arr;
    // printRangeFrame(b);
    Serial.println();
    delay(1000);
}

various_msg rangeMessageToVariousMessage(range_msg * rm){
    various_msg output;
    memcpy(&output, rm, sizeof(range_msg));
    return output;
}

// void comStackToByteArray(uint8_t *arr, com_stack * com){
//     memcpy(arr, com, sizeof(com_stack));
// }

void printRangeFrame(range_msg *rm)
{
    Serial.print("From    -> "); Serial.println(rm->from);
    Serial.print("To      -> "); Serial.println(rm->to);
    Serial.print("Msg Seq -> "); Serial.println(rm->msg_seq);
    Serial.print("Hops    -> "); Serial.println(rm->hops);
    Serial.print("Range   -> "); Serial.println(rm->range);
    Serial.print("Heading -> "); Serial.println(rm->heading);
    Serial.println();
}

// 0x21c8a254b1
// to   from seq
// 2--- 1--- 12--
// 0010 0001 1100 1000 1010 0010 0101 0100 1011 0001