#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif  // __arm__

#ifdef MNSLAC_NODE_M0
  float getBattVoltage() { return analogRead(BATT) * BATT_MEAS_COEFF; }
  uint16_t getBattVoltageBits() { return analogRead(BATT); }
#else
  float getBattVoltage() { return 0.0f; }
#endif

int freeMemory() {
  char top;
#ifdef __arm__
  return &top - reinterpret_cast<char*>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
  return &top - __brkval;
#else  // __arm__
  return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif  // __arm__
}



// Print a message struct to the console
void printMessage(Message msg) {
  inc();
  pcln("Message Contents");
  sprintf(medium_buf, "| → %-8s%d", "Valid: ", msg.valid); pcln(medium_buf);
  sprintf(medium_buf, "| → %-8s%d", "From: ", msg.from); pcln(medium_buf);
  sprintf(medium_buf, "| → %-8s%d", "Seq: ", msg.seq); pcln(medium_buf);
  sprintf(medium_buf, "| → %-8s%s", "Type: ", MsgTypes[msg.type]); pcln(medium_buf);
  sprintf(medium_buf, "| → %-8s0x%.*X", "Data: ", msg.len, msg.data); pcln(medium_buf);
  sprintf(medium_buf, "| → %-8s%d", "Mem: ", freeMemory()); pcln(medium_buf);
  dec();
}

void receiver() {
  	DW1000.newReceive();
  	DW1000.setDefaults();
  	// so we don't need to restart the receiver manually
  	DW1000.receivePermanently(true);
  	DW1000.startReceive();
}

// Get a message from the DW1000 and return its message format
Message getMessage() {

  // Create a data buffer
  byte data[MAC_LEN + DATA_LEN];

  // Populate the main data buffer
  DW1000.getData(data, MAC_LEN + DATA_LEN);

  // Get the length of this message
  uint8_t dataLength = DW1000.getDataLength();

  return getMessage(data, dataLength);
}

// Get a message struct from the given byte array with the given length
Message getMessage(byte data[], uint8_t dataLength) {

  // Create Message Struct
  Message msg;

  // Check if the message is the correct min length. Also confirm that the
  // Check Byte (Byte 0) is 0x9B (unique to our project)
  if (dataLength >= 3 && data[0] == 0x9B) {

    // Get our From bits
    msg.from = data[1] >> 4;

    // Get our Type bits
    msg.type = (Msg_Type) (data[2] & 0x0F);

    // Get our Seq bits
    msg.seq = (data[1] << 4) | (data[2] >> 4);

    // Create a pointer for data (for easy manipulation)
    byte *ptr = (byte *)data;

    // Copy the payload portion of data into msg.data. Notice that this starts
    // at data[MAC_LEN] and steps for dataLength-MAC_LEN bytes. This can be
    // zero bytes if no payload is recieved
    memcpy(msg.data, ptr + MAC_LEN, dataLength - MAC_LEN);

    // Store the length of the payload in msg
    msg.len = dataLength - MAC_LEN;

    // Set this message to valid
    msg.valid = 1;
  }
  return msg;
}


// Returns true if we can send this sequence number (and thus
// we should store it until our next bcast)
boolean processSequenceNumber(uint8_t from, uint8_t seq) {

  if (from > LEN_NODES_LIST) {
    pcln("Invalid From Address Detected", C_RED);
    return false;
  }

  if (seq > 31) {
    pcln("Invalid Sequance Detected", C_RED);
    return false;
  }

  boolean ret = (packet_sendable[from] >> seq) & 0x1;


  if (ret) {

    // If this sequence is sendable, then we should assume it will
    // be sent. Mark this sequence as unsendable to prevent us from
    // receiving any additional copies
    // packet_sendable[from] = packet_sendable[from] | (0x1 << seq);
    packet_sendable[from] = packet_sendable[from] & ~(0x1 << seq); // Set to false
    // number &= ~(1UL << n);

    // Clear (set to true) the opposite half of the packet sendable list to allow
    // new packets from this node to come in under different
    // sequence numbers

    // 0b0000000////////////////000000001 idx 0
    // 0b00000000000000000000000000000001
    // 0b00000001111111111111111000000000

    // 0b////////////////0000000010000000 idx 7
    // 0b00000000000000000000000010000000
    // 0b11111111111111110000000000000000
    if (seq <= 7) {
      packet_sendable[from] = packet_sendable[from] | (0xFFFF0000 >> (7 - seq));
    }

    // 0b///////////////0000000010000000/ idx 8
    // 0b00000000000000000000000100000000
    // 0b11111111111111100000000000000001

    // 0b/0000000010000000/////////////// idx 22
    // 0b00000000010000000000000000000000
    // 0b10000000000000000111111111111111
    else if (seq <= 22) {
      packet_sendable[from] = packet_sendable[from] | (0x00007FFF >> (22 - seq));
      packet_sendable[from] = packet_sendable[from] | (0xFFFE0000 << (seq - 8));
    }


    // 0b0000000010000000//////////////// idx 23
    // 0b00000000100000000000000000000000
    // 0b00000000000000001111111111111111

    // 0b10000000////////////////00000000 idx 31
    // 0b10000000000000000000000000000000
    // 0b00000000111111111111111100000000
    else {
      packet_sendable[from] = packet_sendable[from] | (0x0000FFFF << (seq - 23));
    }
  }

  return ret;
}


void processMessage(Message msg){
  if (msg.valid && msg.len == sizeof(com_stack)) {
    com_stack *msg_com_stack = (com_stack *)msg.data;
    pcln("Decoded COM Stack", C_ORANGE);
    sprintf(medium_buf, "| → node_id: %u", msg_com_stack -> node_id); pcln(medium_buf);
    sprintf(medium_buf, "| → number_msgs: %u", msg_com_stack -> number_msgs); pcln(medium_buf);

    for (uint8_t i = 0; i < msg_com_stack->number_msgs; i++){
      if (msg_com_stack->msg_id[i] == RANGE_PACKET){ // Range Message
        range_msg * rm = (range_msg *)&msg_com_stack->msg[i];

        if ((rm->hops > 0 || isBase) && rm->from != nodeNumber && processSequenceNumber(rm->from, rm->seq)) {

          rm->hops --;

          if (!putPacketInBuffer((various_msg *)rm, RANGE_PACKET)) {
            pcln("Packet Buffer Full. Overwrote Message", C_RED);
          }

          sprintf(medium_buf, "Processed Range Packet: %u", packet_sendable[rm->from]); pcln(medium_buf);
          sprintf(medium_buf, "| → From:  %u", rm -> from); pcln(medium_buf);
          sprintf(medium_buf, "| → To:    %u", rm -> to); pcln(medium_buf);
          sprintf(medium_buf, "| → Seq:   %u", rm -> seq); pcln(medium_buf);
          sprintf(medium_buf, "| → Hops:  %u", rm -> hops); pcln(medium_buf);
          sprintf(medium_buf, "| → Range: %u", rm -> range); pcln(medium_buf);

        } else if (isBase){
          Serial5.print("Discarded Range Packet due to Seq (Host: "); Serial5.print(msg.from);
          Serial5.print(", Seq: "); Serial5.print(rm->seq);
          Serial5.print(", From: "); Serial5.print(rm->from);
          Serial5.print(", To: "); Serial5.print(rm->to);
          Serial5.print(", Hops: "); Serial5.print(rm->hops);
          Serial5.print(", Sendable: "); Serial5.print(packet_sendable[rm->from]);
          Serial5.println(")");
        } else {
          pcln("Range Packet Failed Seq or Hops", C_ORANGE);
          sprintf(medium_buf, "| → From:  %u", rm -> from); pcln(medium_buf);
          sprintf(medium_buf, "| → To:    %u", rm -> to); pcln(medium_buf);
          sprintf(medium_buf, "| → Seq:   %u", rm -> seq); pcln(medium_buf);
          sprintf(medium_buf, "| → Hops:  %u", rm -> hops); pcln(medium_buf);
          sprintf(medium_buf, "| → Range: %u", rm -> range); pcln(medium_buf);
        }

        // memcpy(&range_array[range_array_pointer], rm, sizeof(range_msg));
        // packet_life

      } else if (msg_com_stack->msg_id[i] == STATUS_PACKET){
        stats_msg * sm = (stats_msg *)&msg_com_stack->msg[i];

        if ((sm->hops > 0 || isBase) && sm->from != nodeNumber && processSequenceNumber(sm->from, sm->seq)) {
          sm->hops --;

          if (!putPacketInBuffer((various_msg *)sm, STATUS_PACKET)) {
            pcln("Packet Buffer Full. Overwrote Message", C_RED);
          }

          sprintf(medium_buf, "Processed Stats Packet: %u", packet_sendable[sm->from]); pcln(medium_buf);
          // sprintf(medium_buf, "| → From:  %u", sm -> from); pcln(medium_buf);
          // sprintf(medium_buf, "| → Seq:   %u", sm -> seq); pcln(medium_buf);
          // sprintf(medium_buf, "| → Hops:  %u", sm -> hops); pcln(medium_buf);
          // sprintf(medium_buf, "| → Bat: %u", sm -> bat); pcln(medium_buf);
          // sprintf(medium_buf, "| → Heading: %u", sm -> heading); pcln(medium_buf);
          // sprintf(medium_buf, "| → Temp: %u", sm -> temp); pcln(medium_buf);

        }

        else if (isBase){
          Serial5.print("Discarded Stats Packet due to Seq (Host: "); Serial5.print(msg.from); Serial5.println(")");
        //   Serial5.print(", Seq: "); Serial5.print(sm->seq);
        //   Serial5.print(", From: "); Serial5.print(sm->from);
        //   Serial5.print(", Hops: "); Serial5.print(sm->hops);
        //   Serial5.print(", Bat: "); Serial5.print(sm->bat);
        //   Serial5.print(", Heading: "); Serial5.print(sm->heading);
        //   Serial5.print(", Temp: "); Serial5.print(sm->temp);
        //   Serial5.print(", Sendable: "); Serial5.print(packet_sendable[sm->from]);
        //   Serial5.println(")");
        } else {
          sprintf(medium_buf, "Stats Packet Failed Seq or Hops: %u", packet_sendable[sm->from]); pcln(medium_buf);
        }

      } else if (msg_com_stack->msg_id[i] == CMD_PACKET) {

        // The base station does not relay or process command packets. Continue to
        // the next packet
        if (isBase)
          continue;

        // Cast our data to a command packet type
        cmd_msg * cm = (cmd_msg *)&msg_com_stack->msg[i];

        // Ensure that the packet is good (note that they are always from
        // the base station [node 0])
        if (processSequenceNumber(0, cm->seq)) {

          pcln("Command Packet Received", C_ORANGE);
          sprintf(medium_buf, "| → Seq:    %u", cm -> seq); pcln(medium_buf);
          sprintf(medium_buf, "| → Hops:   %u", cm -> hops); pcln(medium_buf);
          sprintf(medium_buf, "| → Cmd_ID: %u", cm -> cmd_id); pcln(medium_buf);

          // Handle this command
          switch (cm->cmd_id) {
            case COM_TRANSMIT_AUTH:
              {
                sprintf(medium_buf, "| → Transmit Authorization Accepted"); pcln(medium_buf, C_GREEN);

                // If our transmit authorization was zero, then we should clear the
                // packet buffer before moving on (so that we don't send really old
                // messages)
                if (transmitAuthorization == 0) {
                  // Clear the packet buffer
                  clearPacketBuffer();
                }

                uint32_t dateData = cm->data;

                uint8_t s = (uint8_t) (cm->data);
                uint8_t m = (uint8_t) (cm->data >> 8);
                uint8_t h = (uint8_t) (cm->data >> 16);

                updateTime(h,m,s);

                min = m; sec = s; hour = h;

                sprintf(medium_buf, "| → Got Time: %02d:%02d:%02d", h, m, s); pcln(medium_buf, C_GREEN);

                validTime = true;

                // Up our transmit authorization and cycle valid status
                transmitAuthorization = TRANSMIT_AUTH_CAP;
                cycleValid = CYCLE_VALID_CAP;
              }
              break;
            case SOFT_RESET:
              sprintf(medium_buf, "| → Soft Reset", cm -> cmd_id); pcln(medium_buf, D_BLINK_C_CYAN);
              softReset = true;
              break;
            case SLEEP:
              if (sleepCounter == -1) {

                uint16_t seconds = (uint16_t) cm->data;

                uint8_t min = seconds / 60;

                sleepTime = (uint16_t) cm->data;
                sleepCounter = SLEEP_CYCLE_DELAY;

                sprintf(medium_buf, "| → Got Sleep Command (%d seconds from now)", sleepTime); pcln(medium_buf, D_BLINK_C_CYAN);
              }
              break;
            default:
              sprintf(medium_buf, "Unknown Command Id: %u", cm->cmd_id); pcln(medium_buf, C_RED);
              break;
          }

          // Only add this command to our buffer if it has at least one hop left
          if (cm->hops > 0) {
            // Age the packet by one hop
            cm->hops --;
            if (!putPacketInBuffer((various_msg *)cm, CMD_PACKET)) {
              pcln("Packet Buffer Full. Overwrote Message", C_RED);
            }
          }
        }
      }else{
        sprintf(medium_buf, "Unknown Packet Type: %u", msg_com_stack->msg_id[i]); pcln(medium_buf, C_RED);
      }
    }
  }
}

void queueReset() {
  cmd_msg rstcmd = {.seq = getMsgSeq(), .cmd_id = SOFT_RESET, .hops = ALLOWABLE_HOPS, .data = 0};
  memcpy(&cmd_buffer[cmd_buffer_len], (various_msg *)&rstcmd, sizeof(various_msg));
  cmd_buffer_len ++;
  softReset = true;
}

void queueSleep(uint16_t delay) {
  cmd_msg rstcmd = {.seq = getMsgSeq(), .cmd_id = SLEEP, .hops = ALLOWABLE_HOPS, .data = delay};
  memcpy(&cmd_buffer[cmd_buffer_len], (various_msg *)&rstcmd, sizeof(various_msg));
  cmd_buffer_len ++;
}

void checkSleep() {

  if (sleepCounter > 0) {
    sprintf(medium_buf, "Sleeping in %d cycle(s) (%d seconds)", sleepCounter, sleepTime); pcln(medium_buf, C_ORANGE);
    sleepCounter --;
  } else if (sleepCounter == 0) {

    // Constrain the sleepTime
    if (sleepTime > MAX_SLEEP_TIME) {
      sleepTime = MAX_SLEEP_TIME;
    } else if (sleepTime < MIN_SLEEP_TIME) {
      sleepTime = MIN_SLEEP_TIME;
    }

    // Grab the current epoch time
    uint32_t epoch = rtc.getEpoch();

    // Add our sleep time
    epoch += sleepTime;

    sprintf(medium_buf, "Sleeping for %d seconds (%d)", sleepTime, epoch); pcln(medium_buf, C_ORANGE);

    // shut down the main watchdog timer
    disableWatchdog();

    // Set the alarm to the epoch time
    rtc.setAlarmEpoch(epoch);

    // Enable the alarm matched to hour, min, second
    rtc.enableAlarm(rtc.MATCH_HHMMSS);

    // Go into standby mode
    standby();

    // Reset the system
    reset();

    // // Start the main watchdog timer back up;
    // enableWatchdog();
    //
    // // Update the RTC time after the sleep
    // updateRTC();
    //
    // // Update flags
    // transmitAuthorization = 0;
    // cycleValid = 0;
    // cycle_counter = 0;
    //
    // sprintf(medium_buf, "Sleep Compleate (%02d:%02d:%02d)", hour, min, sec); pcln(medium_buf, C_ORANGE);
    //
    // // Reset our cleep counter to disable sleep next time
    // sleepCounter = -1;
  }
}

// Disable the main protection watchdog timer
void disableWatchdog() {
  rtc.disableAlarm();
  rtc.detachInterrupt();
}

// Enable the main protection watchdog timer
void enableWatchdog() {

  // Attach the watchdogCallback function
  rtc.attachInterrupt(watchdogCallback);

  // Grab the current epoch time
  uint32_t epoch = rtc.getEpoch();

  // Add our sleep time
  epoch += WATCHDOG_INTERVAL;

  // Set the alarm to the epoch time
  rtc.setAlarmEpoch(epoch);

  // Enable the alarm matched to hour, min, second
  rtc.enableAlarm(rtc.MATCH_SS);
}

void updateTime(uint8_t h, uint8_t m, uint8_t s) {

  set_hour = h;
  set_min = m;
  set_sec = s;
  clockUpdateReceived = true;
}

// Main protection watchdog callback function
void watchdogCallback() {
  // If the watchdog value has not changed, then it is time to reset
  if (__watchdog_last ==__watchdog_comp) {
    reset();
  } else {

    // Record what the watchdog value was this time to check against next time
    __watchdog_last = __watchdog_comp;

    // Update the time if needed
    if (clockUpdateReceived) {
        clockUpdateReceived = false;

        // Turn off the watchdog timer
        rtc.disableAlarm();

        // Set the new time
        rtc.setSeconds(set_sec);
        rtc.setMinutes(set_min);
        rtc.setHours(set_hour);
    }

    // Set the alarm to the epoch time plus the watchdog interval
    rtc.setAlarmEpoch(rtc.getEpoch() + WATCHDOG_INTERVAL);

    // Enable the alarm matched to hour, min, second
    rtc.enableAlarm(rtc.MATCH_SS);
  }
}

// Hold the watchdog off for another alarm rotation
void holdWatchdog() {
  // Only change the watchdog value if it hasnt changed since the last time
  // the watchdog was fired
  if (__watchdog_comp == __watchdog_last)
    __watchdog_comp = !__watchdog_last;
}

void standby() {

  // Stop all of the timers
  stopTimers();

  // Put the DW1000 to sleep
  DW1000.deepSleep();

  // Turn all of the LEDs off
  setLed(LED_AUX, MODE_OFF);
  setLed(LED_RED, MODE_OFF);
  setLed(LED_GREEN, MODE_OFF);
  setLed(LED_BLUE, MODE_OFF);

  // Wait for the DW1000 to fully enter sleep mode
  delay(10);

  // Enter standby mode
  rtc.standbyMode();

  // Start the DW1000 back up
  DW1000.spiWakeup();

  // Wait for the DW1000 to boot up
  delay(10);

  // TODO Reset here?
}

void checkReset() {
  // If our reset flag is high, then force the system to reset
  if (softReset) {
    reset();
  }
}

void reset() {
  stopTimers();
  disableWatchdog();
  // setup();
  NVIC_SystemReset();
}

void printDate() {
  Serial.print(hour);
  Serial.print(":");
  Serial.print(min);
  Serial.print(":");
  Serial.println(sec);
}

// typedef struct cmd_msg
// {                      // 5 Bytes
//   uint8_t seq : 5; // Unique message number
//   uint8_t cmd_id : 5;  // Command ID
//   uint8_t hops : 3;    // Total allowed remaining hops for this message
//   uint32_t data : 27;  // Command data
// } cmd_msg;

void loadPackets(Message * msg)
{
  uint8_t num_messages = 0;
  uint8_t ids[MSGS_PER_COM_STACK];
  various_msg v_packets[MSGS_PER_COM_STACK];

  if (isBase) {
    if (cmd_buffer_len == 0) {
      msg->len = 0;
      return;
    }

    msg->len = (uint8_t)sizeof(com_stack);
    for (uint8_t i = 0; i < MSGS_PER_COM_STACK; i++){
      if (cmd_buffer_len == 0) {
        break;
      } else {
        cmd_buffer_len --;
        memcpy(&v_packets[i], &cmd_buffer[cmd_buffer_len], sizeof(various_msg));
        ids[i] = CMD_PACKET;
        num_messages ++;
      }
    }
  } else {
    if (getBufferLength() == 0) {
      msg->len = 0;
      return;
    }


    msg->len = (uint8_t)sizeof(com_stack);
    for (uint8_t i = 0; i < MSGS_PER_COM_STACK; i++){
      uint8_t id;
      if (getPacketFromBuffer(&v_packets[i], &ids[i])) {
        num_messages ++;
      } else {
        break;
      }
      // ids[i] = RANGE_PACKET;
      // memcpy(&v_packets[i], &range_array[range_array_pointer], sizeof(various_msg));
    }
  }

  com_stack msg_com_stack = {.node_id = nodeNumber, .number_msgs = num_messages};
  memcpy(&msg_com_stack.msg_id, &ids, MSGS_PER_COM_STACK);
  memcpy(&msg_com_stack.msg, &v_packets, sizeof(various_msg)*MSGS_PER_COM_STACK);
  memcpy(&msg->data, &msg_com_stack, sizeof(com_stack));
  sprintf(medium_buf, "Loaded %u packets into message", num_messages); pcln(medium_buf, C_BLUE);
}
// uint8_t range_array_pointer = 0;
// range_msg range_array[MAX_RANGE_MESSAGES];

// Get the length of a message object
int getMessageLength(Message msg) {
  return MAC_LEN + msg.len;
}

// Create a message from the given message struct and place the result into
// the given byte buffer
void createMessage(byte data[], Message msg) {

  // Set the Check Byte
  data[0] = 0x9B;

  // Set byte 1
  data[1] = (msg.from << 4) | (msg.seq >> 4);

  // Set byte 2
  data[2] = (msg.seq << 4) | msg.type;

  // Create a pointer for data
  byte *ptr = (byte *)data;

  // Copy the data from msg.data to the byte array starting at its third
  // byte
  memcpy(ptr + 3, msg.data, msg.len);
}

void getRangeRecDelay(DW1000Time& t, uint8_t nodeNum, Settings s) {
  if (nodeNum == 0) {
    t.setTime((int32_t) 0);
  } else {
    t.setTime((int32_t) (s.t_rx + s.t_r * nodeNum));
  }
}

uint32_t getRangeRecDelay(uint8_t nodeNum, Settings s) {
  if (nodeNum == 0) {
    return 0;
  } else {
    return (int32_t) (s.t_rx + s.t_r * nodeNum);
  }
}

// uint8_t buffer_start = 0;
// uint8_t buffer_num = 0;
// various_msg packet_buffer[MAX_RANGE_MESSAGES];
// uint8_t packet_buffer_IDs[MAX_RANGE_MESSAGES];

uint8_t getBufferLength() {
  return buffer_num;
}

void clearPacketBuffer() {
  buffer_num = 0;
  buffer_start = 0;
}

boolean getPacketFromBuffer(various_msg * packet, uint8_t * id)
{
  if (buffer_num == 0)
    return false;

  // packet = &packet_buffer[buffer_start];
  memcpy(packet, &packet_buffer[buffer_start], sizeof(various_msg));
  *id = packet_buffer_IDs[buffer_start];

  buffer_start++;
  if (buffer_start >= MAX_RANGE_MESSAGES)
    buffer_start = 0;

  buffer_num--;

  return true;
}

boolean putPacketInBuffer(various_msg * packet, uint8_t id)
{
  boolean ret = true;

  if (buffer_num == MAX_RANGE_MESSAGES) {
    ret = false;
    buffer_start ++;
    buffer_num --;
  }

  uint8_t idx = buffer_start + buffer_num;

  if (idx >= MAX_RANGE_MESSAGES)
  {
    idx -= MAX_RANGE_MESSAGES;
  }

  memcpy(&packet_buffer[idx], packet, sizeof(various_msg));
  packet_buffer_IDs[idx] = id;

  buffer_num ++;

  return ret;
}


void printSettings(Settings s) {
  section("System Settings", C_ORANGE);
  sprintf(medium_buf, "%-8s%d", "n:", s.n); pcln(medium_buf, C_RED);
  sprintf(medium_buf, "%-8s%d", "t_rx:", s.t_rx); pcln(medium_buf, C_RED);
  sprintf(medium_buf, "%-8s%d", "t_s:", s.t_s); pcln(medium_buf, C_RED);
  sprintf(medium_buf, "%-8s%d", "t_fs:", s.t_fs); pcln(medium_buf, C_RED);
  sprintf(medium_buf, "%-8s%d", "t_c:", s.t_c); pcln(medium_buf, C_RED);
  sprintf(medium_buf, "%-8s%d", "n_com:", s.n_com); pcln(medium_buf, C_RED);
  sprintf(medium_buf, "%-8s%d", "t_b:", s.t_b); pcln(medium_buf, C_RED);
  sprintf(medium_buf, "%-8s%d", "t_r:", s.t_r); pcln(medium_buf, C_RED);
  sprintf(medium_buf, "%-8s%d", "t_cl:", s.t_cl); pcln(medium_buf, C_RED);
  sprintf(medium_buf, "%-8s%d", "t_br:", s.t_br); pcln(medium_buf, C_RED);
  sprintf(medium_buf, "%-8s%d", "t_fr:", s.t_fr); pcln(medium_buf, C_RED);
  sprintf(medium_buf, "%-8s%d", "t_bc:", s.t_bc); pcln(medium_buf, C_RED);
  sprintf(medium_buf, "%-8s%d", "t_fc:", s.t_fc); pcln(medium_buf, C_RED);
  sprintf(medium_buf, "%-8s%d", "t_rn:", s.t_rn); pcln(medium_buf, C_RED);
  sprintf(medium_buf, "%-8s%d", "bits_c:", s.bits_c); pcln(medium_buf, C_RED);
  endSection();
  pcln("");
}

float runningTotal;
float computeRange(DW1000Time& rec, uint8_t fromId) {


  // The total time between our poll message being sent and the reply being
  // received.
  DW1000Time total = (rec - timePollSent).wrap();

  // The time that the distant node was suppsed to wait to reply based on their
  // Node Id
  DW1000Time turn = DW1000Time(settings.t_r +  (settings.t_r * fromId), DW1000Time::MICROSECONDS);

  DW1000Time tof = (total - turn) / 2;

  float distance = tof.getAsMeters();

  // Serial.print("Distance Data: Total("); Serial.print(total.getAsNanoSeconds());
  // Serial.print("NodeID: "); Serial.print(fromId);
  // Serial.print("\tTurn("); Serial.print(turn.getAsNanoSeconds());
  // Serial.print("), TOF("); Serial.print(tof.getAsNanoSeconds());
  // Serial.print("); Sent:"); Serial.print(timePollSent.getAsMicroSeconds());
  // Serial.print(", Received:"); Serial.print(rec.getAsMicroSeconds());
  // Serial.print(", Dist:"); Serial.println(distance);

  return distance;
}

// TODO Finish adjust clock
boolean adjustClock(Message msg, uint32_t m0rx, float dwrx, boolean confirmCycle = true) {

  // Figure out what frame we are in
  Frame curFrame = getFrameFromSeq(msg.seq, settings);
  // Figure out if this is the start of this block
  boolean blockStart = isBlockStart(curFrame, msg.seq, settings);

  // If this message is the start of a block,
  // the we adjust our clocks
  if (blockStart) {

    // Reset our DWOffset based on the message we got. Usage of this offset value
    // us explained at its def.
    DWOffset = m0rx - dwrx;

    // From the message, get the exact micros at which that message should have
    // been received
    uint32_t curCycleTime = getTimeAtSeq(msg.seq);

    // At the exact moment that the message was recieved, the cycle was at cycle
    // time curCycleTime. Therefore the start of the cycle occured curCycleTime
    // micros before the message was received in M0 time.
    cycleStart = m0rx - curCycleTime;

    // Because we have just confirmed our cycle, set cycleValid to true
    if (confirmCycle)
      cycleValid = CYCLE_VALID_CAP;

    // Return true to denote a clock adjustment
    return true;
  }

  // Return false to denote no clock adjustment
  return false;
}

uint32_t getTimeAtSeq(uint8_t seq) {
  Settings s = settings;

  // Start with the sleep frame time (which is always at the start of the cycle)
  uint32_t ret = s.t_fs;

  // Find the very first sequence number of the COM frame
  uint8_t comFirstSeq = s.n * (s.n + 1);

  // See if we are before the first com seq number
  uint8_t isRangeFrame = seq < comFirstSeq;

  if (isRangeFrame) {
    // Find what block we are in within this frame. This relies on interger
    // truncation as a "floor" operation.
    uint8_t blockNum = seq / (s.n + 1);

    // Find what seq we are within this block.
    uint8_t seqIdx = (seq % (s.n + 1));

    // For each block that we are already past, we add one block length.
    // Remember that the block number starts from zero. This means that if we
    // are in the 0th block, we should add no additional time. If we are the
    // 1st block, then we need only add the time of the 0th block.
    ret += (s.t_br * blockNum);

    // Now within our block, we add the time to get to the given sequence id.
    // Just as with the block numbers, the sequence index starts at zero. If
    // the 0th message comes through, then we only add in the rx time. If the
    // 1st message comes through, we add the rx time plus one range division
    // (t_r)
    ret += (s.t_r * seqIdx) + s.t_rx;
  }
  // If we are not in the range frame, then we are in the com frame
  else {
    // Because we are past the ENTIRE range frame, we can simply add the time
    // to compleate the whole thing now
    ret += s.t_fr;

    // Using the value we calculated earlier for the first seq number of the
    // com frame, we can see how many sequances we are into this frame. As with
    // the Range block number, this block number starts from 0
    uint8_t blockNum = seq - comFirstSeq;

    // Add the time for all the blocks in the COM frame before this one.
    // Because the block number starts at 0, we need only do a simple
    // multiplication to achieve this. Finally we add a single t_rx to account
    // for the fact that each com message is t_rx deep into its block
    ret += (s.t_bc * blockNum) + s.t_rx;
  }
  return ret;
}

// Get the block that the given Node Id should be in, given that the current
// message has the given sequence number.
Block getBlockFromSeq(uint8_t seq, uint8_t nodeId, Settings s) {
  // Figure out what frame we are in
  Frame curFrame = getFrameFromSeq(seq, s);

  // If we are in the range frame, then use the following calculation
  if (curFrame == FRAME_RANGE) {
    // Figure out which block within the range frame we are in
    uint8_t blockNum = seq / (s.n + 1);

    // If our nodeid matches the block number, then it is our turn to be the
    // requester. Otherwise, we should be an accepter.
    if (nodeId == blockNum) {
      return BLOCK_RANGE_REQUEST;
    } else {
      return BLOCK_RANGE_ACCEPT;
    }
  }

  // if we are in the com frame, then we are always in a com block
  else if (curFrame == FRAME_COMS) {
    return BLOCK_COM;
  }

  // if we are in the com frame, then we are always in a com block
  else if (curFrame == FRAME_SLEEP) {
    return BLOCK_SLEEP;
  }

  // If not any others, then assume we are in the sleep block
  else {
    return BLOCK_NONE;
  }
}

// Gets a message's index within a single block
uint8_t getMsgIdx(Frame frame, uint8_t seq, Settings s) {
  // When we are in the range frame, the following formula will result in the
  // message index
  if (frame == FRAME_RANGE) {
    return (seq % (s.n + 1));
  } else {
    return 0;
  }
}

// Return whether or not the given message sequance number is the start of its
// block
boolean isBlockStart(Frame frame, uint8_t seq, Settings s) {
  return getMsgIdx(frame, seq, s) == 0;
}

// Return whether or not the given message sequance number is the end of its
// block
boolean isBlockEnd(Frame frame, uint8_t seq, Settings s) {
  if (frame == FRAME_RANGE) {
    return getMsgIdx(frame, seq, s) == s.n;
  } else {
    return true;
  }
}

// LEFT OFF:
// Setting up get state to work. This will go in the decode states in order to
// help figure out whether or not we need to move to a different state based on
// the message we got. Also might help with moving us between blocks and frames

// Gets state we should move to based on the given sequence number and message
// type. This function will return the state to move to next.
// To be execute if we do not receive the expected message
state_fn * getState(Message msg) {
  Settings s = settings;
  state_fn * nextState;

  // See what we should be doing next sequence
  Frame curFrame = getFrameFromSeq(msg.seq + 1, s);
  Block curBlock = getBlockFromSeq(msg.seq + 1, nodeNumber, s);

  // If this message index is 0, then it is the first in this block
  boolean blockStart = isBlockStart(curFrame, msg.seq + 1, s);

  // If this messsage index is the number of nodes in the network, then it is
  // the last message in this block
  boolean blockEnd = isBlockEnd(curFrame, msg.seq + 1, s);

  if (curFrame == FRAME_RANGE) {

    // Check if we should be in the range accept block
    if (curBlock == BLOCK_RANGE_ACCEPT) {
      switch (msg.type) {
        // If we just got a range request, jump right to the ra_resp state
        case RANGE_REQ:
          nextState = { ra_resp };
          break;
        // For all other message types (should only be RANGE_RESP), simply
        // enter the ra_init state
        default:
          nextState = { ra_init };
          break;
      }
    }

    // If we are suposed to be in the range request block, the move to rb_range
    else if (curBlock == BLOCK_RANGE_REQUEST) {
      nextState = { rb_range };
    }

    else {
      nextState = { ra_init };
    }
  } else if (curFrame == FRAME_COMS) {
    nextState = { sleep };
  } else {
    nextState = { sleep };
  }

  return nextState;
}

// Get the frame the given sequence number falls in, given the current settings
Frame getFrameFromSeq(uint8_t seq, Settings s) {
  // There are n * (n+1) messages required for a full ranging frame. Given that
  // the sequence numbers start from 0, we can get away with using a < operator
  // for this alone (Eg for n=5, the sequences in the Range frame would number
  // from 0 to 29 (accounting for one "blank" seq for the Request node). If our
  // current sequence is 29, that is less than 5*6. If our seq is 30, that is
  // equal to 5*6 and would therefore fail this if statement)
  if (seq < (s.n * (s.n + 1)))
    return FRAME_RANGE;
  else if (seq < (s.n * (s.n + 1) + s.n * s.n_com))
    return FRAME_COMS;
  else
    return FRAME_SLEEP;
}


void stopTimers() {
  M0Timer.stop(_BLOCK_TIMER);
  M0Timer.stop(_FRAME_TIMER);
  if (M0Timer.getFired(_FRAME_TIMER)) {
    pcln("Frame Timer Stopped Before Handle", C_RED);
  }
  if (M0Timer.getFired(_BLOCK_TIMER)) {
    pcln("Block Timer Stopped Before Handle", C_RED);
  }

  blockTimerSet = false;
  frameTimerSet = false;
}

// Get the time from now until the event at cycleDelay micros from cycleStart
// will occur.
long getDelayMicros(uint32_t cycleDelay, uint32_t now) {
  long delay = (long) (cycleStart - now + cycleDelay);
  return delay;
}

// Set the frame timer based on the cycle delay and the offset. The cycle delay
// should be the number of micros between the start of the cycle and when the
// timer should fire. The offset should be the current time
boolean setFrameTimer(uint32_t cycleDelay, uint32_t now, boolean wrapTime = false) {
  // inc();
  // Set the delay based on the cycle start time, the offset (now time), and
  // the callback delay (referenced to the cycle start time)
  long delay = getDelayMicros(cycleDelay, now);

  // If wrapping is enabled, then we ensure that the delay is greater than
  // min delay by wrapping an ENTIRE cycle time onto it until that is the case.
  // The means that the time will always be the NEXT cycleDelay offset from a
  // cycleStart with a value greater than MIN_DELAY. IF wrapping is disabled
  // and the delay is less than MIN_DELAY, then we cannot set the timer
  if (wrapTime) {
    while (delay < MIN_DELAY) {
      // sprintf(medium_buf, "| → Wrap: %lu", delay); pcln(medium_buf);
      delay += settings.t_c; //settings.t_fs + settings.t_fr + settings.t_fc;
    }
  } else if (delay < MIN_DELAY) {
    // Serial.println("FT False");
    sprintf(medium_buf, "> Frame Timer Delay too Short %ld", delay); pcln(medium_buf);
    return false;
  }

  // pcln("a");

  // Start the frame timer with the given delay. Set it to calculate an internal
  // offset such that the timer starts as close as possible to right now.
  // TODO Should we use now instead of micros() for the offset?
  M0Timer.start(delay, _FRAME_TIMER, now);

  // pcln("b");
  frameTimerSet = true;


  sprintf(medium_buf, "> Frame Timer set: %ld", delay + now); pcln(medium_buf);
  sprintf(medium_buf, "| → Delay: %lu", delay); pcln(medium_buf);
  sprintf(medium_buf, "| → CycleDelay: %lu", cycleDelay); pcln(medium_buf);
  sprintf(medium_buf, "| → Now:   %ld", now); pcln(medium_buf);

  // dec();
  return true;
}

boolean setBlockTimer(uint32_t cycleDelay, uint32_t now) {
  // Set the delay based on the cycle start time, the offset (now time), and
  // the callback delay (referenced to the cycle start time)
  long delay = getDelayMicros(cycleDelay, now);

  if (delay < MIN_DELAY) {
    sprintf(medium_buf, "> Block Timer Delay too Short %ld", delay); pcln(medium_buf);
    return false;
  }

  // Start the block timer with the given delay. Set it to calculate an internal
  // offset such that the timer starts as close as possible to right now.
  // TODO Should we use now instead of micros() for the offset?
  M0Timer.start(delay, _BLOCK_TIMER, micros());
  blockTimerSet = true;

  sprintf(medium_buf, "> Block Timer set: %ld", delay + now); pcln(medium_buf);
  sprintf(medium_buf, "| → Delay: %lu", delay); pcln(medium_buf);
  sprintf(medium_buf, "| → Now:   %ld", now); pcln(medium_buf);

  return true;
}

boolean updateTimers(struct State * state, state_fn * frameState, state_fn * blockState, uint32_t frameDelay, uint32_t blockDelay, uint32_t now) {

  // Set the frame timer. IF we are too close to the frame timer's time, then we
  // set our state to the frameState value and return false to alert the user
  boolean prompt = setFrameTimer(frameDelay, now);
  if (!prompt) {
    M0Timer.stop(_BLOCK_TIMER); // TODO Not Needed?
    state->next = frameState;
    pcln("> Frame Timer late!", C_RED);
    return false;
  }

  // Check to see that we should set the timer
  if(blockDelay > 0){
    // Set up the block timer to fire when it is time for us to move to our
    // ranging block. If that is too soon, then move to blockState and return
    // false to alert the user
    prompt = setBlockTimer(blockDelay, now);
    if (!prompt) {
      state->next = blockState;
      pcln("> Block Timer late!", C_RED);
      return false;
    }
  }else{
    // Reset the timer
    M0Timer.stop(_BLOCK_TIMER);
    blockTimerSet = false;
    pcln("> Disabled Block Timer", C_BLUE);
  }

  // If both timers were set correctly, return true
  return true;
}

// Check the block and frame timers to see if they have fired. If they have,
// then set the next state to one of the given states (frame and block). Returns
// true if a timer fired. Returns false if no timer fired
boolean checkTimers(struct State * state, state_fn * frameState, state_fn * blockState) {
  // Frame Timer
  if (M0Timer.getFired(_FRAME_TIMER)) {
    sprintf(medium_buf, "> Frame Timer Fired: %lu", micros()); pcln(medium_buf);
    state->next = frameState;
    return true;
  }

  if (M0Timer.getFired(_BLOCK_TIMER)) {
    sprintf(medium_buf, "> Block Timer Fired: %lu", micros()); pcln(medium_buf);
    state->next = blockState;
    return true;
  }

  if (millis() - milliTimer > MAX_STALL_TIME) {
    sprintf(medium_buf, "> Stall Timer Fired %lu", micros()); pcln(medium_buf);
    state->next = sleep;
    cycleValid = 0;
    return true;
  }

  return false;
}

boolean checkMillis(struct State * state) {
  if (millis() - milliTimer > MAX_STALL_TIME) {
    sprintf(medium_buf, "> Stall Timer Fired %lu", micros()); pcln(medium_buf);
    state->next = sleep;
    cycleValid = 0;
    return true;
  }
  return false;
}

uint8_t getMsgSeq(){
  uint8_t tmp = msg_seq;
  msg_seq++;
  if(msg_seq > 31){
    msg_seq = 0;
  }
  return tmp;
}


void hexDump(char *desc, void *addr, int len)
{
  int i;
  unsigned char buff[17];
  unsigned char *pc = (unsigned char *)addr;

  // Output description if given.
  if (desc != NULL)
  {
    sprintf(large_buf, "%s:\n\r", desc);
    Serial.print(large_buf);
  }

  if (len == 0)
  {
    Serial.print("  ZERO LENGTH\n\r");
    return;
  }
  if (len < 0)
  {
    sprintf(large_buf, "  NEGATIVE LENGTH: %i\n\r", len);
    Serial.print(large_buf);
    return;
  }

  // Process every byte in the data.
  for (i = 0; i < len; i++)
  {
    // Multiple of 16 means new line (with line offset).

    if ((i % 16) == 0)
    {
      // Just don't print ASCII for the zeroth line.
      if (i != 0)
      {
        sprintf(large_buf, "  %s\n\r", buff);
        Serial.print(large_buf);
      }

      // Output the offset.
      sprintf(large_buf, "  %04x ", i);
      Serial.print(large_buf);
    }

    // Now the hex code for the specific character.
    sprintf(large_buf, " %02x", pc[i]);
    Serial.print(large_buf);

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
  sprintf(large_buf, "  %s\n\r", buff);
  Serial.print(large_buf);
}
