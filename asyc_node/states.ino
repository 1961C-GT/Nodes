// ========================================================================== //
// Flags
//
// -------------------------------------------------------------------------- //
uint8_t post_rb; // True when exited RB states. Reset in Sleep frame.
uint8_t bcst_blocks; // True when exited RB states. Reset in Sleep frame.
uint8_t com_acpt_blocks; // True when exited RB states. Reset in Sleep frame.
// ========================================================================== //

double n = 0;
double s = 0;
double m = 0;
double m_prev = 0;

// ========================================================================== //
// STATE: SLEEP
//
// -------------------------------------------------------------------------- //
state_fn sleep, sleep_loop, sleep_decode;
void sleep(struct State * state)
{

  // If our reset flag is high, then force the system to reset
  if (softReset) {
    stopTimers();
    setup();
    NVIC_SystemReset();
    return;
  }
  // blinkInit();

  // section("Frame: SLEEP", C_ORANGE);
  header("SLEEP FRAME", C_BLACK, BG_GREEN);
  pcln("[State] SLEEP_INIT", C_ORANGE);
  inc();
  // Make sure that we do not have any block or frame timers set. (which means
  // we will be stuck in sleep until one is set at some point)
  stopTimers();

  // Start RX Mode
  receiver();

  // Deal with LED control
  setLed(LED_AUX,MODE_OFF);
  setLed(LED_RED,MODE_OFF);
  setLed(LED_BLUE,MODE_OFF);
  setLed(LED_GREEN,MODE_OFF);

  digitalWrite(LED_PIN, LOW);
  digitalWrite(BOARD_RGB_RED, LOW);
  digitalWrite(BOARD_RGB_GREEN, LOW);
  digitalWrite(BOARD_RGB_BLUE, LOW);

  milliTimer = millis();

  pcln("Setting Flags");
  post_rb = false;
  bcst_blocks = 0;
  com_acpt_blocks = 0;
  cycle_counter++;

  // Set the default next state
  state->next = sleep_loop;

  // If we are the base, make some exceptions to the transmit rules
  if (cycleValid <= 0 && isBase) {
    cycleStart = micros();
    cycleValid = 10;
  }
  if (transmitAuthorization <= 0 && isBase) {
    transmitAuthorization = TRANSMIT_AUTH_CAP;
  }

  // If our cycle is valid here at the start of sleep, then go ahead and set our
  // sleep timeout. Otherwise we will have to wait until we can validate it
  // later.
  if (cycleValid > 0 && transmitAuthorization > 0) {

    pcln("Current DateTime:");
    printDate();

    sprintf(medium_buf, "Cycle Valid: %d", cycleValid); pcln(medium_buf);
    sprintf(medium_buf, "Old Cycle Start: %d", cycleStart); pcln(medium_buf);

    // Step the cycle start forward
    cycleStart += settings.t_c;

    sprintf(medium_buf, "New Cycle Start: %d", cycleStart); pcln(medium_buf);
    sprintf(medium_buf, "Now: %d", micros()); pcln(medium_buf);

    if (!isBase) {
      byte temp = 0;
      byte vbat = 0;
      DW1000.getTempAndVbatByte(temp, vbat);

      int heading = 0xFFFF;

      if (magEnabled) {
        sensors_event_t event;
        mag.getEvent(&event);
        heading = (int) ((atan2(event.magnetic.z,event.magnetic.x) * 180) / 3.14159f);
        if (heading < 0) {
          heading = 360 + heading;
        }
      }

      stats_msg sm = {.from = nodeNumber, .seq = getMsgSeq(), .hops = ALLOWABLE_HOPS, .bat = getBattVoltageBits(), .temp = temp, .heading = heading};
      if (!putPacketInBuffer((various_msg *)&sm, STATUS_PACKET)) {
        pcln("Packet Buffer Full. Overwrote Message", C_RED);
      }
    }

    boolean prompt = setFrameTimer(settings.t_fs, cycleStart, true);

    // DW1000.deepSleep();

    // If we change states now, then blink the green light
    if (!prompt)
      setLed(LED_GREEN,MODE_CHIRP);

    setLed(LED_BLUE, MODE_OFF);
  } else {
    setLed(LED_BLUE, MODE_BLINK);
  }

  // Decrement our cycle valid
  cycleValid --;
  if (cycleValid < 0)
    cycleValid = 0;

  // Decrement our transmit authorization
  transmitAuthorization --;
  if (transmitAuthorization < 0)
    transmitAuthorization = 0;

  sprintf(medium_buf, "Cycle Valid Value: %d", cycleValid); pcln(medium_buf);
  sprintf(medium_buf, "Transmit Authorization Value: %d", transmitAuthorization); pcln(medium_buf);


  // If we are are the base station, then we want to serial all of our
  // data out
  if (isBase) {

    // Build our cycle's transmit authorization message

    // 23-16 15----8 7---0
    // Hours Minutes Hours
    uint32_t timeData = 0;
    timeData = timeData | rtc.getSeconds();
    timeData = timeData | (rtc.getMinutes() << 8);
    timeData = timeData | (rtc.getHours() << 16);

    sprintf(medium_buf, "Sent Time %12X", timeData); pcln(medium_buf, C_GREEN);

    cmd_msg cm = {.seq = getMsgSeq(), .cmd_id = COM_TRANSMIT_AUTH, .hops = ALLOWABLE_HOPS, .data = timeData};
    memcpy(&cmd_buffer[0], (various_msg *)&cm, sizeof(various_msg));
    cmd_buffer_len = 1;

    if (Serial5 && Serial5.available()) {
      while(Serial5.available())
        Serial5.read();

      Serial5.println("Queueing Sleep Command!!");

      cmd_msg rstcmd = {.seq = getMsgSeq(), .cmd_id = SOFT_RESET, .hops = ALLOWABLE_HOPS, .data = 0};
      memcpy(&cmd_buffer[cmd_buffer_len], (various_msg *)&rstcmd, sizeof(various_msg));
      cmd_buffer_len ++;

      softReset = true;
    }


    // Print out our incomming messages
    various_msg packet;
    uint8_t packet_id;
    while (getPacketFromBuffer(&packet, &packet_id)) {
      switch(packet_id) {
        case RANGE_PACKET:
          {
            range_msg * msg = (range_msg *) &packet;

            Serial5.print("Range Packet | Cycle:"); Serial5.print(cycle_counter);
            Serial5.print(",\tFrom:"); Serial5.print(msg -> from);
            Serial5.print(",\tTo:"); Serial5.print(msg -> to);
            Serial5.print(",\tSeq:"); Serial5.print(msg -> seq);
            Serial5.print(",\tHops:"); Serial5.print(msg -> hops);
            Serial5.print(",\tRange:"); Serial5.println(msg -> range);

            // int rng = msg->range;
            //
            // if (rng < 500 || rng > 500000) {
            //   continue;
            // }
            //
            // n ++;
            // m_prev = m;
            // m = m + (rng - m) / n;
            // s = s + (rng - m) * (rng - m_prev);
            //
            // // runningAverage += (msg -> range);
            // // measurement_counter++;
            // Serial5.print("#: "); Serial5.print(n);
            // Serial5.print(", R: "); Serial5.print(msg->range);
            // Serial5.print(", Mean:"); Serial5.print(m);
            // Serial5.print(", Var:"); Serial5.println(s/n);
          }
          break;
        case STATUS_PACKET:
          {
            // stats_msg * msg = (stats_msg *) &packet;
            //
            // Serial5.print("Stats Packet | Cycle:"); Serial5.print(cycle_counter);
            // Serial5.print(",\tFrom:"); Serial5.print(msg->from);
            // Serial5.print(",\t\tSeq:"); Serial5.print(msg->seq);
            // Serial5.print(",\tHops:"); Serial5.print(msg->hops);
            // Serial5.print(",\tBat:"); Serial5.print(msg->bat * BATT_MEAS_COEFF);
            // Serial5.print(",\tTemp:"); Serial5.print(DW1000.convertTemp(msg->temp));
            // Serial5.print(",\tHeading:"); Serial5.println(msg->heading);
          }
          break;
        default:
          // Serial5.print("Unknown Packet Type: "); Serial5.println(packet_id);
          break;
      }
    }

    // // Limit ourselves to 500 cycles
    // if (isBase && n > 250) {
    //   Serial5.print("Test Complete. Mean:"); Serial5.print(m);
    //   Serial5.print(", Var:"); Serial5.println(s/n);
    //   setLed(LED_RED, MODE_BLINK);
    //   while (1) {
    //     delay(500);
    //   }
    // }

  }

  /*
  typedef struct stats_msg
  {                      // 5 Bytes
    uint8_t from : 4;    // Node ID of the originating node
    uint8_t seq : 5; // Unique message number
    uint8_t hops : 3;    // Total allowed remaining hops for this message
    uint16_t bat : 8;    // Raw battery voltage value
    uint16_t heading : 9; // Raw heading value
    int pad : 11;        // Extra padding to make 5 byte aligned
  } stats_msg;*/

  // #ifdef DEBUG
  // endSection();
  // section("Frame: SLEEP_LOOP");
  dec();
  pcln("[State] SLEEP_LOOP", C_ORANGE);
  inc();
  // #endif
}
void sleep_loop(struct State * state)
{
  // See if we have gotten a message
  if (checkRx()) {
    pcln("Received Message", C_GREEN);
    state->next = sleep_decode;
    return;
  }

  // See if it is time to leave the sleep frame. The block state is set to the
  // sleep frame so that if it is set (incorrecly), we will not leave sleep mode
  if (checkTimers(state, range_frame_init, sleep)) {

    // DW1000.spiWakeup();
    receiver();

    setLed(LED_GREEN,MODE_CHIRP);
    // cycleStart = micros() - settings.t_fs;
    pcln("Timer Triggered", C_GREEN);
    dec();
    return;
  }

  // // If we do not have a frame timer set but we DO have a valid cycle time, then
  // // we should go ahead and set our frame timer
  // if (!frameTimerSet && cycleValid > 0) {
  //   pcln("Cycle Now Valid. Setting Frame Timer", C_GREEN);
  //   setFrameTimer(settings.t_fs, cycleStart, true);
  // }
}
void sleep_decode(struct State * state)
{
  // #ifdef DEBUG
  section("[state] SLEEP_DECODE", C_ORANGE);
  // pcln("Sleep Decode");
  // #endif
  // blinkInit();

  // Set the defualt next state
  state->next = sleep_loop;

  sprintf(medium_buf, "Rx @:  %lu", rxTimeM0); pcln(medium_buf);

  // Get the message from the DW1000 and parse it
  rxMessage = getMessage();
  processMessage(rxMessage);

  if (rxMessage.valid)
  {

    printMessage(rxMessage);

    // See if we can adjust our clock based on this message
    boolean adjusted = adjustClock(rxMessage, rxTimeM0, rxTimeDW.getAsMicroSeconds());

    // If we did adjust our clock, then we should set (or reset) our frame
    // callback if it is authorized
    if (adjusted && cycleValid > 0 && transmitAuthorization > 0) {
      pcln("Adjusted clock. Setting new frame timer", C_BLUE);
      setFrameTimer(settings.t_fs, cycleStart, true);
    }
  }
  endSection("End Decode");
}
// ========================================================================== //

// ========================================================================== //
// STATE: RANGE_FRAME_INIT
//
// -------------------------------------------------------------------------- //
state_fn range_frame_init;
void range_frame_init(struct State * state)
{
  // Save the current time before we do any calculations
  uint32_t now = micros();

  header("RANGE FRAME", C_WHITE, BG_RED);
  pcln("[State] RANGE_FRAME_INIT", C_ORANGE);
  inc();
  // Serial.println("[STATE] SLEEP_LOOP -> RANGE_FRAME_INIT");

  // Go ahead and stop any block and frame timers now
  stopTimers();

  if (cycleValid) {

    state->next = ra_init;

    boolean ret = updateTimers(state, com_frame_init, rb_range,
      settings.t_fs + settings.t_fr,
      settings.t_fs + settings.t_br * nodeNumber, now);

    if (!ret) {
      pcln("Callback Timer not set. Moving to alternate state", C_RED);
      sprintf(medium_buf, "| → Cycle Start:  %lu", cycleStart); pcln(medium_buf);
      sprintf(medium_buf, "| → Now:          %lu", micros()); pcln(medium_buf);
      sprintf(medium_buf, "| → Cycle Length: %lu", settings.t_fs + settings.t_fr); pcln(medium_buf);
      sprintf(medium_buf, "| → Own RB Time:  %lu", settings.t_fs + settings.t_br * nodeNumber); pcln(medium_buf);
    }

    // Serial.print("Frame Time "); Serial.println(settings.t_fs + settings.t_fr);
    // Serial.print("Block Time "); Serial.println(settings.t_fs + settings.t_br * nodeNumber);

    // // Set up our frame timer for the range frame
    // boolean prompt = setFrameTimer(settings.t_fs + settings.t_fr, now);
    // if (!prompt) {
    //   state->next = sleep;
    //   return;
    // }
    //
    // // Set up the block timer to fire when it is time for us to move to our
    // // ranging block. If that is too soon, then move right to the range
    // // block
    // prompt = setBlockTimer(settings.t_fs + settings.t_br * nodeNumber, now);
    // if (!prompt) {
    //   state->next = rb_range;
    // } else {
    //   state->next = ra_init;
    // }

  } else {
    pcln("Invalid Clock. Returning to sleep", C_RED);
    // If our cycle is not valid, then we need to wait in sleep instead
    state->next = sleep;
  }
  dec();
}


// ========================================================================== //


// ========================================================================== //
// STATE: RA_INIT
//
// -------------------------------------------------------------------------- //
state_fn ra_init, ra_init_loop;
void ra_init(struct State * state)
{
  pcln("[State] RA_INIT", C_ORANGE); inc();

  // blinkInit();

  // Start RX Mode
  receiver();

  // blinkLoop();
  state->next = ra_init_loop;

  dec();
  pcln("[State] RA_INIT_LOOP", C_ORANGE); inc();
}
void ra_init_loop(struct State * state)
{
  // Check our timers. If one of them fires, then our state will be set as
  // needed, based on the values we pass in.
  if (checkTimers(state, com_frame_init, rb_range)) {
    pcln("Timer Triggered", C_GREEN);
    dec();
    return;
  }

  if (checkRx()) {
    pcln("Received Message", C_GREEN);
    state->next = ra_decode;
    return;
  }

}
// ========================================================================== //


// ========================================================================== //
// STATE: RA_DECODE
//
// -------------------------------------------------------------------------- //
state_fn ra_decode;
void ra_decode(struct State * state)
{
  boolean changeState = false;

  section("[state] RA_DECODE", C_ORANGE);

  // blinkInit();

  // Set the defualt next state
  state->next = ra_init;

  // Message temp value
  rxMessage = getMessage();
  processMessage(rxMessage);

  sprintf(medium_buf, "Rx @:  %lu", rxTimeM0); pcln(medium_buf);

  if (rxMessage.valid) {

    printMessage(rxMessage);

    // Adjust our clock based on the message we received.
    sprintf(medium_buf, "Cycle Time (pre adjust):  %lu", cycleStart); pcln(medium_buf);
    boolean adj = adjustClock(rxMessage, rxTimeM0, rxTimeDW.getAsMicroSeconds());
    sprintf(medium_buf, "Cycle Time (post adjust): %lu", cycleStart); pcln(medium_buf);
    switch(rxMessage.type) {
      case RANGE_REQ:
        // If we got a range request, move to range response
        rangeRequestFrom = rxMessage.from;
        rangeRequestSeq = rxMessage.seq;
        state->next = ra_resp;
        changeState = true;
        pcln("Got Range Request", C_GREEN);
        break;
      default:
        changeState = true;
        pcln("Bad Message Type", C_RED);
        // state->next = getState(rxMessage);
        break;
    }

    if (adj) {

      pcln("Adjusting Timers From New Clock Settings", C_BLUE);

      // If we got a new clock adjustment, then we should reset our block timer
      // and frame timer to match this more accurate value

      uint64_t blockTime;
      if(post_rb){
        blockTime = 0; // Don't set at all
      } else {
        blockTime = settings.t_fs + settings.t_br * nodeNumber;
      }
      boolean ret = updateTimers(state, com_frame_init, rb_range,
        settings.t_fs + settings.t_fr,
        blockTime, micros());

      // TODO: If we are late setting this timer, then we should proceed directly to
      // the next block
      if (!ret) {
        pcln("Callback Timer not set. Moving to alternate state", C_RED);
        sprintf(medium_buf, "| → Cycle Start:  %lu", cycleStart); pcln(medium_buf);
        sprintf(medium_buf, "| → Now:          %lu", micros()); pcln(medium_buf);
        sprintf(medium_buf, "| → Cycle Length: %lu", settings.t_fs + settings.t_fr); pcln(medium_buf);
        sprintf(medium_buf, "| → Own RB Time:  %lu", settings.t_fs + settings.t_br * nodeNumber); pcln(medium_buf);
        changeState = true;
      }
    }
  } else {
    pcln("Malformed Message", C_RED);
    changeState = true;
  }
  endSection("End Decode");

  // If the state was changed, then we are about to leave our current global
  // state. We should decrement the log to show this
  if (changeState)
    dec();
}
// ========================================================================== //


// ========================================================================== //
// STATE: RA_RESP
//
// -------------------------------------------------------------------------- //
state_fn ra_resp, ra_resp_loop;
void ra_resp(struct State * state)
{
  // blinkTx();

  pcln("[State] RA_RESP", C_ORANGE); inc();

  // Configure for a new transmit
  DW1000.newTransmit();
  DW1000.setDefaults();

  // Set the message to be from us
  txMessage.from = nodeNumber;

  // TODO Maybe compare the calculated block start sequence to the measured
  // one (rangeRequestSeq) in order to ensure we are all on the same page? They
  // should match unless there is some sort of issue
  // uint8_t blockStartSeq = (settings.n + 1 ) * rangeRequestFrom;
  // rangeRequestSeq

  // Apply the sequence number to the message. This is based on our node number
  // as an offset from the seq number of the range request itself.
  txMessage.seq = rangeRequestSeq + nodeNumber + 1;

  // Set the message type
  txMessage.type = RANGE_RESP;

  // Set the message data length to 0 (empty message)
  txMessage.len = 0;

  // Load any data into this range response
  loadPackets(&txMessage);

  // get the length of the full message (data + MAC)
  uint8_t len = getMessageLength(txMessage);

  // Create a buffer array to accept the contents of the message
  byte data[len];

  // Create the message and place it in the data buffer
  createMessage(data, txMessage);


  uint32_t now = micros();

  // Schedule the message to send exatly settings.t_r micros from the last
  // rx
  DW1000.setDelayFromRx(t_r); // DW1000Time(settings.t_rn, DW1000Time::MICROSECONDS)

  // Set the data of the message
  DW1000.setData(data, len);

  // Start the message transmission
  DW1000.startTransmit();

  dec();
  pcln("[State] RA_RESP_LOOP", C_ORANGE); inc();
  sprintf(medium_buf, "Message Scheduled for %lu", settings.t_r + rxTimeM0); pcln(medium_buf);
  sprintf(medium_buf, "| → Now: %lu", now); pcln(medium_buf);

  // Proceed to loop state
  state->next = ra_resp_loop;
}
void ra_resp_loop(struct State * state)
{
  // ***###TODO Do we need these here?
  // // Check our timers. If one of them fires, then our state will be set as
  // // needed, based on the values we pass in.
  if (checkTimers(state, com_frame_init, rb_range)) {
    dec();
    return;
  }

  if(checkTx()) {
    // DW1000.readSystemEventStatusRegister();
    // Serial.print("Late: "); Serial.println(DW1000.isLate());

    // DW1000Time txTime;
    // DW1000.getTransmitTimestamp(txTime);
    //
    // DW1000Time turn = (txTime - timePollSent).wrap();
    // Serial.print("Turn: "); Serial.print(turn.getAsNanoSeconds());
    // Serial.print("ns, Goal: "); Serial.println(t_r.getAsMicroSeconds());
    // Serial.print("ns, TX:"); Serial.print(txTime.getAsMicroSeconds());
    // Serial.print("ns, ");
    receiver();
    sprintf(medium_buf, "Message Sent At %lu", txTimeM0); pcln(medium_buf, C_GREEN);
    state->next = ra_init;
    dec();
  }
}
// ========================================================================== //


// ========================================================================== //
// STATE: RB_RANGE
//
// -------------------------------------------------------------------------- //
state_fn rb_range, rb_range_loop;
void rb_range(struct State * state)
{

  pcln("[State] RB_RANGE", C_ORANGE); inc();

  // Set this block to end at the end of this node's range block. We do not
  // set the frame timer here because we do not have new and more accurate
  // information here. The block timer is being set to tell us to move back
  // to the other state
  boolean prompt = setBlockTimer(settings.t_fs + settings.t_br * (nodeNumber + 1), micros());

  // If it is time to move on now, then do so
  if (!prompt) {
    pcln("Behind Schedule (Callback Timer). Moving state", C_RED);
    post_rb = true;
    state->next = ra_init;
    dec();
    return;
  }

  // Configure out LED
  // blinkTx();

  // Configure for transmission
  DW1000.newTransmit();
  DW1000.setDefaults();

  // Set this message to be from us
  txMessage.from = nodeNumber;

  // Set the sequence number to be the start of this node's range request
  // block. This is the specific number associated with this request
  txMessage.seq = (settings.n + 1) * nodeNumber;

  // Set the message type to a range request
  txMessage.type = RANGE_REQ;

  // Set the length of the message data to 0
  txMessage.len = 0;

  // Load any data into this range request
  loadPackets(&txMessage);

  // Get the full length of the message (data + MAC)
  uint8_t len = getMessageLength(txMessage);

  // Create a buffer to store the byte form of the message
  byte data[len];

  // Create the message and store it in data
  createMessage(data, txMessage);

  // Set the message data to data
  DW1000.setData(data, len);

  // Set the message delay so that it sends exacty when it should (exactly t_rx
  // after the start of this block)
  int64_t delay = getDelayMicros(settings.t_fs + (settings.t_br * nodeNumber) + settings.t_rx, micros());

  // Only set the delay of the message if our delay value is greater than the
  // MIN_DELAY. Otherwise, we send out message right away (Without settings a
  // delay)
  if (delay > MIN_TX_DELAY) {
    DW1000.setDelay(DW1000Time(delay, DW1000Time::MICROSECONDS));
  }

  // Start the transmission
  DW1000.startTransmit();

  // Proceed to loop state
  state->next = rb_range_loop;
  dec();

  pcln("[State] RB_RANGE_LOOP", C_ORANGE); inc();
  sprintf(medium_buf, "Message Scheduled for %lu", micros() + delay); pcln(medium_buf);
  sprintf(medium_buf, "| → Delay: %lu", delay); pcln(medium_buf);
}
void rb_range_loop(struct State * state)
{

  if (checkTimers(state, com_frame_init, ra_init)) {
    receiver();
    pcln("Timer Fired Before Message Sent", C_RED);
    dec();
    return;
  }

  if(checkTx()) {
    receiver();
    sprintf(medium_buf, "Message Sent At %lu", txTimeM0); pcln(medium_buf, C_GREEN);
    state->next = rb_rec;
    dec();
    return;
  }

  // TODO***### Do we need this?
  // // Check our timers. If one of them fires, then our state will be set as
  // // needed, based on the values we pass in.
  // if (checkTimers(state, com_frame_init, ra_init)) {
  //   return;
  // }
}
// ========================================================================== //


// ========================================================================== //
// STATE: RB_REC
//
// -------------------------------------------------------------------------- //
state_fn rb_rec, rb_rec_loop;
void rb_rec(struct State * state)
{
  pcln("[State] RB_REC", C_ORANGE); inc();

  // blinkInit();

  // Enter receiver mode right away to catch messages as fast as possible
  receiver();

  // Reset our block timer based on the time at which our transmission sent
  // (which is the NEW block sync)
  boolean adjusted = adjustClock(txMessage, txTimeM0, txTimeDW.getAsMicroSeconds(), false);

  if (!adjusted) {
    // pcln("Clock Not Adjusted on TX", C_RED);
    // printMessage(txMessage);
  } else {
    pcln("Set new Frame and Block Timers", C_BLUE);
    // If we got a new clock adjustment, then we should reset our block timer
    // and frame timer to match this more accurate value
    boolean ret = updateTimers(state, com_frame_init, ra_init,
      settings.t_fs + settings.t_fr,
      settings.t_fs + settings.t_br * (nodeNumber + 1), micros());

    // If we are late setting this timer, then we should proceed directly to
    // the next block
    if (!ret) {
      pcln("Callback Timer not set. Moving to alternate state", C_RED);
      sprintf(medium_buf, "| → Cycle Start:  %lu", cycleStart); pcln(medium_buf);
      sprintf(medium_buf, "| → Now:          %lu", micros()); pcln(medium_buf);
      sprintf(medium_buf, "| → Cycle Length: %lu", settings.t_fs + settings.t_fr); pcln(medium_buf);
      sprintf(medium_buf, "| → Own RB Time:  %lu", settings.t_fs + settings.t_br * nodeNumber); pcln(medium_buf);
      post_rb = true;
      dec();
      return;
    }
  }

  // Check our timers. If one of them fires, then our state will be set as
  // needed, based on the values we pass in.
  if (checkTimers(state, com_frame_init, ra_init)) {
    pcln("Timer Triggered", C_GREEN);
    post_rb = true;
    dec();
    return;
  }

  // Store the time at which we sent this message (for calculating TOF)
  timePollSent = txTimeDW;

  // Proceed to loop state
  // blinkLoop();
  state->next = rb_rec_loop;

  dec();

  pcln("[State] RB_REC_LOOP", C_ORANGE); inc();
}
void rb_rec_loop(struct State * state)
{
  // Check our timers. If one of them fires, then our state will be set as
  // needed, based on the values we pass in.
  if (checkTimers(state, com_frame_init, ra_init)) {
    pcln("Timer Triggered", C_GREEN);
    post_rb = true;
    dec();
    return;
  }

  if(checkRx()) {
    pcln("Received Message", C_GREEN);
    state->next = rb_decode;
    // dec();
    return;
  }
}
// ========================================================================== //

// ========================================================================== //
// STATE: RB_DECODE
//
// -------------------------------------------------------------------------- //
state_fn rb_decode;
void rb_decode(struct State * state)
{
  // blinkInit();
  section("[state] RB_DECODE", C_ORANGE);

  // Set the defualt next state
  state->next = rb_rec_loop;

  // Load the message into our rxMessage Object
  rxMessage = getMessage();
  processMessage(rxMessage);

  sprintf(medium_buf, "Rx @:  %lu", rxTimeM0); pcln(medium_buf);


  // Ensure that the message is valid
  if (rxMessage.valid) {

    printMessage(rxMessage);

    // Adjust our clock based on the message we received
    boolean adjusted = adjustClock(rxMessage, rxTimeM0, rxTimeDW.getAsMicroSeconds());

    // See what type of message it is
    switch(rxMessage.type){
      case RANGE_RESP:

        if (rxMessage.from < settings.n) {
          DW1000Time rec;
          DW1000.getReceiveTimestamp(rec);
          float dist = computeRange(rec, rxMessage.from);
          if(dist < 0){
            pcln("Dist calculated as less than 0", C_RED);
            dist = 0;
          }
          setLed(LED_AUX, MODE_CHIRP);

          pcln("Message Type: Range RESP", C_PURPLE);
          sprintf(medium_buf, "| → From: %d", rxMessage.from); pcln(medium_buf, C_PURPLE);
          sprintf(medium_buf, "| → Seq: %d", rxMessage.seq); pcln(medium_buf, C_PURPLE);
          sprintf(medium_buf, "| → Distance: %d", dist); pcln(medium_buf, C_PURPLE);
          Serial.println(dist);
          // Store this range to the distance_meas array
          // distance_meas[rxMessage.from] = dist;

          if (dist <= MAX_VALID_DISTANCE) { // distance is in meters here
            range_msg rm = {.from = nodeNumber, .to = rxMessage.from, .seq = getMsgSeq(), .hops = ALLOWABLE_HOPS, .range = (uint32_t)(dist*1000.0)}; // Distance is encoded in mm
            if (!putPacketInBuffer((various_msg *)&rm, RANGE_PACKET)) {
              pcln("Packet Buffer Full. Overwrote Message", C_RED);
            }
          }

        } else {
          pcln("From ID Out Of Bounds", C_RED);
          sprintf(medium_buf, "| → Node ID: %d", rxMessage.from); pcln(medium_buf, C_RED);
        }

        break;
      case SETTINGS:
        // TODO
      default:
        pcln("Bad Message Type", C_RED);
        break;
    }

    // If we adjused our clock, then reset our callback timers
    if (adjusted) {
      pcln("Set new Frame and Block Timers", C_BLUE);

      // If we got a new clock adjustment, then we should reset our block timer
      // and frame timer to match this more accurate value
      boolean ret = updateTimers(state, com_frame_init, ra_init,
        settings.t_fs + settings.t_fr,
        settings.t_fs + settings.t_br * (nodeNumber + 1), micros());

      // If we are late setting this timer, then we should proceed directly to
      // the next block
      if (!ret) {
        pcln("Callback Timer not set. Moving to alternate state", C_RED);
        sprintf(medium_buf, "| → Cycle Start:  %lu", cycleStart); pcln(medium_buf);
        sprintf(medium_buf, "| → Now:          %lu", micros()); pcln(medium_buf);
        sprintf(medium_buf, "| → Cycle Length: %lu", settings.t_fs + settings.t_fr); pcln(medium_buf);
        sprintf(medium_buf, "| → Own RB Time:  %lu", settings.t_fs + settings.t_br * nodeNumber); pcln(medium_buf);
        endSection("End Decode");
        post_rb = true;
        dec();
        return;
      }
    }
  } else {
      pcln("Malformed Message", C_RED);
  }

  // // Check our timers. If one of them fires, then our state will be set as
  // // needed, based on the values we pass in.
  // if (checkTimers(state, com_frame_init, ra_init)) {
  //   Serial.println( "[RB_DECODE] Timer Triggered");
  //   return;
  // }
  endSection("End Decode");
}
// ========================================================================== //




// ========================================================================== //
// FRAME: COM_FRAME
//
// -------------------------------------------------------------------------- //
state_fn com_frame_init, com_acpt, com_acpt_loop, com_decode, com_bcst, com_bcst_loop;
void com_frame_init(struct State * state)
{

  // Save the current time before we do any calculations
  uint32_t now = micros();

  // Go ahead and stop any block and frame timers now
  stopTimers();

  header("COM FRAME", C_BLACK, BG_YELLOW);

  pcln("[State] COM_FRAME_INIT", C_ORANGE); inc();

  if (isBase) {
    ta_msg_seq = getMsgSeq();
  }

  state->next = com_acpt;

  // Set only the frame timer
  boolean prompt = setFrameTimer(settings.t_c, now);

  if (!prompt) {
    pcln("Callback Timer not set. Moving to alternate state", C_RED);
    sprintf(medium_buf, "| → Cycle Start:  %lu", cycleStart); pcln(medium_buf);
    sprintf(medium_buf, "| → Now:          %lu", micros()); pcln(medium_buf);
    sprintf(medium_buf, "| → Cycle Length: %lu", settings.t_fs + settings.t_fr); pcln(medium_buf);
    sprintf(medium_buf, "| → Own RB Time:  %lu", settings.t_fs + settings.t_br * nodeNumber); pcln(medium_buf);
    state->next = sleep;
  }

  dec();
}
void com_acpt(struct State * state)
{
  pcln("[State] COM_ACPT", C_ORANGE); inc();

  com_acpt_blocks ++;

  if (com_acpt_blocks > settings.n_com + 2) {
    state->next = sleep;
    dec();
    return;
  }

  if (checkMillis(state)){
    state->next = sleep;
    dec();
    return;
  }

  // Set this block timer
  uint32_t delay = (settings.t_fs + settings.t_fr) + (settings.t_cl + settings.t_b + settings.t_rx) * nodeNumber + settings.t_bc * settings.n * bcst_blocks;
  Serial.print("Calculated Delay:"); Serial.println(delay);
  boolean prompt = setBlockTimer(delay, micros());
  if (!prompt) {
    pcln("Behind Schedule (Block Timer). Moving state", C_RED);
    state->next = com_bcst;
    dec();
    return;
  }

  receiver();

  state->next = com_acpt_loop;

  dec();
  pcln("[State] COM_ACPT_LOOP", C_ORANGE); inc();
}
void com_acpt_loop(struct State * state)
{

  if (com_acpt_blocks > settings.n_com + 2) {
    state->next = sleep;
    dec();
    return;
  }

  // Check our timers. If one of them fires, then our state will be set as
  // needed, based on the values we pass in.
  if (checkTimers(state, sleep, com_bcst)) {
    pcln("Timer Triggered", C_GREEN);
    dec();
    return;
  }

  if(checkRx()) {
    pcln("Received Message", C_GREEN);
    state->next = com_decode;
    return;
  }
}
void com_decode(struct State * state)
{
  section("[state] COM_DECODE", C_ORANGE);

  // Set the defualt next state
  state->next = com_acpt_loop;

  // Message temp value
  rxMessage = getMessage();
  processMessage(rxMessage);

  sprintf(medium_buf, "Rx @:  %lu", rxTimeM0); pcln(medium_buf);

  if (rxMessage.valid) {

    printMessage(rxMessage);

    // Adjust our clock based on the message we received.
    sprintf(medium_buf, "Cycle Time (pre adjust):  %lu", cycleStart); pcln(medium_buf);
    boolean adj = adjustClock(rxMessage, rxTimeM0, rxTimeDW.getAsMicroSeconds());
    sprintf(medium_buf, "Cycle Time (post adjust): %lu", cycleStart); pcln(medium_buf);
    switch(rxMessage.type) {
      case COM_MSG:
        pcln("Got Com Message", C_PURPLE);
        break;
      default:
        pcln("Bad Message Type", C_RED);
        break;
    }

    if (adj) {
      pcln("Adjusting Timers From New Clock Settings", C_BLUE);

      // If we got a new clock adjustment, then we should reset our block timer
      // and frame timer to match this more accurate value

      // TODO ###
      uint32_t blockTime = (settings.t_fs + settings.t_fr) + (settings.t_cl + settings.t_b + settings.t_rx) * nodeNumber + settings.t_bc * settings.n * bcst_blocks;
      // if(post_rb){
      //   blockTime = 0; // Don't set at all
      // } else {
      //   blockTime = 0; // TODO ### settings.t_fs + settings.t_br * nodeNumber;
      // }
      boolean ret = updateTimers(state, sleep, com_bcst,
        settings.t_c,
        blockTime, micros());

      if (!ret) {
        pcln("Callback Timer not set. Moving to alternate state", C_RED);
        sprintf(medium_buf, "| → Cycle Start:  %lu", cycleStart); pcln(medium_buf);
        sprintf(medium_buf, "| → Now:          %lu", micros()); pcln(medium_buf);
        sprintf(medium_buf, "| → Cycle Length: %lu", settings.t_fs + settings.t_fr); pcln(medium_buf);
        sprintf(medium_buf, "| → Own RB Time:  %lu", settings.t_fs + settings.t_br * nodeNumber); pcln(medium_buf);
        endSection("End Decode");
        dec();
        return;
      }
    }
  } else {
    pcln("Malformed Message", C_RED);
  }

  endSection("End Decode");
}
void com_bcst(struct State * state)
{
  pcln("[State] COM_BCST", C_ORANGE); inc();

  if (com_acpt_blocks > settings.n_com + 2) {
    state->next = sleep;
    dec();
    return;
  }

  if (checkMillis(state)){
    dec();
    return;
  }

  // Set this block timer
  uint64_t delay = (settings.t_fs + settings.t_fr) + (settings.t_cl + settings.t_b + settings.t_rx) * nodeNumber + settings.t_bc * settings.n * bcst_blocks + settings.t_bc;
  boolean prompt = setBlockTimer(delay, micros());
  if (!prompt) {
    pcln("Behind Schedule (Block Timer). Moving state", C_RED);
    state->next = com_acpt;
    dec();
    return;
  }

  // #################### Send the Com Message #################### //
  // Configure for transmission
  DW1000.newTransmit();
  DW1000.setDefaults();

  // Set this message to be from us
  txMessage.from = nodeNumber;

  // Set the sequence number to be the start of this node's range request
  // block. This is the specific number associated with this request
  txMessage.seq = ((settings.n + 1) * settings.n) + nodeNumber + bcst_blocks * settings.n;

  // Set the message type to a range request
  txMessage.type = COM_MSG;

  // Set the length of the message data to 0
  txMessage.len = 0;

  // Load avaliable packets into the message
  loadPackets(&txMessage);


  // txMessage.data = []; // TODO ##

  // Get the full length of the message (data + MAC)
  uint8_t len = getMessageLength(txMessage);

  // Create a buffer to store the byte form of the message
  byte data[len];

  // Create the message and store it in data
  createMessage(data, txMessage);

  // Set the message data to data
  DW1000.setData(data, len);

  // Set the message delay so that it sends exacty when it should (exactly t_rx
  // after the start of this block)
  delay = getDelayMicros((settings.t_fs + settings.t_fr) + settings.t_rx + (settings.t_cl + settings.t_b + settings.t_rx) * nodeNumber + settings.t_bc * settings.n * bcst_blocks, micros());

  // Only set the delay of the message if our delay value is greater than the
  // MIN_DELAY. Otherwise, we send out message right away (Without settings a
  // delay)
  if (delay > MIN_TX_DELAY) {
    DW1000.setDelay(DW1000Time(delay, DW1000Time::MICROSECONDS));
  }

  // Start the transmission
  DW1000.startTransmit();
  // #################### End Message Transmission ################ //

  dec();
  pcln("[State] COM_BCST_LOOP", C_ORANGE); inc();
  state->next = com_bcst_loop;
}
void com_bcst_loop(struct State * state)
{

  if (isBase) {
    pcln("Except from bcast");
    state->next = com_acpt;
    dec();
    bcst_blocks++;
    return;
  }

  if (com_acpt_blocks > settings.n_com + 2) {
    state->next = sleep;
    dec();
    return;
  }

  // Check our timers. If one of them fires, then our state will be set as
  // needed, based on the values we pass in.
  if (checkTimers(state, sleep, com_acpt)) {
    pcln("Timer Triggered Before Message Sent", C_RED);
    dec();
    bcst_blocks++;
    return;
  }

  if(checkTx()) {
    pcln("Sent Message", C_GREEN);
    state->next = com_acpt;
    dec();
    bcst_blocks++;
    return;
  }
}
