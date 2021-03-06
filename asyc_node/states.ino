// ========================================================================== //
// Flags
//
// -------------------------------------------------------------------------- //
uint8_t post_rb; // True when exited RB states. Reset in Sleep frame.
uint8_t bcst_blocks; // True when exited RB states. Reset in Sleep frame.
uint8_t com_acpt_blocks; // True when exited RB states. Reset in Sleep frame.
// ========================================================================== //

// ========================================================================== //
// STATE: SLEEP
//
// -------------------------------------------------------------------------- //
state_fn sleep, sleep_loop, sleep_decode;
void sleep(struct State * state)
{
  rst();
  header("SLEEP FRAME", C_BLACK, BG_GREEN);
  pcln("[State] SLEEP_INIT", C_ORANGE);
  inc();

  // Make sure that we do not have any block or frame timers set. (which means
  // we will be stuck in sleep until one is set at some point)
  stopTimers();

  // reset our milli timer
  milliTimer = millis();

  // Start RX Mode
  receiver();

  // Check for reset and sleep
  checkReset();
  checkSleep();

  // Print out how much free memory we currently have
  sprintf(medium_buf, "Current Mem: ", freeMemory()); pcln(medium_buf);

  // Deal with LED control (write them all to low)
  setLed(LED_AUX,MODE_OFF);
  setLed(LED_RED,MODE_OFF);
  setLed(LED_BLUE,MODE_OFF);
  setLed(LED_GREEN,MODE_OFF);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(BOARD_RGB_RED, LOW);
  digitalWrite(BOARD_RGB_GREEN, LOW);
  digitalWrite(BOARD_RGB_BLUE, LOW);

  // Reset cycle flags
  post_rb = false;
  bcst_blocks = 0;
  com_acpt_blocks = 0;
  cycle_counter++;

  // Every 5th cycle, we update the RTC
  // if (cycle_counter % 5 == 0) {
  //   updateRTC();
  //   sprintf(medium_buf, "Current Time: %02d:%02d:%02d", hour, min, sec); pcln(medium_buf);
  // }

  // Set the default next state
  state->next = sleep_loop;

  // If we are the base...
  if (isBase) {
    // Go ahead and queue an xmit auth message to send to the network when
    // we get going. Only send a new time correction every 25 cycles
    boolean sendTime = false; //cycle_counter % TRANSMIT_AUTH_TIME_INTERVAL == 0;
    // if (sendTime) {
    //   updateRTC();
    //   sprintf(medium_buf, "Sending Time: %02d:%02d:%02d", hour, min, sec); pcln(medium_buf, C_GREEN);
    // }
    queueTransmitAuth(sendTime);

    // Make come exceptions to the transmit rules for the base
    if (cycleValid <= 0) {
      cycleStart = micros();
      cycleValid = 10;
    }
    if (transmitAuthorization <= 0) {
      transmitAuthorization = TRANSMIT_AUTH_CAP;
    }
  }

  // If our cycle is valid here at the start of sleep, then go ahead and set our
  // sleep timeout. Otherwise we will have to wait until we can validate it
  // later.
  if (cycleValid > 0 && transmitAuthorization > 0) {
    // Step the cycle start forward
    cycleStart += settings.t_c;

    // If we are not the base station, then we want to send some information
    // about battery and temperature to the network
    // if (!isBase) {
    byte temp = 0;
    byte vbat = 0;
    DW1000.getTempAndVbatByte(temp, vbat);

    // Get our current heading from the mag/acell sensor
    int heading = 0xFFFF;
    if (magEnabled) {
      sensors_event_t event;
      mag.getEvent(&event);
      heading = (int) ((atan2(event.magnetic.z,event.magnetic.x) * 180) / 3.14159f);
      if (heading < 0) {
        heading = 360 + heading;
      }
    }

    // Build and queue a status message
    stats_msg sm = {.from = nodeNumber, .seq = getMsgSeq(), .hops = ALLOWABLE_HOPS, .bat = getBattVoltageBits(), .temp = temp, .heading = heading};
    if (!putPacketInBuffer((various_msg *)&sm, STATUS_PACKET)) {
      pcln("Packet Buffer Full. Overwrote Message", C_RED);
    }
    // }

    // Set the frame timer for the sleep frame, now that we are valid and
    // authorized
    boolean prompt = setFrameTimer(settings.t_fs, cycleStart, true);

    // Decrement our cycle valid value
    cycleValid --;
    if (cycleValid < 0)
      cycleValid = 0;

    // Decrement our transmit authorization value
    transmitAuthorization --;
    if (transmitAuthorization < 0)
      transmitAuthorization = 0;

    // If we change states now, then blink the green light
    if (!prompt)
      setLed(LED_GREEN,MODE_CHIRP);
    setLed(LED_BLUE, MODE_OFF);
  }

  // If we are ot valid or we do not have transmit authorization, then set
  // the blue light to mode_blink
  else {
    setLed(LED_BLUE, MODE_BLINK);
  }

  // Print our current cycle valid and transmit authorization values
  sprintf(medium_buf, "Cycle Valid Value: %d", cycleValid); pcln(medium_buf);
  sprintf(medium_buf, "Transmit Authorization Value: %d", transmitAuthorization); pcln(medium_buf);

  // If we are are the base station, then we want to serial all of our
  // data out
  if (isBase) {
    handleSerial();
  }

  // Move to the sleep loop state
  dec();
  pcln("[State] SLEEP_LOOP", C_ORANGE);
  inc();
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
  holdWatchdog();
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
  holdWatchdog();
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
  holdWatchdog();
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
          // Serial.println(dist);
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
  // Serial.print("Calculated Delay:"); Serial.println(delay);
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

  // If we have had too many com acpt blocks, then move on
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
  holdWatchdog();
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

  // If we have had too many com acpt blocks, then move on
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

  // if (isBase) {
  //   pcln("Excempt from bcast");
  //   state->next = com_acpt;
  //   dec();
  //   bcst_blocks++;
  //   return;
  // }

  // If we have had too many com acpt blocks, then move on
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

  // Check if the TX maessage has sent
  if(checkTx()) {
    pcln("Sent Message", C_GREEN);
    state->next = com_acpt;
    dec();
    bcst_blocks++;
    return;
  }
}
