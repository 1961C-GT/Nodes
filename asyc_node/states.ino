// ========================================================================== //
// STATE: RA_INIT
//
// -------------------------------------------------------------------------- //
state_fn ra_init, ra_init_loop;
void ra_init(struct State * state)
{
  blinkInit();
  #ifdef DEBUG
    //Serial.println(F("[State] RA_INIT_ENTER"));
  #endif

  // Set timer that tells us when we go to
  M0Timer.start(settings.t_br, _BLOCK_TIMER);

  // Start RX Mode
  receiver();


  // Proceed to loop state
  #ifdef DEBUG
    //Serial.println(F("[State] RA_INIT_LOOP"));
  #endif

  blinkLoop();
  state->next = ra_init_loop;
}
void ra_init_loop(struct State * state)
{
  #ifdef DEBUG
    // Serial.println(F("[State] RA_INIT_LOOP"));
  #endif

  if (M0Timer.getFired(_BLOCK_TIMER)) {
    Serial.println("Soft Reset");
    // reciever();

    state->next = ra_init;
    return;
  }

  if (checkRx()) {
    state->next = ra_decode;
    return;
  }

  // while(!checkRx());
  // state->next = ra_decode;
  // ra_decode(state);



  // DW1000.readSystemEventStatusRegister();
  // if (DW1000.isReceiveFailed()) {
  //   Serial.println("REC Fail");
  // }

}
// ========================================================================== //


// ========================================================================== //
// STATE: RA_DECODE
//
// -------------------------------------------------------------------------- //
state_fn ra_decode;
void ra_decode(struct State * state)
{
  blinkInit();
  #ifdef DEBUG
    //Serial.print(F("[State] RA_Decode: "));
  #endif

  // Set the defualt next state
  state->next = ra_init;

  // uint32_t val = SINCE(rxTime);
  // Serial.print("rx->decode: "); Serial.println(val);
  // tmpTime = micros();

  Message msg = getMessage();

  // val = SINCE(tmpTime);
  // Serial.print("decode->getMsg: "); Serial.println(val);
  // tmpTime = micros();

  #ifdef DEBUG
    // Serial.print("Rx @: "); Serial.print(rxTime); Serial.print(' ');
    // printMessage(msg);
  #endif

  if (msg.valid) {
    switch(msg.type){
      case RANGE_REQ:
        // delay(100);
        // Serial.println(t1);

        // Store the time we got the recieve timestamp
        DW1000.getReceiveTimestamp(timePollSent);

        state->next = ra_resp;
        break;
      case SETTINGS:
        // TODO
      default:
        Serial.println("BAD MESSAGE TYPE");
        break;
    }
  }
}
// ========================================================================== //


// ========================================================================== //
// STATE: RA_RESP
//
// -------------------------------------------------------------------------- //
state_fn ra_resp, ra_resp_loop;
void ra_resp(struct State * state)
{
  blinkTx();
  #ifdef DEBUG
    //Serial.println(F("[State] RA_RESP_ENTER"));
  #endif

  // uint32_t val = SINCE(tmpTime);
  // Serial.print("getMsg->ra_resp: "); Serial.println(val);
  // tmpTime = micros();

  DW1000.newTransmit();
  DW1000.setDefaults();

  // val = SINCE(tmpTime);
  // Serial.print("ra_resp->defaults: "); Serial.println(val);
  // tmpTime = micros();

  Message msg;
  msg.from = nodeNumber;
  msg.seq = 2; // TODO CALCULATE SEQ
  msg.type = RANGE_RESP;
  msg.len = 0;

  uint8_t len = getMessageLength(msg);
  byte data[len];
  createMessage(data, msg);

  // val = SINCE(tmpTime);
  // Serial.print("defaults->createMessage: "); Serial.println(val);
  // tmpTime = micros();


  // Serial.print("T:"); Serial.println(getRangeRecDelay(nodeNumber, settings));
  DW1000.setDelayFromRx(msgDelay); // DW1000Time(settings.t_rn, DW1000Time::MICROSECONDS)

  // val = SINCE(tmpTime);
  // Serial.print("createMessage->delay: "); Serial.println(val);
  // tmpTime = micros();

  DW1000.setData(data, len);

  // val = SINCE(tmpTime);
  // Serial.print("delay->setData: "); Serial.println(val);
  // tmpTime = micros();

  DW1000.startTransmit();

  // val = SINCE(tmpTime);
  // Serial.print("setData->xmit: "); Serial.println(val);
  // tmpTime = micros();

  // uint32_t val = SINCE(rxTime);

  // Proceed to loop state
  #ifdef DEBUG
    //Serial.println(F("[State] RA_RESP_LOOP"));
  #endif
  state->next = ra_resp_loop;
}
void ra_resp_loop(struct State * state)
{
  #ifdef DEBUG
    //Serial.println(F("[State] RA_RESP_LOOP"));
  #endif

  if(checkTx()) {
    // DW1000.readSystemEventStatusRegister();
    // Serial.print("Late: "); Serial.println(DW1000.isLate());

    DW1000Time txTime;
    DW1000.getTransmitTimestamp(txTime);

    DW1000Time turn = (txTime - timePollSent).wrap();
    Serial.print("Turn: "); Serial.print(turn.getAsNanoSeconds());
    Serial.print("ns, Goal: "); Serial.println(msgDelay.getAsMicroSeconds());
    // Serial.print("ns, TX:"); Serial.print(txTime.getAsMicroSeconds());
    // Serial.print("ns, ");

    receiver();
    state->next = ra_init;
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
  #ifdef DEBUG
    //Serial.println(F("[State] RB_RANGE_ENTER"));
  #endif

  // Set our block timer just incase we have an issue transmitting. This will
  // pull us out of this block
  M0Timer.start(settings.t_br, _BLOCK_TIMER);

  // Configure out LED
  blinkTx();

  // Start a RANGE_REQ Xmit
  DW1000.newTransmit();
  DW1000.setDefaults();
  Message msg;
  msg.from = 0x2;
  msg.seq = 2;
  msg.type = RANGE_REQ;
  msg.len = 0;
  uint8_t len = getMessageLength(msg);
  byte data[len];
  createMessage(data, msg);
  // DW1000.setDelay(msgDelay);
  DW1000.setData(data, len);
  DW1000.startTransmit();

  // Proceed to loop state
  #ifdef DEBUG
    // Serial.println(F("[State] RB_RANGE_LOOP"));
  #endif
  state->next = rb_range_loop;
}
void rb_range_loop(struct State * state)
{
  #ifdef DEBUG
    // Serial.println(F("[State] RB_RANGE_LOOP"));
  #endif

  if(checkTx()) {

    // TODO: Store the exact DW1000 time at which our message was sent
    DW1000.getTransmitTimestamp(timePollSent);

    receiver();
    state->next = rb_rec;
    return;
  }

  // ## BLOCK TIMER ## //
  if (M0Timer.getFired(_BLOCK_TIMER)) {
    Serial.println("Block Timeout - RB_RANGE_LOOP");
    // In this case, this will force us to send another message
    state->next = rb_range;
    return;
  }
}
// ========================================================================== //


// ========================================================================== //
// STATE: RB_REC
//
// -------------------------------------------------------------------------- //
state_fn rb_rec, rb_rec_loop;
void rb_rec(struct State * state)
{

  // Reset our block timer based on the time at which our transmission sent
  // (which is the NEW block sync)
  M0Timer.start(settings.t_br, _BLOCK_TIMER, txTime);

  blinkInit();
  #ifdef DEBUG
    //Serial.println(F("[State] RB_REC_ENTER"));
  #endif

  receiver();

  // ## Check our BLOCK TIMER ## //
  if (M0Timer.getFired(_BLOCK_TIMER)) {
    Serial.println("Block Timeout - RB_REC_ENTER");
    // In this case, this will force us to send another message
    state->next = rb_range;
    return;
  }

  // Proceed to loop state
  #ifdef DEBUG
    //Serial.println(F("[State] RB_REC_LOOP"));
  #endif
  blinkLoop();
  state->next = rb_rec_loop;
}
void rb_rec_loop(struct State * state)
{
  #ifdef DEBUG
    // Serial.println(F("[State] RB_REC_LOOP"));
  #endif

  if(checkRx()) {
    state->next = rb_decode;
    return;
  }

  // ## BLOCK TIMER ## //
  if (M0Timer.getFired(_BLOCK_TIMER)) {
    // Serial.println("Block Timeout - RB_RANGE_LOOP");
    state->next = rb_range;
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
  blinkInit();
  #ifdef DEBUG
    // Serial.print(F("[State] RB_DECODE: "));
  #endif

  // Set the defualt next state
  state->next = rb_rec;

  Message msg = getMessage();

  if (msg.valid) {
    switch(msg.type){
      case RANGE_RESP:
        // state->next = rb_range;
        // TODO Store this information

        // printMessage(msg);

        if (msg.from < settings.n) {
          DW1000Time rec;
          DW1000.getReceiveTimestamp(rec);
          float dist = computeRange(rec, msg.from);

          // Serial.print("Distance("); Serial.print(msg.from); Serial.print("): "); Serial.println(dist);


        } else {
          Serial.print("Out of Bounds Node ID: "); Serial.println(msg.from);
        }

        break;
      case SETTINGS:
        // TODO
      default:
        Serial.println("BAD MESSAGE TYPE");
        break;
    }
  }

  // ## Check our BLOCK TIMER ## //
  if (M0Timer.getFired(_BLOCK_TIMER)) {
    // Serial.println("Block Timeout - RB_DECODE");
    state->next = rb_range;
    return;
  }
}
// ========================================================================== //
