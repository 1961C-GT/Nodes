// ========================================================================== //
// STATE: RA_INIT
//
// -------------------------------------------------------------------------- //
state_fn ra_init, ra_init_loop;
void ra_init(struct State * state)
{
  #ifdef DEBUG
    Serial.println(F("[State] RA_INIT_ENTER"));
  #endif

  // Set timer that tells us when we go to
  // M0Timer.stopTimer(3);
  // M0Timer.startTimer(500, 3);

  // Start RX Mode
  receiver();


  // Proceed to loop state
  #ifdef DEBUG
    Serial.println(F("[State] RA_INIT_LOOP"));
  #endif
  state->next = ra_init_loop;
}
void ra_init_loop(struct State * state)
{
  #ifdef DEBUG
    // Serial.println(F("[State] RA_INIT_LOOP"));
  #endif

  // if (M0Timer.getTC3Fired()) {
  //   state->next = rb_range;
  // }

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
  #ifdef DEBUG
    Serial.print(F("[State] RA_Decode: "));
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
    printMessage(msg);
  #endif

  if (msg.valid) {
    switch(msg.type){
      case RANGE_REQ:
        // delay(100);
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
  #ifdef DEBUG
    Serial.println(F("[State] RA_RESP_ENTER"));
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
  msg.from = 0x1;
  msg.seq = 2;
  msg.type = RANGE_RESP;
  msg.len = 0;

  uint8_t len = getMessageLength(msg);
  byte data[len];
  createMessage(data, msg);

  // val = SINCE(tmpTime);
  // Serial.print("defaults->createMessage: "); Serial.println(val);
  // tmpTime = micros();

  DW1000.setDelayFromRx(msgDelay);

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

  uint32_t val = SINCE(rxTime);
  Serial.print("Time Since RX: "); Serial.println(val);

  // Proceed to loop state
  #ifdef DEBUG
    Serial.println(F("[State] RA_RESP_LOOP"));
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
    Serial.println(F("[State] RB_RANGE_ENTER"));
  #endif

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

  uint32_t val = SINCE(rxTime);
  Serial.print("Time Since RX: "); Serial.println(val);

  // SET UP THE BLOCK END TIMER

  // Proceed to loop state
  #ifdef DEBUG
    Serial.println(F("[State] RB_RANGE_LOOP"));
  #endif
  state->next = rb_range_loop;
}
void rb_range_loop(struct State * state)
{
  #ifdef DEBUG
    // Serial.println(F("[State] RB_RANGE_LOOP"));
  #endif

  if(checkTx()) {
    // DW1000.readSystemEventStatusRegister();
    // Serial.print("Late: "); Serial.println(DW1000.isLate());
    receiver();
    state->next = rb_rec;
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
  #ifdef DEBUG
    Serial.println(F("[State] RB_REC_ENTER"));
  #endif

  receiver();

  M0Timer.stopTimer(3);
  M0Timer.startTimer(10, 3);

  // Proceed to loop state
  #ifdef DEBUG
    Serial.println(F("[State] RB_REC_LOOP"));
  #endif
  state->next = rb_rec_loop;
}
void rb_rec_loop(struct State * state)
{
  #ifdef DEBUG
    // Serial.println(F("[State] RB_REC_LOOP"));
  #endif

  if (M0Timer.getTC3Fired()) {
    Serial.println("Timeout");
    state->next = rb_range;
  }

  if(checkRx()) {
    state->next = rb_decode;
    return;
  }

  // DW1000.readSystemEventStatusRegister();
  // if (DW1000.isReceiveFailed()) {
  //   Serial.println("REC Fail");
  // }
}
// ========================================================================== //

// ========================================================================== //
// STATE: RB_DECODE
//
// -------------------------------------------------------------------------- //
state_fn rb_decode;
void rb_decode(struct State * state)
{
  #ifdef DEBUG
    Serial.print(F("[State] RB_Decode: "));
  #endif

  // Set the defualt next state
  state->next = rb_rec;

  Message msg = getMessage();

  #ifdef DEBUG
    printMessage(msg);
  #endif

  if (msg.valid) {
    switch(msg.type){
      case RANGE_RESP:
        // delay(100);
        state->next = rb_range;
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