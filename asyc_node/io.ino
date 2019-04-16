int indents = 0;
int sectionIndents = 0;
boolean isSection = false;

void inc() {
  if (isBase)
    return;
    
  indents ++;
}

void inc(int level) {
  if (isBase)
    return;

  indents += level;
}

void dec() {
  if (isBase)
    return;

  indents --;
  if (indents < 0)
    indents = 0;
  if (isSection && indents < sectionIndents)
    indents = sectionIndents;
}

void dec(int level) {
  if (isBase)
    return;

  indents -= level;
  if (indents < 0)
    indents = 0;
  if (isSection && indents < sectionIndents)
    indents = sectionIndents;
}

void rst() {
  if (isBase)
    return;

  indents = 0;
  if (isSection && indents < sectionIndents)
    indents = sectionIndents;
}

void rst(int level) {
  if (isBase)
    return;

  indents = level;
  if (isSection && indents < sectionIndents)
    indents = sectionIndents;
}

void pind() {
  if (isBase)
    return;

  pind(indents);
}

void pind(int num) {
  if (isBase)
    return;

  if (num > 0) {
    uint8_t indentstr[num*2];
    for (int i = 0; i < num*2; i++) {
      indentstr[i] = (uint8_t) 32;
    }
    Serial.write(indentstr, num*2);
  }
}

void header(char * str, char * fg, char * bg) {
  if (isBase)
    return;

  sprintf(large_buf, "\n\r%s→ %s%s %-80s%s\n\r", C_PURPLE, bg, fg, str, D_CLEAR);
  Serial.println(large_buf);
}

void pcln(char * str) {
  if (isBase)
    return;

  if (DEBUG >= indents) {
    if (isSection) {
      pind(sectionIndents);
      Serial.print("| ");
      if (indents > sectionIndents)
        pind(indents - sectionIndents);
      Serial.println(str);
    } else {
      pind();
      Serial.println(str);
    }
  }
}

void pcln(char * str, char * color) {
  if (isBase)
    return;

  if (DEBUG >= indents) {
    if (isSection) {
      pind(sectionIndents);
      Serial.print("| ");
      if (indents > sectionIndents)
        pind(indents - sectionIndents);
      sprintf(large_buf, "%s%s%s", color, str, D_CLEAR);
      Serial.println(large_buf);
    } else {
      pind();
      sprintf(large_buf, "%s%s%s", color, str, D_CLEAR);
      Serial.println(large_buf);
    }
  }
}

void section(char * title) {
  if (isBase)
    return;

  if (DEBUG >= indents) {
    sectionIndents = indents;
    isSection = true;
    Serial.print(D_CLEAR);
    pind(sectionIndents);
    Serial.print("> ");
    Serial.println(title);
  }
}

void section(char * title, char * color) {
  if (isBase)
    return;

  if (DEBUG >= indents) {
    isSection = true;
    sectionIndents = indents;
    Serial.print(D_CLEAR);
    pind(sectionIndents);
    sprintf(large_buf, "> %s%s%s", color, title, D_CLEAR);
    Serial.println(large_buf);
  }
}

// void scln(char * str) {
//
//   pind();
//   sprintf(large_buf, " | %s", str);
//   Serial.println(large_buf);
// }
//
// void scln(char * str, char * color) {
//   pind();
//   sprintf(large_buf, " | %s%s%s", color, str, D_CLEAR);
//   Serial.println(large_buf);
// }

void endSection(){
  if (isBase)
    return;

  if (DEBUG >= sectionIndents) {
    isSection = false;
    indents = sectionIndents;
  }
}

void endSection(char * result){
  if (isBase)
    return;

  if (DEBUG >= sectionIndents) {
    Serial.print(D_CLEAR);
    pind();
    sprintf(large_buf, "| → %s", result);
    Serial.println(large_buf);
    isSection = false;
    indents = sectionIndents;
  }
}

void endSection(char * result, char * color){
  if (isBase)
    return;

  if (DEBUG >= sectionIndents) {
    Serial.print(D_CLEAR);
    pind();
    sprintf(large_buf, "| → %s%s%s", color, result, D_CLEAR);
    Serial.println(large_buf);
    isSection = false;
    indents = sectionIndents;
  }
}
