/////////////////////////////////////
// uart0
// tx put routine
void uart0_put (uint8_t d) {
  Serial.write(d);
}

// print a unsigned 32-bit value
void printValue0_U32(uint32_t msec) {
  uint8_t pr = 0;
  uint8_t d = 0;
  while (msec > 999999999) {
    d++;
    msec -= 1000000000;
  }
  if (pr || d) {
    uart0_put(d+0x30);
    pr = 1;
  }

  d = 0;
  while (msec > 99999999) {
    d++;
    msec -= 100000000;
  }
  if (pr || d) {
    uart0_put(d+0x30);
    pr = 1;
  }

  d = 0;
  while (msec > 9999999) {
    d++;
    msec -= 10000000;
  }
  if (pr || d) {
    uart0_put(d+0x30);
    pr = 1;
  }

  d = 0;
  while (msec > 999999) {
    d++;
    msec -= 1000000;
  }
  if (pr || d) {
    uart0_put(d+0x30);
    pr = 1;
  }

  d = 0;
  while (msec > 99999) {
    d++;
    msec -= 100000;
  }
  if (pr || d) {
    uart0_put(d+0x30);
    pr = 1;
  }

  d = 0;
  while (msec > 9999) {
    d++;
    msec -= 10000;
  }
  if (pr || d) {
    uart0_put(d+0x30);
    pr = 1;
  }

  d = 0;
  while (msec > 999) {
    d++;
    msec -= 1000;
  }
  if (pr || d) {
    uart0_put(d+0x30);
    pr = 1;
  }
  
  d = 0;
  while (msec > 99) {
    d++;
    msec -= 100;
  }
  if (pr || d) {
    uart0_put(d+0x30);
    pr = 1;
  }

  d = 0;
  while (msec > 9) {
    d++;
    msec -= 10;
  }
  if (pr || d)
    uart0_put(d+0x30);

  d = msec;
  uart0_put(d+0x30);
}

// print new-line (CR,LF)
void printNewline0() {
  uart0_put(0x0d);
  uart0_put(0x0a);
}

// print a string
void printString0(char *line) {
  while (*line)
    uart0_put(*line++);
}

// print a string + new-line
void printlnString0(char *line) {
  while (*line)
    uart0_put(*line++);
  printNewline0();
}


/////////////////////////////////////
// uart1 - this uart is used to send output commands to MouseoVeR
void uart1_put (uint8_t d) {
  Serial1.write(d);
}

// print new-line (CR,LF) 
void printNewline1() {
  uart1_put(0x0d); // CR line
//  uart1_put(0x0a); // LF line
  
  // MouseoVeR uses QChar (a QT app) to parse the strings.
  // QChar only requires the CR, and not the LF
}

// print a string
void printString1(char *line) {
  while (*line)
    uart1_put(*line++);
}

// print a string + new-line
void printlnString1(char *line) {
  while (*line)
    uart1_put(*line++);
  printNewline1();
}



/////////////////////////////////////
// uart2
void uart2_put (uint8_t d) {
  Serial2.write(d);
}


/////////////////////////////////////
// uart3
void uart3_put (uint8_t d) {
  Serial3.write(d);
}

