#include <avr/pgmspace.h>
//#include <AltSoftSerial.h>
//#include <config/AltSoftSerial_Boards.h>

#include "MKSerial.h"

//AltSoftSerial mySerial;
MKSerial mkSerial;


#define FRAME_SIZE 20
#define FRAMES_INIT_NUM 3
#define FRAMES_OUT_NUM 2
#define FRAMES_IN_NUM 6
#define IGNORES_MAX 3

uint8_t frame_buf[FRAME_SIZE];
uint8_t buf_ptr = 0;

uint8_t odu_frame_buf[FRAME_SIZE];
uint8_t odu_buf_ptr = 0;


const uint8_t init_frames[FRAMES_INIT_NUM][FRAME_SIZE] = {
  {17, 33, 0, 8, 32, 25, 8, 0, 16, 0, 20, 23, 27, 0, 8, 0, 0, 0, 0, 217},
  {18, 33, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 51},
  {47, 33, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 80}
};

const PROGMEM uint8_t frames_out_const[FRAMES_OUT_NUM][FRAME_SIZE] = {
  {49, 33, 0, 3, 128, 0, 64, 66, 0, 66, 1, 0, 0, 0, 0, 0, 0, 0, 0, 195},
  {50, 33, 0, 128, 0, 0, 0, 0, 34, 0, 0, 0, 0, 0, 0, 16, 0, 0, 0, 5},
//  {64, 33, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 97},
};

bool isOn = false;
char fan = '1';

uint8_t frames_out[FRAMES_OUT_NUM][FRAME_SIZE];
uint8_t frames_out_ptr = 0;

uint8_t frames_in[FRAMES_IN_NUM][FRAME_SIZE];
uint8_t frames_in_max = 0;

void send_frame(const uint8_t *frame, uint8_t channel) {
  uint8_t i, s;
  mkSerial.write(frame[0]);
  i = (frame[1] & 0xF0) | (channel & 0xf);
  mkSerial.write(i);
  s = frame[0] + i;

  for (i = 2; i < FRAME_SIZE-1; i++) {
    mkSerial.write(frame[i]);
    s += frame[i];
  }
  mkSerial.write(s);
}

uint8_t checksum_frame(const uint8_t *frame) {
  uint8_t i, s = frame[0];
  for (i = 1; i < FRAME_SIZE-1; i++) {
    s += frame[i];
  }
  return s;
}


uint8_t check_frame(const uint8_t *frame) {
  return checksum_frame(frame) == frame[FRAME_SIZE-1];
}

void dump_frame(const uint8_t *frame, bool compact) {
  uint8_t i;
  for(i=0; i<FRAME_SIZE; i++) {
    if (!compact) {
      if (frame[i]<100) Serial.print(F(" "));
      if (frame[i]<10) Serial.print(F(" "));
    }
    Serial.print(frame[i]);
    Serial.print(F(" "));
  }
  Serial.println();
}


void dump_frame(const uint8_t *frame) {
  dump_frame(frame, true);
}

int8_t find_in_frame(const uint8_t *frame) {
  uint8_t i;
  for(i=0; i<frames_in_max; i++) {
    if (frames_in[i][0] == frame[0]) return i;
  }
  if (frames_in_max<FRAMES_IN_NUM) {
    memcpy(frames_in[frames_in_max++], frame, FRAME_SIZE);
  } else {
    Serial.println(F("Frame store full"));
  }
  return -1;
}


uint8_t find_out_frame(byte id, byte ch) {
  uint8_t i;
  for(i=0; i<FRAMES_OUT_NUM; i++) {
    if (frames_out[i][0] == id && (ch == 0 || frames_out[i][1] == ch)) return i;
  }
  return -1;
}

unsigned long int ignores[IGNORES_MAX] = {
  (49UL << 24) | (16UL << 16) | (1UL << 10),
  (50UL << 24) | (16UL << 16),
  (64UL << 24) | (16UL << 16),
};

unsigned long int frames_different(uint8_t *frame1, uint8_t *frame2) {
  byte i = 0;
  unsigned long diff = 0;
  unsigned long int sig = ((unsigned long int)frame1[0] << 24) | ((unsigned long int)(frame1[1] & 0xf0) << 16);
  for (i = 0; i < IGNORES_MAX; i++) {
    if ((ignores[i] & 0xfff00000) == sig) {
      sig = ignores[i] & 0xfffff;
      break;
    }
  }
  
  for (i = 2; i < FRAME_SIZE-1; i++) {
    if (frame1[i] != frame2[i]) {
      diff |= (1UL << i);
      if (sig & (1UL << i)) continue;
    }
  }
  return (diff & (~sig));
}

unsigned long int compare_frames(uint8_t *frame1, uint8_t *frame2) {
  byte i = 0;
  unsigned long diff = 0;
  unsigned long int sig = ((unsigned long int)frame1[0] << 24) | ((unsigned long int)(frame1[1] & 0xf0) << 16);
  for (i = 0; i < IGNORES_MAX; i++) {
    if ((ignores[i] & 0xfff00000) == sig) {
      sig = ignores[i] & 0xfffff;
      break;
    }
  }
  
  for (i = 2; i < FRAME_SIZE-1; i++) {
    if (frame1[i] != frame2[i]) {
      diff |= (1UL << i);
      if (sig & (1UL << i)) continue;
    }
  }
  if (!(diff & (~sig))) return 0;

  //               "000 001 002 003 004 005 006 007 008 009 010 011 012 013 014 015 016 017 018 019"
  Serial.println(F("          2   3   4   5   6   7   8   9  10  11  12  13  15  15  16  17  18  19"));
  dump_frame(frame1,false);
  dump_frame(frame2, false);
  for (i = 0; i < FRAME_SIZE-1; i++) {
    if (frame1[i] == frame2[i]) {
      Serial.print(F("    "));
    } else {
      Serial.print(F("  * "));
    }
  }
  Serial.println();
  return (diff & (~sig));
}


void init_out_frames(void) {
  memcpy_P(frames_out, frames_out_const, FRAMES_OUT_NUM*FRAME_SIZE);
}


void setup()
{
  Serial.begin(115200);
  // Open serial communications and wait for port to open:
  init_out_frames();
  memset(frames_in, 0, sizeof(frames_in));
  //mySerial.begin(1200, SERIAL_8E1);
  mkSerial.begin(1200);

  Serial.println(F("Setup complete"));
  return;
}

#define STATE_IDLE 0
#define STATE_RECV 1
#define STATE_WAIT2TX 2
#define STATE_TX 3


byte state = STATE_IDLE;

unsigned long rx_end;

char frame = 0;

char cmdline[128];
byte cmdlineptr = 0;


void handle_idu(void) {
  switch (state) {
    case STATE_IDLE:
      if (mkSerial.rxIdle()) return;
      buf_ptr = 0;
      state = STATE_RECV;

    case STATE_RECV:
      while (mkSerial.available()) {
        if (buf_ptr == FRAME_SIZE) {
          Serial.println(F("Frame buffer overflow"));
          state = STATE_IDLE;
          return;
        }
        frame_buf[buf_ptr++] = mkSerial.read();
      }
      if (!mkSerial.rxIdle()) return;
      rx_end = millis();
      if (buf_ptr < FRAME_SIZE) {
        if (buf_ptr > 0) {
          Serial.print(F("Frame size error: "));
          Serial.println(buf_ptr);
          dump_frame(frame_buf);
        }
        state = STATE_IDLE;
        return;
      }
      if (!check_frame(frame_buf)) {
        Serial.println(F("Checksum error"));
        dump_frame(frame_buf);
        state = STATE_IDLE;
        return;
      }

      state = STATE_WAIT2TX;
      frame = find_in_frame(frame_buf);
      if (frame != -1) {
        compare_frames(frames_in[frame], frame_buf);
        memcpy(frames_in[frame], frame_buf, FRAME_SIZE);
      } else {
        Serial.print(F("IDU NEW: "));
        dump_frame(frame_buf);
      }
      break;

    case STATE_WAIT2TX:
      {
      if (millis()-rx_end < 61) return; //70-9,1666ms for 'idle' byte
      //disable RX
      mkSerial.rxOff();
      uint8_t channel = frame_buf[1] & 0xf;
      if (frame_buf[0] < 49) {
        //init frames
        for(uint8_t i = 0; i < FRAMES_INIT_NUM; i++) {
          if (init_frames[i][0] == frame_buf[0]) {
            send_frame(init_frames[i], channel);
            state = STATE_TX;
            return;
          }
        }
        Serial.println(F("Init frame not found"));
        dump_frame(frame_buf);
        state = STATE_IDLE;
        return;
      } else {
        send_frame(frames_out[frames_out_ptr++], channel);
        if (frames_out_ptr == FRAMES_OUT_NUM) frames_out_ptr = 0;
      }
      state = STATE_TX;
      }
      break;

    case STATE_TX:
      if (!mkSerial.txIdle()) return;
      state = STATE_IDLE;
      mkSerial.rxOn();
      break;
  }
}


void process_command_set(void) {
  //frame_id [in_out_channel = 0] byte_num value
  byte i=0, b[4];
  char * ptr;
  for(i=0; i<4; i++) {
    ptr = strtok(NULL, " ");
    if (!ptr) {
      b[3] = b[2];
      b[2] = b[1];
      b[1] = 0;
      break;
    }
    b[i] = atoi(ptr);
  }
  i = find_out_frame(b[0], b[1]);
  if (i == -1) {
    Serial.print(F("Frame "));
    Serial.print(b[0]);
    Serial.print(" ");
    Serial.print(b[1]);
    Serial.println(F(" not found"));
    return;
  }
  frames_out[i][b[2]] = b[3];
  frames_out[i][FRAME_SIZE-1] = checksum_frame(frames_out[i]);

}


void process_command_on(void) {
  byte i;
  i = find_out_frame(50, 0);
  frames_out[i][3] = 0;
  frames_out[i][FRAME_SIZE-1] = checksum_frame(frames_out[i]);
  
  i = find_out_frame(49, 0);
  frames_out[i][3] = 19;  //mode=fan
  frames_out[i][4] = 1;  //fan speed = low
  
  frames_out[i][FRAME_SIZE-1] = checksum_frame(frames_out[i]);
}


void process_command_temp(void) {
  char * ptr = strtok(NULL, " ");

  byte i = find_out_frame(49, 0);
  byte off = 0;
  while (ptr != NULL) {
    if (off == 0) {
      if (strcmp(ptr, "s") == 0 || strcmp(ptr, "set") == 0) {
        off = 6;
      } else if (strcmp(ptr, "a") == 0 || strcmp(ptr, "air") == 0) {
        off = 7;
      } else if (strcmp(ptr, "e") == 0 || strcmp(ptr, "evap") == 0) {
        off = 9;
      } else {
        Serial.print(F("ERROR: Invalid temp selector: "));
        Serial.println(ptr);
        return;
      }
      ptr = strtok(NULL, " ");
    } else {
      //TODO process .5
      char *endptr;
      int m = strtol(ptr, &endptr, 10);
      if (ptr == endptr || *endptr != 0 || m > 255) {
        Serial.print(F("ERROR: invalid temp value: "));
        Serial.println(ptr);
        return;
      }
      frames_out[i][off] = m+40;
      off = 0;
      ptr = strtok(NULL, " ");
    }
  }
}


void process_command_mode(void) {
  char * ptr = strtok(NULL, " ");

  byte i = find_out_frame(49, 0);
  if (frames_out[i][3] == 3) process_command_on();  
  if (strcmp(ptr, "cool") == 0) {
    frames_out[i][3] = 17;
  } else if (strcmp(ptr, "heat") == 0) {
    frames_out[i][3] = 20;
  } else if (strcmp(ptr, "dry") == 0) {
    frames_out[i][3] = 18;
  } else if (strcmp(ptr, "fan") == 0) {
    frames_out[i][3] = 19;
  } else {
    char *endptr;
    unsigned long m = strtol(ptr, &endptr, 10);
    if (ptr == endptr || *endptr != 0 || m > 255) {
      Serial.print(F("ERROR: invalid mode: "));
      Serial.println(ptr);
      return;
    }
    frames_out[i][3] = m;
  }
}


void process_command_fan(void) {
  char * ptr = strtok(NULL, " ");

  byte i = find_out_frame(49, 0);
  if (frames_out[i][3] == 3) process_command_on();  

  frames_out[i][12] &= ~(1<<4);  //16
  if (strcmp(ptr, "t") == 0 || strcmp(ptr, "turbo") == 0) {
    frames_out[i][4] = 4;
  } else if (strcmp(ptr, "h") == 0 || strcmp(ptr, "high") == 0) {
    frames_out[i][4] = 3;
  } else if (strcmp(ptr, "m") == 0 || strcmp(ptr, "medium") == 0) {
    frames_out[i][4] = 2;
  } else if (strcmp(ptr, "l") == 0 || strcmp(ptr, "low") == 0) {
    frames_out[i][4] = 1;
  } else if (strcmp(ptr, "q") == 0 || strcmp(ptr, "quiet") == 0) {
    frames_out[i][4] = 7;
    frames_out[i][12] |= (1<<4);  //16
  } else {
    char *endptr;
    unsigned long m = strtol(ptr, &endptr, 10);
    if (ptr == endptr || *endptr != 0 || m > 255) {
      Serial.print(F("ERROR: invalid fan mode: "));
      Serial.println(ptr);
      return;
    }
    frames_out[i][4] = m;
  }
}


void process_command_odu(void) {
  char * ptr = strtok(NULL, " ");

  byte i = find_out_frame(49, 0);
  if (frames_out[i][3] != 24 && frames_out[i][3] != 25) {
    Serial.println(F("Need mode 24/25 to control ODU directly"));
    return;
  }
  byte off = 0;
  if (strcmp(ptr, "fan") == 0) {
    off = 13;
  } else if (strcmp(ptr, "comp") == 0) {
    off = 14;
  } else {
    Serial.print(F("ERROR: invalid param: "));
    Serial.println(ptr);
    return;
  }
  ptr = strtok(NULL, " ");
  if (!ptr) {
    Serial.println(F("ERROR: missing argument"));
  }
  char *endptr;
  unsigned long m = strtol(ptr, &endptr, 10);
  if (ptr == endptr || *endptr != 0 || m > 255) {
    Serial.print(F("ERROR: invalid argument: "));
    Serial.println(ptr);
    return;
  }
  frames_out[i][off] = m;
}


void process_command_dump(void) {
  char * ptr = strtok(NULL, " ");
  char src = *ptr;
  ptr = strtok(NULL, " ");
  byte num = atoi(ptr);

  if (src == 'o')
    dump_frame(frames_out[num]);
  else
    dump_frame(frames_in[num]);
}


void process_uptime(void) {
//  char buf[20];
  unsigned long t = millis();
  unsigned int ms = t % 1000;
  t = t/1000;
  Serial.print(F("Uptime: "));
  unsigned long days = t/(86400);
  if (days) {
    Serial.print(days, DEC);
    if (days == 1)
      Serial.print(F(" day "));
    else
      Serial.print(F(" days "));
  }
  t -= days * 86400;
  byte h = t/3600;
  if (h) {
    Serial.print(h, DEC);
    Serial.print(F(":"));
  }
  unsigned int t2 = t-h*3600;
  byte mi = t2/60;
  if (h || mi) {
    if (mi<10) Serial.print(F("0"));
    Serial.print(mi, DEC);
    Serial.print(F(":"));
  }
  byte s = t2 - mi * 60;
  if (s<10) Serial.print(F("0"));
  Serial.print(s, DEC);
  Serial.print(F("."));
  if (ms<100) Serial.print(F("0"));
  if (ms<10) Serial.print(F("0"));
  Serial.println(ms, DEC);
}

void process_command(char *buf) {
  Serial.print(F("Processing: "));
  Serial.println(buf);
  char * ptr = strtok(buf, " ");
  Serial.println(ptr);
  if (strcmp(ptr, "set") == 0) {
    process_command_set();
  } else if (strcmp(ptr, "on") == 0) {
    process_command_on();
  } else if (strcmp(ptr, "off") == 0) {
    init_out_frames();
  } else if (strcmp(ptr, "mode") == 0) {
    process_command_mode();
  } else if (strcmp(ptr, "fan") == 0) {
    process_command_fan();
  } else if (strcmp(ptr, "temp") == 0) {
    process_command_temp();
  } else if (strcmp(ptr, "dump") == 0) {
    process_command_dump();
  } else if (strcmp(ptr, "odu") == 0) {
    process_command_odu();
  } else if (strcmp(ptr, "uptime") == 0) {
    process_uptime();
  }
}


void handle_cli(void) {
  while (Serial.available()) {
    char b = Serial.read();
    if (cmdlineptr == 0 && (b <= ' ')) continue;

    if (b == ';' || b == '\n' || b == '\r') {
      if (!cmdlineptr) continue;
      cmdline[cmdlineptr] = 0;
      process_command(cmdline);
      cmdlineptr = 0;
    } else cmdline[cmdlineptr++] = b;
  }
}


void loop() // run over and over
{
  handle_idu();
  handle_cli();
}
