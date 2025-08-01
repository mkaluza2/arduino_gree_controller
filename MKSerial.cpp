#include "MKSerial.h"
#include <util/parity.h>

#define RX_BUF_SIZE 20
#define RX_IDLE 0
#define RX_START 0x1
#define RX_PARITY 0x10
#define RX_STOP 0x30

static uint8_t data_bits;
static uint8_t ticks_per_bit;
static volatile uint8_t rx_state;
static volatile uint8_t rx_byte;
static volatile uint8_t rx_meta_byte;
static volatile uint8_t rx_buffer_head;
static volatile uint8_t rx_buffer_tail;
static volatile uint8_t rx_buffer[RX_BUF_SIZE];
static volatile uint8_t rx_parity_ok;
static volatile uint8_t parity_errors;

#define TX_BUF_SIZE 20
#define TX_START 0x1
#define TX_PARITY 0x10
#define TX_STOP 0x30

static volatile uint8_t tx_state=0;
static volatile uint8_t tx_byte=0;
static volatile uint8_t tx_buffer_head=0;
static volatile uint8_t tx_buffer_tail=0;
static volatile uint8_t tx_buffer[TX_BUF_SIZE];
static uint8_t tx_parity;

static uint8_t rx_wait = 0;
//RX

ISR(INT1_vect)
{
  if (rx_wait) {
    rx_wait = 0;
    bitClear(EICRA, ISC10); //don't trigger on raising edge but on falling edge, wait for start bit
    return;
  }
  rx_state = RX_START;
  OCR2B = TCNT2 + (ticks_per_bit >> 1);
  bitSet(TIFR2, OCF2B);   //clear int flag
  bitSet(TIMSK2, OCIE2B); //timer int enable
  bitClear(EIMSK, INT1);  //disable int1
}

ISR(TIMER2_COMPB_vect)
{
  uint8_t state, byte, rx_bit, head;
  state = rx_state;
  rx_bit = (PIND & bit(PD3)) << 4;
  if (state == RX_START) {
    //start bit
    state++;
    rx_byte = 0;
    rx_meta_byte = rx_bit;

    goto next;
  }
  if (state <= (data_bits + RX_START)) {
    //data bits
    byte = rx_byte;
    rx_byte = (byte >> 1) | rx_bit;
    state++;
    goto next;
  }
  if (state < RX_PARITY) {
    //parity bit
    state = RX_PARITY;
    rx_meta_byte = (rx_meta_byte >> 1) | rx_bit;
    //TODO check parity
    rx_parity_ok = (parity_even_bit(rx_byte) == (rx_bit >> 7));
    goto next;
  }
  if (state == RX_PARITY) {
    //stop bit
    rx_meta_byte = (rx_meta_byte >> 1) | rx_bit;
    //TODO check errors
    
    if ((rx_byte == 0xFF && rx_meta_byte == 0xE0) || (rx_byte == 0 && rx_meta_byte == 0)) {
      //idle
      bitClear(TIMSK2, OCIE2B); //timer int disable
      bitSet(EIFR, INTF1); //clear flag
      bitSet(EIMSK, INT1); //enable int1
      rx_state = RX_IDLE;
      return;
    }

    if (rx_parity_ok) {
    	head = rx_buffer_head + 1;
    	if (head >= RX_BUF_SIZE) head = 0;
    	if (head != rx_buffer_tail) {
    		rx_buffer[head] = rx_byte;
    		rx_buffer_head = head;
    	} // else buf overflow
    } else parity_errors++;
    //in case there is some delay, but shouldn't be... and for resync
    bitSet(EIFR, INTF1); //clear flag
    bitSet(EIMSK, INT1); //enable int1

    state = RX_START; //timer int left on b/c waiting for another byte or idle
    /*
      bitClear(TIMSK2, OCIE2B); //timer int disable
      bitSet(EIFR, INTF1); //clear flag
      bitSet(EIMSK, INT1); //enable int1
      state = RX_IDLE;
      */
    goto next;
  }
next:
  OCR2B += ticks_per_bit;
  rx_state = state;
}


uint8_t MKSerial::available(void) {
    uint8_t head, tail;

    head = rx_buffer_head;
    tail = rx_buffer_tail;
    if (head >= tail) return head - tail;
    return RX_BUF_SIZE + head - tail;
}


uint8_t MKSerial::read(void) {
    uint8_t head, tail, out;

    head = rx_buffer_head;
    tail = rx_buffer_tail;
    if (head == tail) return -1;
    if (++tail >= RX_BUF_SIZE) tail = 0;
    out = rx_buffer[tail];
    rx_buffer_tail = tail;
    return out;
}


uint8_t MKSerial::rxIdle(void) {
    return rx_state == RX_IDLE;
}

uint8_t MKSerial::getParityErrors(uint8_t clear) {
    uint8_t err = parity_errors;
    if (clear) parity_errors = 0;
    return err;
}
// TX

ISR(TIMER2_COMPA_vect)
{  // triggered by a state change during transmission 
  // there's now at least one bit time for the next interrupt 
  uint8_t state, byte, bit, head, tail;
  uint16_t target;
  state = tx_state;  // 1 = startbit has gone LOW right now 
                      // 2 .. 9 = databit (0..7) a change has happened 
                      // 10 = parity bit has started 
                      // 11 = stopbits have just started ( 1 or 2 stopbits )
                      // 20 = stopbit time has elapsed, 
                      //      if there's more to send that can be done immediately
                      //      else stop timer and set tx_state to 0 
  if (state == TX_START) {
    tail = tx_buffer_tail;
    byte = tx_buffer[tail];
    tx_parity = parity_even_bit(byte);
  } else {
    byte = tx_byte;
  }
  if (state < (data_bits + TX_START)) {
    state++;
    bit = byte & 1;
    byte >>= 1;

    if (bit)  bitSet(TCCR2A, COM2A0); //out=1
    else      bitClear(TCCR2A, COM2A0); //out=0

    tx_byte = byte;
    tx_state = state;
    goto next;
  }
  //  no more change for the remaining data bits
  if (state < TX_PARITY) {  // parity not yet handled
    // PORTD |= 0x80;  // DEBUG: Here we add parity info

    if (tx_parity)  bitSet(TCCR2A, COM2A0); //out=1
    else            bitClear(TCCR2A, COM2A0); //out=0
    tx_state = TX_PARITY;
    // PORTD &= ~0x80;  // DEBUG: ISR done
    goto next;
  }
  if (state == TX_PARITY) {
    bitSet(TCCR2A, COM2A0); //out=1, stop bit
    tx_state = TX_STOP;
    goto next;
  }

  head = tx_buffer_head;
  tail = tx_buffer_tail;
  if (head == tail) {
    tx_state = 0;
    //TCCR2A = (TCCR2A & ~bit(COM2A1)) & (~bit(COM2A0));
    bitClear(TCCR2A, COM2A0); //out=0
    bitClear(TCCR2A, COM2A1); //comp normal
    bitClear(TIMSK2, OCIE2A); //int disable
    //TCCR2B &= ~(0x7);  //clock off
     goto next;
  }
  else {
    tx_state = TX_START;
    if (++tail >= TX_BUF_SIZE) tail = 0;
    tx_buffer_tail = tail;
    bitClear(TCCR2A, COM2A0); //out=0
  }
next:
  OCR2A += ticks_per_bit;
}


uint8_t MKSerial::getTXstate(void) {
  return tx_state;
}


void MKSerial::rxOn(void) {
  wait(WAIT_FALLING);
}

void MKSerial::rxOff(void) {
  bitClear(EIMSK, INT1);  //disable int1
}

uint8_t MKSerial::txIdle(void) {
  return tx_state == 0;
}


void MKSerial::write(uint8_t b) {
  uint8_t intr_state, head;

  head = tx_buffer_head + 1;
  if (head >= TX_BUF_SIZE) head = 0;
  while (tx_buffer_tail == head) ; // wait until space in buffer
  intr_state = SREG;
  cli();
  tx_buffer[head] = b;
  tx_buffer_head = head;
  if (!tx_state) {
    tx_state = TX_START;
    tx_buffer_tail = head;

    OCR2A = TCNT2 + (ticks_per_bit >> 1);    //can be less
    TCCR2A = (TCCR2A | bit(COM2A1)) & (~bit(COM2A0));
    //bitClear(TCCR2A, COM2A0); //out=0
    //bitSet(TCCR2A, COM2A1);  //enable Clear(0)/Set(1) OC2A on compare match   (COM2A0 determines the actual action)
    //TCCR2A &= ~bit(COM2A0);
    //TCCR2A |= bit(COM2A1);


    bitSet(TIFR2, OCF2A);   //clear int flag
    bitSet(TIMSK2, OCIE2A); //int enable
  }
  SREG = intr_state;
}

//min br is 62bps, max probably 9600-14400 (1666-1111 Clock ticks per bit...)
void MKSerial::begin(unsigned long br) {
  data_bits = 8;
  parity_errors = 0;

  TCCR2A = 0;
  TCCR2B = 0;
  //TX
  //bitSet(TCCR2A, COM2A1);  //enable Clear(0)/Set(1) OC2A on compare match   (COM2A0 determines the actual action)
  TCCR2A |= bit(COM2A1) | bit(COM2A0);	//set initial output compare to HIGH
  bitSet(TCCR2B, FOC2A);
  TCCR2A = 0;

  digitalWrite(11, HIGH);  //TX out
  pinMode(11, OUTPUT);  //TX out
  pinMode(3, INPUT_PULLUP);  //RX

  unsigned long f = F_CPU/br;
  int i = 0;
  const uint8_t divs[6] = {3, 2, 1, 1, 1, 2};  //shift right
  for (i=0; i < 6 && f > 255; i++) f = f >> divs[i];
  Serial.println(F("MKSerial begin: "));
  Serial.println(i);
  Serial.println(f);
  ticks_per_bit = f;
  TCCR2B |= (i+1);	//set prescaller CS22-CS20

  // RX
  bitSet(EICRA, ISC11); //trigger on falling edge
  bitSet(EIFR, INTF1); //clear flag
  bitSet(EIMSK, INT1); //enable int1
}


void MKSerial::wait(uint8_t state) {		//0 - falling, 1 - raising
  // RX
  rx_wait = state;
  bitSet(EICRA, ISC11); //trigger on falling edge
  if (state == WAIT_RISING) bitSet(EICRA, ISC10); //trigger on raising edge
  bitSet(EIFR, INTF1); //clear flag
  bitSet(EIMSK, INT1); //enable int1
}

void MKSerial::debugRX(void) {
  Serial.print(rx_state);
  Serial.print(" ");
  Serial.print(rx_buffer_head);
  Serial.print(" ");
  Serial.print(rx_buffer_tail);
  Serial.print(" ");
  Serial.print(OCR2B);
  Serial.print(" ");
  Serial.print(rx_byte);
  Serial.print(" ");
  Serial.print(rx_meta_byte);
  Serial.print(" ");
}


MKSerial::MKSerial() {
}
