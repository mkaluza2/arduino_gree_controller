#ifndef MKSerial_h
#define MKSerial_h

#include "Arduino.h" 

#define WAIT_FALLING 0
#define WAIT_RISING 1

class MKSerial {
	public:
		MKSerial();
		void begin(unsigned long br);
		void rxOn(void);
		void rxOff(void);
		void wait(uint8_t state);
		uint8_t available();
		uint8_t read();
		void write(uint8_t b);
		void debugRX(void);
		uint8_t rxIdle(void);
		uint8_t txIdle(void);
		uint8_t getTXstate(void);
		uint8_t getParityErrors(uint8_t clear);
};

#endif
