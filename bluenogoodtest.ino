

/* 
 drive remote control car wit bluetooth
 */
 
//#include "SoftwareSerial.h"


#include <util/delay_basic.h>

#ifndef _SS_MAX_RX_BUFF
#define _SS_MAX_RX_BUFF 64 // RX buffer size
#endif

const int BTBAUD = 19200;  //tank 115200, test 19200 //5 19200 //6 38400,  57600 7, 115200 8  AT+BAUD8
const int SERBAUD = 19200; //tank 115200, test 19200



uint8_t _receivePin;
  uint8_t _receiveBitMask;
  volatile uint8_t *_receivePortRegister;
  uint8_t _transmitBitMask;
  volatile uint8_t *_transmitPortRegister;
  volatile uint8_t *_pcint_maskreg;
  uint8_t _pcint_maskvalue;

  // Expressed as 4-cycle delays (must never be 0!)
  uint16_t _rx_delay_centering;
  uint16_t _rx_delay_intrabit;
  uint16_t _rx_delay_stopbit;
  uint16_t _tx_delay;


  // static data
  uint8_t _receive_buffer[_SS_MAX_RX_BUFF]; 
  volatile uint8_t _receive_buffer_tail;
  volatile uint8_t _receive_buffer_head;  

  // private methods
  //inline void recv() __attribute__((__always_inline__));
  uint8_t rx_pin_read();
  void setTX(uint8_t transmitPin);
  void setRX(uint8_t receivePin);

  bool _buffer_overflow;
  //inline void setRxIntMsk(bool enable) __attribute__((__always_inline__));

  // Return num - sub, or 1 if the result would be < 1
  //static uint16_t subtract_cap(uint16_t num, uint16_t sub);

  // private static method for timing
  //static inline void tunedDelay(uint16_t delay);
  

void setTX(uint8_t tx)
{
  // First write, then set output. If we do this the other way around,
  // the pin would be output low for a short while before switching to
  // output high. Now, it is input with pullup for a short while, which
  // is fine. With inverse logic, either order is fine.
  digitalWrite(tx, HIGH);
  pinMode(tx, OUTPUT);
  _transmitBitMask = digitalPinToBitMask(tx);
  uint8_t port = digitalPinToPort(tx);
  _transmitPortRegister = portOutputRegister(port);
}
void setRX(uint8_t rx)
{
  pinMode(rx, INPUT);
  digitalWrite(rx, HIGH);  // pullup for normal logic!
  _receivePin = rx;
  _receiveBitMask = digitalPinToBitMask(rx);
  uint8_t port = digitalPinToPort(rx);
  _receivePortRegister = portInputRegister(port);
}

uint16_t subtract_cap(uint16_t num, uint16_t sub) {
  if (num > sub)
    return num - sub;
  else
    return 1;
}

inline void tunedDelay(uint16_t delay) { 
  _delay_loop_2(delay);
}

void begin(long speed)
{
  _rx_delay_centering = _rx_delay_intrabit = _rx_delay_stopbit = _tx_delay = 0;

  // Precalculate the various delays, in number of 4-cycle delays
  uint16_t bit_delay = (F_CPU / speed) / 4;

  // 12 (gcc 4.8.2) or 13 (gcc 4.3.2) cycles from start bit to first bit,
  // 15 (gcc 4.8.2) or 16 (gcc 4.3.2) cycles between bits,
  // 12 (gcc 4.8.2) or 14 (gcc 4.3.2) cycles from last bit to stop bit
  // These are all close enough to just use 15 cycles, since the inter-bit
  // timings are the most critical (deviations stack 8 times)
  _tx_delay = subtract_cap(bit_delay, 15 / 4);

  // Only setup rx when we have a valid PCINT for this pin
  if (digitalPinToPCICR((int8_t)_receivePin)) {
    #if GCC_VERSION > 40800
    // Timings counted from gcc 4.8.2 output. This works up to 115200 on
    // 16Mhz and 57600 on 8Mhz.
    //
    // When the start bit occurs, there are 3 or 4 cycles before the
    // interrupt flag is set, 4 cycles before the PC is set to the right
    // interrupt vector address and the old PC is pushed on the stack,
    // and then 75 cycles of instructions (including the RJMP in the
    // ISR vector table) until the first delay. After the delay, there
    // are 17 more cycles until the pin value is read (excluding the
    // delay in the loop).
    // We want to have a total delay of 1.5 bit time. Inside the loop,
    // we already wait for 1 bit time - 23 cycles, so here we wait for
    // 0.5 bit time - (71 + 18 - 22) cycles.
    _rx_delay_centering = subtract_cap(bit_delay / 2, (4 + 4 + 75 + 17 - 23) / 4);

    // There are 23 cycles in each loop iteration (excluding the delay)
    _rx_delay_intrabit = subtract_cap(bit_delay, 23 / 4);

    // There are 37 cycles from the last bit read to the start of
    // stopbit delay and 11 cycles from the delay until the interrupt
    // mask is enabled again (which _must_ happen during the stopbit).
    // This delay aims at 3/4 of a bit time, meaning the end of the
    // delay will be at 1/4th of the stopbit. This allows some extra
    // time for ISR cleanup, which makes 115200 baud at 16Mhz work more
    // reliably
    _rx_delay_stopbit = subtract_cap(bit_delay * 3 / 4, (37 + 11) / 4);
    #else // Timings counted from gcc 4.3.2 output
    // Note that this code is a _lot_ slower, mostly due to bad register
    // allocation choices of gcc. This works up to 57600 on 16Mhz and
    // 38400 on 8Mhz.
    _rx_delay_centering = subtract_cap(bit_delay / 2, (4 + 4 + 97 + 29 - 11) / 4);
    _rx_delay_intrabit = subtract_cap(bit_delay, 11 / 4);
    _rx_delay_stopbit = subtract_cap(bit_delay * 3 / 4, (44 + 17) / 4);
    #endif


    // Enable the PCINT for the entire port here, but never disable it
    // (others might also need it, so we disable the interrupt by using
    // the per-pin PCMSK register).
    *digitalPinToPCICR((int8_t)_receivePin) |= _BV(digitalPinToPCICRbit(_receivePin));
    // Precalculate the pcint mask register and value, so setRxIntMask
    // can be used inside the ISR without costing too much time.
    _pcint_maskreg = digitalPinToPCMSK(_receivePin);
    _pcint_maskvalue = _BV(digitalPinToPCMSKbit(_receivePin));

    tunedDelay(_tx_delay); // if we were low this establishes the end
  }

  listen();
}

void setRxIntMsk(bool enable)
{
    if (enable)
      *_pcint_maskreg |= _pcint_maskvalue;
    else
      *_pcint_maskreg &= ~_pcint_maskvalue;
}

bool listen()
{
  if (!_rx_delay_stopbit)
    return false;
  
  {
  
    _buffer_overflow = false;
    _receive_buffer_head = _receive_buffer_tail = 0;

    setRxIntMsk(true);
    return true;
  }

}


size_t write(uint8_t b)
{
  if (_tx_delay == 0) {
    Serial.println("Write error");
    return 0;
  }

  // By declaring these as local variables, the compiler will put them
  // in registers _before_ disabling interrupts and entering the
  // critical timing sections below, which makes it a lot easier to
  // verify the cycle timings
  volatile uint8_t *reg = _transmitPortRegister;
  uint8_t reg_mask = _transmitBitMask;
  uint8_t inv_mask = ~_transmitBitMask;
  uint8_t oldSREG = SREG;
  uint16_t delay = _tx_delay;
  

  cli();  // turn off interrupts for a clean txmit

  // Write the start bit
  
    *reg &= inv_mask;

  tunedDelay(delay);

  // Write each of the 8 bits
  for (uint8_t i = 8; i > 0; --i)
  {
    if (b & 1) // choose bit
      *reg |= reg_mask; // send 1
    else
      *reg &= inv_mask; // send 0

    tunedDelay(delay);
    b >>= 1;
  }

  // restore pin to natural state  
   *reg |= reg_mask;

  SREG = oldSREG; // turn interrupts back on
  tunedDelay(_tx_delay);
  
  return 1;
}

int available()
{
  return (_receive_buffer_tail + _SS_MAX_RX_BUFF - _receive_buffer_head) % _SS_MAX_RX_BUFF;
}

void SoftwareSerialInit(uint8_t receivePin, uint8_t transmitPin)   
{
  _rx_delay_centering = 0;
  _rx_delay_intrabit = 0;
  _rx_delay_stopbit = 0;
  _tx_delay = 0;
  _buffer_overflow = false;  
  setTX(transmitPin);
  setRX(receivePin);
}


int read()
{
  // Empty buffer?
  if (_receive_buffer_head == _receive_buffer_tail)
    return -1;

  // Read from "head"
  uint8_t d = _receive_buffer[_receive_buffer_head]; // grab next byte
  _receive_buffer_head = (_receive_buffer_head + 1) % _SS_MAX_RX_BUFF;
  return d;
}

//SoftwareSerial BTSerial(2,4); // blue tx, blue rx

inline uint8_t rx_pin_read()
{
  return *_receivePortRegister & _receiveBitMask;
}
inline void handle_interrupt()
{
  uint8_t d = 0;

  // If RX line is high, then we don't see any start bit
  // so interrupt is probably not for us
  if (!rx_pin_read())
  {
    // Disable further interrupts during reception, this prevents
    // triggering another interrupt directly after we return, which can
    // cause problems at higher baudrates.
    setRxIntMsk(false);

    // Wait approximately 1/2 of a bit width to "center" the sample
    tunedDelay(_rx_delay_centering);

    // Read each of the 8 bits
    for (uint8_t i=8; i > 0; --i)
    {
      tunedDelay(_rx_delay_intrabit);
      d >>= 1;
      if (rx_pin_read())
        d |= 0x80;
    }

    

    // if buffer full, set the overflow flag and return
    uint8_t next = (_receive_buffer_tail + 1) % _SS_MAX_RX_BUFF;
    if (next != _receive_buffer_head)
    {
      // save new data in buffer: tail points to where byte goes
      _receive_buffer[_receive_buffer_tail] = d; // save new byte
      _receive_buffer_tail = next;
    } 
    else 
    {
      _buffer_overflow = true;
    }

    // skip the stop bit
    tunedDelay(_rx_delay_stopbit);

    // Re-enable interrupts when we're sure to be inside the stop bit
    setRxIntMsk(true);

  }
}

#if defined(PCINT0_vect)
ISR(PCINT0_vect)
{
  handle_interrupt();
}
#endif

void setup() {
  SoftwareSerialInit(2,4);
    Serial.begin(SERBAUD);
    Serial.println("serial initialized");
    //BTSerial.begin(BTBAUD);  //5 19200 //6 38400,  57600 7, 115200 8  AT+BAUD8
    begin(BTBAUD);
}


void loop() { 
  loop_bt();;  
}

//int c = 0;
void loop_bt() { 
  //Serial.println(c++);
  while (available()){
        int c = read();
        Serial.write(c);        
  }
  // Keep reading from Arduino Serial Monitor and send to HC-05
  while(Serial.available()){
    int c = Serial.read();
    Serial.write(c);
    //BTSerial.write(c);          
    write(c);          
  }
}
