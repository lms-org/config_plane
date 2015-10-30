#ifndef ownI2C_H
#define ownI2C_H
#include "Arduino.h"

////////////////////////////////////////////////////////////////////////////////
void setup_i2c(uint8_t speed, uint8_t pullup);                      // sets the i2c-com up
void i2c_stop();                                                     // sends NACK
void i2c_waitTransmission();                                          // waits if/til the transmission is over
void i2c_rep_start(uint8_t address);                                 // sends ACK
void i2c_write(uint8_t data);                                        // writes data via i2c (is calles by writeReg() )
uint8_t i2c_read(uint8_t ack);                                        // reads data via i2c (is calles by readReg() )
uint8_t i2c_readAck();
uint8_t i2c_readNak();
size_t i2c_read_to_buf(uint8_t add, void *buf, size_t size);
size_t i2c_read_reg_to_buf(uint8_t add, uint8_t reg, void *buf, size_t size);
void swap_endianness(void *buf, size_t size);
void i2c_writeReg(uint8_t add, uint8_t reg, uint8_t val);            // writes data to a specified register of a specified slave
void i2c_writeReg(uint8_t add, uint8_t reg, uint8_t val);
uint8_t i2c_readReg(uint8_t add, uint8_t reg);                       // reads data from a specified register of a specified slave
////////////////////////////////////////////////////////////////////////////////


//#define INTERNAL_I2C_PULLUPS

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
  #define ATMEGA328
#elif defined(__AVR_ATmega32U4__) || defined(TEENSY20)
  #define PROMICRO
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
  #define ATMEGA2560
#endif

#if defined (ATMEGA328)
  #define I2C_PULLUPS_ENABLE         PORTC |= 1<<4; PORTC |= 1<<5;   // PIN A4&A5 (SDA&SCL)
  #define I2C_PULLUPS_DISABLE        PORTC &= ~(1<<4); PORTC &= ~(1<<5);
#endif

#if defined (PROMICRO)
  #define I2C_PULLUPS_ENABLE         PORTD |= 1<<0; PORTD |= 1<<1;   // PIN 2&3 (SDA&SCL)
  #define I2C_PULLUPS_DISABLE        PORTD &= ~(1<<0); PORTD &= ~(1<<1);
#endif

#if defined (ATMEGA2560)
  #define I2C_PULLUPS_ENABLE         PORTD |= 1<<0; PORTD |= 1<<1;       // PIN 20&21 (SDA&SCL)
  #define I2C_PULLUPS_DISABLE        PORTD &= ~(1<<0); PORTD &= ~(1<<1);
#endif

#endif // ownI2C_H
