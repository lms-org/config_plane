/*
////////////////////////////////////////////////////////////////////////////////
void setup_i2c();                                                    // sets the i2c-com up
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
*/

#include <ownI2c.h>

//#define FAST_MODE

void setup() {

}

void loop() {
  
}
