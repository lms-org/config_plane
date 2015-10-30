#include "Arduino.h"
#include "ownI2C.h"

uint32_t neutralizeTime;
uint16_t i2c_errors_count;
boolean i2c_display_available = true;
boolean i2c_sonar_available = true;

void setup_i2c(uint8_t speed, uint8_t pullup) {
  if (pullup) {
    I2C_PULLUPS_ENABLE
  }
  else {
    I2C_PULLUPS_DISABLE
  }
  TWSR = 0;                                              // no prescaler => prescaler = 1
  if (speed) TWBR = ((F_CPU / 400000UL) - 16) / 2;					 // change the I2C clock rate
  else       TWBR = ((F_CPU / 100000UL) - 16) / 2;					 // change the I2C clock rate
  TWCR = 1 << TWEN;                                      // enable twi module, no interrupt
  i2c_errors_count = 0;
}


uint8_t i2c_readReg(uint8_t add, uint8_t reg) {
  uint8_t val;
  i2c_read_reg_to_buf(add, reg, &val, 1);
  return val;
}

void i2c_writeReg(uint8_t add, uint8_t reg, uint8_t val) {
  i2c_rep_start(add << 1); // I2C write direction
  i2c_write(reg);        // register selection
  i2c_write(val);        // value to write in register
  i2c_stop();
}

uint8_t i2c_read(uint8_t ack) {
  TWCR = (1 << TWINT) | (1 << TWEN) | (ack ? (1 << TWEA) : 0);
  i2c_waitTransmission();
  uint8_t r = TWDR;
  if (!ack) i2c_stop();
  return r;
}

void i2c_write(uint8_t data) {
  TWDR = data;                                 // send data to the previously addressed device
  TWCR = (1 << TWINT) | (1 << TWEN);
  i2c_waitTransmission();
}

void i2c_rep_start(uint8_t address) {
  TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN) ; // send REPEAT START condition
  i2c_waitTransmission();                       // wait until transmission completed
  TWDR = address;                              // send device address
  TWCR = (1 << TWINT) | (1 << TWEN);
  i2c_waitTransmission();                       // wail until transmission completed
}

void i2c_waitTransmission() {
  uint16_t count = 255;
  while (!(TWCR & (1 << TWINT))) {
    count--;
    if (count == 0) {            //we are in a blocking state => we don't insist
      TWCR = 0;                  //and we force a reset on TWINT register
      neutralizeTime = micros(); //we take a timestamp here to neutralize the value during a short delay
      i2c_errors_count++;
      break;
    }
  }
}

uint8_t i2c_readAck() {
  return i2c_read(1);
}

uint8_t i2c_readNak(void) {
  return i2c_read(0);
}

size_t i2c_read_to_buf(uint8_t add, void *buf, size_t size) {
  i2c_rep_start((add << 1) | 1); // I2C read direction
  size_t bytes_read = 0;
  uint8_t *b = (uint8_t*)buf;
  while (size--) {
    /* acknowledge all but the final byte */
    *b++ = i2c_read(size > 0);
    /* TODO catch I2C errors here and abort */
    bytes_read++;
  }
  return bytes_read;
}

size_t i2c_read_reg_to_buf(uint8_t add, uint8_t reg, void *buf, size_t size) {
  i2c_rep_start(add << 1); // I2C write direction
  i2c_write(reg);        // register selection
  i2c_stop();
  return i2c_read_to_buf(add, buf, size);
}

/* transform a series of bytes from big endian to little
   endian and vice versa. */
void swap_endianness(void *buf, size_t size) {
  /* we swap in-place, so we only have to
  * place _one_ element on a temporary tray
  */
  uint8_t tray;
  uint8_t *from;
  uint8_t *to;
  /* keep swapping until the pointers have assed each other */
  for (from = (uint8_t*)buf, to = &from[size - 1]; from < to; from++, to--) {
    tray = *from;
    *from = *to;
    *to = tray;
  }
}

void i2c_stop() {
  TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
  //  while(TWCR & (1<<TWSTO));                // <- can produce a blocking state with some WMP clones
}
