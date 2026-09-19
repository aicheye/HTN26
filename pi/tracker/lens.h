// Direct control of the Camera Module 3 focus motor. QNX's camera driver for this module reports no
// autofocus and no manual focus steps, so the lens is driven over I2C: a DW9807 voice-coil driver at
// address 0x0c on the camera's own bus (/dev/i2c6 for camera unit 3, /dev/i2c4 for unit 4).
// Position is a code from 0 to 1023. Raspberry Pi's tuning for this module maps focus distance to
// code = 445 + 32 / distance in metres, so infinity is about 445 and 1 m is about 477.
#pragma once
#include <devctl.h>
#include <fcntl.h>
#include <hw/i2c.h>
#include <unistd.h>

#include <cstdint>
#include <cstring>

inline const char* lensBusForUnit(int unit) { return unit == 4 ? "/dev/i2c4" : "/dev/i2c6"; }

inline int lensCodeForDistance(double metres) { return (int)(445 + 32 / metres + 0.5); }

// Returns 0 on success, or the errno-style error from open() or devctl().
inline int lensWrite(const char* bus, const uint8_t* bytes, uint32_t count) {
  int fd = open(bus, O_RDWR);
  if (fd < 0) return errno;
  struct {
    i2c_send_t header;
    uint8_t data[8];
  } message;
  std::memset(&message, 0, sizeof(message));
  message.header.slave.addr = 0x0c;
  message.header.slave.fmt = I2C_ADDRFMT_7BIT;
  message.header.len = count;
  message.header.stop = 1;
  std::memcpy(message.data, bytes, count);
  int error = devctl(fd, DCMD_I2C_SEND, &message, sizeof(message.header) + count, nullptr);
  close(fd);
  return error;
}

inline int lensSetCode(int unit, int code) {
  const char* bus = lensBusForUnit(unit);
  code = code < 0 ? 0 : code > 1023 ? 1023 : code;
  const uint8_t powerOn[] = {0x02, 0x00};  // control register: leave power-down
  const uint8_t position[] = {0x03, (uint8_t)(code >> 8), (uint8_t)(code & 0xff)};  // position MSB, then LSB
  int error = lensWrite(bus, powerOn, sizeof(powerOn));
  return error ? error : lensWrite(bus, position, sizeof(position));
}
