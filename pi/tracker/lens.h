// Direct control of the Camera Module 3 focus motor. QNX's camera driver for this module reports no
// autofocus and no manual focus steps, so the lens is driven over I2C: a DW9807 voice-coil driver at
// address 0x0c on the camera's own bus (/dev/i2c6 for camera unit 3, /dev/i2c4 for unit 4).
// Position is a code from 0 to 1023. Measured on our module: the sharpest code for a scene about 0.85 m away
// was 440, and sharpness fell to a third within 20 codes either side, so the right code must be found by
// sweeping (pi/run-focus.sh) at the real working distance.
// The motor has hysteresis: the same code gives a different lens position when reached from above than from
// below. A jump from 800 down to 440 landed visibly out of focus. lensApproach() therefore always arrives
// from below in small steps, which is also how the sweep moves.
#pragma once
#include <devctl.h>
#include <fcntl.h>
#include <hw/i2c.h>
#include <unistd.h>

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

inline const char* lensBusForUnit(int unit) { return unit == 4 ? "/dev/i2c4" : "/dev/i2c6"; }

// Which call failed last: "open" or "devctl". The I2C devices belong to root on the QNX image, so opening
// them fails with "Permission denied" until they are made writable (pi/run-focus.sh does that with sudo).
inline const char*& lensFailedAt() {
  static const char* stage = "";
  return stage;
}

// Returns 0 on success, or the errno-style error from open() or devctl().
inline int lensWrite(const char* bus, const uint8_t* bytes, uint32_t count) {
  int fd = open(bus, O_RDWR);
  lensFailedAt() = "open";
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
  lensFailedAt() = "devctl";
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

// Moves to `code` from below in steps of 20 with a short pause each, so the lens ends where a sweep found it.
inline int lensApproach(int unit, int code) {
  for (int step = code - 120; step < code; step += 20) {
    int error = lensSetCode(unit, step);
    if (error) return error;
    usleep(step == code - 120 ? 250000 : 80000);
  }
  int error = lensSetCode(unit, code);
  usleep(150000);
  return error;
}

// The focus tool saves the code it found per camera unit, and the tracker reads it back, so nobody has to
// copy the number by hand.
inline std::string lensCodeFile(int unit) {
  const char* home = std::getenv("HOME");
  return std::string(home ? home : ".") + "/lens-unit" + std::to_string(unit) + ".txt";
}

inline void lensSaveCode(int unit, int code) {
  if (FILE* file = std::fopen(lensCodeFile(unit).c_str(), "w")) {
    std::fprintf(file, "%d\n", code);
    std::fclose(file);
  }
}

// Returns -1 when no code has been saved for this unit.
inline int lensLoadCode(int unit) {
  int code = -1;
  if (FILE* file = std::fopen(lensCodeFile(unit).c_str(), "r")) {
    if (std::fscanf(file, "%d", &code) != 1) code = -1;
    std::fclose(file);
  }
  return code;
}
