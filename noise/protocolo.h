#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>

// Especificacao comum. A mesma copia acompanha cada sketch.
namespace Proto {
constexpr uint32_t BAUD = 115200;
constexpr uint32_t PERIOD_US = 100000;
constexpr uint32_t QUIET_US = 8000;
constexpr uint32_t LINK_TIMEOUT_US = 250000;
constexpr uint8_t SOF0 = 0xA5, SOF1 = 0x5A;
constexpr uint8_t LEN = 7; // bytes DEPOIS do campo LEN
constexpr size_t FRAME_SIZE = 10, BODY_SIZE = 8, PAYLOAD_SIZE = 4;
constexpr uint8_t DBG[4] = {0xD3, 0x91, 0xC7, 0x6E};
constexpr size_t DEBUG_SIZE = 18; // marcador + quadro original + tempo TX uint32
enum Algorithm : uint8_t { SUM8, CRC8 };

inline const char *name(Algorithm a) { return a == SUM8 ? "SUM8" : "CRC8"; }
inline uint16_t seq(const uint8_t *frame) {
  return uint16_t(frame[3]) | (uint16_t(frame[4]) << 8);
}
inline uint8_t sum8(const uint8_t *p, size_t n) {
  uint8_t s = 0;
  for (size_t i = 0; i < n; ++i) s = uint8_t(s + p[i]);
  return s;
}
// CRC-8/SMBUS: poly=0x07, init=0, refin=false, refout=false, xorout=0.
inline uint8_t crc8(const uint8_t *p, size_t n) {
  uint8_t c = 0;
  for (size_t i = 0; i < n; ++i) {
    c ^= p[i];
    for (uint8_t k = 0; k < 8; ++k)
      c = (c & 0x80) ? uint8_t((c << 1) ^ 0x07) : uint8_t(c << 1);
  }
  return c;
}
inline uint8_t code(Algorithm a, const uint8_t *p, size_t n) {
  return a == SUM8 ? uint8_t(0 - sum8(p, n)) : crc8(p, n);
}
inline uint8_t residue(Algorithm a, const uint8_t *p, size_t n) {
  return a == SUM8 ? sum8(p, n) : crc8(p, n);
}
inline bool shape(const uint8_t *p, size_t n) {
  return n == FRAME_SIZE && p[0] == SOF0 && p[1] == SOF1 && p[2] == LEN;
}
inline bool debugShape(const uint8_t *p, size_t n) {
  return n == DEBUG_SIZE && memcmp(p, DBG, 4) == 0 && shape(p + 4, FRAME_SIZE);
}
inline void hex(const uint8_t *p, size_t n, char *out) {
  const char *digits = "0123456789ABCDEF";
  for (size_t i = 0; i < n; ++i) {
    out[2*i] = digits[p[i] >> 4]; out[2*i+1] = digits[p[i] & 15];
  }
  out[2*n] = '\0';
}
inline void put32(uint8_t *p, uint32_t v) {
  for (uint8_t k = 0; k < 4; ++k) p[k] = uint8_t(v >> (8*k));
}
inline uint32_t get32(const uint8_t *p) {
  return uint32_t(p[0]) | (uint32_t(p[1]) << 8) |
         (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24);
}
// Pausas de 8 ms servem para recuperar um bloco incompleto, nao como EDC.
struct Block {
  uint8_t data[64] = {};
  size_t size = 0;
  bool overflow = false;
  uint32_t first = 0, last = 0;
  void clear() { size = 0; overflow = false; }
  void push(uint8_t b, uint32_t now) {
    if (size == 0) first = now;
    if (size < sizeof(data)) data[size++] = b; else overflow = true;
    last = now;
  }
  bool expired(uint32_t now) const {
    return size != 0 && uint32_t(now - last) >= QUIET_US;
  }
};
// Analise operacional. Nunca recebe a copia de depuracao.
struct Sequence {
  uint16_t expected = 0;
  uint32_t rejected = 0;
  void reset() { expected = 0; rejected = 0; }
  void discard() { ++rejected; }
  // -1: repetido/antigo/fora de ordem (janela menor que 32768).
  int32_t accept(uint16_t received) {
    uint16_t delta = uint16_t(received - expected);
    if (delta >= 32768) return -1;
    uint32_t lost = delta > rejected ? delta - rejected : 0;
    expected = uint16_t(received + 1);
    rejected = 0;
    return int32_t(lost);
  }
};
}
