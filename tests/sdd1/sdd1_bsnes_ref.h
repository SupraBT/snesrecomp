#ifndef SDD1_BSNES_REF_H
#define SDD1_BSNES_REF_H

#include <stdint.h>

/* Standalone C port of bsnes's S-DD1 streaming decompressor
 * (bsnes/bsnes/sfc/coprocessor/sdd1/decompressor.cpp, Andreas Naive's
 * public-domain algorithm as ported by byuu). Unlike the runner's
 * whole-block sdd1_decompress(), this implementation emits one output byte
 * per call -- the shape the real chip presents to DMA. It is the
 * differential oracle for sdd1_test.c. */

typedef struct Sdd1Ref {
  /* Input manager */
  const uint8_t *rom;
  uint32_t rom_size;
  uint32_t offset;
  int bit_count;

  /* Bits generators (one per Golomb code number) */
  uint8_t bg_mps_count[8];
  uint8_t bg_lps_index[8];

  /* Probability estimation module */
  struct {
    uint8_t status;
    uint8_t mps;
  } context_info[32];

  /* Context model */
  uint8_t bitplanes_info;
  uint8_t context_bits_info;
  uint8_t current_bitplane;
  uint16_t previous_bitplane_bits[8];
  uint32_t bit_number;

  /* Output logic */
  uint8_t ol_bitplanes_info;
  uint8_t r0, r1, r2;
} Sdd1Ref;

void sdd1_ref_init(Sdd1Ref *ref, const uint8_t *rom, uint32_t rom_size,
                   uint32_t offset);
uint8_t sdd1_ref_read_byte(Sdd1Ref *ref);

#endif
