#include <string.h>

#include "sdd1_bsnes_ref.h"

/* Transcribed verbatim from bsnes sfc/coprocessor/sdd1/decompressor.cpp
 * (runCount table of the Golomb-code decoder). */
/* bsnes indexes this table with codeWord >> (codeNumber ^ 0x07), so it
 * spans the full 256-entry space: the top bit selects the LPS side. */
static const uint8_t ref_run_count[256] = {
  0x00, 0x00, 0x01, 0x00, 0x03, 0x01, 0x02, 0x00,
  0x07, 0x03, 0x05, 0x01, 0x06, 0x02, 0x04, 0x00,
  0x0f, 0x07, 0x0b, 0x03, 0x0d, 0x05, 0x09, 0x01,
  0x0e, 0x06, 0x0a, 0x02, 0x0c, 0x04, 0x08, 0x00,
  0x1f, 0x0f, 0x17, 0x07, 0x1b, 0x0b, 0x13, 0x03,
  0x1d, 0x0d, 0x15, 0x05, 0x19, 0x09, 0x11, 0x01,
  0x1e, 0x0e, 0x16, 0x06, 0x1a, 0x0a, 0x12, 0x02,
  0x1c, 0x0c, 0x14, 0x04, 0x18, 0x08, 0x10, 0x00,
  0x3f, 0x1f, 0x2f, 0x0f, 0x37, 0x17, 0x27, 0x07,
  0x3b, 0x1b, 0x2b, 0x0b, 0x33, 0x13, 0x23, 0x03,
  0x3d, 0x1d, 0x2d, 0x0d, 0x35, 0x15, 0x25, 0x05,
  0x39, 0x19, 0x29, 0x09, 0x31, 0x11, 0x21, 0x01,
  0x3e, 0x1e, 0x2e, 0x0e, 0x36, 0x16, 0x26, 0x06,
  0x3a, 0x1a, 0x2a, 0x0a, 0x32, 0x12, 0x22, 0x02,
  0x3c, 0x1c, 0x2c, 0x0c, 0x34, 0x14, 0x24, 0x04,
  0x38, 0x18, 0x28, 0x08, 0x30, 0x10, 0x20, 0x00,
  0x7f, 0x3f, 0x5f, 0x1f, 0x6f, 0x2f, 0x4f, 0x0f,
  0x77, 0x37, 0x57, 0x17, 0x67, 0x27, 0x47, 0x07,
  0x7b, 0x3b, 0x5b, 0x1b, 0x6b, 0x2b, 0x4b, 0x0b,
  0x73, 0x33, 0x53, 0x13, 0x63, 0x23, 0x43, 0x03,
  0x7d, 0x3d, 0x5d, 0x1d, 0x6d, 0x2d, 0x4d, 0x0d,
  0x75, 0x35, 0x55, 0x15, 0x65, 0x25, 0x45, 0x05,
  0x79, 0x39, 0x59, 0x19, 0x69, 0x29, 0x49, 0x09,
  0x71, 0x31, 0x51, 0x11, 0x61, 0x21, 0x41, 0x01,
  0x7e, 0x3e, 0x5e, 0x1e, 0x6e, 0x2e, 0x4e, 0x0e,
  0x76, 0x36, 0x56, 0x16, 0x66, 0x26, 0x46, 0x06,
  0x7a, 0x3a, 0x5a, 0x1a, 0x6a, 0x2a, 0x4a, 0x0a,
  0x72, 0x32, 0x52, 0x12, 0x62, 0x22, 0x42, 0x02,
  0x7c, 0x3c, 0x5c, 0x1c, 0x6c, 0x2c, 0x4c, 0x0c,
  0x74, 0x34, 0x54, 0x14, 0x64, 0x24, 0x44, 0x04,
  0x78, 0x38, 0x58, 0x18, 0x68, 0x28, 0x48, 0x08,
  0x70, 0x30, 0x50, 0x10, 0x60, 0x20, 0x40, 0x00,
};

/* Probability estimation module evolution table, verbatim from bsnes. */
static const struct {
  uint8_t code_number;
  uint8_t next_if_mps;
  uint8_t next_if_lps;
} ref_evolution_table[33] = {
  {0, 25, 25}, {0,  2,  1}, {0,  3,  1}, {0,  4,  2}, {0,  5,  3},
  {1,  6,  4}, {1,  7,  5}, {1,  8,  6}, {1,  9,  7}, {2, 10,  8},
  {2, 11,  9}, {2, 12, 10}, {2, 13, 11}, {3, 14, 12}, {3, 15, 13},
  {3, 16, 14}, {3, 17, 15}, {4, 18, 16}, {4, 19, 17}, {5, 20, 18},
  {5, 21, 19}, {6, 22, 20}, {6, 23, 21}, {7, 24, 22}, {7, 24, 23},
  {0, 26,  1}, {1, 27,  2}, {2, 28,  4}, {3, 29,  8}, {4, 30, 12},
  {5, 31, 16}, {6, 32, 18}, {7, 24, 22},
};

static uint8_t ref_mmc_read(Sdd1Ref *ref, uint32_t offset) {
  return offset < ref->rom_size ? ref->rom[offset] : 0;
}

/* Input manager */
static void ref_im_init(Sdd1Ref *ref, uint32_t offset) {
  ref->offset = offset;
  ref->bit_count = 4;
}

static uint8_t ref_im_get_codeword(Sdd1Ref *ref, uint8_t code_length) {
  uint8_t code_word = (uint8_t)(ref_mmc_read(ref, ref->offset) << ref->bit_count);
  ref->bit_count++;
  if (code_word & 0x80) {
    code_word |= (uint8_t)(ref_mmc_read(ref, ref->offset + 1) >> (9 - ref->bit_count));
    ref->bit_count += code_length;
  }
  if (ref->bit_count & 0x08) {
    ref->offset++;
    ref->bit_count &= 0x07;
  }
  return code_word;
}

/* Golomb-code decoder */
static void ref_gcd_get_run_count(Sdd1Ref *ref, uint8_t code_number,
                                  uint8_t *mps_count, uint8_t *lps_index) {
  uint8_t code_word = ref_im_get_codeword(ref, code_number);
  if (code_word & 0x80) {
    *lps_index = 1;
    *mps_count = ref_run_count[code_word >> (code_number ^ 0x07)];
  } else {
    *mps_count = (uint8_t)(1u << code_number);
  }
}

/* Bits generator */
static void ref_bg_init(Sdd1Ref *ref, int n) {
  ref->bg_mps_count[n] = 0;
  ref->bg_lps_index[n] = 0;
}

static uint8_t ref_bg_get_bit(Sdd1Ref *ref, int n, uint8_t *end_of_run) {
  if (!(ref->bg_mps_count[n] || ref->bg_lps_index[n])) {
    ref_gcd_get_run_count(ref, (uint8_t)n, &ref->bg_mps_count[n],
                          &ref->bg_lps_index[n]);
  }
  uint8_t bit;
  if (ref->bg_mps_count[n]) {
    bit = 0;
    ref->bg_mps_count[n]--;
  } else {
    bit = 1;
    ref->bg_lps_index[n] = 0;
  }
  *end_of_run = (uint8_t)(!(ref->bg_mps_count[n] || ref->bg_lps_index[n]));
  return bit;
}

/* Probability estimation module */
static void ref_pem_init(Sdd1Ref *ref) {
  for (int n = 0; n < 32; n++) {
    ref->context_info[n].status = 0;
    ref->context_info[n].mps = 0;
  }
}

static uint8_t ref_pem_get_bit(Sdd1Ref *ref, uint8_t context) {
  uint8_t current_status = ref->context_info[context].status;
  uint8_t current_mps = ref->context_info[context].mps;
  const uint8_t code_number = ref_evolution_table[current_status].code_number;
  uint8_t end_of_run = 0;
  uint8_t bit = ref_bg_get_bit(ref, code_number, &end_of_run);
  if (end_of_run) {
    if (bit) {
      if (!(current_status & 0xfe)) ref->context_info[context].mps ^= 0x01;
      ref->context_info[context].status = ref_evolution_table[current_status].next_if_lps;
    } else {
      ref->context_info[context].status = ref_evolution_table[current_status].next_if_mps;
    }
  }
  return (uint8_t)(bit ^ current_mps);
}

/* Context model */
static void ref_cm_init(Sdd1Ref *ref, uint32_t offset) {
  ref->bitplanes_info = (uint8_t)(ref_mmc_read(ref, offset) & 0xc0);
  ref->context_bits_info = (uint8_t)(ref_mmc_read(ref, offset) & 0x30);
  ref->bit_number = 0;
  for (int n = 0; n < 8; n++) ref->previous_bitplane_bits[n] = 0;
  switch (ref->bitplanes_info) {
  case 0x00: ref->current_bitplane = 1; break;
  case 0x40: ref->current_bitplane = 7; break;
  case 0x80: ref->current_bitplane = 3; break;
  }
}

static uint8_t ref_cm_get_bit(Sdd1Ref *ref) {
  switch (ref->bitplanes_info) {
  case 0x00:
    ref->current_bitplane ^= 0x01;
    break;
  case 0x40:
    ref->current_bitplane ^= 0x01;
    if (!(ref->bit_number & 0x7f)) {
      ref->current_bitplane = (uint8_t)((ref->current_bitplane + 2) & 0x07);
    }
    break;
  case 0x80:
    ref->current_bitplane ^= 0x01;
    if (!(ref->bit_number & 0x7f)) ref->current_bitplane ^= 0x02;
    break;
  case 0xc0:
    ref->current_bitplane = (uint8_t)(ref->bit_number & 0x07);
    break;
  }

  uint16_t context_bits = ref->previous_bitplane_bits[ref->current_bitplane];
  uint8_t current_context = (uint8_t)((ref->current_bitplane & 0x01) << 4);
  switch (ref->context_bits_info) {
  case 0x00:
    current_context |= (uint8_t)(((context_bits & 0x01c0) >> 5) | (context_bits & 0x0001));
    break;
  case 0x10:
    current_context |= (uint8_t)(((context_bits & 0x0180) >> 5) | (context_bits & 0x0001));
    break;
  case 0x20:
    current_context |= (uint8_t)(((context_bits & 0x00c0) >> 5) | (context_bits & 0x0001));
    break;
  case 0x30:
    current_context |= (uint8_t)(((context_bits & 0x0180) >> 5) | (context_bits & 0x0003));
    break;
  }

  uint8_t bit = ref_pem_get_bit(ref, current_context);
  ref->previous_bitplane_bits[ref->current_bitplane] =
      (uint16_t)((context_bits << 1) | bit);
  ref->bit_number++;
  return bit;
}

/* Output logic */
static void ref_ol_init(Sdd1Ref *ref, uint32_t offset) {
  ref->ol_bitplanes_info = (uint8_t)(ref_mmc_read(ref, offset) & 0xc0);
  ref->r0 = 0x01;
}

static uint8_t ref_ol_decompress(Sdd1Ref *ref) {
  switch (ref->ol_bitplanes_info) {
  case 0x00:
  case 0x40:
  case 0x80:
    if (ref->r0 == 0) {
      ref->r0 = (uint8_t)~ref->r0;
      return ref->r2;
    }
    for (ref->r0 = 0x80, ref->r1 = 0, ref->r2 = 0; ref->r0; ref->r0 >>= 1) {
      if (ref_cm_get_bit(ref)) ref->r1 |= ref->r0;
      if (ref_cm_get_bit(ref)) ref->r2 |= ref->r0;
    }
    return ref->r1;
  case 0xc0:
    for (ref->r0 = 0x01, ref->r1 = 0; ref->r0; ref->r0 <<= 1) {
      if (ref_cm_get_bit(ref)) ref->r1 |= ref->r0;
    }
    return ref->r1;
  }
  return 0;
}

/* Core */
void sdd1_ref_init(Sdd1Ref *ref, const uint8_t *rom, uint32_t rom_size,
                   uint32_t offset) {
  memset(ref, 0, sizeof(*ref));
  ref->rom = rom;
  ref->rom_size = rom_size;
  ref_im_init(ref, offset);
  for (int n = 0; n < 8; n++) ref_bg_init(ref, n);
  ref_pem_init(ref);
  ref_cm_init(ref, offset);
  ref_ol_init(ref, offset);
}

uint8_t sdd1_ref_read_byte(Sdd1Ref *ref) {
  return ref_ol_decompress(ref);
}
