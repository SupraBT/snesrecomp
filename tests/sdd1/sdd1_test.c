#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "sdd1.h"
#include "sdd1_bsnes_ref.h"

/* Differential test for the S-DD1 decompressor.
 *
 * 1. The runner's whole-block core (sdd1_decompress, a Snes9x-style port of
 *    Andreas Naive's algorithm) must produce byte-identical output to a
 *    standalone port of bsnes's streaming Decompressor for every bitplane
 *    type x context configuration over deterministic pseudo-random streams.
 * 2. The runner's streaming DMA path (sdd1_dma_get_byte) must reproduce the
 *    whole-block output byte for byte -- the chip presents the same stream
 *    to DMA one byte at a time, so the two shapes must agree.
 *
 * ROM-free by design, matching the other tests/ C harnesses.
 */

#define INPUT_SIZE 0x10000u
#define OUT_LEN 0x4000

static uint64_t rng_state = 0x9e3779b97f4a7c15ull;
static uint32_t rng_next(void) {
  rng_state ^= rng_state << 13;
  rng_state ^= rng_state >> 7;
  rng_state ^= rng_state << 17;
  return (uint32_t)rng_state;
}

static int check(int condition, const char *message) {
  if (!condition) {
    fprintf(stderr, "FAIL: %s\n", message);
    return 1;
  }
  return 0;
}

static int first_diff(const uint8_t *a, const uint8_t *b, size_t n) {
  for (size_t i = 0; i < n; i++) {
    if (a[i] != b[i]) return (int)i;
  }
  return -1;
}

int main(void) {
  int fails = 0;
  uint8_t *ref_out = (uint8_t *)malloc(OUT_LEN);
  uint8_t *block_out = (uint8_t *)malloc(OUT_LEN);
  uint8_t *stream_out = (uint8_t *)malloc(OUT_LEN);
  uint8_t *input = (uint8_t *)malloc(INPUT_SIZE);
  if (!ref_out || !block_out || !stream_out || !input) return 2;

  /* Corpus: all 4 bitplane types x 4 context configurations. */
  for (int t = 0; t < 4; t++) {
    for (int c = 0; c < 4; c++) {
      for (uint32_t i = 0; i < INPUT_SIZE; i++) input[i] = (uint8_t)rng_next();
      input[0] = (uint8_t)((t << 6) | (c << 4));
      input[1] = (uint8_t)rng_next();

      char msg[128];

      /* Whole-block core vs the bsnes streaming oracle. */
      Sdd1Ref ref;
      sdd1_ref_init(&ref, input, INPUT_SIZE, 0);
      for (uint32_t i = 0; i < OUT_LEN; i++) ref_out[i] = sdd1_ref_read_byte(&ref);
      sdd1_decompress(block_out, input, (int)OUT_LEN);
      snprintf(msg, sizeof(msg), "core vs bsnes: bitplane=%d ctx=%d", t, c);
      fails += check(memcmp(ref_out, block_out, OUT_LEN) == 0, msg);
      if (memcmp(ref_out, block_out, OUT_LEN) != 0) {
        fprintf(stderr, "  first divergence at byte %d\n",
                first_diff(ref_out, block_out, OUT_LEN));
      }

      /* Streaming DMA path must reproduce the whole-block output exactly. */
      Sdd1 *s = sdd1_create(input, INPUT_SIZE, NULL, 0);
      if (!s) {
        fails += check(0, "sdd1_create");
        continue;
      }
      /* Channel 0 enabled. Source $C0:0000 maps through MMC page 0
       * (r4804 resets to 0) to linear ROM offset 0 -- the stream's header
       * byte -- matching the whole-block core's input. */
      sdd1_reset(s);
      sdd1_write(s, 0x4800, 0x01); /* hard enable, channel 0 */
      sdd1_write(s, 0x4801, 0x01); /* soft enable, channel 0 */
      sdd1_dma_init(s, 0, 0xC00000u, OUT_LEN);
      for (uint32_t i = 0; i < OUT_LEN; i++) stream_out[i] = sdd1_dma_get_byte(s, 0);
      snprintf(msg, sizeof(msg), "streaming vs block: bitplane=%d ctx=%d", t, c);
      fails += check(memcmp(stream_out, block_out, OUT_LEN) == 0, msg);
      if (memcmp(stream_out, block_out, OUT_LEN) != 0) {
        int d = first_diff(stream_out, block_out, OUT_LEN);
        fprintf(stderr, "  first divergence at byte %d\n", d);
        fprintf(stderr, "    block   : ");
        for (int i = d < 8 ? 0 : d - 8; i < d + 8; i++) {
          fprintf(stderr, "%02x ", block_out[i]);
        }
        fprintf(stderr, "\n    stream  : ");
        for (int i = d < 8 ? 0 : d - 8; i < d + 8; i++) {
          fprintf(stderr, "%02x ", stream_out[i]);
        }
        fprintf(stderr, "\n");
      }
      sdd1_destroy(s);
    }
  }

  /* Register model (bsnes semantics). */
  {
    uint8_t *big_rom = (uint8_t *)calloc(1, 8u * 1024u * 1024u);
    if (!big_rom) return 2;
    Sdd1 *s = sdd1_create(big_rom, 8u * 1024u * 1024u, NULL, 0);
    if (!s) return 2;
    sdd1_reset(s);

    /* $4800/$4801 are enable bytes; $4802/$4803 are not registers. */
    sdd1_write(s, 0x4800, 0x83);
    sdd1_write(s, 0x4801, 0x05);
    sdd1_write(s, 0x4802, 0xff); /* ignored */
    sdd1_write(s, 0x4803, 0xff); /* ignored */
    fails += check(sdd1_read(s, 0x4800) == 0x83, "r4800 hard enable");
    fails += check(sdd1_read(s, 0x4801) == 0x05, "r4801 soft enable");
    fails += check(sdd1_read(s, 0x4802) == 0x00, "r4802 is not a register");
    fails += check(sdd1_read(s, 0x4803) == 0x00, "r4803 is not a register");

    /* MMC bank selects default to 0,1,2,3 and mask writes to &0x8f. */
    fails += check(sdd1_read(s, 0x4804) == 0x00, "r4804 default");
    fails += check(sdd1_read(s, 0x4805) == 0x01, "r4805 default");
    fails += check(sdd1_read(s, 0x4806) == 0x02, "r4806 default");
    fails += check(sdd1_read(s, 0x4807) == 0x03, "r4807 default");
    sdd1_write(s, 0x4804, 0x95);
    fails += check(sdd1_read(s, 0x4804) == 0x85, "r4804 masked to &0x8f");
    sdd1_write(s, 0x4804, 0x00);

    /* MMC mapping is linear: each 1MB window selects a 1MB ROM page. */
    fails += check(sdd1_mmc_offset(s, 0xC00000) == 0u * 0x100000, "MMC C0 -> page 0");
    fails += check(sdd1_mmc_offset(s, 0xD00000) == 1u * 0x100000, "MMC D0 -> page 1");
    fails += check(sdd1_mmc_offset(s, 0xE00000) == 2u * 0x100000, "MMC E0 -> page 2");
    fails += check(sdd1_mmc_offset(s, 0xF00000) == 3u * 0x100000, "MMC F0 -> page 3");
    fails += check(sdd1_mmc_offset(s, 0xE80000) == 2u * 0x100000 + 0x80000,
                   "MMC within-page address");
    sdd1_write(s, 0x4805, 0x07);
    fails += check(sdd1_mmc_offset(s, 0xD00000) == 7u * 0x100000,
                   "MMC D0 -> page 7 after write");
    sdd1_write(s, 0x4805, 0x01); /* restore default */

    /* LoROM window ($00-$3F/$80-$BF:8000-FFFF) maps linearly by default,
     * and the bit-7 MMC override folds banks 20-3F / A0-BF onto the first
     * 1MB (bsnes mcuRead). */
    fails += check(sdd1_lorom_window_offset(s, 0x00, 0x8000) == 0u,
                   "LoROM 00:8000 -> 0");
    fails += check(sdd1_lorom_window_offset(s, 0x1F, 0xFFFF) == 0xFFFFFu,
                   "LoROM 1F:FFFF -> 1MB-1");
    fails += check(sdd1_lorom_window_offset(s, 0x20, 0x8000) == 0x100000u,
                   "LoROM 20:8000 -> 1MB (default)");
    fails += check(sdd1_lorom_window_offset(s, 0x3F, 0xFFFF) == 0x1FFFFFu,
                   "LoROM 3F:FFFF -> 2MB-1 (default)");
    fails += check(sdd1_lorom_window_offset(s, 0x80, 0x8000) == 0u,
                   "LoROM 80:8000 -> 0");
    fails += check(sdd1_lorom_window_offset(s, 0xA0, 0x8000) == 0x100000u,
                   "LoROM A0:8000 -> 1MB (default)");
    fails += check(sdd1_lorom_window_offset(s, 0xBF, 0xFFFF) == 0x1FFFFFu,
                   "LoROM BF:FFFF -> 2MB-1 (default)");
    fails += check(sdd1_lorom_window_offset(s, 0x00, 0x7FFF) == UINT32_MAX,
                   "LoROM addr < 8000 is not ROM");
    fails += check(sdd1_lorom_window_offset(s, 0x40, 0x8000) == UINT32_MAX,
                   "bank 40 is outside the LoROM window");

    sdd1_write(s, 0x4805, 0x81); /* bit 7 set: 20-3F aliases first 1MB */
    fails += check(sdd1_lorom_window_offset(s, 0x20, 0x8000) == 0u,
                   "r4805 bit7: 20:8000 aliases first 1MB");
    fails += check(sdd1_lorom_window_offset(s, 0x3F, 0x8000) == 0xF8000u,
                   "r4805 bit7: 3F:8000 aliases first 1MB");
    fails += check(sdd1_lorom_window_offset(s, 0x10, 0x8000) == 0x80000u,
                   "r4805 bit7: 10:8000 unaffected");
    fails += check(sdd1_lorom_window_offset(s, 0xA0, 0x8000) == 0x100000u,
                   "r4805 bit7: A0:8000 unaffected");

    sdd1_write(s, 0x4805, 0x01);
    sdd1_write(s, 0x4807, 0x83); /* bit 7 set: A0-BF aliases first 1MB */
    fails += check(sdd1_lorom_window_offset(s, 0xA0, 0x8000) == 0u,
                   "r4807 bit7: A0:8000 aliases first 1MB");
    fails += check(sdd1_lorom_window_offset(s, 0xBF, 0x8000) == 0xF8000u,
                   "r4807 bit7: BF:8000 aliases first 1MB");
    fails += check(sdd1_lorom_window_offset(s, 0x80, 0x8000) == 0u,
                   "r4807 bit7: 80:8000 unaffected");
    fails += check(sdd1_lorom_window_offset(s, 0x20, 0x8000) == 0x100000u,
                   "r4807 bit7: 20:8000 unaffected");
    sdd1_write(s, 0x4807, 0x03); /* restore default */

    /* Beyond-cartridge reads resolve to UINT32_MAX. */
    {
      uint8_t *small_rom = (uint8_t *)calloc(1, 0x180000u); /* 1.5MB */
      if (!small_rom) return 2;
      Sdd1 *s2 = sdd1_create(small_rom, 0x180000u, NULL, 0);
      if (!s2) return 2;
      sdd1_reset(s2);
      fails += check(sdd1_lorom_window_offset(s2, 0x20, 0x8000) == 0x100000u,
                     "1.5MB rom: 20:8000 in range");
      fails += check(sdd1_lorom_window_offset(s2, 0x3F, 0xFFFF) == UINT32_MAX,
                     "1.5MB rom: 3F:FFFF beyond size");
      sdd1_destroy(s2);
      free(small_rom);
    }

    /* The chip clears the channel's soft enable when the transfer ends. */
    sdd1_write(s, 0x4800, 0x01);
    sdd1_write(s, 0x4801, 0x01);
    sdd1_dma_init(s, 0, 0xC00000u, 4);
    for (int i = 0; i < 4; i++) sdd1_dma_get_byte(s, 0);
    fails += check(!sdd1_dma_active(s, 0), "session deactivates after transfer");
    fails += check(sdd1_read(s, 0x4801) == 0x00, "r4801 cleared at transfer end");

    /* A session requires both hard and soft enable for the channel. */
    sdd1_write(s, 0x4800, 0x00);
    sdd1_write(s, 0x4801, 0x02); /* channel 1 soft only */
    sdd1_dma_init(s, 1, 0xC00000u, 4);
    fails += check(!sdd1_dma_active(s, 1), "init requires both enables");

    sdd1_destroy(s);
    free(big_rom);
  }

  free(input);
  free(stream_out);
  free(block_out);
  free(ref_out);
  if (fails) return 1;
  puts("sdd1_test: PASS");
  return 0;
}
