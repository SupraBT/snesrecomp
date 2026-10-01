
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>


#include "apu.h"
#include "snes.h"
#include "spc.h"
#include "dsp.h"
#include "../audio_trace.h"

static const uint8_t bootRom[0x40] = {
  0xcd, 0xef, 0xbd, 0xe8, 0x00, 0xc6, 0x1d, 0xd0, 0xfc, 0x8f, 0xaa, 0xf4, 0x8f, 0xbb, 0xf5, 0x78,
  0xcc, 0xf4, 0xd0, 0xfb, 0x2f, 0x19, 0xeb, 0xf4, 0xd0, 0xfc, 0x7e, 0xf4, 0xd0, 0x0b, 0xe4, 0xf5,
  0xcb, 0xf4, 0xd7, 0x00, 0xfc, 0xd0, 0xf3, 0xab, 0x01, 0x10, 0xef, 0x7e, 0xf4, 0x10, 0xeb, 0xba,
  0xf6, 0xda, 0x00, 0xba, 0xf4, 0xc4, 0xf4, 0xdd, 0x5d, 0xd0, 0xdb, 0x1f, 0x00, 0x00, 0xc0, 0xff
};

Apu* apu_init(void) {
  Apu* apu = calloc(1, sizeof(Apu));  /* zero padding: saveload/co-sim hash determinism */
  apu->spc = spc_init(apu);
  apu->dsp = dsp_init(apu->ram);
  apu_clearPortQueue(apu);
  return apu;
}

void apu_free(Apu* apu) {
  spc_free(apu->spc);
  dsp_free(apu->dsp);
  free(apu);
}

void apu_reset(Apu* apu) {
  apu->romReadable = true; // before resetting spc, because it reads reset vector from it
  spc_reset(apu->spc);
  dsp_reset(apu->dsp);
  memset(apu->ram, 0, sizeof(apu->ram));
  apu->dspAdr = 0;
  apu->cycles = 0;
  memset(apu->inPorts, 0, sizeof(apu->inPorts));
  memset(apu->outPorts, 0, sizeof(apu->outPorts));
  for(int i = 0; i < 3; i++) {
    apu->timer[i].cycles = 0;
    apu->timer[i].divider = 0;
    apu->timer[i].target = 0;
    apu->timer[i].counter = 0;
    apu->timer[i].enabled = false;
  }
  apu->cpuCyclesLeft = 7;
  apu->portClock = 0;
  apu_clearPortQueue(apu);
}

void apu_clearPortQueue(Apu* apu) {
  apu->portQHead = apu->portQTail = 0;
  apu->portGuestAnchor = 0;
  apu->portTargetAnchor = 0;
  apu->portLastGuest = 0;
  apu->portLastTarget = 0;
  apu->portTimeValid = false;
}

/* Diagnostico: escrituras de puerto que EL INVITADO ha hecho llegar al SPC y
 * profundidad de la cola de puertos pendientes.  Si la cola no baja de cero el
 * invitado entrega su audio pero el DSP nunca lo ve (silencio con musica
 * sonando por dentro). */
uint64_t g_apu_port_writes = 0;
/* Escrituras del SPC700 a sus propios puertos ($F4-$F7) y al DSP ($F2/$F3).
 * El hardware produce 11.002 / 3.095 / ~3,2 por fotograma; si aqui sale 0, el
 * motor de sonido esta esperando una respuesta que nunca le llega. */
uint64_t g_spc_port_w = 0;
uint64_t g_spc_dsp_adr = 0;
uint64_t g_spc_dsp_dat = 0;
uint64_t g_apu_port_queue_max = 0;
uint64_t g_apu_port_dropped = 0;

/* PERDIDA DE BYTES EN LOS PUERTOS DE ENTRADA. Los puertos $F4-$F7 del SPC700
 * son un unico registro de un byte: si el invitado escribe el byte N+1 antes de
 * que el SPC700 lea el byte N, el N se pierde y no hay ningun aviso (la cola de
 * eventos sigue vacia, porque la escritura anterior YA se aplico). El driver de
 * sonido de Star Ocean transfiere ~7.460 bytes de golpe con $
 * 2140, asi que basta un instante de desfase del SPC700 para corromper el BRAM
 * entero y dejar el motor de sonido mudo. Este contador mide exactamente eso. */
uint64_t g_inp_overwritten = 0;
/* Lecturas de $F4-$F7 por el SPC700. El motor de sonido sano LEE el flujo de
 * bytes a ritmo constante; si se estanca, el SPC esta aparcado en un bucle de
 * espera y no esta leyendo nada. */
uint64_t g_spc_port_reads[4] = {0, 0, 0, 0};
/* HUELLA DEL FLUJO DE PUERTOS: FNV-1a sobre la secuencia (puerto, valor) de
 * TODAS las escrituras que el invitado entrega al SPC700, en orden. Comparar la
 *.huella acumulada en un frame de invitado dado entre dos configuraciones dice
 * si el invitado entrego los MISMOS bytes, sin volcar 60.000 lineas. */
uint64_t g_apu_stream_hash = 1469598103934665603ull;
/* ESCRITURAS AL BRAM POR PAGINA de 4 KiB. El IPL del SPC700 descomprime el
 * motor de sonido dentro del BRAM; si esa subida se queda a medias, el motor
 * arranca (hace su tick por frame) pero le faltan las tablas de secuencia y las
 * muestras, y entonces: los SFX suenan, la musica no, y en el cambio de tema
 * sale un chasquido de BRR basura. 16 contadores, uno por pagina. */
uint64_t g_bram_page_w[16] = {0};
static uint8_t g_inp_dirty[4] = {0, 0, 0, 0};

void apu_writePortNow(Apu* apu, uint8_t port, uint8_t val) {
  g_apu_port_writes++;
  port &= 3;
  { extern void audio_trace_emit(const char *, const char *, unsigned, unsigned,
                                 const char *);
    audio_trace_emit("w214x", "cpu", 0x2140 + port, val, ""); }
  if (g_inp_dirty[port]) g_inp_overwritten++;
  g_inp_dirty[port] = 1;
  g_apu_stream_hash ^= (uint64_t)((port << 8) | val);
  g_apu_stream_hash *= 1099511628211ull;
  apu->inPorts[port] = val;
  audio_trace_on_cpu_port_apply(port, val);
}

static void apu_applyPortWrite(Apu* apu, const ApuPortWrite *w) {
  apu_writePortNow(apu, w->port, w->val);
}

uint32_t apu_portQueueDepth(const Apu* apu) {
  return apu->portQTail - apu->portQHead;
}

bool apu_schedulePortWrite(Apu* apu, uint8_t port, uint8_t val,
                           uint64_t guest_cycle) {
  if (apu_portQueueDepth(apu) >= APU_PORT_QUEUE_LEN) {
    g_apu_port_dropped++;
    return false;
  }
  if (apu_portQueueDepth(apu) > g_apu_port_queue_max)
    g_apu_port_queue_max = apu_portQueueDepth(apu);

  /* Establish a correspondence between guest time and wherever the
   * callback-driven SPC is now. If the callback later runs ahead, rebase at
   * the new current point; future guest deltas are still preserved. */
  if (!apu->portTimeValid || guest_cycle < apu->portLastGuest) {
    apu->portGuestAnchor = guest_cycle;
    apu->portTargetAnchor = apu->portClock;
    apu->portTimeValid = true;
  }

  uint64_t target = apu->portTargetAnchor +
                    (guest_cycle - apu->portGuestAnchor);
  if (target < apu->portClock) {
    apu->portGuestAnchor = guest_cycle;
    apu->portTargetAnchor = apu->portClock;
    target = apu->portClock;
  }
  if (target < apu->portLastTarget)
    target = apu->portLastTarget;

  ApuPortWrite *w = &apu->portQueue[apu->portQTail & (APU_PORT_QUEUE_LEN - 1)];
  w->target_cycle = target;
  w->port = (uint8_t)(port & 3);
  w->val = val;
  apu->portQTail++;
  apu->portLastGuest = guest_cycle;
  apu->portLastTarget = target;
  return true;
}

/* Apply all CPU bus events due before this SPC cycle executes. */
static void apu_drainPortQueue(Apu* apu) {
  while (apu->portQHead != apu->portQTail) {
    ApuPortWrite *w = &apu->portQueue[apu->portQHead & (APU_PORT_QUEUE_LEN - 1)];
    if (w->target_cycle > apu->portClock)
      break;
    apu_applyPortWrite(apu, w);
    apu->portQHead++;
  }
}

bool apu_runUntilPortQueueEmpty(Apu* apu, uint32_t max_cycles) {
  while (apu->portQHead != apu->portQTail) {
    if (max_cycles-- == 0)
      return false;
    apu_cycle(apu);
  }
  return true;
}

bool apu_runToGuestCycle(Apu* apu, uint64_t guest_cycle,
                         uint32_t max_cycles) {
  if (!apu->portTimeValid || guest_cycle < apu->portGuestAnchor)
    return true;

  uint64_t target = apu->portTargetAnchor +
                    (guest_cycle - apu->portGuestAnchor);
  if (target < apu->portLastTarget)
    target = apu->portLastTarget;

  for (;;) {
    bool due_event = false;
    if (apu->portQHead != apu->portQTail) {
      const ApuPortWrite *w =
          &apu->portQueue[apu->portQHead & (APU_PORT_QUEUE_LEN - 1)];
      due_event = w->target_cycle <= target;
    }
    if (apu->portClock >= target && !due_event)
      return true;
    if (max_cycles-- == 0)
      return false;
    apu_cycle(apu);
  }
}

bool apu_waitForTransferReady(Apu* apu, uint8_t request_port,
                              uint8_t request_value, uint32_t max_cycles) {
  for (;;) {
    if (apu->outPorts[0] == 0xaa && apu->outPorts[1] == 0xbb)
      return true;
    if (apu->inPorts[request_port & 3] != request_value)
      apu_writePortNow(apu, request_port, request_value);
    if (max_cycles-- == 0)
      return false;
    apu_cycle(apu);
  }
}

bool apu_finishHleTransfer(Apu* apu, uint16_t final_pc,
                           uint32_t max_cycles) {
  apu_writePortNow(apu, 2, (uint8_t)final_pc);
  apu_writePortNow(apu, 3, (uint8_t)(final_pc >> 8));
  apu_writePortNow(apu, 1, 0);
  apu_writePortNow(apu, 0, 0xcc);
  for (;;) {
    if (apu->outPorts[0] == 0xcc) {
      memset(apu->inPorts, 0, 4);
      return true;
    }
    if (max_cycles-- == 0)
      return false;
    apu_cycle(apu);
  }
}

void apu_saveload(Apu *apu, SaveLoadInfo *sli) {
  sli->func(sli, apu->ram, offsetof(Apu, pad) + 6 - offsetof(Apu, ram));
  dsp_saveload(apu->dsp, sli);
  spc_saveload(apu->spc, sli);
}

extern uint64_t g_spc_pc_histogram[0x10000];
extern int g_spc_pc_max_seen;

void apu_cycle(Apu* apu) {
  apu_drainPortQueue(apu);
  if(apu->cpuCyclesLeft == 0) {
    /* Sample PC right BEFORE running the opcode — so PC reflects the
     * instruction we're about to execute, not the post-opcode PC. */
    g_spc_pc_histogram[apu->spc->pc]++;
    if (apu->spc->pc > g_spc_pc_max_seen) g_spc_pc_max_seen = apu->spc->pc;
    apu->cpuCyclesLeft = spc_runOpcode(apu->spc);
  }
  apu->cpuCyclesLeft--;

  if((apu->cycles & 0x1f) == 0) {
    // every 32 cycles
    dsp_cycle(apu->dsp);
  }

  // handle timers
  extern uint64_t g_apu_timer0_total_ticks;
  for(int i = 0; i < 3; i++) {
    if(apu->timer[i].cycles == 0) {
      apu->timer[i].cycles = i == 2 ? 16 : 128;
      if(apu->timer[i].enabled) {
        apu->timer[i].divider++;
        if(apu->timer[i].divider == apu->timer[i].target) {
          apu->timer[i].divider = 0;
          apu->timer[i].counter++;
          apu->timer[i].counter &= 0xf;
          if (i == 0) g_apu_timer0_total_ticks++;
        }
      }
    }
    apu->timer[i].cycles--;
  }

  apu->cycles++;
  apu->portClock++;
}

uint8_t apu_cpuRead(Apu* apu, uint16_t adr) {
  switch(adr) {
    case 0xf0:
    case 0xf1:
    case 0xfa:
    case 0xfb:
    case 0xfc: {
      return 0;
    }
    case 0xf2: {
      return apu->dspAdr;
    }
    case 0xf3: {
      return dsp_read(apu->dsp, apu->dspAdr & 0x7f);
    }
    case 0xf4:
    case 0xf5:
    case 0xf6:
    case 0xf7: {
      uint8_t v = apu->inPorts[adr - 0xf4];
      g_spc_port_reads[adr - 0xf4]++;
      g_inp_dirty[adr - 0xf4] = 0;
      audio_trace_on_spc_port_read((uint8_t)(adr - 0xf4), v);
#if defined(SNESRECOMP_TRACE) && SNESRECOMP_TRACE
      if (getenv("SNESRECOMP_SPC_PORT_TRACE")) {
        static unsigned long n;
        if (n++ < 2000000)
          fprintf(stderr, "[spcport] pc=%04X port=%d val=%02X clock=%llu q=%u\n",
                  apu->spc->pc, adr - 0xf4, v,
                  (unsigned long long)apu->portClock,
                  (unsigned)(apu->portQTail - apu->portQHead));
      }
#endif
      return v;
    }
    case 0xf8:
    case 0xf9: {
      return apu->inPorts[adr - 0xf4];
    }
    case 0xfd:
    case 0xfe:
    case 0xff: {
      uint8_t ret = apu->timer[adr - 0xfd].counter;
      apu->timer[adr - 0xfd].counter = 0;
      return ret;
    }
  }
  if(apu->romReadable && adr >= 0xffc0) {
    return bootRom[adr - 0xffc0];
  }
  return apu->ram[adr];
}

/* Diagnostic counters: track SPC writes to specific addresses so we
 * can see whether the engine is touching outPorts at all. */
uint64_t g_spc_write_counts[0x100] = {0};

/* SPC PC histogram. Sampled once per apu_cycle that starts a new
 * opcode. Lets us answer "which PCs does the SPC spend time in?" */
uint64_t g_spc_pc_histogram[0x10000] = {0};
int g_spc_pc_max_seen = 0;
/* Per-value count for outPorts $F4-$F7. Index = (port_idx * 256) + val. */
uint64_t g_spc_outport_value_counts[4 * 256] = {0};
/* Last 32 outPort writes as a ring buffer: [adr, val] pairs. */
typedef struct { uint8_t adr; uint8_t val; } SpcWriteRec;
SpcWriteRec g_spc_recent_outport_writes[32];
int g_spc_recent_outport_idx = 0;

void apu_cpuWrite(Apu* apu, uint16_t adr, uint8_t val) {
  if (adr < 0x100) g_spc_write_counts[adr]++;
  if (adr >= 0xF4 && adr <= 0xF7) {
    int port = adr - 0xF4;
    g_spc_outport_value_counts[port * 256 + val]++;
    int i = g_spc_recent_outport_idx++ & 31;
    g_spc_recent_outport_writes[i].adr = (uint8_t)adr;
    g_spc_recent_outport_writes[i].val = val;
  }
  switch(adr) {
    case 0xf0: {
      break; // test register
    }
    case 0xf1: {
      for(int i = 0; i < 3; i++) {
        if(!apu->timer[i].enabled && (val & (1 << i))) {
          apu->timer[i].divider = 0;
          apu->timer[i].counter = 0;
        }
        apu->timer[i].enabled = val & (1 << i);
      }
      if(val & 0x10) {
        apu->inPorts[0] = 0;
        apu->inPorts[1] = 0;
        g_inp_dirty[0] = g_inp_dirty[1] = 0;
      }
      if(val & 0x20) {
        apu->inPorts[2] = 0;
        apu->inPorts[3] = 0;
        g_inp_dirty[2] = g_inp_dirty[3] = 0;
      }
      apu->romReadable = val & 0x80;
      break;
    }
    case 0xf2: {
      apu->dspAdr = val;
      g_spc_dsp_adr++;
      { extern void audio_trace_emit(const char *, const char *, unsigned,
                                     unsigned, const char *);
        audio_trace_emit("sdsp_addr", "spc", 0x00F2, val, ""); }
      break;
    }
    case 0xf3: {
      /* La nota lleva reg=XX, igual que la sonda de Mesen, para que las dos
       * trazas se puedan alinear sin depender del latch de $F2. */
      { extern void audio_trace_emit(const char *, const char *, unsigned,
                                     unsigned, const char *);
        char nota[16];
        snprintf(nota, sizeof nota, "reg=%02X", apu->dspAdr);
        audio_trace_emit("sdsp_data", "spc", 0x00F3, val, nota); }
      if(apu->dspAdr < 0x80) dsp_write(apu->dsp, apu->dspAdr, val);
      g_spc_dsp_dat++;
      break;
    }
    case 0xf4:
    case 0xf5:
    case 0xf6:
    case 0xf7: {
      audio_trace_on_spc_port_write((uint8_t)(adr - 0xf4), val);
      { extern void audio_trace_emit(const char *, const char *, unsigned,
                                     unsigned, const char *);
        audio_trace_emit("w214x", "spc", 0x2140 + (adr - 0xf4), val, "spc"); }
      apu->outPorts[adr - 0xf4] = val;
      g_spc_port_w++;
      break;
    }
    case 0xf8:
    case 0xf9: {
      apu->inPorts[adr - 0xf4] = val;
      break;
    }
    case 0xfa:
    case 0xfb:
    case 0xfc: {
      apu->timer[adr - 0xfa].target = val;
      break;
    }
  }
#if defined(SNESRECOMP_TRACE) && SNESRECOMP_TRACE
  if ((adr >= 0x0200 && adr < 0x02c0) || (adr >= 0x3900 && adr < 0x3c00)) {
    if (getenv("SNESRECOMP_RAMWRITE_TRACE")) {
      static unsigned long n;
      if (n++ < 4000000)
        fprintf(stderr, "[ramw] pc=%04X adr=%04X val=%02X\n", apu->spc->pc, adr, val);
    }
  }
#endif
  g_bram_page_w[adr >> 12]++;
  apu->ram[adr] = val;
}
