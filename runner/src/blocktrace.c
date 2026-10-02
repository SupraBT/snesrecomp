/* blocktrace.c — traza de BLOQUE (no de fotograma) del estado del invitado.
 *
 * POR QUE EXISTE. El oraculo de estado por fotograma esta rechazado
 * (docs/DESCARTADAS.md §29, "Los registros de CPU por frame NO sirven como
 * oraculo de estado"): la frontera de fotograma del motor cae ~0,15 fotogramas
 * mas tarde que la de Mesen, y en ese punto A/X/Y son valores de trabajo, asi
 * que el acuerdo medido es ruido (A coincidia 0,2 %). La leccion de §29 fue
 * que el instante de muestreo manda sobre el valor.
 *
 * Este modulo ataca justo eso: en vez de UN valor por fotograma, escribe un
 * registro (master, A, X, Y, S, D, DB, PB, P, PC) en CADA frontera de bloque
 * basico. Con miles de muestras por fotograma, la chance de que exista un
 * instante del mismo PC con el mismo estado es alta, y el comparador puede
 * emparejar por ciclo en vez de por indice de fotograma.
 *
 * MEDIDA que justifica el muestreo denso (traza de Mesen por instruccion,
 * frames 934-936, contra la corrida del motor con la misma entrada):
 *   - muestreo por fotograma (1 muestra/frame): 1 de 6 campos coincidentes
 *   - muestreo por bloque (denso):             5 de 6 campos coincidentes
 *   (A, Y, S, DP y DB exactos; solo X se mueve 4 unidades = otra instruccion)
 *
 * POR QUE cpu_dispatch_pc_from. Es el UNICO choke point por el que pasa toda
 * frontera de bloque basico: el codigo AOT resuelve el flujo de control con
 * llamadas C nativas y NO mantiene un PC vivo entre instrucciones (ver
 * SNES_COSIM.md: "No live PC field"), asi que ahi el PC de 24 bits es la
 * direccion REAL del bloque que se va a ejecutar. Un unico hook nos da
 * cobertura completa sin tocar el codigo generado.
 *
 * COMPILACION FUERA. Todo el modulo cuelga de un static inicializado a -1 y
 * de un getenv leido UNA vez; con la variable ausente el unico coste es una
 * comparacion de int por bloque, y el fprintf no llega a ejecutarse nunca.
 *
 * Variables de entorno:
 *   SO_BLOCKTRACE=<fichero>   activa la traza (obligatorio; sin esto no hace nada)
 *   SO_BLOCKTRACE_FROM=<n>    primer fotograma a trazar (por defecto 0 = todos)
 *   SO_BLOCKTRACE_UNTIL=<n>   ultimo fotograma (por defecto = infinito)
 *   SO_BLOCKTRACE_EVERY=<n>   1 de cada N bloques (por defecto 1 = denso)
 *
 * Formato de salida (TSV, una linea por bloque):
 *   master<TAB>cycles<TAB>A<TAB>X<TAB>Y<TAB>S<TAB>D<TAB>DB<TAB>PB<TAB>P<TAB>pc24
 * Todo en mayusculas hex, sin 0x. `master` y `cycles` son el reloj MAESTRO y
 * el de CPU respectively: son las dos columnas con las que se alinea contra
 * la traza de Mesen (que reporta `master` y `cyc`).
 */

#include "cpu_state.h"
#include <stdio.h>
#include <stdlib.h>

/* La instrumentacion de traza densa es OPCIONAL y por defecto esta
 * DESACTIVADA en tiempo de compilacion. Con SNESRECOMP_BLOCKTRACE=0 (defecto)
 * los llamantes en el bucle caliente ni siquiera existen, asi que el bucle LLE
 * queda exactamente como estaba antes de añadir esta herramienta: cero
 * instrucciones, cero llamadas, cero coste. Eso importa porque el punto de
 * enganche esta DENTRO del bucle de opcodes del interprete, que se ejecuta
 * millones de veces por fotograma; una llamada a funcion por opcode, aunque
 * solo compare un puntero nulo, es lo bastante cara como para ensuciar las
 * medidas de FPS que se quieren cerrar.
 *
 * Para medir (solo diagnostico, nunca en produccion):
 *   cmake -S . -B build-trace -DSNESRECOMP_BLOCKTRACE=ON ...
 *   SO_BLOCKTRACE=<fichero> ./StarOcean.exe
 */
#ifndef SNESRECOMP_BLOCKTRACE
#define SNESRECOMP_BLOCKTRACE 0
#endif

#if !SNESRECOMP_BLOCKTRACE

/* Compilado fuera: el fichero sigue formando parte del proyecto para que el
 * objetivo exista cuando se activa, pero no aporta ni un solo simbolo. */
int snes_frame_counter_bt_unused_;

#else

int snes_frame_counter;

FILE *g_blocktrace_fp = NULL;
int  g_blocktrace_on = 0;
int  g_blocktrace_every = 1;
unsigned long g_blocktrace_n = 0;
unsigned long g_blocktrace_calls = 0;
int  g_blocktrace_from = 0;
int  g_blocktrace_until = -1;

/* Llamado una vez desde el arranque; es una sola lectura de getenv. */
void blocktrace_init(void) {
    const char *pth;
    if (g_blocktrace_fp || g_blocktrace_on) return;   /* ya inicializado */
    g_blocktrace_on = 1;                              /* evita reentrada */
    pth = getenv("SO_BLOCKTRACE");
    if (!pth || !pth[0]) { g_blocktrace_on = 0; return; }
    g_blocktrace_fp = fopen(pth, "w");
    if (!g_blocktrace_fp) {
        fprintf(stderr, "[blocktrace] no he podido abrir '%s'\n", pth);
        g_blocktrace_on = 0;
        return;
    }
    {
        const char *e;
        if ((e = getenv("SO_BLOCKTRACE_FROM")) && e[0]) g_blocktrace_from = atoi(e);
        if ((e = getenv("SO_BLOCKTRACE_UNTIL")) && e[0]) g_blocktrace_until = atoi(e);
        if ((e = getenv("SO_BLOCKTRACE_EVERY")) && e[0]) {
            int v = atoi(e);
            if (v >= 1) g_blocktrace_every = v;
        }
    }
    setvbuf(g_blocktrace_fp, NULL, _IOFBF, 1 << 20);
    { extern void blocktrace_atexit(void); atexit(blocktrace_atexit); }
    fprintf(g_blocktrace_fp,
            "# SO_BLOCKTRACE master\tcycles\tA\tX\tY\tS\tD\tDB\tPB\tP\tpc24\n");
    fprintf(stderr, "[blocktrace] activo -> %s (from=%d until=%d every=%d)\n",
            pth, g_blocktrace_from, g_blocktrace_until, g_blocktrace_every);
}

/* Hook de frontera de bloque. `cpu` es el estado ANTES de ejecutar el bloque,
 * que es justo el instante en que el oraculo por instruccion tambie samplea. */
void blocktrace_emit(CpuState *cpu, uint32_t pc24) {
    FILE *f = g_blocktrace_fp;
    if (!f) return;
    g_blocktrace_calls++;
    if (g_blocktrace_from && snes_frame_counter < g_blocktrace_from) return;
    if (g_blocktrace_until >= 0 && snes_frame_counter > g_blocktrace_until) return;
    if (g_blocktrace_every > 1 && (g_blocktrace_n % (unsigned long)g_blocktrace_every)) {
        g_blocktrace_n++;
        return;
    }
    g_blocktrace_n++;
    fprintf(f, "%llu\t%llu\t%04X\t%04X\t%04X\t%04X\t%04X\t%02X\t%02X\t%02X\t%06X\n",
            (unsigned long long)cpu->master_cycles,
            (unsigned long long)cpu->cycles,
            cpu->A, cpu->X, cpu->Y, cpu->S, cpu->D,
            cpu->DB, cpu->PB, cpu->P,
            (unsigned)(pc24 & 0xFFFFFFu));
}

/* Cierre voluntary desde el codigo de salida del anfitrion. */
void blocktrace_flush(void) {
    if (g_blocktrace_fp) {
        fflush(g_blocktrace_fp);
        fclose(g_blocktrace_fp);
        g_blocktrace_fp = NULL;
    }
}

/* Registrado al abrir el fichero: el bucle principal puede morir por un
 *SNESRECOMP_EXIT_AT_FRAME limpio o por un kill externo, y en ambos casos el
 * anillo de 1 MB debe llegar a disco. Sin esto, una traza que se lea mientras
 * el motor corre sale vacia. */
static void blocktrace_atexit(void) {
    fprintf(stderr, "[blocktrace] bloques emitidos=%lu (filas=%lu)\n",
            g_blocktrace_calls, g_blocktrace_n);
    blocktrace_flush();
}
#endif /* SNESRECOMP_BLOCKTRACE */
