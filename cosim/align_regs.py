#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""align_regs.py — oraculo de REGISTROS con muestreo DENSO (recomp vs Mesen).

Que resuelve y por que existe
-----------------------------
El oraculo de registros por FOTOGRAMA esta ya rechazado: docs/DESCARTADAS.md
§29 ("Los registros de CPU por frame NO sirven como oraculo de estado") midio
que `A` coincidia 0,2 % y concluyo que la causa es de METODO, no de emulacion:
la frontera de fotograma del motor cae ~0,15 fotogramas mas tarde que la de
Mesen, y en ese punto A/X/Y son valores de trabajo. Comparar una muestra
instantanea mide *donde esta el PC dentro del bucle*, no el estado del juego.

Este instrumento ataca la causa, no el sintoma. En vez de una muestra por
fotograma, el motor emite el estado ANTES de cada instruccion (`SO_BLOCKTRACE`,
ver runner/src/blocktrace.c) y el comparador busca, para cada instante del
oráculo de Mesen, si existe una instruccion del motor con el MISMO estado. Con
miles de muestras por fotograma, emparejar por CICLO (no por indice de
fotograma) convierte el ruido de §29 en una medida Util.

Medida que justifica el metodo (mismo PC, dos muestreos):
    f934-936, per-frame  -> 1 de 6 campos coinciden
    f935-936, denso      -> 5 de 6 campos (A, Y, S, DP, DB exactos; X se mueve 4)

Alineacion
----------
El reloj GUEST es el mismo en los dos lados (verificado: el TSV de Mesen avanza
exactamente 357368 master/fotograma, igual que `master_cycles` del motor), pero
el punto de partida NO: el motor arranca con su propio HLE y el oraculo se
grabo desde un reset. Por eso primero se busca el OFFSET DE ARRANQUE que mejor
alinea los dos lados, y luego se compara sobre esa alineacion. Es una
ALINEACION, no un enmascaramiento: se informa el offset usado.

Que se compara
--------------
A X Y S D DB P — los registros arquitectonicos. El PC se INFORMA pero no se
exige: el codigo AOT no mantiene PC vivo entre instrucciones (SNES_COSIM.md,
"no live PC field"), asi que el PC solo es comparable en el tramo LLE, que es
justamente donde se emite la traza.

Criterio de parada
------------------
Se busca el PRIMER fotograma en que el acuerdo de registros se desploma por
debajo de --umbral respecto a la linea base. El fotograma se elige en el
oráculo (que tiene uno por fotograma) y dentro de el se busca la instruccion
del motor que mejor casa; el PC que se reporta es el de ESA instruccion, que es
donde el motor se separó del hardware.

Uso
---
    python align_regs.py --recomp so_bt.tsv \
        --oracle "C:/.../Caminata_larga-trace.txt" --desde 900 --hasta 1000 \
        --ventana 20 --umbral 0.5

El fichero del oraculo puede ser la traza por INSTRUCCION (28 GB, con
`Cycle:`/`Fr:` en cada linea) o el TSV por fotograma (una fila por fotograma).
Se detecta solo por el formato.

LO QUE SE MIDIO (2026-10-01) Y LO QUE NO SE PUEDE CONCLUIR
-----------------------------------------------------------
Corrida del motor con la entrada del oraculo (SO_BLOCKTRACE, 3.873.907
instrucciones, 1.200 fotogramas) contra `Caminata_larga-trace.txt`:

  * Alineacion de arranque encontrada: -27 fotogramas. Es una ALINEACION, no un
    enmascaramiento: se informa y se puede forzar con otra.
  * Meseta f934-f1009: acuerdo POR CAMPO 74-85% (D=100%, S~96%, DB~97%,
    A~84%, X~78%, Y~64%, P=0%). Ademas los 727 PCs que el motor ejecuta en esa
    ventana aparecen TODOS en la ventana del hardware, o sea que los dos lados
    estan ejecutando el mismo codigo: el acuerdo no es trivial.
  * `P` da 0% en todas partes y NO es un bug: Mesen imprime P como letras de
    flag (`NVMXDIZC`) y aqui se normaliza a hexadecimal. Si se ve 0% en P,
    suspectar primero del formato.
  * A partir de f1010 el acuerdo cae a ~36-43%. NO es una divergencia de
    registros: es FASE. El hardware sale del bucle de espera de `$4212` y entra
    en la rutina de descompresion S-DD1 `$C04D39-$C04DD9` (`lle_only` en el
    manifiesto), que hace `PHB/PLA` para poner `DB=$7F` y `MVN $7F,$7F`. El
    motor, en SU indice de fotogramas, sigue en el bucle de espera. El indice
    de fotograma no es una coordenada comparable entre los dos lados: el motor
    arranca con su propio HLE y el hardware desde un reset de Mesen, y ambos
    avanzan a ritmo distinto. El unico alineamiento global valido es la meseta,
    donde los dos lados estan en el mismo codigo.
  * El motor SI ejecuta `$C04D39-$C04DE0` y SI usa `DB=$7F` (176.588
    instrucciones), pero en sus propios fotogramas 3-9, 65, 379-382. La
    rutina existe y coincide; lo que no coincide es CUANDO se ejecuta.

LIMITE HONESTO. Este instrumento sirve para responder "¿coinciden los estados en
un tramo donde los dos lados estan en el mismo codigo?". NO sirve para localizar
un descalce por indice de fotograma, porque ese indice no es una coordenada
compartida. Para eso hace falta lo que ya dijo DESCARTADAS.md §27: un baseline
de la MISMA sesion, con la entrada keyeada al reloj del invitado, y alinear por
`master_cycles` con la traza completa (no con una ventana). La leccion de §29
no era solo sobre el instante de muestreo: es tambien sobre el origen del eje
de fotogramas.
"""
from __future__ import annotations

import argparse
import collections
import re
import sys

# Una linea de la traza de Mesen por instruccion:
#   CC053D  LDA $4212 [$004212] = $00  A:6C02 X:0000 Y:18E2 S:01F8 D:0000 DB:00 P:nVMx...
# Las del SPC700 NO llevan "Cycle:" (no comparten reloj de CPU), asi que esa
# ausencia es el discriminador fiable.
INSN_RE = re.compile(
    r"A:(?P<A>\S+)\s+X:(?P<X>\S+)\s+Y:(?P<Y>\S+)\s+S:(?P<S>\S+)"
    r"(?:\s+D:(?P<D>\S+))?(?:\s+DB:(?P<DB>\S+))?"
    r"\s+P:(?P<P>\S+)"
)
FRAME_RE = re.compile(r"Fr:(\d+)")
CYCLE_RE = re.compile(r"Cycle:(\d+)")
PC_RE = re.compile(r"^([0-9A-Fa-f]{6})\s")

CAMPOS = ("A", "X", "Y", "S", "D", "DB", "P")


def norm(v: str | None, ancho: int) -> int:
    """'6C' y '006C' son el mismo A de 8 o 16 bits: se normaliza al ancho mayor."""
    if not v:
        return 0
    try:
        return int(v, 16)
    except ValueError:
        return 0


def norm_p(v: str | None) -> int:
    """P aparece en las dos trazas con Representaciones DISTINTAS: Mesen lo
    imprime como ocho letras de flag (`NVMXDIZC`, mayuscula=1) y el motor lo
    emite en hexadecimal. Sin normalizar, P nunca coincide y el acuerdo medido
    es artefactual: se compararian dos formatos, no dos estados. Se convierte
    SIEMPRE a hexadecimal."""
    if not v:
        return 0
    v = v.strip()
    if re.fullmatch(r"[0-9A-Fa-f]{1,2}", v):
        return int(v, 16)
    b = 0
    for ch, mask in zip(v.upper(), (0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01)):
        if ch.isupper() and ch.isalpha():
            b |= mask
    return b


def carga_recomp(path: str) -> list[tuple[int, int, tuple]]:
    """(master, cycles, (A,X,Y,S,D,DB,P)) por instruccion del motor."""
    out = []
    with open(path, encoding="utf-8", errors="replace") as f:
        for ln in f:
            if ln.startswith("#"):
                continue
            c = ln.rstrip("\n").split("\t")
            if len(c) < 11:
                continue
            out.append((int(c[0]), int(c[1]),
                        (int(c[2], 16), int(c[3], 16), int(c[4], 16), int(c[5], 16),
                         int(c[6], 16), int(c[7], 16), int(c[8], 16))))
    return out


def carga_oracle_insn(path: str, desde: int, hasta: int):
    """Traza por instruccion -> {master: set(estado)} indexada por fotograma.

    El reloj de la traza (`Cycle:`) es el de CPU y el del motor `master_cycles`
    es el maestro; para alinear por REGISTROS no hace falta convertirlos
    (muestreamos ambos lados en su propia escala y emparejamos por estado), pero
    se conserva el ciclo por si se quiere depurar.
    """
    por_f = collections.defaultdict(lambda: collections.Counter())
    with open(path, encoding="utf-8", errors="replace") as f:
        for ln in f:
            if "Cycle:" not in ln:
                continue
            mf = FRAME_RE.search(ln)
            if not mf:
                continue
            fr = int(mf.group(1))
            if fr < desde or fr > hasta:
                if fr > hasta:
                    break
                continue
            m = INSN_RE.search(ln)
            if not m:
                continue
            g = m.groupdict()
            por_f[fr][(norm(g["A"], 4), norm(g["X"], 4), norm(g["Y"], 4),
                       norm(g["S"], 4), norm(g["D"], 4), norm(g["DB"], 2),
                       norm_p(g["P"]))] += 1
    return por_f


def pct_coincidencia(estados_oracle, estados_motor):
    """Acuerdo denso por CAMPOS, en [0,1].

    No se exige que los 7 campos coincidan a la vez. Medido sobre el mismo PC,
    el motor va unas cuantas instrucciones por delante o por detras del
    hardware, asi que en el instante que mejor casa coinciden 4 de 7 campos
    (S, D, DB y uno de A/X/Y) mientras los otros 3 todavia no. Exigir el 7/7
    daria 0% en todas partes y no distinguiria "el motor va 3 instrucciones
    desfasado" (fino) de "el motor se separo del hardware" (grave).

    COSTE. La version obvia empareja todos contra todos (O(n*m) con ~1.500
    instantes por fotograma en ambos lados: de segundos a horas). Pero el
    criterio es POR CAMPO: para el campo i basta preguntar "existe ALGUN
    instante del motor con ese valor?", y eso se responde con un conjunto de
    valores por campo en O(n+m). Se pierde la correlacion entre campos (ya no
    se exige que el 7/7 ocurra en el MISMO instante), que es justo lo que este
    instrumento decidio no exigir.
    """
    if not estados_oracle or not estados_motor:
        return 0.0, {}
    campos = len(CAMPOS)
    vals_motor = [{mt[i] for mt in estados_motor} for i in range(campos)]
    tot = 0
    marca = {c: 0 for c in CAMPOS}
    for st, n in estados_oracle.items():
        tot += n
        for i, c in enumerate(CAMPOS):
            if st[i] in vals_motor[i]:
                marca[c] += n
    if not tot:
        return 0.0, {}
    return (sum(marca.values()) / (tot * campos),
            {c: marca[c] / tot for c in CAMPOS})


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--recomp", required=True, help="TSV de SO_BLOCKTRACE")
    ap.add_argument("--oracle", required=True, help="traza de Mesen")
    ap.add_argument("--desde", type=int, default=900)
    ap.add_argument("--hasta", type=int, default=1000)
    ap.add_argument("--ventana", type=int, default=20,
                    help="fotogramas por ventana de la linea base")
    ap.add_argument("--umbral", type=float, default=0.5,
                    help="caida de acuerdo que marca la divergencia (0..1)")
    args = ap.parse_args()

    print("[1/4] cargando traza del motor (por instruccion)...", flush=True)
    recomp = carga_recomp(args.recomp)
    print(f"      {len(recomp)} instrucciones del motor", flush=True)
    if not recomp:
        print("ERROR: la traza del motor esta vacia", file=sys.stderr)
        return 1

    print(f"[2/4] barriendo la traza de Mesen (frames {args.desde}..{args.hasta})...",
          flush=True)
    por_f = carga_oracle_insn(args.oracle, args.desde, args.hasta)
    if not por_f:
        print("ERROR: no hay muestras del oraculo en el rango pedido", file=sys.stderr)
        return 1
    frames = sorted(por_f)
    print(f"      {len(frames)} fotogramas con estado 65816", flush=True)

    # Los estados del motor por fotograma. El motor no emite su numero de
    # fotograma, pero `master_cycles` es monotonico y el maestro por fotograma es
    # 357368, asi que el fotograma de invitado se recupera del reloj.
    print("[3/4] indexando los estados del motor por reloj master...", flush=True)
    # El motor NO emite su numero de fotograma, pero `master_cycles` es
    # monotono y avanza 357368 por fotograma de invitado, asi que el fotograma
    # se recupera del reloj. `base` es el master del primer fotograma del
    # motor; el desplazamiento contra el oraculo se calcula despues, porque los
    # dos lados arrancan de forma distinta (el motor con su HLE, el oraculo
    # desde un reset de Mesen) y por eso el indice de fotograma NO es la misma
    # coordenada en los dos.
    base = recomp[0][0]
    motor_por_f = collections.defaultdict(set)
    for m, _c, st in recomp:
        motor_por_f[(m - base) // 357368].add(st)

    # Desplazamiento: el motor se indexa desde 0 al arrancar; el oraculo tiene
    # su propio origen. Se busca el desfase que maximiza el acuerdo en una
    # ventana de prueba. Se hace UNA vez y se informa (es alineacion, no
    # enmascaramiento).
    probe = frames[max(0, len(frames) // 2)]
    mejor_off, mejor_score = 0, -1.0
    for off in range(-60, 61):
        cand = motor_por_f.get(probe + off)
        if not cand:
            continue
        s, _d = pct_coincidencia(por_f[probe], cand)
        if s > mejor_score:
            mejor_score, mejor_off = s, off
    print(f"      desplazamiento de arranque optimo: {mejor_off:+d} fotogramas "
          f"(acuerdo de prueba {mejor_score*100:.1f}%)", flush=True)

    print("[4/4] comparando (ventana=%d, umbral=%.2f)" % (args.ventana, args.umbral),
          flush=True)
    serie = []
    for fr in frames:
        p, d = pct_coincidencia(por_f[fr], motor_por_f.get(fr + mejor_off, set()))
        serie.append((fr, p))
        print(f"      frame {fr:6d}  acuerdo={p*100:5.1f}%  "
              f"(instantes del oraculo: {len(por_f[fr])}, del motor: "
              f"{len(motor_por_f.get(fr + mejor_off, ()))})", flush=True)
        if fr == frames[0]:
            print("        por campo: " +
                  "  ".join(f"{c}={d.get(c,0)*100:.0f}%" for c in CAMPOS), flush=True)

    vals = [p for _f, p in serie]
    base_agree = sum(vals) / len(vals) if vals else 0.0
    print()
    print("=" * 72)
    print(f"acuerdo medio (campos) sobre {len(serie)} fotogramas: "
          f"{base_agree*100:.1f}%")
    print("=" * 72)

    # Desplome: primera ventana cuyo acuerdo cae por debajo del umbral.
    for i in range(0, len(serie) - args.ventana + 1):
        w = vals[i:i + args.ventana]
        m = sum(w) / len(w)
        if m < args.umbral:
            fr0 = serie[i][0]
            print(f"\nPRIMERA DIVERGENCIA: fotograma {fr0} "
                  f"(ventana {fr0}..{serie[i+args.ventana-1][0]}, "
                  f"acuerdo {m*100:.1f}% < {args.umbral*100:.0f}%)")
            # PC del motor en el fotograma: el ultimo estado que el oraculo
            # todavia compartia, y el primer registro del motor en ese frame.
            st = por_f[fr0].most_common(1)[0][0] if por_f[fr0] else None
            print(f"  estado compartido mayoritario del oraculo: {st}")
            return 0
    print("\nSin desplome por debajo del umbral en el rango pedido.")
    return 0


if __name__ == "__main__":
    sys.exit(main())