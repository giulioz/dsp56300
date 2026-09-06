# DSP56300 architectural regression audit

The September 2026 repairs use the DSP56300 Family Manual as the architectural
reference. Interpreter/JIT agreement alone is not the oracle. The independent
`source/dsp56kTestRunner/manualAudit.cpp` harness computes expected results
from architectural integers and explicit tables, without calling the core's
arithmetic or address-generation helpers.

## References and scope

- [DSP56300 Family Manual, revision 5](https://www.nxp.com/docs/en/reference-manual/DSP56300FM.pdf).
- [Family Manual addendum, revision 6](https://www.nxp.com/docs/en/reference-manual/DSP56300FMAD.pdf).
  Its PLL diagram and ASL/ASR typography corrections do not change these arithmetic expectations.
- [DSP56362 User Manual](https://www.nxp.com/docs/en/user-guide/DSP56362UM.pdf).
  Derivative peripherals and interrupt arbitration remain separate from family-core instruction semantics.

| Behavior | Manual basis | Regression coverage |
| --- | --- | --- |
| SA accumulator arithmetic and long transfers | FM 3.4.2, figure 3-10, tables 3-3/3-4 | Concatenate the 40 significant bits during arithmetic; restore the two padding gaps. Check long-memory transfers, ignored input padding, signed multiply, rounding, logical operations and scaling. |
| ADD/SUB/CMP and combined shift/arithmetic | FM table 5-1; instruction definitions in chapter 13; ADDL/ADDR pp. 13-9/10, SUBL/SUBR pp. 13-174/175 | Signed boundaries, carry/borrow, V/N/Z and sticky L. Combined left operations retain overflow from either stage. Do not assert C where shift overflow makes it unspecified. |
| Arithmetic saturation | FM 3.2.3, table 3-1; SA constant in 3.4.2 | EXT[7]/EXT[0]/MSP[23] detection, all table patterns, V/L, three scaling modes, post-rounding detection and scale-independent rounded constants. Selected excluded operations behave identically with SM clear/set. |
| ABS and NEG | FM pp. 13-5 and 13-144; table 5-1 | Ordinary minimum-value overflow, SM limiting, preserved C, sticky L, both accumulators and normal/SA modes. |
| Transfer S | FM table 5-1 | Sticky adjacent-bit equations from both pre-transfer accumulators, three scaling modes, register/memory/long transfers and parallel CLR. |
| Modulo addressing | FM 4.5.3 and table 4-2 | Underflow below address zero, legal offsets and boundaries, ignored Mn high byte with full readback, immediate/register/memory/control-register writes. |
| Shift, rotate and normalization flags | FM 3.4 and chapter 13, including CLB and NORMF p. 13-147 | Full-width and zero shifts, carry at architectural word boundaries, CLB result flags and NORMF carry preservation. |
| REP and DO execution | FM chapter 5 and chapter 13 instruction definitions | REP owns its body across cached-code boundaries and invalidation; DO can yield to a board scheduler without changing iteration counts or long-interrupt restoration. |
| Assembler operands | FM tables 12-13/12-16 and MOVEC p. 13-130 | Assert signed MPY/MAC, CMP B,A and TFR B,A encodings. Control-register copies are accepted. |
| External serial cadence | Derivative PLL clock ratios; integer deadline arithmetic | Exact cumulative deadlines for fractional periods, delayed service, rate/speed changes and fractional PLL frequency. |

The suite includes 800 valid DIV scenarios per engine as a regression for
REP ownership and quotient behavior; this is not complete DIV conformance.

The JIT metadata and register-allocation repairs address emulator implementation
defects, rather than additional chip behaviors: pooled block metadata must not
inherit stale CCR overwrite masks, and a parallel move must not exhaust fixed
scratch registers or skip required register loads. The manual audit forces
metadata reuse; the existing full core runner exercises parallel X/Y moves and
dynamic-mode flag helpers. Interpreter CCR-cache tests likewise verify that
partial flag writes preserve the architectural result of preceding instructions.

Runtime engine selection now consistently controls interrupt dispatch, cycle
accounting and threaded execution. The optional interpreter DO-yield interface
is a scheduler mechanism; REP remains atomic. These implementation choices are
covered by core tests and are not claims of cycle-exact pipeline emulation.

## Running the tests

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target dsp56kManualAudit dsp56kTestRunner -j18
ctest --test-dir build -R '^dsp56300_(manualAudit|unitTests)$' --output-on-failure
```

`dsp56kManualAudit` runs the interpreter, unlinked JIT and linked JIT against
the same expectations. `--tables` selects the original 120 saturation/transfer
table checks, which are also included in the default suite. Those checks
previously reproduced 78 failures per engine and now pass unchanged.

The default suite has 42,474 interpreter checks and 42,476 per JIT configuration.
Both ARM64 and x86-64 generators have passed, as has the full core/optimizer
runner on both architectures. On a universal macOS build, run the copied audit
and core executables with `arch -x86_64` to exercise the x86-64 slices. Copy
executables only after the build completes, and do not rebuild an executable
while it is running.

## Remaining gaps

Passing these subsets does not certify the entire core or hardware audio fidelity.
In particular, this pass does not complete SM coverage for single-bit ASL/ASR,
INC/DEC, NORM/TST, DMACss, every multiply/parallel form or mode transitions.
Exclusion tests establish SM independence, not independent conformance of every
excluded operation. SA transition latency and every AGU/loop boundary also need
further work.

Programmable DSP56362 interrupt priorities, pending-event arbitration, hardware
instruction-cache behavior, unsupported instruction forms, all DMA/serial modes,
reset defaults and pipeline/bus timing remain open audit areas. Reserved modes
and explicitly undefined outcomes are excluded from test assertions.
