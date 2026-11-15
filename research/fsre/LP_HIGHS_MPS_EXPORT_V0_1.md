# FSRE exact MPS export contract for HiGHS

The LP certificate crate now emits deterministic free-format MPS for the canonical model:

\[
\max c^\top x\quad\text{s.t.}\quad Ax\le b,\quad x\ge0.
\]

## Exact decimal policy

MPS numeric fields are decimal text. A rational \(p/q\) has an exact finite decimal representation only when the normalized denominator has no prime factors other than 2 and 5. The exporter:
- normalizes rationals before inspection;
- converts only finite-decimal rationals, with checked arithmetic;
- rejects non-terminating denominators (for example, 1/3);
- rejects values that need an unsupported decimal scale or overflow;
- never rounds or approximates the LP during export.

The initial fixture contains integer coefficients and reduces three columns to two under the exact equality-column rule. Its MPS output is frozen and compared byte-for-byte in CI.

## MPS meaning

- Objective sense is maximization.
- All rows are upper-bound (L) rows representing \(Ax\le b\).
- The objective row is N / OBJ.
- Variables use MPS's default lower bound of zero.
- No integer/binary marker is emitted; this exporter supports continuous variables only.
- Row/column names are preserved and restricted to safe free-format tokens.

If a future model adds variable bounds, integrality, SOS, indicator constraints, or quadratic terms, the exporter must reject unsupported semantics until those are represented explicitly. Silent dropping is forbidden.

## Solver integration

The CLI supports:

    cargo run --quiet --locked --offline --manifest-path tools/fsre-partial-id-lp-certificate/Cargo.toml -- --export-mps input.psv

The result bytes are hashed by the calling harness and passed to HiGHS with a frozen option set. Preserve both the canonical FSRE input bytes and exported MPS bytes. Solver versions, options, hardware, build hash and termination state belong in the run receipt.

HiGHS's floating-point primal and dual vectors remain proposals. FSRE reconstructs rationals only when exact conversion is defensible, then sends those vectors through the exact primal/dual and Farkas verifiers. Numerical-only residuals never become exact proof.
