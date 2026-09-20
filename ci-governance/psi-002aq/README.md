# PSI-002AQ v0.4 — runner-neutral exact-source qualifier

This qualifier binds exact PSI-002A product `2be72da2acfd9903bfca168035c9ee087059f46f` and reconstructs the isolated three-crate experiment graph from canonical Git objects.

It adds no workflow and performs no GitHub or Git mutation.

## Required invocation

Run the canonical in-checkout qualifier itself with isolated Python:

```text
python3 -I -S -B ci-governance/psi-002aq/qualify.py --repo . --receipt-output /path/outside/checkout/receipt.json
```

The qualifier fails unless the executing file resolves to that canonical path and all four qualifier working files are regular, non-symlink files whose bytes exactly equal `HEAD` both before and after execution.

Python isolation requires isolated mode, ignored Python environment, no user site, no `site` initialization, safe path mode, and disabled bytecode writes.

## Git/object boundary

The qualifier disables global/system Git configuration for its subprocesses, rejects registered Git environment redirects/config injection, rejects dangerous local config (`core.worktree`, include paths, fsmonitor, external attributes/hooks, alternate-ref commands), and rejects grafts, alternates and replace refs.

It requires the exact one-commit/four-file qualifier topology above the exact PSI product and re-verifies the full subject after Rust execution.

## Toolchain boundary

The executable qualification profile is pinned to the inherited PEC line:

```text
rustc release = 1.96.0
cargo release = 1.96.0
```

Any other observed release is a FAIL rather than a new evidence lineage.

## Backend boundary

The qualifier independently requires:

```text
voprf crate version      0.5.0
upstream Git commit      f0531f0812387cd6be01923b21e2157399a9b295
crates.io package hash   28f59c30c76e2fea54cdece6a054e2662feffa7ab19658a7887524265ee39470
ciphersuite              ristretto255-SHA512
```

The generated offline `Cargo.lock` must contain exactly one registry-resolved `voprf 0.5.0` carrying that checksum.

## Authority ceiling

Even a successful execution remains synthetic and `Experimental`. It does not establish generic PSI security, enumeration resistance, anonymity, transport privacy, registry authenticity/freshness, operational key lifecycle, wire-format safety, real-data admission, production admission, or application authority.
