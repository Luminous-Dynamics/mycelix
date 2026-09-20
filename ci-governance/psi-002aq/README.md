# PSI-002AQ v0.5 — exact verifier, toolchain, and build-environment qualifier

This runner-neutral qualifier binds exact PSI-002A product `2be72da2acfd9903bfca168035c9ee087059f46f` without adding a GitHub Actions workflow.

## Required invocation

```text
python3 -I -S -B ci-governance/psi-002aq/qualify.py --repo . --receipt-output /path/outside/checkout/receipt.json
```

The verifier itself is evidence. It must execute from the canonical in-checkout path and all four qualifier working files must byte-match `HEAD` before and after the Rust run.

## Git and Python isolation

The qualifier requires isolated Python (`-I -S -B`), disables global/system Git configuration, rejects Git object/worktree/index/config/namespace/shallow redirects, rejects dangerous local Git config, and rejects grafts, alternates and replacement refs.

## Exact Rust/Cargo source identities

The inherited PEC toolchain is now bound beyond release strings:

```text
rustc release       1.96.0
rustc commit        ac68faa20c58cbccd01ee7208bf3b6e93a7d7f96
rustc commit date   2026-05-25
cargo release       1.96.0
cargo commit        30a34c6821b57de0aaec83a901aca39f88f6778c
cargo commit date   2026-05-25
```

`rustc -Vv` and `cargo -Vv` must match these identities exactly.

## Cargo/Rust build environment

The qualifier rejects semantic build overrides such as `RUSTC`, `RUSTFLAGS`, Rust wrappers, encoded Cargo rustflags, build target/target-dir/build-dir overrides, Cargo profile overrides, target-specific overrides, registry overrides and source overrides.

Before Cargo executes, the qualifier also rejects any `.cargo/config.toml` or `.cargo/config` in Cargo's workspace-ancestor search path or the effective Cargo home. Offline registry/package caches may remain present; configuration that can alter resolution or compilation may not.

## Backend identity

The PSI experiment remains bound to:

```text
voprf crate version      0.5.0
upstream Git commit      f0531f0812387cd6be01923b21e2157399a9b295
crates.io package hash   28f59c30c76e2fea54cdece6a054e2662feffa7ab19658a7887524265ee39470
ciphersuite              ristretto255-SHA512
```

The generated offline Cargo.lock must contain exactly one registry-resolved `voprf 0.5.0` carrying that checksum.

## Authority ceiling

Even PASS is only exact execution evidence for the frozen synthetic experiment. It does not establish generic PSI security, enumeration resistance, anonymity, transport privacy, registry authenticity/freshness, operational key lifecycle, wire-format safety, real-data admission, production admission, or application authority.
