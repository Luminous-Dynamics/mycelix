# Integral Reference Solution shell

This directory is the first executable structural tranche of `MYC-INT-017B`.

It intentionally does **not** implement Integral governance, ITC economics, OAD certification, COS actuation, FRS authority, PostgreSQL persistence, Holochain federation, or Symthaea analysis. Those remain owned by their respective qualified tranches.

What this shell does own:

- exact external source/status identities for the six current `SPEC-DS-*` structures and three current `SPEC-IF-*` interfaces;
- five distinct Integral adapter namespaces (`CDS`, `OAD`, `ITC`, `COS`, `FRS`);
- a maturity model that cannot display designed/source-implemented/local-test work as operational;
- a UI-neutral navigation contract that the canonical Leptos surface can consume later.

Core boundaries:

```text
Integral object != canonical Mycelix primitive
Integral schema identity != database row ID != Holochain ActionHash
Integral ITC != MYCEL != SAP != TEND
FRS recommendation != CDS decision != authorization
DecisionPacket != Authorization != EffectReceipt
Designed/queued/source-implemented != repository-qualified
```

The package is temporarily an independent Cargo workspace so its structural tests can be run without forcing unqualified Integral runtime dependencies into the parent Mycelix workspace.

## Local qualification target

```bash
cd mycelix-workspace/integrations/integral
cargo fmt --check
cargo test
cargo clippy --all-targets -- -D warnings
```

A successful local run would establish only this structural shell. It would not establish a working Integral node or any authority/runtime theorem.
