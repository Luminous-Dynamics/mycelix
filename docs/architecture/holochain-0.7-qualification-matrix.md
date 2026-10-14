# Holochain 0.7 Qualification Matrix

**Status:** working qualification record  
**Scope:** Mycelix shared-crate consumers and Hearth 0.7 migration  
**Created:** 2026-09-29

This document distinguishes source migration from runtime qualification. A declaration, queued CI run, skipped job, or unexecuted test is not a PASS.

## Compatibility baseline

| Component | 0.7 target |
|---|---|
| Holochain / conductor | 0.7.0 |
| HDK | 0.7.0 |
| HDI | 0.8.0 |
| Rust client | 0.9.0 |
| JavaScript client | 0.21.0 |
| Lair | 0.7.1 |
| Kitsune2 bootstrap | 0.5.0 |
| hc | 0.7.0 |
| hc-scaffold | 0.700.0 |

Source: official Holochain 0.7 compatibility table.

## Consumer matrix

| Consumer | Boundary | Holochain baseline | Shared 0.7 dependency exposure | WASM | DNA pack | Runtime qualification | Evidence |
|---|---|---|---|---|---|---|---|
| Hearth isolated workspace | standalone workspace | 0.7 target on canary branch | bridge-common, bridge-entry-types, zome-helpers | yes | required | required | #3542/#3572 |
| Unified mycelix-workspace | outer workspace | 0.6.1 on main | consumes shared crates | mixed | existing hApps | required before platform promotion | #3543 |
| bridge-common | excluded shared crate | 0.6 on main; 0.7 on Hearth canary | direct Holochain dependency | consumer-dependent | consumer-dependent | consumer-specific | #3542/#3572 |
| bridge-entry-types | excluded shared crate | 0.6 on main; 0.7 on Hearth canary | direct Holochain dependency | consumer-dependent | consumer-dependent | consumer-specific | #3542/#3572 |
| zome-helpers | excluded shared crate | 0.6 on main; 0.7 on Hearth canary | direct Holochain dependency | consumer-dependent | consumer-dependent | consumer-specific | #3542/#3572 |
| Hearth Rust conductor probe | standalone test surface | Rust client 0.9 target | independent conductor oracle | no | no | required | #3575 |
| Hearth TypeScript client | custom browser transport | JS client 0.21 target | custom transport, not holochain_client | no | no | required | #3542/#3575 |
| Sweettest harnesses | standalone/cluster-specific | must match conductor | direct Holochain test dependencies | test-dependent | yes where applicable | required | TBD |

## Qualification gates

### G0 — dependency inventory

Every claimed 0.7 consumer has an explicit dependency inventory. Any remaining 0.6 dependency must be classified as:

- not a 0.7 consumer;
- isolated legacy consumer;
- or a migration defect.

### G1 — lock coherence

Regenerate Cargo.lock, flake.lock, and package-lock files with the intended toolchain. Do not hand-edit generated lock data.

### G2 — compile

Compile all intended coordinator/integrity targets, including wasm32-unknown-unknown where the consumer produces zomes.

### G3 — package

Pack the DNA using the same Holochain 0.7 toolchain family used for conductor qualification.

### G4 — runtime

Run Sweettest or real-conductor tests against Holochain 0.7. Source compilation is not runtime evidence.

### G5 — authority

For each identity-bearing field whose semantics claim agency:

1. matching signer -> Valid;
2. mismatching signer -> Invalid.

For delegated or subject fields:

1. signer != subject can be Valid when the domain model permits it;
2. the test must demonstrate that the field was not accidentally signer-bound.

### G6 — client oracle

The independent Rust client 0.9 probe must pass against a real 0.7 conductor. The custom browser transport is qualified separately.

### G7 — promotion evidence

Record:

- exact commit SHA;
- workflow run ID;
- Holochain/conductor version;
- HDK/HDI versions;
- hc version;
- DNA hash;
- test command;
- result;
- relevant artifact/log reference.

## Current evidence

### #3542 — Hearth 0.7 canary

Head: 7fa909ef144d052e6a3a1b9ca9c8ae68a64208b1

The PR ports the 11 Hearth integrity zomes to the 0.7 action/FlatOp model and moves the shared crates used by Hearth to the 0.7 family for canary qualification.

CI run 1371 is queued. This is not a PASS.

### #3572 — signer-bound actor claims

Head: e845f2f215490389e00e9e33c305c5ae33235a0f

The PR adds the deterministic claimed-agent equality helper and binds explicit agency claims while intentionally leaving delegated/subject fields unbound.

CI run 1370 is queued. This is not a PASS.

The remaining qualification requirement is to exercise the actual validation dispatch paths, including CreateEntry/CreateRecord paths, with adversarial signer mismatch tests.

### #3575 — independent Rust client

Head: d34e57cb80ecf0fb4b6371264af2bbd759f542b3

The ignored real-conductor probe moves from Rust client 0.6-era 0.8.x to Rust client 0.9.0.

CI run 1389 is queued. Runtime qualification is still required.

## Known blockers

1. The unified workspace remains on Holochain 0.6.1 on main.
2. Hearth's 0.7 flake declaration changes the Holonix input, but the generated flake.lock must be regenerated and inspected before claiming a 0.7 Nix toolchain.
3. The generic Hearth CI job runs cargo test --workspace but does not by itself prove hc dna pack or real-conductor qualification.
4. 0.6 DNA data is not directly migratable to 0.7; the 0.7 boundary is a new DNA/network qualification boundary.
5. Shared-crate consumers must be migrated independently; do not infer that changing the root workspace is required solely because Hearth consumes a shared crate.

## Decision rule

The migration is qualified only when the relevant consumer satisfies G0-G7 with recorded evidence. A green source build without lock, packaging, conductor, authority, and client evidence is a partial qualification result, not a foundation PASS.
