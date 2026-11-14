# FSRE independent recovery reference evaluator

A separately authored, dependency-free Rust implementation of the seven-predicate recovery-option decision rule. It does not import or call `fsre-recovery-qualification`, its parser, structs or helper functions.

The independent reference reads the same frozen scenario-options and predicate-evidence files, then checks:
- exact scenario ID and digest;
- service binding;
- owner/schema namespace;
- common time basis and conservative clock intervals;
- all seven required dispositions;
- mobilisation horizon.

It emits only per-option dispositions and reason codes. It deliberately does not render the production evidence report or evaluate debt groups. That keeps the differential comparison focused on the critical accept/reject decision while reducing shared code paths.

## Run

```sh
cargo test --locked --offline --manifest-path tools/fsre-recovery-reference/Cargo.toml
cargo run --locked --offline --manifest-path tools/fsre-recovery-reference/Cargo.toml -- \
  tools/fsre-recovery-qualification/fixtures/scenario.psv \
  tools/fsre-recovery-qualification/fixtures/predicate-evidence.psv
```

Compare the output byte-for-byte with `fixtures/expected-options.txt`. The separate recovery qualification workflow performs this comparison.

The crate has four deterministic unit tests for digest shape and conservative clock-boundary handling. On the current qualified revision, its independent output matched all seven frozen synthetic candidate dispositions byte-for-byte.

## Independence boundary

This is independent implementation, not independent evidence. Both evaluators consume the same files and share the same written contract and owner map. It can detect implementation/parser differences but cannot detect an incorrect specification, an unauthenticated upstream record or a shared fixture error. A separate reviewer and independently prepared challenge inputs remain necessary.
