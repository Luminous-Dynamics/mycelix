# Pulse Trace Verifier

Independent Rust verifier for the Pulse authorization lifecycle corpus and six-plane runtime traces.

## Commands

From the repository root:

```bash
cargo test --locked \
  --manifest-path mycelix-workspace/mycelix-pulse/holochain/Cargo.toml \
  -p pulse-trace-verifier

cargo run --locked \
  --manifest-path mycelix-workspace/mycelix-pulse/holochain/Cargo.toml \
  -p pulse-trace-verifier -- \
  validate-corpus \
  mycelix-workspace/mycelix-pulse/docs/security/authorization-lifecycle-v1.corpus.json \
  mycelix-workspace/mycelix-pulse/docs/security/authorization-lifecycle-v1.schema.json
```

To replay a redacted runtime trace:

```bash
cargo run --locked \
  --manifest-path mycelix-workspace/mycelix-pulse/holochain/Cargo.toml \
  -p pulse-trace-verifier -- \
  verify-trace <corpus.json> <corpus-schema.json> <trace-schema.json> <trace.json>
```

The CLI writes a deterministic JSON receipt to stdout and exits non-zero on any contract violation.

## Trace contract

Each event is one observation row covering the six planes:

- `host_decision`
- `domain_result`
- `source_chain`
- `validator`
- `projection`
- `observer`

It also binds exact grant/source ActionHashes, logical operation ID, separate attempt ID/number, host grant transition, observed log index and explicit causal predecessors.

`observed_index` is a local log position only. It is not used to infer causality. Only `causal_predecessors` establishes happens-before. A protected call causally after a `REVOKED` transition for the same exact grant ActionHash may not be `AUTHORIZED`.

The verifier also checks:

- corpus/schema closure, scenario/event uniqueness and all references;
- event/action/actor/request/attempt parity with the synthetic scenario;
- all checkpoints in the six-plane corpus;
- acyclic and complete causal edges;
- exact grant identity across retries and at most one CREATED effect per logical operation;
- no source-action/validator disposition mismatch;
- rejected remote validation plus acknowledged observation requires equal before/after canonical state digests;
- unknown fields fail closed, including secret-bearing fields such as `cap_secret`.

The receipt includes SHA-256 for raw and canonical corpus/trace bytes, both schema files, source commit, runtime tuple, DNA/WASM digests and claim ceiling.

## Security boundary

This tool verifies structure and internal consistency of supplied evidence. It cannot prove that the conductor, source-chain observer, validator or exporter told the truth. A hash does not authenticate an issuer. Production evidence still needs signed provenance, independent observer collection and exact-head artifact binding.

This verifier is deliberately independent from the candidate coordinator's decision code. It must not import production authorization functions or accept a precomputed `PASS` from the system under test.
