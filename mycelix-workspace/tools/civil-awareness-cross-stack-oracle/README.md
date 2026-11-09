# Civil Awareness Cross-Stack Golden-Trace Oracle

An offline, effects-disabled oracle for typed synthetic owner-result fixtures across UAS, maritime and civil aviation.

## What it evaluates

The oracle composes four *declared* owner outputs:
- Mycelix source-integrity, rights/authorization, dependency-state and action-authority declarations;
- Sol-Atlas spatial/temporal context status;
- Symtropy synthetic scenario/replay status;
- Symthaea read-only triage and contradiction status.

It produces one evidence disposition per synthetic case:
- `REVIEWABLE_WITHIN_DECLARED_EVIDENCE`;
- `REVIEW_WITH_CORRELATED_SOURCES`;
- `UNRESOLVED`;
- `CONFLICT`;
- `QUARANTINED`;
- `REJECTED`;
- `NOT_EVALUABLE`.

A reviewable disposition is not a safety verdict, a claim of physical truth or source independence, and is not permission for field use. A no-declared-common-domain result is bounded to the exact complete/current Mycelix dependency snapshot and queried failure dimension. It is not physical independence.

## Composition; no parallel evidence architecture

The CLI takes four exact-byte inputs:
1. frozen golden-trace corpus;
2. paired synthetic owner-result inputs;
3. the empty civil handoff contract template;
4. JSON report emitted by the existing Mycelix #5972 maritime handoff verifier.

It validates the report against the exact manifest byte digest and requires the handoff verifier to report `pass` for the expected empty `contract_template`. The template is intentionally empty: all owner results in this first corpus are synthetic fixtures, not owner-native production artifacts.

The tool consumes the dependency disposition produced by Mycelix #5923/#5904. It does not implement a graph or compute independence. Dataset/split/run-receipt semantics remain with Symthaea #10875. The cross-stack profile documents the owner-native artifact handoff boundary.

## Run

From the repository root:

```sh
cargo fmt --manifest-path mycelix-workspace/tools/maritime-handoff-verifier/Cargo.toml --check
cargo fmt --manifest-path mycelix-workspace/tools/civil-awareness-cross-stack-oracle/Cargo.toml --check

cargo test --manifest-path mycelix-workspace/tools/maritime-handoff-verifier/Cargo.toml
cargo test --manifest-path mycelix-workspace/tools/civil-awareness-cross-stack-oracle/Cargo.toml

cargo run --manifest-path mycelix-workspace/tools/maritime-handoff-verifier/Cargo.toml -- \
  mycelix-workspace/docs/civil-awareness/integration/civil-awareness-handoff-template-v0.1.json \
  > /tmp/civil-awareness-handoff-report.json

cargo run --manifest-path mycelix-workspace/tools/civil-awareness-cross-stack-oracle/Cargo.toml -- \
  mycelix-workspace/docs/civil-awareness/integration/civil-awareness-cross-stack-golden-traces-v0.1.json \
  mycelix-workspace/docs/civil-awareness/integration/civil-awareness-cross-stack-oracle-inputs-v0.1.json \
  mycelix-workspace/docs/civil-awareness/integration/civil-awareness-handoff-template-v0.1.json \
  /tmp/civil-awareness-handoff-report.json
```

The oracle prints a JSON report with corpus/input/template SHA-256, per-case actual/expected disposition, reason code and match state. It exits nonzero for any case mismatch or invalid handoff/report input.

## Frozen synthetic corpus

The current profile includes 22 cases:
- UAS positive, shared clock/collector, unknown receiver lineage, duplicate/relay, stale source and prohibited-field negative;
- maritime conflict, missing AIS coverage, shared port pipeline, stale chart/context and positive control;
- civil aviation shared ADS-B/MLAT timing, unknown coverage, weather/NOTAM conflict and positive control;
- cross-domain rights pending, missing owner handoff, action-authority attempt, failed source integrity, untrusted clock, stale calibration and unsupported simulation replay.

The input JSON is bound to exact SHA-256 digests of the corpus and handoff template. Parsing uses typed `deny_unknown_fields` structs. The test suite covers the full 22-case oracle, action-authority promotion, corpus digest tampering and non-pass handoff reports.

## Hard-gate order

The disposition rule table is deliberately non-compensatory:
1. malformed/invalid schema, prohibited fields, privacy violation, failed source integrity or any non-`none` action authority -> `REJECTED`;
2. rights/authorization not cleared or privacy quarantine -> `QUARANTINED`;
3. required handoff missing -> `NOT_EVALUABLE`;
4. unresolved source contradiction -> `CONFLICT`;
5. stale/unverified source, stale context, unqualified clock/calibration, incomplete evidence or failed/unavailable replay -> `UNRESOLVED`;
6. shared dependencies or duplicate/relay -> `REVIEW_WITH_CORRELATED_SOURCES`;
7. complete/current source/context and `NO_DECLARED_COMMON_DOMAIN` within the captured inventory -> `REVIEWABLE_WITHIN_DECLARED_EVIDENCE`.

No result grants navigation, flight, vessel, VTS, ATC, mitigation, provider or any external action authority.

## Claim ceiling

Passing tests proves only that the independent rule table matches the 22 frozen synthetic fixtures, and that the empty contract template is structurally accepted by the separate handoff verifier. It does not establish:
- Mycelix/Sol-Atlas/Symtropy/Symthaea production integration;
- source authentication or physical truth;
- physical or statistical independence of any real sensor;
- aircraft/vessel/drone detection, track or identification performance;
- regulatory compliance, airworthiness, vessel safety or navigation safety;
- industry/SOTA superiority or field performance.

No live sensor, private communication, personal/crew/controller/operator location, hostile-intent inference, effectors, mitigation, jamming/spoofing, navigation/flight-control interface or action execution is implemented.

## Parent issue

[GCPN-CIVIL-CROSS-STACK-001 — #5998](https://github.com/Luminous-Dynamics/mycelix/issues/5998)
