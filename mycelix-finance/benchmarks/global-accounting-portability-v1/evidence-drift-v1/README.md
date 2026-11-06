# Financial Evidence Drift Analyzer v1.0.0 — S0

This package implements [FIN-EVIDENCE-DRIFT-001 / #5894](https://github.com/Luminous-Dynamics/mycelix/issues/5894). It computes the downstream impact of changed financial evidence and projection dependencies from exact baseline/candidate bytes. It does not decide accounting treatment, authorize a new journal, or prove that the underlying source population is complete.

## Paradigm: continuous evidence integrity, not point-in-time green checks

A projection that passed yesterday can become stale when a source revision, assertion, event relation, policy profile, projector build or knowledge frontier changes. An opaque `stale=true` flag is not enough: the system must explain exactly which objects changed, which downstream decisions/journal lines depend on them, and what work is now required.

The analyzer uses typed identity and actual dependency edges. Caller-provided affected-object lists are untrusted hints. They are compared with the independently derived closure and never used to narrow it.

## Impact classes

- `no_material_impact`: graph bytes changed only in non-semantic source display metadata, or an unrelated source object was added without joining the projection's evidence closure.
- `review_required`: an assertion became disputed/unknown/invalidated, source integrity degraded, or a same-identity revision family has no explicit supersession chain.
- `reproject_required`: material source/assertion/event/relation semantics changed, or the policy/projector digest changed.
- `historical_replay_required`: evidence that was unavailable at the earlier knowledge frontier becomes available only at a later frontier. The earlier frontier remains reproducible.
- `projection_invalid`: graph binding or projection dependencies are invalid, including a recognized decision backed by unavailable evidence.

Precedence is fail-closed: projection invalidity wins over review, review wins over historical replay, historical replay wins over ordinary reprojection, then no material impact.

## Files

- `financial_evidence_drift_request_v1.0.0.schema.json`: closed Draft 2020-12 request schema. The request contains ten candidate cases.
- `financial_evidence_drift_report_v1.0.0.schema.json`: closed deterministic report schema.
- `financial_evidence_drift_oracle_v1.0.0.schema.json`: separate oracle schema.
- `fixtures/financial_evidence_drift_candidates_v1.0.0.json`: ten candidate cases; each baseline/candidate graph is a full eight-case event-graph bundle serialized as an embedded raw JSON string, with a separately embedded projection string.
- `fixtures/financial_evidence_drift_oracle_v1.0.0.json`: expected classification and impact closure. The analyzer does not read this oracle.
- `../economic-event-core-v1/economic_event_graph_v1.0.0.schema.json`: parent graph schema; this branch adds optional `display_metadata` to a source object. Its display label and ingestion-batch reference are explicitly non-semantic and must not trigger financial replay.
- `../rust-resolver/src/bin/analyze_financial_evidence_drift_s0.rs`: standalone Rust analyzer and report writer.

## S0 scenarios

1. **Amount correction:** source assertion and event amount change; the corresponding decision, journal entry and both journal lines must be invalidated for reprojection.
2. **Metadata-only edit:** source display metadata changes, while source identity/evidence semantics remain the same; no financial replay is needed.
3. **Disputed assertion:** a previously asserted amount becomes disputed; the decision and its prior journal lines require review.
4. **Late evidence:** service-acceptance evidence arrives after the baseline frontier; a new event and relation are added at the later frontier. Historical replay is required, while the old frontier remains stable.
5. **Policy digest change:** all decisions and journal outputs in that projection require reprojection even though graph bytes are unchanged.
6. **Projector digest change:** all decisions and journal outputs require reprojection even though graph bytes are unchanged.
7. **Event relation change:** a new evidence relation changes the dependency graph and invalidates the dependent decision/journal.
8. **Cross-tenant ID collision:** the same external ID appears in another tenant namespace but has no relation to the original projection; the analyzer must not merge them.
9. **Unlinked revision:** a second revision with the same tenant/source identity is added without a supersession link; the dependent decision requires review.
10. **Under-reported impact set:** the caller names the decision but omits its journal entry and lines; the analyzer still calculates the full closure and reports the caller's claim as incomplete.

All actors, source identifiers, amounts, policies, rules and evidence are synthetic. Candidate snapshots preserve seven unchanged event-graph control cases in addition to the case under test, so the parent eight-case schema remains intact.

## Dependency closure

The analyzer:
- compares baseline/candidate objects by typed refs (`source_ref`, `assertion_ref`, `event_ref`, `relation_ref`);
- computes semantic source changes while excluding only the declared non-semantic `display_metadata` object;
- traverses source → assertion → event and event/relation → policy decision dependencies;
- follows policy decision → journal line → journal entry references in both baseline and candidate projections, so invalidated historical outputs are included;
- invalidates all decisions when policy or projector digests change;
- discovers late evidence from source received-at, assertion recorded-at and the exact knowledge frontier;
- detects unlinked revision families using namespace, external ID, source kind and revision;
- keeps tenant namespace in identity so equal external IDs across tenants do not create false dependencies;
- compares caller-reported affected refs with the independently computed decision/journal closure.

The current policy-projection v1 contract does not own statement-line objects; statement compilation is a downstream reporting artifact. This S0 therefore reports journal-line/entry closure only and does not claim statement-line dependency coverage.

## Local Rust validation

From `mycelix-finance/benchmarks/global-accounting-portability-v1/rust-resolver`:

```sh
cargo test --locked --bin analyze_financial_evidence_drift_s0
cargo run --locked --bin analyze_financial_evidence_drift_s0
cargo fmt -- --check
cargo clippy --locked --all-targets -- -D warnings
```

Optional arguments:
- `--data-dir PATH`: evidence-drift artifact directory (default: sibling `evidence-drift-v1`);
- `--output PATH`: write the report JSON to a file;
- `--help`: show usage.

The analyzer validates the request, nested event-graph bundles, embedded policy projections and generated report against Draft 2020-12 schemas. It hashes exact embedded graph/projection byte strings with SHA-256. It performs no network calls, source mutations, policy execution or ledger posting.

Eleven Rust tests are authored for all ten expected classifications plus projection-binding and unavailable-evidence negative controls. **The Rust binary/tests have not been executed.** No Cargo compile/test, rustfmt, Clippy, independent schema-runtime pass or CI pass is claimed.

## Static authoring evidence

A separate JS simulation of fetched artifacts found:
- ten candidate cases and ten oracle rows;
- every embedded baseline/candidate graph has eight cases and the same request case identity;
- both graph and projection bindings use SHA-256 over the exact embedded JSON strings;
- all ten classifications match the public oracle;
- metadata-only change and cross-tenant ID collision produce no dependent decision/journal refs;
- amount correction reaches the decision, journal entry and both journal lines;
- disputed evidence and unlinked revisions require review;
- late evidence requires historical replay;
- policy and projector digest changes reproject the full decision/journal closure;
- the only deliberate caller-claim mismatch is the under-reported impact set in case 10.

This is a static harness simulation, not Rust execution or an accounting-policy judgement.

## Benchmark targets

S0 is ten synthetic controls. Proposed progression: >=100 dual-reviewed S1 change/impact cases and >=250 S2 cases plus hidden holdout spanning source systems, currencies, charts/account mappings, frameworks, policy/projector versions, late arrival, corrections and temporal frontiers.

Hard gates: zero missed material dependents; zero false historical rewrites; zero policy/projector changes treated as no-op; zero false cross-tenant dependencies; zero stale/disputed/invalidated support accepted by a recognized projection; 100% deterministic impact output for frozen input bytes.

Report changed-object detection, dependent-closure precision/recall, caller-set mismatches, false-negative impact count, false-positive reprojection count, historical replay stability, p50/p95 analyzer latency and reviewer effort separately. No single aggregate score may hide a missed material dependency.

## Integration boundaries

- #5858 owns candidate accounting tasks, oracle and expected ledger outputs.
- #5890 owns the bitemporal event graph and policy-neutral event semantics.
- #5892 owns independent policy-projection verification.
- #5893 owns agent-output and evidence-to-decision evaluation.
- #5799 owns canonical benchmark run manifests and receipts.

This package introduces no new run-manifest schema and does not duplicate projection auditing or candidate evaluation.

## Claim ceiling

S0 proves only that the declared synthetic dependency changes and their impact closures are computed deterministically under the supplied schemas. It does not prove that all real-world sources were ingested, that an accounting policy is correct, that a projection is substantively right, that a close is complete or that any real ledger should be modified.