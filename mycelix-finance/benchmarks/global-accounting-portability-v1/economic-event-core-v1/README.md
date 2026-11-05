# Bitemporal Economic Event Core v1.0.0 — S0

This package starts [FIN-EVENT-CORE-001 / #5865](https://github.com/Luminous-Dynamics/mycelix/issues/5865). It defines a synthetic, vendor-neutral economic-event graph and a structural validator. It is not an accounting policy engine, general ledger, legal system, or authorization to post entries.

## Paradigm: documents are assertions, not transactions

An invoice is a source document asserting an amount and an obligation. It does not, by itself, prove service delivery, recognition timing, settlement or expense treatment. A bank payment is a settlement event, not a second expense-recognition event. A credit note is a correction claim with an explicit relationship to its parent obligation.

The core separates:

1. **SourceObject** — tenant namespace, external ID, immutable revision, source kind, received-at, integrity state and optional supersession reference.
2. **Assertion** — typed source fact, asserted/unknown/disputed/invalidated state, valid time, recorded-at time, source object and exact locator.
3. **EconomicEvent** — typed economic occurrence (service delivery, obligation, invoice issuance, credit, settlement, prepayment, accrual, reversal, FX remeasurement, etc.), entity, counterparty, occurred-at, recorded-at, amount and currency.
4. **EventRelation** — explicit supports, contradicts, supersedes, settles, reverses, allocates, corrects, causally-depends-on and duplicate-of edges.

The event graph is not a journal. It is a stable semantic layer from which separately versioned accounting-policy projections can be produced. It does not encode GAAP/IFRS recognition rules in the event types.

## S0 files and coverage

- `economic_event_graph_v1.0.0.schema.json`: closed Draft 2020-12 graph schema.
- `economic_event_validation_report_v1.0.0.schema.json`: structural/temporal validation report schema.
- `fixtures/economic_event_graph_candidates_v1.0.0.json`: eight synthetic graph cases.
- `../rust-resolver/src/bin/validate_economic_event_graph_s0.rs`: Rust validator and report generator.

The eight cases cover:
1. invoice issued without service-acceptance evidence;
2. service-delivery evidence and later invoice issuance as distinct events;
3. settlement event linked to an invoice without a second expense-recognition event;
4. credit note explicitly correcting its parent invoice event;
5. same external invoice ID in two tenant namespaces, preserved as two source objects/events;
6. source and assertion received/recorded after a historical knowledge frontier;
7. contradictory amount assertions preserved with a contradiction edge;
8. two source revisions with no supersession chain, surfaced rather than silently merged.

Counts: 8 cases, 16 source objects, 21 assertions, 13 economic events and 4 typed relations. All values, actors, namespaces, IDs and locators are synthetic.

## Bitemporal and identity invariants

- Date-time strings are canonical whole-second UTC: `YYYY-MM-DDTHH:MM:SSZ`. Offsets and fractional-second timestamps are rejected.
- Date-only valid-time values are canonical `YYYY-MM-DD`.
- `recorded_at` and source `received_at` must be no later than `knowledge_at` for an assertion/event to be available at that frontier.
- Assertion valid time must overlap the case period before it is considered available in this S0 evaluation.
- Superseded source revisions become unavailable only after the superseding source is received at the requested frontier.
- Source identity is the tuple (tenant namespace, external ID, revision, source kind); a same-ID object in another tenant remains distinct.
- Multiple revisions of one source identity require an explicit ordered supersession chain. Unlinked revision families are surfaced.
- Every assertion points to one source object and locator. Every event points to one or more assertions. Every relation has typed endpoints and source assertion refs.
- Contradictions are preserved as edges; the validator never selects one conflicting assertion as true.
- The validator reports future evidence, integrity mismatches, contradictions and unlinked revision families. It never resolves them into accounting conclusions.

## Local Rust validation

From `mycelix-finance/benchmarks/global-accounting-portability-v1/rust-resolver`:

```sh
cargo test --locked --bin validate_economic_event_graph_s0
cargo run --locked --bin validate_economic_event_graph_s0
cargo fmt -- --check
cargo clippy --locked --all-targets -- -D warnings
```

Optional arguments:
- `--data-dir PATH`: event graph artifact directory (default is the sibling `economic-event-core-v1` folder);
- `--output PATH`: write the deterministic validation report to a file;
- `--help`: show usage.

The validator checks Draft 2020-12 schemas, canonical timestamps, unique identities, tenant-scoped revision chains, assertion/source references, event/typed assertion compatibility, event and relation endpoints, self-relations, temporal availability, and event-supersession cycles. The report fingerprints the exact candidate bytes with SHA-256. There is no network or ledger side effect.

Eleven Rust tests are authored but **not executed**. No Cargo compile/test, rustfmt, Clippy, independent schema runtime pass or CI pass is claimed.

## Integration boundary

- #5858 owns evidence-qualified accounting tasks, expected journals/statements and the independent agent-output evaluation.
- #5840/#5848 own typed jurisdiction/applicability context and fact resolution.
- #5849 owns cross-border financial data-flow review and data governance.
- #5855 owns read-only vendor platform interchange.
- #5799 owns canonical benchmark run manifests/receipts and matched evaluation arms.

The economic-event graph should eventually consume normalized, provenance-preserving facts from those packages. It must not redefine their source identity, data-flow permission, run receipts or benchmark oracles. Multi-framework accounting outputs belong in distinct policy-projection artifacts that reference the same frozen event graph, policy digest and assumptions. Those projection artifacts are a later workstream, not part of S0.

## Proposed benchmarks and targets

Compare direct document-to-journal generation, source-fact-to-journal generation, typed-event-graph normalization, event graph plus deterministic replay, and multi-policy projection from the same frozen graph.

Proposed hard gates:
- 100% assertion provenance closure;
- zero dangling typed relation refs;
- zero false merges across tenant namespaces;
- zero historical rewrites from future evidence;
- zero unlinked revision families silently normalized;
- 100% exact replay for frozen graph bytes and policy digests;
- zero journal imbalance in any later projection verifier.

S0 is eight synthetic structural cases, not an expert-adjudicated accounting benchmark. Expand to 50 adversarial synthetic graphs, then >=100 dual-reviewed S1 cases and >=250 S2 cases with hidden holdout. Event-normalization precision/recall, event identity resolution quality, reviewer effort, runtime and cost require expert-labelled real-world-style cases and matched baselines.

## Claim ceiling

This graph establishes only a typed, bitemporal representation and reference-validation contract. It does not establish whether a service was recognized under GAAP/IFRS, whether a credit note changes revenue or expenses, whether a transfer is legally permitted, whether an entity is a real counterparty, or whether any journal entry should be posted.
