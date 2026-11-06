# Bitemporal Economic Event Core v1.0.0 — S0

This package starts [FIN-EVENT-CORE-001 / #5865](https://github.com/Luminous-Dynamics/mycelix/issues/5865). It defines a synthetic, vendor-neutral economic-event graph, a structural/temporal validator, and a separate accounting policy-projection contract. It is not an accounting policy engine, general ledger, legal system, or authorization to post entries.

## Paradigm: documents are assertions, not transactions

An invoice is a source document asserting an amount and an obligation. It does not, by itself, prove service delivery, recognition timing, settlement or expense treatment. A bank payment is a settlement event, not a second expense-recognition event. A credit note is a correction claim with an explicit relationship to its parent obligation.

The core separates:

1. **SourceObject** — tenant namespace, external ID, immutable revision, source kind, received-at, integrity state and optional supersession reference.
2. **Assertion** — typed source fact, asserted/unknown/disputed/invalidated state, valid time, recorded-at time, source object and exact locator.
3. **EconomicEvent** — typed economic occurrence (service delivery, obligation, invoice issuance, credit, settlement, prepayment, accrual, reversal, FX remeasurement, etc.), entity, counterparty, occurred-at, recorded-at, amount and currency.
4. **EventRelation** — explicit supports, contradicts, supersedes, settles, reverses, allocates, corrects, causally-depends-on and duplicate-of edges.

The event graph is not a journal. It is a stable semantic layer from which separately versioned accounting-policy projections can be produced. It does not encode GAAP/IFRS recognition rules in the event types. **Evidence assertions are immutable; corrections add new evidence and relations rather than rewriting history.**

## S0 files and coverage

- `economic_event_graph_v1.0.0.schema.json`: closed Draft 2020-12 graph schema.
- `economic_event_validation_report_v1.0.0.schema.json`: structural/temporal validation report schema.
- `fixtures/economic_event_graph_candidates_v1.0.0.json`: eight synthetic graph candidates.
- `accounting_policy_projection_v1.0.0.schema.json`: policy-specific output contract binding graph, policy and projector digests separately.
- `accounting_policy_projection_validation_report_v1.0.0.schema.json`: deterministic projection-verification report.
- `accounting_policy_projection_bundle_v1.0.0.schema.json` and `fixtures/accounting_policy_projection_review_controls_v1.0.0.json`: two review-only projection examples over one immutable graph digest.
- `accounting_policy_projection_audit_report_v1.0.0.schema.json` and `accounting_policy_projection_audit_v1/README.md`: independent projection-integrity auditor contract and report.
- `../rust-resolver/src/bin/validate_economic_event_graph_s0.rs`: Rust graph validator/report generator.
- `../rust-resolver/src/bin/validate_accounting_policy_projection_bundle_s0.rs`: separate graph-bound policy-projection verifier/report generator.
- `accounting_policy_projection_validation_report_v1.0.0.schema.json`: projection-verifier report schema.
- `../rust-resolver/src/bin/validate_accounting_policy_projection_s0.rs`: exact graph/policy/projector binding and journal-lineage validator.

The eight graph cases cover:
1. invoice issuance without service-acceptance evidence;
2. service-delivery evidence and later invoice issuance as distinct events;
3. settlement event linked to an invoice without a second expense-recognition event;
4. credit note explicitly correcting its parent invoice event;
5. same external invoice ID in two tenant namespaces, preserved as two source objects/events;
6. source and assertion received/recorded after a historical knowledge frontier;
7. contradictory amount assertions preserved with a contradiction edge;
8. two source revisions with no supersession chain, surfaced rather than silently merged.

Counts: 8 cases, 16 source objects, 21 assertions, 13 economic events and 4 typed relations. All values, actors, namespaces, IDs and locators are synthetic.

## Bitemporal, identity and evidence-consistency invariants

- Date-time strings are canonical whole-second UTC: `YYYY-MM-DDTHH:MM:SSZ`. Offsets and fractional-second timestamps are rejected.
- Date-only valid-time values are canonical `YYYY-MM-DD`.
- `recorded_at` and source `received_at` must be no later than `knowledge_at` for an assertion/event to be available at that frontier.
- Assertion valid time must overlap the case period before it is considered available in this S0 evaluation.
- Superseded source revisions become unavailable only after the superseding source is received at the requested frontier.
- Source identity is the tuple (tenant namespace, external ID, revision, source kind); a same-ID object in another tenant remains distinct.
- Multiple revisions of one source identity require an explicit ordered supersession chain. Unlinked revision families are surfaced.
- Every assertion points to one source object and locator. Every event points to one or more assertions. Every relation has typed endpoints and source assertion refs.
- Event amount/currency must equal the cited invoice, credit or payment amount assertion when the amount is unconflicted. If linked amount assertions conflict, every competing assertion must be included and explicitly connected by a `contradicts` relation; the event amount/currency must remain null until resolution. Service-delivery and settlement event dates must match the cited acceptance/payment-date assertions. Where service-period boundaries are present, start must not follow end.
- A `corrects` relation must link a `credit_issued` event to an `invoice_issued` event with a parent invoice source object in the same tenant namespace. Cross-tenant ID coincidences cannot create parentage.
- Contradictions are preserved as edges; the validator never selects one conflicting assertion as true.
- The validator reports future assertions/events, unlinked revisions, contradictions and integrity mismatches. It does not resolve them into accounting conclusions.

## Policy-neutral, multi-projection boundary

The event graph is the shared evidence substrate; policy and projector versions are independent bindings.

The policy-projection contract requires:
- exact event graph case ID and SHA-256 digest;
- policy profile ID/version/framework and policy SHA-256 digest;
- projector ID/version and projector SHA-256 digest;
- per-event decision state, treatment-rule reference, source assertions and assumptions;
- journal lines linked to event refs, source assertions and policy decision refs;
- unresolved items and review state.

The review-control fixture has two synthetic policy profiles and two synthetic projector versions bound to the **same exact graph-bundle digest** (`f3f6ddeca12df82fb06bca9c7711a47b9125f3447a0e7e5f310db1f2dcab6abd`). Both outputs remain `needs_review` with no journal lines because the graph contains only invoice issuance and lacks service-acceptance evidence. Their policy/projector digests differ. Each projection now has an explicit assumption register; decision references must resolve into that register. These are schema examples only—not GAAP, IFRS or any real framework outputs.

This demonstrates the architectural boundary: a chart of accounts or policy change must not mutate the source evidence graph. Multiple policy projections can be regenerated from the same frozen graph and compared without rewriting the underlying facts.

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

The event validator checks Draft 2020-12 schemas; source identity and supersession; assertion/source provenance; valid time and knowledge time; event-to-assertion type consistency; exact event amount/currency/date agreement with cited assertions; same-tenant credit-note parent resolution; typed relation endpoints; self-relations; event supersession cycles; and deterministic input SHA-256. The projection validator separately checks exact graph-byte digest, projection graph case, policy and projector pins, event/source assertion cross-links, entity/currency/period consistency, one-to-one policy decision lineage, per-entry balance, and journal debit/credit totals against recognized measurements. It reports future assertions/events, unlinked revisions, contradictions and integrity mismatches without resolving them into accounting conclusions.

Seventeen event-graph Rust tests and thirteen projection-validator Rust tests are authored. The event tests include source/event amount mismatch, currency mismatch, acceptance/payment-date mismatch, cross-tenant credit-note parent rejection, future evidence, contradiction preservation without attaching the conflicting assertion to the invoice event, unlinked revisions, dangling refs and supersession cycles. Projection tests cover graph digest mismatch, wrong graph binding, dangling event/assertion refs, recognized measurement requirements, exact event/period journal binding, balanced-but-wrong measurement, unresolved-item references and independent policy/projector digests. Tests are authored but **not executed**. No Cargo compile/test, rustfmt, Clippy, independent Draft 2020-12 runtime pass or CI pass is claimed.

## Integration and benchmark design

- #5858 owns evidence-qualified accounting tasks and ledger oracles.
- #5868 owns evaluation of agent questions, evidence traces and final deliverables.
- #5840/#5848 own typed jurisdiction/applicability context and fact resolution.
- #5849 owns cross-border financial data-flow review and data governance.
- #5855 owns read-only vendor platform interchange.
- #5799 owns canonical benchmark run manifests/receipts and matched evaluation arms.
- #5891 / `accounting_policy_projection_audit_v1` independently checks policy/projector bindings, event decision coverage, assumption evidence, journal lineage, amount/currency agreement, per-entry balance and blocked-state non-posting.

Compare document-to-journal direct generation, source-fact-to-journal generation, typed-event-graph normalization, event graph plus independent replay, and multiple policy projections from the same frozen graph. The policy projection verifier makes cross-policy comparisons reproducible by pinning graph, policy and projector digests independently. Measure event normalization precision/recall on expert-labelled cases, provenance closure, contradiction retention, false event merges, replay stability, journal invariants, reviewer effort and cost separately.

Proposed hard gates:
- 100% assertion provenance closure;
- zero dangling typed relation refs;
- zero false merges across tenant namespaces;
- zero historical rewrites from future evidence;
- zero unlinked revision families silently normalized;
- 100% exact replay for frozen graph bytes and policy digests;
- zero event/source amount or currency mismatch;
- zero journal imbalance in any later projection verifier.

S0 is eight synthetic structural cases, not an expert-adjudicated accounting benchmark. Expand to 50 adversarial synthetic graphs, then at least 100 dual-reviewed S1 cases and 250 S2 cases with a hidden holdout.

## Claim ceiling

This graph establishes only a typed, bitemporal representation and reference/evidence-consistency contract. It does not establish whether a service was recognized under GAAP/IFRS, whether a credit note changes revenue or expenses, whether a transfer is legally permitted, whether an entity is a real counterparty, or whether any journal entry should be posted.