# Procurement Outcome Ledger v1 — benchmark and claim contract

**Status:** experimental reference proposal for `src/outcomes.rs`; no production, correctness, or calibration claim.
**Tracking issue:** [PROC-OUTCOME-001 #5120](https://github.com/Luminous-Dynamics/mycelix/issues/5120).
**Adjacent work:** [ex-ante strategy PR #5119](https://github.com/Luminous-Dynamics/mycelix/pull/5119).

## Scope

The append-only in-memory reference links versioned forecasts, authorized commitments, shipment claims, receipts, acceptance/rejection, returns, disputes, dispute resolutions, and finalized outcomes. It is not a durable ledger, signature checker, evidence authenticator, distributed consensus protocol, payment authority, or supplier reputation engine.

- Source claim IDs are upstream idempotency keys. Exact retries are idempotent; conflicting reuse fails closed.
- Corrections append replacements and mark their predecessor superseded in the derived view. Prior event objects remain available for audit, and reports expose a superseded-event count.
- Replay sorts active events deterministically by occurrence time and event ID, not ingestion order.
- Only events labelled both verified and live-domain affect trusted outcome totals. Other active events remain visible as excluded/unverified counts.
- Open disputes suppress OTIF and forecast-calibration outputs until resolved.
- Finalized quantities must reconcile with detailed acceptance/return evidence when that detail is present; detailed accepted/rejected quantities cannot exceed the reported receipt quantity.
- Caller-supplied status, origin, role and digest references are assertions; this module does not cryptographically authenticate them. Stable source claim IDs are required for idempotency; differently keyed duplicate assertions cannot reliably be recognized as the same real-world fact without stronger identity rules.

## Benchmarks

### Correctness gates (proposed, not measured)

| Property | Target |
|---|---:|
| Exact duplicate claim retries | 100% idempotent |
| Conflicting reuse of event/claim identity | 100% rejected |
| Corrections erase prior event history | 0 |
| Superseded event history count visibility | 100% of corrected case reports |
| Duplicate active commitment/finalization accepted | 0 |
| Accepted + rejected quantity exceeding receipt or commitment | 0 |
| Returned quantity exceeding previously accepted quantity | 0 |
| Contested/unverified/synthetic outcome silently scored as truth | 0 |
| Open dispute with publishable OTIF/calibration metric | 0 |
| Identical frozen event set yields different replay output | 0 |
| Missing/pending/contested outcomes represented as success | 0 |

### Adversarial corpus

Start with at least 40 hand-authored stable cases: exact retries; conflicting claim IDs; duplicate ledger IDs; two distinct IDs over a shared evidence reference; late-arriving/backdated events; corrections and correction chains; duplicate corrections; cross-case correction; invalid issuer role; synthetic forecast/outcome mixing; no forecast; multiple pre-award forecasts; intervals that omit the point estimate; partial receipt/acceptance; rejection; return after acceptance; disputed and resolved finalization; duplicate finalizations; quantity mismatches; currency mismatch; accepted-after-deadline; quote revisions after commitment; and permanently missing final outcomes.

After the independent replay oracle passes, generate at least 10,000 deterministic event streams, with pinned generator/version/seed, manifest digest and expected replay output. Include random ingestion-order permutations, duplicated messages, partial batches, corrected events, unresolved disputes, invalid provenance labels, and arithmetic-boundary quantities. Shrink any counterexample into a named fixture and retain it as a regression.

### Outcome/calibration metrics

Expose every metric with its denominator, event-time cut-off, versioned policy, sample count and evidence coverage. Separate:
- On-time-in-full (OTIF) only after an undisputed finalization.
- Net accepted quantity, rejected quantity, returns and missing finalization.
- Forecast delivery-time error (actual accepted time minus the pre-commitment forecast point).
- Forecast interval coverage and width only when the forecast contains a valid interval.
- Realized landed-cost variance versus authorized landed cost, with currency match.
- Evidence status/origin coverage and unresolved dispute rate.

Do not produce Brier/log scores unless forecasts carry well-defined probabilities and outcomes are adjudicated. Report calibration only out of sample with a declared cutoff/window and sufficient outcome count. No universal supplier score is emitted.

### Runtime and storage

After semantic tests qualify, report p50/p95/max replay latency, event throughput, memory/storage per active case, and deterministic replay cost for 100, 1,000, 10,000 and 100,000 events. Pin hardware, toolchain, fixture corpus, seeds and source revision. These are measurement goals, not present performance claims.

## Interoperability and evidence boundaries

OCDS links planning, tender, award, contract and implementation using a contracting-process identifier. Map Mycelix case/commitment identities to suitable OCDS identifiers in an adapter rather than claiming this event schema is itself OCDS-conformant:
https://standard.open-contracting.org/latest/en/primer/how/

GS1 EPCIS/CBV provides vocabulary and event interfaces for product status, location, movement and chain of custody. Map shipment/receipt events through an adapter; do not treat a syntactically valid event as proof that the physical claim is true:
https://www.gs1.org/standards/epcis

## Cross-project responsibilities

- **Symthaea:** versioned advisory forecasts and calibration summaries with data cutoffs.
- **Mycelix:** bind claim IDs, role/authority receipts, provenance and dispute transitions.
- **Sol-Atlas:** distinguish observed movement/location evidence from inferred routes and forecasts.
- **Symtropy:** test event loss, duplication, delay, correction and outage sequences without converting synthetic output into real outcome evidence.
