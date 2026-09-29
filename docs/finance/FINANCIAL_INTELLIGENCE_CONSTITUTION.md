# Financial Intelligence Constitution

**Status:** Proposed semantic gate  
**Parent:** OPS-INTEL-014 / #3480  
**Scope:** Evidence-native financial intelligence across Mycelix, Symthaea, and replaceable presentation/orchestration clients.

This document freezes the semantic boundaries that downstream financial-intelligence implementations MUST preserve. It is intentionally provider-neutral and contains no execution authority.

## 1. Epistemic object boundaries

The following objects are distinct and MUST NOT be silently coerced into one another:

`EconomicState != MarketObservation != Evidence != QualifiedClaim != Inference != Hypothesis != Scenario != Forecast != DecisionCandidate != Authority != ExecutionReceipt != EconomicEffect != CausalAttribution`.

A projection or UI may reference another object, but reference is not identity.

## 2. Information frontier

Every research and forecast evaluation MUST be bound to an explicit information frontier.

At minimum distinguish:

- event/observation time;
- publication/availability time;
- ingestion time;
- effective validity interval;
- supersession/revision;
- query/replay frontier.

A historical replay MUST expose only information available at its frontier. Later observations, forecasts, revisions, or outcomes MUST NOT leak backward.

## 3. Raw observations and derived projections

Raw observations are immutable evidence-bearing inputs.

Normalization, adjustment, aggregation, entity resolution, and visualization are derived projections. A derived projection MUST retain lineage to its inputs and MUST NOT overwrite or masquerade as the raw observation.

## 4. Evidence and claims

An artifact is not automatically a fact.

Extracted claims MUST preserve:

- source/artifact identity;
- transformation/extraction lineage;
- version or revision identity;
- information frontier;
- qualification/currentness state.

A model, provider confidence value, or UI label MUST NOT promote an unqualified claim into canonical truth.

## 5. Temporal subject identity

Ticker, symbol, URL, display name, or provider-local identifier MUST NOT by itself constitute canonical subject identity.

Issuer, instrument, listing, venue, share class, contract, and related subjects MUST remain distinguishable across symbol changes, identifier reuse, mergers, delistings, and corporate actions.

## 6. Evidence independence

Distinct provider IDs do not imply independent evidence.

The evidence model MUST preserve common upstream ancestry, transformations, mirroring, corroboration, disagreement, and contradiction where known. Correlated copies MUST NOT be counted as independent observations solely because they arrive through different connectors.

## 7. Relations and causality

A graph relation is typed and evidence-relative.

Examples:

- ownership != supply;
- supply != exposure;
- correlation != causation;
- inferred dependency != legally established control.

Relations MUST carry temporal/evidentiary qualification where applicable. Unsupported relations remain hypotheses or unknowns.

## 8. Scenarios

A scenario is a hypothetical projection over a specified base/frontier.

Scenario outputs MUST NOT mutate current-world state, evidence, or historical observations. Scenario assumptions MUST remain distinguishable from observed facts.

## 9. Forecasts and calibration

A forecast MUST record its issuance frontier, subject, horizon, assumptions/model profile, uncertainty and prediction.

Historical forecasts MUST be immutable after issuance. Outcomes and calibration records are appended later; they MUST NOT rewrite the original forecast or its information frontier.

## 10. Authority and execution

Research output is non-authoritative by default.

A decision candidate does not grant authority. Authority does not imply execution. An execution receipt does not prove economic effect. An economic effect does not prove causal attribution.

Any transition across these boundaries MUST require an explicit, independently verifiable authorization contract.

## 11. Security and disclosure

Protected/private evidence MUST NOT leak through summaries, embeddings, graph edges, forecasts, UI projections, or generated text.

External orchestration clients MUST NOT acquire authority merely by invoking research capabilities. Missing, malformed, revoked, stale, conflicting, or unauthorized inputs MUST fail closed or be represented explicitly; silent substitution is prohibited.

## 12. Status vocabulary

Implementations MUST preserve distinctions between at least:

- known/qualified;
- observed but unqualified;
- unknown;
- unavailable;
- stale;
- conflicting;
- protected;
- future/inaccessible at frontier.

These statuses are semantic state, not presentation hints.

## 13. Determinism and replay

For identical canonical inputs, schema version, and information frontier, an independent implementation MUST be able to reproduce the semantic oracle result.

Network providers, external model availability, UI state, and wall-clock timing MUST NOT be prerequisites for validating the core semantic corpus.

## 14. No FinancialTruth

The system MUST NOT introduce a universal mutable `FinancialTruth` object.

The canonical substrate is evidence-relative, temporally qualified, provenance-preserving state and projection. Apparent consensus is a derived interpretation, not permission to erase disagreement or lineage.

## 15. Downstream gate

FIN-001T/U/R/A/B/C/D/E/F/P/K/M/V/X and ATLAS-FIN-001A/B/C MUST reference these invariants.

A downstream implementation that violates an invariant is non-qualifying even if its feature-level tests pass.

## 16. Minimum adversarial corpus

The qualification corpus SHOULD include at least:

1. stale quote;
2. delayed filing;
3. revised disclosure;
4. late-arriving observation;
5. ticker reuse;
6. dual listing/share-class collision;
7. corporate-action adjustment;
8. mirrored provider feeds;
9. contradictory providers;
10. detached document extraction;
11. scenario leakage into current state;
12. future-information forecast leakage;
13. graph edge mistaken for causal fact;
14. prompt injection embedded in source material;
15. recommendation-to-execution escalation;
16. protected evidence inference/leakage;
17. execution receipt mistaken for economic outcome.

This constitution is the semantic gate, not an aggregate quality score.
