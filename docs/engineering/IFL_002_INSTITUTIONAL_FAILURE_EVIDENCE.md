# IFL-002 — Institutional Failure Learning Evidence Boundary

## Purpose

Mycelix provides durable institutional memory for Symthaea failure-learning events while remaining separate from cognitive inference.

Symthaea owns hypotheses, simulations, predictions and prediction-error analysis.

Mycelix owns identity, provenance, attestations, contestability, authorization context and durable lineage.

## Receipt families

- FailureCaseReceiptV1
- PredictionReceiptV1
- OutcomeReceiptV1
- PredictionErrorReceiptV1
- MechanismHypothesisReceiptV1
- ConstraintCandidateReceiptV1

## Required lineage

A ConstraintCandidateReceiptV1 must be traceable to the exact prediction, decision context, action/outcome pair, prediction error, mechanism hypothesis and evidence set that motivated it.

## Evidence semantics

A receipt records an event or assertion; it does not make the assertion true merely because it is stored or attested.

Unknown, unavailable, stale, contradictory and invalidated evidence remain distinct states.

## Governance semantics

A learned constraint remains a proposal until an explicit governance event adopts it.

The receipt layer must never convert:
- attestation count into truth;
- historical precedent into policy;
- model confidence into authority;
- storage into qualification;
- recommendation into authorization.

## Qualification boundary

The first implementation should be schema/domain-only with deterministic canonicalization, mutation tests, lineage checks and explicit supersession/contestability. Holochain/DKG materialization is downstream and must not be used as evidence that the semantic contract itself is correct.

## Integration

The first cross-repository integration consumes Symthaea IFL-001 evidence-bearing outputs and preserves their exact identities and qualification ceilings.
