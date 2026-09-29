# ADR-0002: Learning Evidence and Capability-Estimate Authority

- Status: Proposed
- Subject: `PRAX-EVIDENCE-001`
- Scope: `praxis-core` semantic contract

## Context

Praxis currently contains several useful but overlapping learning-state concepts: activity records, assessment outcomes, Bayesian Knowledge Tracing (BKT) `SkillMastery`, Proof of Learning (PoL), goals/progress, recommendations, and credentials.

The legacy adaptive path can accept a caller-provided correctness observation and immediately update a durable `SkillMastery` value. PoL likewise combines heterogeneous evidence with heuristic component scores. Those mechanisms are useful estimators, but their outputs must not silently acquire stronger authority merely because they are stored, summarized, or displayed.

## Decision

Praxis adopts the following non-equivalence as a semantic invariant:

```text
observation
!= admitted evidence
!= capability estimate
!= credential decision
!= authorization
```

`praxis-core::learning_evidence` is the semantic root for the first three layers.

### 1. Observation

A `LearningEvidenceEvent` records what a producing boundary observed. It carries:

- opaque learner and capability identifiers;
- source class;
- explicit assistance class;
- dimensional measures;
- producer/version/source provenance;
- artifact digest when available;
- observation time and descriptive context.

Structural validation means only that the record is well formed. It does not establish truth, sufficiency, authenticity, or mastery.

### 2. Admission

An `EvidenceAdmissionDecision` records the result of applying a named, versioned policy profile to one source event.

Admission is profile-relative. It does not mutate the observation and is not universal proof. The same event may be admissible for practice feedback and inadmissible or inconclusive for a credential claim.

Assistance must remain visible through this boundary. In particular:

```text
direct answer observed
!= independent performance demonstrated
```

No global rule says assisted work is worthless; the relevant admission profile decides what it can support.

### 3. Capability estimate

An `AdvisoryCapabilityEstimate` is a recomputable model output over explicitly referenced evidence. It records estimator identity/version and the admission profile used.

Estimates are dimensional. Praxis should prefer separate observations/estimates for recall, understanding, application, transfer, explanation, practical performance, retention, judgment, and collaboration rather than silently compressing them into a single authoritative percentage.

An advisory estimate cannot issue a credential or grant authorization.

### 4. Credentials remain downstream

Credential issuance, endorsement, progression gates, licensing decisions, and other consequential claims require their own explicit policy boundary. They may consume admitted evidence and advisory estimates, but neither input is sufficient authority on its own.

## Compatibility

This tranche is additive. It does **not** change Holochain entry schemas or remove the legacy adaptive API.

The existing `SkillMastery` and `record_attempt` path therefore remain wire-compatible for now, but are considered legacy projection semantics for future migration. The existing PoL API also remains compatible and should be treated as experimental advisory analytics.

## Required follow-up tranches

1. **PRAX-EVIDENCE-002 — attempt provenance**
   - introduce immutable practice/assessment evidence at the adaptive boundary;
   - bind correctness to source activity/task, assistance, evaluator/version, and artifact provenance;
   - preserve old API through an explicit compatibility adapter rather than pretending legacy attempts are provenance-complete.

2. **PRAX-ESTIMATE-001 — BKT projection repair**
   - derive BKT state from referenced admitted events;
   - rename new semantics to `AdvisoryCapabilityEstimate`/BKT projection;
   - make recomputation deterministic and test event-order/profile behavior;
   - prevent BKT thresholds from directly creating credential authority.

3. **PRAX-POL-001 — PoL authority repair**
   - reframe PoL as a versioned learning-analysis projection;
   - remove claims that model heuristics independently prove genuine learning or cheating;
   - bind every component score to referenced evidence and estimator provenance.

4. **PRAX-STATE-001 — event-derived learner state**
   - derive learner summaries, goal progress, and recommendation inputs from admitted events/projections;
   - retain source events so state can be recomputed under a new profile or model version.

5. **PRAX-CREDENTIAL-001 — credential evidence policy**
   - require explicit versioned evidence policies for credential decisions;
   - record the exact evidence/admission/projection set used by a decision;
   - do not allow a UI progress percentage, BKT threshold, or PoL score to become credential authority implicitly.

## Non-goals

This ADR does not claim that any particular estimator is scientifically validated, that an evidence event is truthful, that imported credentials are trustworthy, or that Holochain persistence itself establishes evidence quality.

## Core theorem

```text
persistence != admission
admission != mastery
estimate != credential
credential != authorization
```
