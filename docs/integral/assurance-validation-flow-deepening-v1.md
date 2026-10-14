# Integral Assurance & Validation Flow — Mycelix Deepening v1

## Research correction

A deeper reading of the current public Development Guide changes the contribution thesis.

Integral already explicitly defines **Assurance and Validation Flows** as one of five federation reciprocity primitives, including certification results, safety data, provenance proofs, and reliability histories. It also defines node dependency state, Viability Envelopes, Coordination Envelopes, federation-aware identifiers, FRS confidence/evidence references, and non-executive recommendations.

Therefore this work must **not** claim that these concepts are absent.

The higher-value Mycelix contribution is:

> turn Integral's existing conceptual assurance vocabulary into a portable, evidence-preserving, executable, adversarially tested and formally refinable assurance layer.

## What Integral already has

### Assurance and Validation Flow

The glossary explicitly identifies this as a federation primitive covering certification results, safety data, provenance proofs, and reliability histories.

### Viability Envelope

Integral defines a node's operational stability range and distinguishes it from the Coordination Envelope used for cross-node response.

### Coordination Envelope

Integral defines a temporary, bounded cross-node coordination context with affected nodes, shared constraints, decision scope, and automatic dissolution.

### FRS evidence and confidence

The public schema includes evidence references, indicators, severity, scope, persistence and confidence on DiagnosticFinding. Recommendations are explicitly non-executive.

### Federation provenance

The guide explicitly requires node-aware identities and originating-node fields, while some cryptographic federation-integrity fields remain deferred.

### Technical seam contracts

Integral's public Technical Specifications page identifies SPEC-IF-01 OAD→COS, SPEC-IF-02 COS→ITC and SPEC-IF-03 FRS→CDS as PENDING, with versioning, authentication, retry, delivery, idempotency and acknowledgement semantics specifically called out as unresolved contract work.

## Therefore: five concrete Mycelix contributions

### 1. Assurance Flow Execution Contract

Do not replace Integral's Assurance and Validation Flow.

Define a neutral contract for moving an assurance artifact through:

\`Observation -> Evidence -> Verification -> Qualification -> Recognition -> Decision-use -> Outcome -> Reassessment\`

Every transition records:
- source;
- subject;
- origin;
- time;
- validity;
- evidence class;
- verifier;
- verification method;
- scope;
- claim ceiling;
- supersession/correction;
- recognition policy.

### 2. Assurance Artifact Algebra

Treat these as distinct types:

\`Observation\`
\`Evidence\`
\`Verification\`
\`Qualification\`
\`Certification\`
\`Recognition\`
\`ReliabilityHistory\`
\`Recommendation\`
\`Decision\`
\`Authorization\`
\`EffectReceipt\`

No implicit widening is permitted.

For example:

\`Certification -> ProductionAuthorization\` is invalid unless an explicit Integral policy transition exists.

\`FRS Recommendation -> Effect\` is invalid without the appropriate receiving-system decision/authority path.

\`Foreign Certification -> Local Certification\` is invalid; recognition can be explicit without changing origin.

### 3. Viability/Coordination Envelope Proof Layer

Map Integral's existing envelopes onto executable state machines:

\`Normal -> Warning -> ThresholdCrossed -> CoordinationActive -> Stabilizing -> ExitEligible -> Dissolved\`

Then formally enforce:
- scope bounds;
- affected-node bounds;
- authority bounds;
- expiration/dissolution;
- reviewability;
- no permanent authority accumulation;
- no silent widening of constraints;
- provenance of triggering observations.

This directly deepens Integral's existing cybernetic model rather than inventing another governance layer.

### 4. Anti-Oracle Evidence Graph

Integral explicitly identifies the oracle problem and proposes physical triangulation and outcome-based verification.

Mycelix can provide the neutral graph underneath that:

\`Claim -> supporting evidence -> independent evidence -> physical observation -> outcome -> contradiction -> review\`

A claim should be able to remain:
- supported;
- weakly supported;
- conflicting;
- stale;
- indeterminate;
- superseded;
- rejected;

without forcing a binary true/false result prematurely.

### 5. Assurance portability

The same assurance workload should run against:
- Mycelix/Holochain;
- PostgreSQL/reference runtime;
- independent Integral implementation;
- simulated federation;
- later physical pilot.

The runtime must not be able to change the semantic result.

That is where Mycelix can offer something much more valuable than another implementation: **an executable assurance reference model**.

## New adversarial cases

Add mutations specifically targeting Integral's already-defined assurance concepts:

1. valid certification presented outside its validity interval;
2. safety evidence from foreign node recognized but origin rewritten;
3. reliability history projected as current capability;
4. Viability Envelope threshold triggered by stale evidence;
5. Coordination Envelope instantiated from an unbound FRS claim;
6. Coordination Envelope fails to dissolve after exit condition;
7. coordination scope expands without explicit transition;
8. assurance artifact loses source provenance during federation;
9. FRS confidence treated as probability;
10. evidence reference treated as verification;
11. verification treated as qualification;
12. qualification treated as authorization;
13. assurance result becomes an effect without decision/authorization;
14. duplicate assurance delivery creates duplicate recognition;
15. retry changes payload while preserving logical delivery ID;
16. correction rewrites rather than supersedes the historical assurance;
17. node-local recognition rewrites foreign origin;
18. reliability history becomes a general capability claim;
19. counterfactual result becomes physical evidence;
20. formal proof receipt becomes physical certification.

## Formal tranche

Add a new family:

- ASSURE-FV-015: Assurance Flow transitions cannot widen claim class implicitly.
- ASSURE-FV-016: Verification does not imply qualification.
- ASSURE-FV-017: Qualification does not imply authorization.
- ASSURE-FV-018: Recognition does not rewrite origin.
- ASSURE-FV-019: Reliability history does not imply current availability.
- ASSURE-FV-020: Viability threshold evaluation requires admissible current evidence.
- ASSURE-FV-021: Coordination Envelope cannot exceed declared scope.
- ASSURE-FV-022: Coordination Envelope dissolution is monotonic once exit criteria are satisfied.
- ASSURE-FV-023: FRS confidence does not become a probability claim without an explicit calibration contract.
- ASSURE-FV-024: Evidence reference does not constitute independent verification.
- ASSURE-FV-025: Formal proof receipt does not establish physical-world qualification.
- ASSURE-FV-026: Assurance retry cannot duplicate semantic recognition.
- ASSURE-FV-027: Assurance correction preserves historical lineage.
- ASSURE-FV-028: Counterfactual/prediction cannot satisfy observation predicates.

## Integration with existing Mycelix work

This should compose with:
- ProductiveLoopV1 evidence separation;
- COS-FV-001..010;
- IF01 OAD→COS seam model;
- IF01 refinement manifest;
- CIV-BOOT capability/replacement closure;
- proof coverage #6027;
- Symthaea's non-authoritative analytical plane.

The resulting chain becomes:

\`Integral semantic object -> Mycelix adapter -> EvidenceEnvelope -> assurance transition -> executable witness -> formal obligation -> runtime receipt -> external/physical evidence\`

No single layer is allowed to claim the authority of the next layer.

## Claim ceiling

This artifact establishes a research/engineering direction only.

It does not establish:
- Integral endorsement;
- Integral ratification;
- completeness of Integral's architecture;
- safety certification;
- real-world productivity;
- ecological/economic/social outcomes;
- correctness of Integral's normative choices;
- Mycelix as the preferred implementation.

The key distinction is:

**Integral already has the assurance concepts. Mycelix can help make their semantics mechanically inspectable and independently testable.**
