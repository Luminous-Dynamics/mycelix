# MYC-INT-007Q — Evidence-Bearing Maturation Checkpoints and Transitions

Status: design / synthetic validation fixture only. Child of MYC-INT-007O/P.

## Purpose

Define how the planned maturation trajectory can later be compared with real evidence without mutating a planned snapshot into an observed claim.

```text
007O planned checkpoint
!= ObservedMaturationCheckpointV1
!= MaturationTransitionRecordV1
!= governance authority
```

The first 007Q objects are `SyntheticValidationFixture` subjects. They exercise the contract only; they do not claim Node A has reached N1.

## ObservedMaturationCheckpointV1

A checkpoint binds one node/profile/window and keeps all 13 capability dimensions present. Each dimension records a descriptive dependency state, source class, evidence references, currentness state, explicit external dependencies and missing/conflicting evidence.

Allowed source classes:

- `DirectObservation`;
- `SourceOwnedOperationalFact`;
- `ReconstructedState`;
- `DerivedAssessment`;
- `OperatorDeclaration`;
- `ImportedForeignEvidence`;
- `Unknown`.

Missing evidence never deletes a dimension; it produces `Unknown` and an explicit gap.

## MaturationTransitionRecordV1

A transition record separately evaluates the frozen 007O evidence requirements for a specific `from_generation -> candidate_to_generation` transition.

Allowed dispositions:

- `EvidenceSatisfiedUnderProfile`;
- `EvidenceIncomplete`;
- `EvidenceConflicting`;
- `NotEstablished`;
- `Superseded`.

Even `EvidenceSatisfiedUnderProfile` is descriptive under the named technical profile. It grants no governance standing, certification, funding, federation membership, voting rights or actuation authority.

## Quantitative claims

A share/rate requires an exact window, system boundary, numerator, denominator and coverage profile. If denominator or coverage is unknown, the quantitative claim is unknown.

```text
solar installed
!= local-energy-share known

recirculating system
!= water-reuse-share known

workshop exists
!= local-repair-share known
```

## Corrections

Observed checkpoints are immutable evidence subjects. Corrections/supersession create new records and retain the prior subject.

## Symthaea boundary

A Symthaea `AnalysisArtifact` may support a `DerivedAssessment` entry but cannot become a direct observation or establish a maturation transition by itself.

## Fixture campaign

007Q includes:

- one synthetic complete N0→N1 checkpoint/transition example;
- one hostile-case manifest covering missing evidence, denominator gaps, hidden critical imports, stale/conflicting evidence, analysis-to-fact promotion, planned-to-observed mutation and authority escalation.

## Nonclaims

007Q establishes no real node maturity, economic independence, ecological sustainability, governance legitimacy, legal compliance or preferred political/economic model.