# ADR-0005: Learning Analysis Is Descriptive, Evidence-Bound, and Non-Authoritative

- Status: Proposed
- Subject: `PRAX-POL-001A`
- Depends on: `PRAX-EVIDENCE-001`, `PRAX-EVIDENCE-002A`
- Follow-up: `PRAX-POL-001B` legacy PoL/MATL quarantine

## Context

The historical `proof_of_learning.rs` combines heterogeneous learning-activity heuristics into one `ProofOfLearning.score`. Its vocabulary includes `error_authenticity`, comments that no failures could indicate gaming, describes transfer as hard to fake, and offers `PoLMATLScore::combine` to alter a general MATL trust score.

Those experiments may contain useful descriptive signals, but the available evidence does not independently establish learner authenticity, cheating, mastery, credential eligibility, or general trustworthiness.

Renaming the scalar is not enough. The replacement contract needs to preserve what evidence supported each analysis component and prevent an aggregate from silently gaining consequential authority.

## Decision

Praxis introduces `LearningAnalysisProjection` as the authority-safe analysis contract.

Core theorem:

```text
learning-pattern analysis
!= authenticity proof
!= cheating proof
!= mastery
!= credential eligibility
!= general trust
!= authorization
```

### No global composite score

The projection contains descriptive components but deliberately has no global analysis score. A consumer that wants to combine components must do so through a separate named/versioned policy and retain the component evidence lineage.

### Descriptive component vocabulary

The core component kinds are:

- `TrajectoryTrend`
- `ErrorPatternDistribution`
- `RetentionPerformance`
- `TransferPerformance`
- `ContributionActivity`
- `TemporalConsistency`

There is no `Authenticity` or `Cheating` component in the core contract.

### Exact evidence topology

Every projection declares a unique global set of stable `EvidenceEventId` inputs. Every component declares the exact subset it consumed.

Validation rejects:

- duplicate global inputs;
- empty component evidence;
- duplicate component inputs;
- component evidence not declared by the projection;
- global inputs that no component actually references;
- duplicate component kinds;
- empty analyzer identity/version/parameter digest;
- non-normalized component estimates/support values.

This prevents provenance padding and makes component recomputation auditable.

### Observation versus admission remains explicit

`AnalysisEvidenceBasis` records whether the analyzer consumed raw observations or evidence admitted under a named/versioned policy. An observed event does not become admitted merely because an analyzer used it.

### No automatic authority

The contract freezes:

```text
LearningAnalysisProjection.grants_credential_authority() == false
LearningAnalysisProjection.grants_trust_authority() == false
```

## Legacy PoL compatibility

`LegacyPoLCompatibilitySummary` can preserve the historical PoL score/confidence for migration or display, but it deliberately has no stable `input_event_ids` and records:

```text
evidence_lineage_complete = false
```

There is intentionally no conversion from this legacy summary into a provenance-complete `LearningAnalysisProjection`.

The adapter also rejects non-finite or out-of-range legacy score/confidence values rather than normalizing malformed history into apparently valid evidence.

## Non-goals

This tranche does not:

- change the historical PoL algorithms;
- rename/remove legacy public fields;
- claim old PoL evidence has stable event provenance;
- change MATL scoring;
- provide an authenticity detector;
- provide a cheating detector;
- issue credentials;
- grant trust or authorization.

## Follow-up: PRAX-POL-001B

The next tranche should quarantine the legacy authority-bearing vocabulary/path while preserving reproducibility:

1. document `ProofOfLearning`, `error_authenticity`, and `PoLMATLScore` as legacy heuristics;
2. remove claims that no failures imply gaming or that a component is intrinsically hard to fake;
3. ensure new code paths use `LearningAnalysisProjection` instead of direct PoL→MATL amplification;
4. require a separate explicit trust-use policy before any learning analysis can influence consequential trust;
5. preserve the old algorithm only as a versioned legacy reproduction path where compatibility requires it.
