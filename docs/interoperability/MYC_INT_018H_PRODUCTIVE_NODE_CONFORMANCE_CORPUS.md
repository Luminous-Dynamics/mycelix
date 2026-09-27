# MYC-INT-018H — Productive-Node Cross-Domain Conformance Corpus

Status: synthetic conformance-corpus architecture. Tracks #3235. Child of MYC-INT-018G / PR #3234 and companion to MYC-INT-018B / #3224.

Observed design date: 2026-09-27.

## 1. Purpose

Freeze the deterministic cross-domain cases required before the productive-node showcase is allowed to compose Food, Water, Energy, Compost, Fabrication, Symthaea analysis and governance/effect paths into executable integration.

The corpus qualifies **semantic/evidence behavior**, not farming science or hardware performance.

```text
synthetic conformance PASS
!= physical-world validation
!= agronomic validation
!= sensor calibration
!= food/water safety
!= autonomous-control safety
```

## 2. Fixture theorem

Every fixture must preserve the complete role chain:

```text
source/native object
-> semantic source identity
-> physical observation(s)
-> optional derived assessment
-> optional analysis artifact
-> optional recommendation
-> optional decision
-> separate authorization
-> optional effect receipt
-> outcome observation
```

No later role may silently replace an earlier role.

## 3. Machine-readable manifest

The companion file:

`fixtures/MYC_INT_018H_PRODUCTIVE_NODE_CONFORMANCE_MANIFEST.json`

freezes the case registry and invariant identifiers.

The manifest is a synthetic protocol fixture index, not a schema for all future productive-node data.

## 4. Required fixture metadata

Each case must bind:

- exact case ID;
- corpus profile/version;
- synthetic-data marker;
- exact source/native schema refs;
- production-environment refs where applicable;
- observation IDs, subject refs and measurement profiles;
- source/temporal/currentness evidence refs;
- optional assessment IDs + profiles;
- optional analysis/recommendation refs;
- optional decision/authorization/effect refs;
- delivery-attempt refs when relevant;
- expected disposition;
- invariant assertions;
- evaluator-only oracle data kept out of candidate analysis input.

## 5. PN-01 — truthful production-environment identity

Positive control:

- native soil crop remains bound to native soil Plot semantics;
- hydroponic fixture uses hydroponic environment profile;
- identities remain distinct.

Negative control:

```text
hydroponic system
-> fabricated food-production::Plot soil_type
```

must be rejected.

## 6. PN-02 — missing/stale observation

Fixture contains a previously valid sensor observation that becomes unavailable/stale.

Expected:

```text
missing != zero
stale != current
last-known != current
```

Historical observation remains available as historical evidence.

## 7. PN-03 — conflicting probes

Two same-profile observations for the same subject/time window materially disagree.

Expected:

- preserve both ObservationIds;
- preserve both SourceIds;
- emit explicit conflict/uncertainty disposition;
- no implicit average becomes source truth.

Any fused estimate must be a separately derived object with method/provenance.

## 8. PN-04 — duplicate/delayed delivery

Same semantic observation is transported twice and one transport attempt arrives later.

Expected:

```text
same observation delivered twice
!= two physical observations

arrival later
!= observed later
!= automatically more current
```

## 9. PN-05 — water standards convenience field

Native water-purity fixture contains:

```text
meets_who_standards = true
```

but no independently qualified standards assessment.

Expected: retain as source-domain assertion only.

It cannot satisfy a requirement for a DerivedAssessment under the exact WHO/profile revision.

## 10. PN-06 — process suitability versus admission

Exact physical water observations produce a positive hydroponic-process suitability assessment.

Expected:

```text
SuitableUnderProfile
!= resource admitted
!= valve authority
!= pump authority
```

A separate local authorization is still required for an effect.

## 11. PN-07 — energy verification convenience field

Native energy fixture contains:

```text
EnergyProduction.verified = true
```

with insufficient independent meter/verification evidence.

Expected:

```text
source verified bool
!= verification receipt
!= qualification-grade telemetry
```

## 12. PN-08 — EvidenceLease versus domain currentness

Fixture supplies a still-valid evidence-reuse lease while the domain subject is stale/revoked/superseded or otherwise non-current under its domain theorem.

Expected: currentness remains negative/indeterminate.

```text
EvidenceLease valid
!= domain current
```

## 13. PN-09 — residue / compost admission

Crop residue is registered as a waste/resource source record.

Expected:

```text
waste record
!= compost input
```

until explicitly admitted to a CompostBatch.

Finished compost remains:

```text
compost output
!= hydroponic nutrient solution
```

without a separately qualified conversion/chemistry process.

## 14. PN-10 — fabrication / repair chain

Fixture chain:

```text
failure observation
-> Design
-> design verification/admission where applicable
-> ManufacturedArtifact
-> installation authorization
-> InstalledArtifact/work receipt
-> post-repair observations
-> RepairOutcomeAssessment
```

Every identity remains distinct.

A design score or successful installation cannot prove repair outcome.

## 15. PN-11 — assessment-profile revision

Same exact input observations are evaluated under method/standard v1 and v2.

Expected:

```text
Assessment(v1)
!= Assessment(v2)
```

Historical v1 result remains intact.

No retroactive rewrite.

## 16. PN-12 — analysis contradicted by later observation

Symthaea fixture emits a prediction/diagnostic candidate.

Later source observation contradicts it.

Expected:

- original AnalysisArtifact remains unchanged;
- later observation remains source-owned;
- contradiction/evaluation produces a new assessment/review candidate;
- analysis does not rewrite source history.

## 17. PN-13 — multi-objective tradeoff

Synthetic operating change yields:

```text
water intensity improved
energy intensity worsened
```

possibly with additional labor/reliability changes.

Expected: preserve the vector of results.

Reject an evaluator that emits a hidden universal:

```text
better = true
```

without explicit weighting/profile authority.

## 18. PN-14 — foreign evidence versus local authority

Foreign/federated node supplies valid evidence/assessment.

Expected:

```text
foreign evidence admitted
!= foreign policy admitted
!= foreign decision authoritative locally
!= local effect authorized
```

## 19. PN-15 — module removal/export

Reconstruct a productive-node history with one optional module absent, such as Energy or Compost.

Expected:

- surviving Food/Water/etc history remains parsable;
- source SemanticRefs remain unchanged;
- historical cross-domain views expose missing module inputs explicitly;
- no remaining source object is reinterpreted.

## 20. PN-16 — ITC disabled

Replay the operational/evidence chain with Integral accounting projection disabled.

Expected:

- source observations remain valid;
- assessments/analysis remain possible;
- decision/authorization/effect semantics remain possible;
- no core identity depends on ITC account/value semantics.

This is a neutrality control.

## 21. PN-17 — synthetic provenance preservation

Synthetic fixture passes through observation, assessment, dashboard, export and analysis paths.

Expected:

```text
synthetic source
-> synthetic provenance retained end-to-end
```

No transform may silently promote it into a physical-world observation.

## 22. PN-18 — dashboard / source-owner separation

Cross-domain view aggregates food, water, energy, waste/material and reliability information.

Expected:

```text
derived dashboard
!= source owner
```

The view must preserve source refs and incomplete/stale/conflict state.

It cannot write its summary back as a source-domain observation.

## 23. Hidden-oracle discipline

Evaluator expected outcomes must not enter candidate analysis/model input.

```text
candidate input
!= evaluator answer key
```

Where a case tests Symthaea or another analysis system, oracle data belongs in evaluator-only fixture material.

## 24. Unit and profile discipline

Every numeric physical fixture value must bind:

- explicit unit;
- measurement profile;
- subject;
- source;
- observation time evidence.

No naked float may carry cross-domain meaning by itself.

## 25. Currentness discipline

Corpus evaluator must not infer currentness from:

- delivery arrival order;
- largest timestamp supplied by caller;
- nonzero `verified`/`current` booleans;
- EvidenceLease alone;
- latest database/DHT row.

Currentness uses the applicable domain/profile theorem.

## 26. Source/derived discipline

Every fixture object must be structurally or explicitly typed as one of the applicable roles:

```text
SourceObject
Observation
DerivedMetric
Assessment
Prediction
Recommendation
Decision
Authorization
EffectReceipt
OutcomeObservation
```

Do not use one generic `event` bucket to erase role boundaries.

## 27. Authority discipline

At minimum, evaluator must reject:

- observation -> direct effect;
- assessment -> direct effect;
- recommendation -> direct effect;
- foreign decision -> local effect;
- expired authorization -> effect;
- source suitability -> automatic admission;
- source `verified` boolean -> authority.

## 28. Translation discipline

Any cross-domain projection must preserve translation receipt semantics where required.

The evaluator must distinguish:

- identity-like wrapper;
- structural projection;
- lossy translation;
- derived interpretation;
- no supported mapping.

Unknown/lossy translation may not silently strengthen meaning.

## 29. Export discipline

A corpus replay/export should preserve:

- source schema/profile identity;
- SemanticRefs;
- EPI role identities;
- provenance;
- synthetic marker;
- observation/assessment profile versions;
- conflicts/unknowns;
- decision/authorization/effect history where applicable.

## 30. Evaluator responsibilities

A deterministic evaluator should be able to assert protocol properties such as:

- role separation;
- identity distinction;
- source ownership;
- required translation receipt presence;
- currentness ceiling behavior;
- duplicate-delivery semantics;
- authority-negative controls;
- module replaceability/export.

It should **not** need to know universal crop recipes, safety thresholds, or sustainability rankings.

## 31. Evaluation result vocabulary

Avoid one `pass: bool` as the only diagnostic surface.

At minimum distinguish conceptually:

- expected admission;
- expected rejection;
- expected partial/unknown;
- invariant violation;
- unsupported profile;
- evaluator fixture error.

Exact executable enum belongs to the later implementation tranche.

## 32. Corpus integrity

Before executable qualification:

- case IDs unique/stable;
- profile version fixed;
- manifest canonicalized deterministically;
- all referenced local fixtures present;
- hidden oracle references unavailable to candidate input;
- no external network required;
- synthetic marker present in every fixture root;
- unknown fields behavior explicit.

## 33. Relationship to 018B H0 hydroponic corpus

018B remains the detailed hydroponic perturbation corpus.

018H adds cross-domain composition cases.

```text
018B
= hydroponic operational/evidence perturbations

018H
= productive-node cross-domain semantic composition
```

Do not duplicate every 018B pump/pH/EC scenario in 018H; reference/reuse them when the cross-domain theorem needs them.

## 34. Qualification ladder

Recommended:

```text
018H manifest + synthetic fixtures
-> deterministic parser/schema checks
-> deterministic invariant evaluator
-> exact-host qualifier
-> only then H1/H2 physical composition
```

No physical actuator is needed to prove these semantic theorems.

## 35. Nonclaims

A future 018H PASS would establish only the exact deterministic cross-domain conformance properties encoded by the qualified corpus/evaluator.

It would not establish:

- agronomic correctness;
- sensor calibration/authenticity;
- food/water safety;
- regulatory compliance;
- commercial viability;
- hardware reliability;
- Symthaea real-world accuracy;
- governance/economic superiority;
- autonomous-control safety.
