# CIV-RES-001B — Privacy projection + reciprocal-accountability composition v1

Status: semantic composition contract only  
Parent: CIV-RES-001A / `bf4f63099e68bd67c02dc741ac4eff340d321b0b`  
Tracking issue: #2019  
Program: #2006

## Purpose

Freeze the universal Civic Resilience privacy projection/release boundary without creating a second person-access accountability system and without hard-coding one deployment's privacy thresholds into the universal protocol.

CIV-RES-001B composes the semantic waist from CIV-RES-001A with the reciprocal-accountability ownership boundary already being developed in `mycelix-accountability-core` / PR #28.

The accountability line remains an architectural dependency reference in this tranche. CIV-RES-001B does not pretend PR #28 is merged, qualified, or present in this branch's executable ancestry.

## Core theorem

```text
protected access != public release
```

and:

```text
valid access accountability
+ valid projection transformation
!= publication authority
```

A deployment needs all applicable boundaries independently satisfied.

## Existing semantic owner: reciprocal accountability

The reciprocal-accountability line already owns concepts including:

- pairwise/pseudonymous subject identifiers;
- person-linked `AccessReceipt` accountability;
- requester/purpose/authority binding;
- disclosure summaries;
- query-budget charges;
- inference disclosures;
- subject rights;
- immediate or expiring delayed notice;
- subject-facing `SubjectNotice` projection.

CIV-RES-001B must not redefine those semantics merely because Civic Resilience consumes protected source data.

In particular:

```text
AccessReceipt != PermissionGrant
AccessReceipt != PublicationAuthority
SubjectNotice != PublicProjection
query/access budget != public-release budget
```

`SubjectNotice` remains owned by reciprocal accountability. There is deliberately no `SubjectNotice` projection class in CIV-RES-001B.

## Universal projection classes

The v1 Civic projection classes are:

```text
PublicAggregate
CommunityScopedAggregate
ResearchOutput
```

They describe the intended destination/surface, not a guarantee that the output is anonymous, safe, authorised, representative, or fit for release.

## Universal projection artifacts

### CivicProjectionRequest

A non-authorizing request to derive a bounded projection from one exact source snapshot.

Semantic refs:

```text
request_id
source_snapshot_ref
source_sensitivity_ref
purpose_ref
projection_class
projection_policy_ref
requested_geographic_precision_ref
requested_temporal_precision_ref
requester_or_process_ref
```

### CivicProjectionPlan

A frozen description of how the requested projection would be constructed.

Semantic refs:

```text
plan_id
request_ref
source_snapshot_ref
transformation_refs[]
suppression_or_threshold_policy_ref
geographic_precision_ref
temporal_precision_ref
access_receipt_refs[]
release_budget_ref
release_history_ref
disclosure_review_policy_ref
limitations[]
```

An empty `access_receipt_refs` set may be valid only when the source path did not require person-linked/protected accountable access under the owning policy. The plan itself does not decide that fact.

### CivicProjectionEvidence

A non-authorizing record that one exact projection computation produced one exact candidate output under one exact plan.

Semantic refs:

```text
projection_evidence_id
plan_ref
source_snapshot_ref
output_commitment_ref
transformation_receipt_refs[]
access_receipt_refs[]
release_budget_state_ref
release_history_state_ref
disclosure_review_ref
release_decision_ref
limitations[]
```

`release_decision_ref` is an opaque external authority reference. CIV-RES-001B cannot mint it.

## Required non-equivalences

```text
ProtectedAccess != PublicRelease
AccessReceipt != PermissionGrant
AccessReceipt != PublicationAuthority
SubjectNotice != PublicProjection
Aggregate != Anonymous
Coarsened != Safe
Redacted != Deidentified
NoDirectIdentifier != NonIdentifiable
OneSafeRelease != SafeReleaseSequence
QueryAccessBudget != PublicReleaseBudget
ValidAccess != SafeAggregate
ValidAggregateTransformation != LegitimateAccess
AnalysisCompleted != OutputReleasable
StatisticalResult != DisclosureSafeResult
ModelOutput != AnonymousOutput
ProjectionEvidence != ReleaseAuthority
ReleaseDecisionRef != ReleaseDecisionValid
PublicProjection != SourceTruth
```

## Mosaic / reconstruction theorem

A release cannot always be evaluated in isolation.

```text
candidate release
+ prior releases
+ auxiliary information
-> potentially identifying information
```

Therefore a future executable gate must consume the current release-history/privacy-budget theorem whenever the selected deployment policy requires it.

```text
required release history unavailable -> fail closed
```

not:

```text
history unavailable -> assume current release is isolated
```

CIV-RES-001B deliberately does not define the numeric release budget itself.

## Access and release are independent boundaries

Where a projection requires person-linked/protected source access, the exact accountability receipt references required by the owning access policy must remain attached to the projection lineage.

But:

```text
valid access receipt != safe aggregate
```

and:

```text
valid aggregate transformation != legitimate access
```

The projection system cannot use a safe-looking aggregate to retroactively legitimize an improper source lookup, and it cannot use a legitimate source lookup to justify unsafe publication.

## No universal privacy magic number

The universal protocol does not freeze one value for any of these:

```text
minimum cohort size
geographic cell size
time delay
retention period
differential-privacy epsilon
suppression threshold
query budget
release budget
```

Those are deployment/policy parameters whose adequacy depends on the exact source data, adversary, purpose, auxiliary information, destination, legal/privacy review, and release history.

A deployment profile may bind exact policy values. It may not mutate CIV-RES-001B's semantic boundaries.

## Aggregate does not mean anonymous

The contract explicitly rejects these shortcuts:

```text
no name field -> anonymous
aggregate row -> anonymous
coarse geography -> safe
old data -> safe
small output -> safe
model statistic -> safe
```

Re-identification/disclosure risk is a property of the release in context, not merely of one row's shape.

## Research output boundary

The `ResearchOutput` class does not create a disclosure bypass.

```text
research enclave access != publication permission
analysis completed != output releasable
statistical result != disclosure-safe result
model output != anonymous output
```

A research result intended for public/community release must still satisfy the exact destination's release/disclosure policy and authority boundary.

## Johannesburg composition

CIV-RES-000B may later bind Johannesburg-specific sensitivity, precision, cross-border, retention and disclosure policies to this universal contract.

The deployment composition must preserve:

```text
universal projection semantics
+ exact Johannesburg profile
+ exact accountability/runtime adapters
+ exact disclosure/release authority
= bounded deployment subject
```

not:

```text
Johannesburg policy value -> universal privacy constant
```

## Runtime ownership

Runtime ownership remains deferred.

A later executable implementation must first resolve the reciprocal-accountability dependency and shared evidence/currentness surface sufficiently to avoid duplicating:

- `AccessReceipt`;
- `SubjectNotice`;
- query-budget/accountability semantics;
- generic evidence/currentness validation.

`architectural composition contract != runtime dependency convergence`.

## Continuation

```text
CIV-RES-001C  commitment -> evidence -> verification -> outcome-observation theorem
CIV-RES-002A  service-issue lifecycle composition
CIV-RES-002C  executable location/temporal projection + mosaic defenses
SYM-CIVIC-000A civic evidence-cut / study-manifest bridge
```

## Qualification claim ceiling

A PASS may establish only that this exact semantic contract preserves the access-vs-release split, reciprocal-accountability ownership, projection vocabularies, closed non-equivalence registry, mosaic/history requirement, externalized policy parameters, deferred runtime ownership and nonclaims.

It does not establish runtime release enforcement, accountability-core qualification, anonymity, de-identification, re-identification resistance, a safe threshold, a safe privacy budget, legal permission, POPIA compliance, publication authority, Johannesburg deployment, or deployment readiness.
