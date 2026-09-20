# Hearth Care Transition Authority — Amendment 002

Status: normative amendment to `CARE_TRANSITION_AUTHORITY.md` and `CARE_TRANSITION_AUTHORITY_AMENDMENT_001.md`

Related issues: #2016, #2089
Qualification lineage at discovery: #2050

This amendment corrects a stronger assumption that survived Amendment 001: a historical legacy `CareSchedule` revision cannot be treated as proof of the current assignee or current lifecycle state.

Where this amendment conflicts with the original contract or Amendment 001, this amendment governs.

## 1. Legacy CareSchedule immutability is narrower than previously assumed

Legacy `hearth_care_integrity::validate_schedule_immutable_fields` freezes only:

```text
hearth_hash
```

Legacy consensus does **not** freeze:

```text
assigned_to
care_type
title
description
recurrence
notes
status
completed_at
```

Those fields may differ across author-valid legacy revisions.

A validated historical CareSchedule ActionHash therefore establishes only:

```text
canonical CareSchedule revision exists
+
revision belongs to Hearth H
+
revision asserted fields X at that revision
```

It does not establish:

```text
current assignee
current status
unique/latest revision
absence of competing update heads
authoritative v2 lifecycle completion
```

## 2. CareCompletion is evidence, not lifecycle admission

The existing immutable `CareCompletion` entry remains useful and valid as an append-only attestation.

Its theorem is now explicitly:

```text
CareCompletion exists
=
actor A, while holding the required fresh Hearth membership authority,
authored completion evidence concerning CareSchedule revision R
```

It does **not** mean:

```text
schedule is completed
current assignee accepted completion
obligation is discharged
legacy lifecycle has converged
v2 lifecycle completion has been admitted
```

The positive state must therefore be described as **completion evidence present** or **completion attested**, never simply `Completed`.

## 3. CareCompletion.assignee is revision-scoped

`CareCompletion.assignee` is retained for evidence provenance and compatibility, but its semantic meaning is narrowed to:

```text
assignee asserted by referenced CareSchedule revision R
```

It must never be presented as:

```text
current assignee
```

without independent current-state proof.

The current transition-integrity rule:

```text
completion.assignee == referenced_revision.assigned_to
```

binds the attestation to the referenced revision. It does not establish that the assignment remained current after that revision.

Likewise, allowing that revision's assignee or a current guardian to author a `CareCompletion` means only that they may author **evidence under the current evidence profile**. It does not give them unilateral lifecycle-finalization authority.

## 4. Stable schedule identity is the original Create/root ActionHash

Legacy update revision hashes are observations in one update graph. They are not stable schedule identities.

For coexistence, the canonical schedule identity is:

```text
original CareSchedule Create ActionHash
```

Every legacy revision, completion-attestation, future admission, projection, digest contribution, and UI referent must be normalized to that root.

A projection must fail closed when it cannot prove a unique root ancestry.

An arbitrary update ActionHash and whichever revision a helper happens to return must never silently become the stable schedule identity.

## 5. Legacy update graphs may fork

Legacy mutation can target a historical revision. Therefore a schedule may have multiple terminal update heads.

Coexistence code must model the update graph explicitly.

```text
one established terminal head
-> one legacy lifecycle observation may be projected

multiple terminal heads
-> LegacyForkConflict

missing/unresolved ancestry
-> Unknown / Unavailable
```

No ordering returned by `get_details`, `get_latest_record`, link traversal, DHT arrival order, action timestamp, or ActionHash may be promoted into a conflict-resolution rule unless a separate canonical-resolution protocol explicitly establishes that authority.

In particular:

```text
last returned update
!= canonical latest state
```

when competing heads exist.

## 6. Lifecycle completion requires a separate immutable admission layer

Before broad client rollout, authoritative v2 lifecycle completion requires a distinct immutable admission concept, provisionally:

```rust
CareCompletionAdmission
```

Raw `CareCompletion` entries are inputs/evidence to admission, not admission themselves.

The admission schema and authority profile will be frozen in a separate subject. The first profile SHOULD satisfy these constraints:

1. admission binds the stable schedule root ActionHash;
2. admission references one or more immutable CareCompletion evidence hashes;
3. admission actor identity comes from the signed Create action, never request payload;
4. admission actor proves fresh Active Hearth membership through the #2010/#2013 authority stack;
5. admission actor is authorized independently of mutable historical assignee state;
6. a candidate initial authority profile is: original schedule creator or a current Hearth guardian;
7. integrity independently verifies the authority profile and every referenced evidence item;
8. duplicate semantically equivalent positive admissions may coalesce in projection;
9. divergent admission claims remain explicit conflict evidence unless a separately specified resolution profile exists.

The candidate creator-or-current-guardian rule is not runtime authority until its own schema, integrity logic, and qualification have been frozen.

## 7. Admission remains profile-relative

Even after an admission layer exists:

```text
AdmittedCompletedUnderProfile(P)
!= universally completed
!= legally discharged
!= beneficiary acceptance
!= admitted under another profile
```

Future profiles may differ by Care domain, Hearth policy, contractual context, beneficiary acceptance requirements, or external legal rules.

The frontend must preserve the profile/evidence context rather than flattening the state to a universal `Completed` badge.

## 8. Coexistence projection keeps evidence families independent

The pure coexistence projection must preserve at least three independent source families:

```text
legacy revision graph
completion evidence / attestations
v2 lifecycle admissions
```

A useful closed presentation vocabulary may include states such as:

```text
LegacyOnly
CompletionEvidenceOnly
AdmittedCompletedV2
LegacyCompletedOnly
LegacyV2Agreement
LegacyV2Conflict
LegacyForkConflict
Unknown
Unavailable
```

Exact public names may evolve, but the following reductions are forbidden:

```text
completion evidence present -> Completed
legacy Completed -> v2 admitted
v2 admitted -> legacy state rewritten
multiple legacy heads -> pick one silently
multiple divergent admissions -> pick one silently
```

## 9. Legacy/v2 agreement and conflict

For one established legacy head:

```text
legacy Completed + v2 completion admission
-> LegacyV2Agreement
```

This represents two evidence systems agreeing. It still counts as one lifecycle completion.

Conversely:

```text
legacy Active/Paused + v2 completion admission
-> LegacyV2Conflict
```

The system must not silently mutate one source to make it agree with the other.

When legacy has multiple terminal heads, the projection is a legacy fork conflict regardless of whether one branch happens to agree with v2.

## 10. Digest semantics count admitted completion once

Raw attestation cardinality is never a fulfilled-care count.

```text
3 CareCompletion attestations
!= 3 completed tasks
```

Likewise:

```text
legacy Completed + equivalent v2 admission
!= 2 completed tasks
```

Care digests may count a stable schedule root at most once for one admitted lifecycle completion under the digest's declared profile.

If lifecycle state is conflicted, unknown, unavailable, or only attested without admission, the digest must preserve that distinction instead of manufacturing a completed count.

## 11. Qualification requirements

Before broad v2 completion rollout, qualification must prove at least:

- legacy root ancestry is derived deterministically;
- competing terminal legacy heads are detected rather than hidden;
- a historical revision naming a former assignee does not become proof of current assignment;
- CareCompletion remains readable as evidence without becoming admission;
- a future admission layer cannot be authored solely because the actor appeared as assignee on an older revision;
- current admission authority is independently proven;
- Legacy Completed + v2 admitted coalesces to one completion in projection/digests;
- Legacy Active/Paused + v2 admitted remains explicit conflict;
- missing ancestry/source data becomes Unknown/Unavailable rather than a positive state.

Qualification-only attack zomes such as #2050 may exercise these rules, but no unsafe raw-publish capability belongs in the production Hearth DNA or hApp.

## 12. Client boundary

Broad client use of `complete_task_v2` remains blocked by #2016.

Until lifecycle admission and coexistence projection are separately implemented and qualified, clients may display immutable completion evidence only as evidence, for example:

```text
Completion attested
Evidence available
```

They must not translate it into:

```text
Completed
Resolved
Satisfied
Discharged
```

unless the corresponding admitted lifecycle state is independently established under the relevant profile.

## 13. Relationship to existing subjects

- #2007 remains the original frozen authority contract.
- #2017 / Amendment 001 remains authoritative for canonical action timestamps, isolated transition zomes, and the rule that historical status is not current status.
- This Amendment 002 supersedes the remaining assumption that historical `assigned_to` is current assignment evidence.
- #2010 and corrected #2013 remain the membership freshness/provenance foundation.
- #2021/#2024 `CareCompletion` is reclassified as immutable completion evidence, not lifecycle admission.
- #2050 adversarial qualification tests the evidence integrity boundary.
- #2016 and #2089 own coexistence/admission work before general client rollout.

No queued or unexecuted workflow run is treated as PASS by this amendment.
