# Hearth Care Completion Admission Profile v1

Status: normative admission contract

Parent semantics:

- `CARE_TRANSITION_AUTHORITY_AMENDMENT_002.md` / #2091
- deterministic coexistence projection / #2097
- admission tracker / #2163

This document freezes the first positive Care lifecycle-admission profile. It does not implement an entry type, coordinator endpoint, DNA mutation, frontend state, digest mutation, legal discharge, or beneficiary acceptance.

Where historical Care transition documents implied that `CareCompletion` itself establishes lifecycle completion, Amendment 002 governs: `CareCompletion` is immutable revision-scoped completion evidence only.

## 1. Governing theorem

```text
CareCompletion evidence exists
!= lifecycle completion admitted
!= current assignment proven
!= current legacy status proven
!= beneficiary acceptance
!= obligation satisfied
!= legal discharge
```

The positive admission state introduced by this profile is therefore named:

```text
AdmittedCompletedUnderProfile(CreatorOrCurrentGuardianV1)
```

It must never be shortened, in consensus or evidence-bearing UI, to a universal `Completed` claim without preserving the profile context.

## 2. Closed v1 profile vocabulary

The first profile identifier is exactly:

```rust
pub enum CareCompletionAdmissionProfileV1 {
    CreatorOrCurrentGuardianV1,
}
```

The enum is closed. Unknown future profile identifiers are unsupported, not aliases for v1.

The initial profile authorizes a positive admission only when the admitting actor is:

1. the original canonical CareSchedule **Create action author**; or
2. a **current guardian** of the schedule's Hearth.

Both paths additionally require fresh Active Hearth membership through the corrected #2010/#2013 authority stack.

Historical `CareSchedule.assigned_to` is never an admission-authority input.

## 3. Stable admission subject

The admission subject is the original legacy CareSchedule **Create/root ActionHash**.

```text
schedule_root_hash = original CareSchedule Create ActionHash
```

An update revision, whichever revision a helper returns, a completion-evidence hash, or a frontend row identifier must never substitute for the stable root.

Admission validation MUST prove that `schedule_root_hash`:

- references a canonical `hearth_care_integrity::CareSchedule` application entry;
- is a **Create** action, not Update;
- has canonical legacy CareSchedule AppEntryDef provenance;
- exposes the Hearth binding used by this admission.

The root action author is the `original_schedule_creator` for this profile.

## 4. Candidate admission schema

The first consensus representation SHOULD have the following semantic fields:

```rust
pub struct CareCompletionAdmissionV1 {
    pub schema_version: u8,
    pub profile: CareCompletionAdmissionProfileV1,
    pub hearth_hash: ActionHash,
    pub schedule_root_hash: ActionHash,
    pub evidence_hashes: Vec<ActionHash>,
    pub actor_membership_hash: ActionHash,
}
```

`actor` is deliberately not caller supplied. The signed Create action author is canonical actor identity.

There is no mutable status field and no duplicate `admitted_at` timestamp. The signed Create action timestamp is canonical admission time.

`schema_version` is exactly `1` for this contract.

## 5. Completion-evidence set

`evidence_hashes` is a bounded canonical set encoded as a vector for deterministic wire representation.

For v1:

```text
1 <= evidence_hashes.len() <= 32
```

Every element must be unique.

The vector MUST be strictly sorted by canonical raw ActionHash bytes. Unsorted input and duplicate references are invalid construction, not alternate semantics.

Order grants no authority and conveys no priority.

Every referenced evidence hash must resolve to a canonical `hearth_care_transitions_integrity::CareCompletion` entry and independently satisfy all of the following:

- the evidence entry itself is valid immutable transition evidence;
- its referenced CareSchedule revision has canonical legacy CareSchedule provenance;
- that revision's explicit update ancestry deterministically reaches `schedule_root_hash`;
- the root Hearth equals admission `hearth_hash`;
- evidence Hearth equals admission `hearth_hash`.

The admission does **not** require the evidence actor to equal the admission actor.

The admission does **not** use `CareCompletion.assignee` as current-assignment proof or admission authority. That field remains revision-scoped provenance only.

## 6. Legacy revision ancestry rule

For one referenced legacy CareSchedule revision `R`, normalization to the stable root must follow explicit Holochain action ancestry:

```text
Create -> root
Update(original_action_address = P) -> recurse(P)
```

Validation fails closed when ancestry is missing, malformed, cyclic, changes Hearth, reaches a non-CareSchedule application entry, or reaches more than one claimed root through inconsistent evidence.

Action timestamp, action sequence, DHT arrival order, link order, `get_latest_record`, and returned-list order are not ancestry rules.

This ancestry proof binds evidence to a stable subject. It does not prove that the referenced revision is a current/terminal legacy head.

## 7. Membership authority

Admission actor identity is:

```text
admission_create_action.author
```

`actor_membership_hash` must reference canonical `hearth_kinship_integrity::HearthMembership` evidence for that same actor and Hearth.

The corrected #2010/#2013 theorem must independently establish that the claimed membership revision is the actor's freshest matching Hearth membership before the admission action and that it is `Active`.

Unresolved host evidence propagates as unresolved/invalid admission; it is never strengthened into authority.

## 8. Creator-or-current-guardian rule

After fresh Active membership is established:

```text
actor == original_schedule_creator
    -> profile role condition satisfied

otherwise
    -> actor's fresh current MemberRole MUST satisfy is_guardian()
```

The profile therefore intentionally allows the original schedule creator to admit completion while still an Active member even if their current role is no longer guardian-level.

A non-creator whose historical role was guardian but whose freshest membership is not currently guardian fails.

A former assignee, current assignee, evidence author, requester, beneficiary, or UI operator receives no admission authority from that fact alone.

## 9. Positive admission scope

A valid immutable admission establishes only:

```text
actor A
under CreatorOrCurrentGuardianV1
admitted lifecycle completion
for stable schedule root S
using evidence set E
at the signed admission action time
```

It does not establish:

```text
universal completion
legacy state convergence
absence of legacy forks
beneficiary acceptance
quality of performance
obligation satisfaction
payment/consideration settlement
legal discharge
admission under any other profile
```

## 10. Relationship to the #2097 projection kernel

This admission layer produces facts suitable for #2097's `ValidatedCompletionAdmission` input.

Admission validation and lifecycle projection remain separate theorems:

```text
valid CareCompletionAdmissionV1
!= LegacyV2Agreement
```

The projection kernel must still combine independently sourced:

- explicit legacy revision graph;
- completion evidence;
- validated admissions;
- source availability.

Thus:

```text
legacy Completed + validated admission
-> LegacyV2Agreement

legacy Active/Paused + validated admission
-> LegacyV2Conflict

multiple legacy terminal heads
-> LegacyForkConflict

observed legacy deletion under the current kernel
-> LegacyDeletionConflict
```

The admission entry itself does not resolve any of those conditions.

## 11. Equivalent and divergent admissions

Two positive v1 admissions are semantically equivalent for lifecycle completion when they bind:

- the same profile;
- the same schedule root;
- the same Hearth.

Their evidence sets and actors may differ and remain independently preserved provenance.

Equivalent positive admissions may coalesce to one positive lifecycle fact in projection/digest computation, but their records are never deleted or rewritten.

A future profile that can express a lifecycle disposition other than positive completion requires a new explicit conflict model. V1 does not invent last-write-wins resolution.

## 12. Exact-once digest semantics

For one stable schedule root and one declared digest profile:

```text
N completion attestations
+ M equivalent positive admissions
+ legacy Completed agreement
-> at most one admitted completed schedule contribution
```

Raw evidence count and admission count remain separate provenance metrics.

Conflict, Unknown, Unavailable, evidence-only, fork, or deletion-conflict states must not contribute a fabricated fulfilled-care count.

## 13. Required adversarial qualification

Before runtime promotion, qualification must prove at least:

- update ActionHash cannot substitute for root Create ActionHash;
- non-CareSchedule root/type lookalikes are rejected;
- evidence from another integrity zome is rejected even when Serde-compatible;
- evidence bound to another root/Hearth is rejected;
- empty, duplicate, unsorted, or >32 evidence sets are rejected;
- admission action author, not payload, is actor identity;
- another actor's membership cannot authorize the admission;
- membership from another Hearth cannot authorize the admission;
- stale Active membership after departure is rejected;
- original creator path works only while the creator is currently Active;
- non-creator path requires current guardian role;
- a historical/former assignee who is neither creator nor current guardian cannot admit completion merely because old evidence named them;
- unresolved membership/ancestry/evidence dependencies fail closed;
- equivalent admissions do not multiply lifecycle/digest completion count;
- legacy/v2 disagreement remains explicit conflict through #2097.

Qualification-only raw-publish/test zomes may exercise the integrity boundary, but no unsafe raw admission endpoint belongs in production Hearth DNA/hApp.

## 14. Runtime ordering

Implementation order is frozen as:

1. this profile contract;
2. pure schema/construction helpers where useful;
3. append immutable admission entry/link variants to the already-published transition-zome vocabulary without moving existing indexes;
4. integrity validation against canonical root/evidence/membership provenance;
5. coordinator derives actor, root, Hearth, membership proof and canonical evidence set without accepting authority claims from the browser;
6. SweetConductor positive/adversarial qualification;
7. projection composition with #2097;
8. exact-once digest composition;
9. frontend admitted-state projection.

## 15. Client boundary

Until the admission implementation and projection composition qualify, clients may display:

```text
Completion attested
Completion evidence available
```

They must not display an unqualified universal:

```text
Completed
Resolved
Satisfied
Discharged
```

When v1 admission eventually qualifies, evidence-bearing UI should preserve the profile, for example:

```text
Admitted complete under Hearth creator/guardian profile
```

rather than erasing the profile-relative authority boundary.

No queued or unexecuted workflow run is a PASS.