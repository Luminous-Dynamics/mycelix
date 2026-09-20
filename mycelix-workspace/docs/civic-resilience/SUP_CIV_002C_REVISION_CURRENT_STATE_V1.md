# SUP-CIV-002C — Revision lineage, conflict and admitted-current-state contract v1

Status: semantic contract only; stacked on SUP-CIV-002A and transitively blocked on parent qualification  
Parent subject: `1733cb6d6e281e16530335c118d951ad939d45d7`  
Tracking issue: #2148  
Hardening program: #2030

## Purpose

Freeze how one stable support-ticket identity acquires reviewable revision lineage and an admitted operational current state without pretending Holochain CRUD automatically chooses a current revision.

## Core theorem

```text
StableSupportTicketId
+ validated genesis
+ evaluated revision-DAG evidence cut
+ admitted typed-mutation edges
+ explicit conflict policy
= AdmittedOperationalTicketState
```

not:

```text
get(entry_hash) = current
latest record seen = current
newest timestamp = current
highest action sequence = current
status index = current
```

## Revision node

A semantic revision node binds:

```text
stable_support_ticket_id
revision_action_ref
parent_revision_action_ref | GENESIS
mutation_event_ref | GENESIS
resulting_ticket_state_ref
actor_ref
policy_or_authority_ref
authored_time_evidence_ref
```

`StableSupportTicketId` and `revision_action_ref` are distinct identifiers with distinct meanings.

## Admitted lineage edge

For `parent -> child` to enter the CIV-safe lineage, a later runtime must establish all of:

1. parent and child belong to the same stable logical ticket identity;
2. child is an actual valid Holochain update of the declared parent;
3. child was produced by an admitted typed mutation under the SUP-CIV-002A contract;
4. the exact command write-set and status transition are satisfied;
5. identity-bound immutable intake facts remain unchanged;
6. mutation provenance binds actor, command, reason and applicable authority/policy evidence;
7. the parent revision equals the mutation's expected revision.

```text
HolochainUpdate != AdmittedSemanticRevision
```

A valid raw/legacy update may remain historical Support data without automatically entering the CIV-safe projection.

## Evidence cut

Every current-state evaluation is relative to an explicit evidence cut.

Semantic refs:

```text
lineage_evidence_cut_ref
observed_at_ref
retrieval_strategy_ref
visible_revision_refs[]
visible_head_refs[]
missing_or_partition_limitations[]
```

The contract refuses an unqualified claim of omniscient network completeness.

```text
NoVisibleCompetingHead != NoCompetingHeadExistsAnywhere
```

## Head derivation

For the admitted revision graph in one evidence cut:

```text
H = admitted revision nodes with no admitted child in that cut
```

### One admitted head

If `|H| == 1`, the result may be `UniqueAdmittedHead`.

That means only that one admitted head was derived under the declared evidence cut and policy.

```text
UniqueAdmittedHead != UniversalRealWorldCurrentTruth
```

### Multiple admitted heads

If `|H| > 1`, the result is `Conflict`.

```text
ConcurrentAdmittedHeads -> Conflict
```

The implementation must not silently choose a winner by timestamp, hash ordering, action sequence, insertion order, local-cache arrival order, or status-index membership.

## Current-state result algebra

The closed v1 result vocabulary is:

```text
Absent
LegacyUnmigrated
UniqueAdmittedHead
Conflict
InsufficientLineageEvidence
InvalidLineage
RetiredOrDeletedUnderProfile
```

A later API should return a tagged result/receipt carrying the relevant refs and limitations rather than collapsing these meanings into `Option<SupportTicket>`.

### Absent

No qualifying support-ticket subject is established for the requested stable identity/reference.

### LegacyUnmigrated

A legacy ticket exists but the stable-identity/runtime lineage prerequisites required by this profile are absent.

### UniqueAdmittedHead

Exactly one admitted head exists in the evaluated graph cut.

### Conflict

Two or more incomparable admitted heads exist.

### InsufficientLineageEvidence

The evaluator cannot establish enough ancestry/evidence to admit or reject a purported lineage/current-state claim safely.

### InvalidLineage

Observed lineage violates an exact invariant, such as cross-ticket ancestry, forbidden mutation, immutable-field rewrite, invalid transition, or malformed conflict-resolution ancestry.

### RetiredOrDeletedUnderProfile

A qualified profile has admitted a distinct retirement/deletion treatment. This state does not mean `Closed` or administrative finality.

## Conflict resolution

A conflict-resolution event must explicitly bind:

```text
stable_support_ticket_id
conflicting_head_refs[]
selected_or_merged_parent_refs[]
resolution_policy_ref
resolver_authority_ref
resolution_reason_ref
new_revision_ref
```

The new revision becomes part of lineage only if it satisfies the profile's resolution policy and ordinary lineage invariants.

Conflict resolution retains conflicting historical branches.

```text
ConflictResolution != DeleteHistory
ConflictResolution != ProveOtherBranchFraudulent
```

## Timestamp/order boundary

Timestamps and action sequence are evidence, not a universal authority rule.

```text
NewestTimestamp != CurrentRevisionAuthority
HighestActionSequence != CurrentRevisionAuthority
```

A future profile may use ordering information inside a qualified conflict policy, but this contract grants no default winner.

## Holochain get/read boundary

A Holochain record/entry lookup is retrieval evidence, not the application current-state theorem.

```text
GetEntryResult != ProvenCurrentState
LatestRecord != ProvenCurrentState
```

The application must walk/evaluate update metadata under its own admitted lineage policy.

## Index boundary

Status/category/requester/assignee indexes are derived projections.

```text
IndexMembership != AdmittedOperationalTicketState
StatusLink != CurrentStatusProof
```

An index disagreement cannot override admitted revision lineage. A later index tranche must repair/rebuild the projection from qualified current-state receipts.

## Delete / retirement boundary

Deletion/liveness, operational closure and administrative finality are separate.

```text
DeleteAction != TicketStatus::Closed
TicketStatus::Closed != DeleteAction
DeleteAction != AdministrativeFinality
```

This contract does not decide whether ordinary ticket deletion should eventually be forbidden, represented as typed retirement, or admitted under a narrow profile. It requires that the policy be explicit.

## Required conflict receipt

A `Conflict` result should expose at least:

```text
stable_support_ticket_id
lineage_evidence_cut_ref
conflicting_head_refs[]
common_ancestor_refs[]
conflict_detected_at_ref
limitations[]
```

It must not carry a hidden selected winner.

## Required unique-head receipt

A `UniqueAdmittedHead` result should expose at least:

```text
stable_support_ticket_id
admitted_head_ref
lineage_evidence_cut_ref
head_derivation_ref
observed_at_ref
limitations[]
```

It must not state globally complete currentness unless a stronger external theorem actually establishes it.

## Required adversarial cases

A runtime qualification must cover at least:

- two admitted children of one parent -> `Conflict`;
- newer timestamp cannot silently win a conflict;
- higher action sequence cannot silently win a conflict;
- cross-ticket child/parent lineage -> `InvalidLineage`;
- raw legacy whole-record update is not automatically admitted as typed lineage;
- requester or created-at mutation -> `InvalidLineage`;
- forbidden status transition -> `InvalidLineage`;
- missing parent -> `InsufficientLineageEvidence`, not fabricated ancestry;
- incomplete evidence cut cannot claim no competing branch exists globally;
- conflict-resolution event retains all conflicting head references;
- stale status index cannot override admitted head;
- delete metadata does not become `Closed` or administrative finality.

## Required non-equivalences

```text
ActionHash != StableSupportTicketId
HolochainUpdate != AdmittedSemanticRevision
LatestRecord != ProvenCurrentState
GetEntryResult != ProvenCurrentState
NewestTimestamp != CurrentRevisionAuthority
HighestActionSequence != CurrentRevisionAuthority
NoVisibleCompetingHead != NoCompetingHeadExistsAnywhere
UniqueAdmittedHead != UniversalRealWorldCurrentTruth
ConcurrentAdmittedHeads != AutomaticWinner
ConflictResolution != DeleteHistory
ConflictResolution != ProveOtherBranchFraudulent
IndexMembership != AdmittedOperationalTicketState
StatusLink != CurrentStatusProof
DeleteAction != TicketStatus::Closed
TicketStatus::Closed != DeleteAction
DeleteAction != AdministrativeFinality
```

## Dependency and qualification rule

This subject is a semantic child of SUP-CIV-002A `1733cb6d6e281e16530335c118d951ad939d45d7` and transitively relies on the stable-identity theorem.

A green qualifier for this exact subject does not qualify its parents and does not establish runtime behavior.

## Deferred work

```text
SUP-CIV-001B  runtime stable identity
SUP-CIV-002B  runtime typed mutations
SUP-CIV-002D  runtime lineage/current-state evaluator
SUP-CIV-003A  exact UTC sharding + legacy shard compatibility
SUP-CIV-003B  current index projection
SUP-CIV-004   autonomy/effect-authority split
SUP-CIV-005   Civic Resilience adapter
```

## Claim ceiling

A PASS may establish only this exact semantic revision-DAG, conflict and evidence-cut-relative admitted-current-state contract. It does not establish globally complete network knowledge, runtime lineage validation, service completion, administrative finality, municipal authority, privacy compliance, Johannesburg readiness, or deployment readiness.
