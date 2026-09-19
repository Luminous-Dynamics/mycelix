# Hearth Care Transition Authority — Amendment 001

Status: normative amendment to `CARE_TRANSITION_AUTHORITY.md`

Parent design: #2007 / #1987
Implementation issue: #2005
Migration issue: #2016
Exact parent lineage for this amendment: `cad25449281ee17fb577da2367e06fb412fd697b`

This amendment preserves the original authority theorem while correcting three implementation details discovered before consensus code was introduced.

Where this amendment conflicts with `CARE_TRANSITION_AUTHORITY.md`, this amendment governs.

## 1. Holochain action timestamp is the transition timestamp

The original contract showed `completed_at` inside `CareCompletion` and `responded_at` inside `CareSwapResponse`, then required those fields to equal the Holochain Create action timestamp.

That duplicate field is unnecessary and creates an avoidable synchronization problem: ordinary `create_entry` flow does not require the application to predeclare the exact timestamp that Holochain will place on the signed action.

### Corrected completion schema

```rust
pub struct CareCompletion {
    pub hearth_hash: ActionHash,
    pub schedule_hash: ActionHash,
    pub assignee: AgentPubKey,
    pub actor: AgentPubKey,
    pub actor_membership_hash: ActionHash,
}
```

### Corrected swap-response schema

```rust
pub enum CareSwapDecision {
    Accepted,
    Declined,
}

pub struct CareSwapResponse {
    pub hearth_hash: ActionHash,
    pub swap_hash: ActionHash,
    pub responder: AgentPubKey,
    pub actor: AgentPubKey,
    pub actor_membership_hash: ActionHash,
    pub decision: CareSwapDecision,
}
```

The canonical completion/response time is the timestamp on the signed Create action that commits the transition entry.

Read/projection DTOs MAY expose that action timestamp as `completed_at` or `responded_at`, but it is derived evidence rather than a second consensus field.

Deterministic ordering SHOULD use `(create_action.timestamp, action_hash)`.

There is therefore no separate “forged transition timestamp” entry-field case to validate.

## 2. V2 transitions live in a new appended zome pair

The original contract proposed appending new transition EntryTypes and LinkTypes to legacy `hearth_care_integrity`.

The stronger compatibility boundary is to isolate v2 transitions in a new zome pair, provisionally named:

- `hearth_care_transitions_integrity`
- `hearth_care_transitions`

### Required compatibility properties

1. `hearth_care_integrity::EntryTypes` remains unchanged.
2. Legacy Care `LinkTypes` remains unchanged.
3. Existing Hearth integrity-zome order remains unchanged.
4. Existing Hearth coordinator-zome order remains unchanged.
5. When the new pair is DNA-wired, each new zome is appended after all historical zomes in its corresponding manifest list.
6. No historical zome, entry, or link index may move as a side effect of v2 rollout.

The transition integrity zome may validate referenced legacy `CareSchedule` / `CareSwap` records by ActionHash and decode them with the canonical legacy integrity crate types.

The transition coordinator may expose versioned v2 endpoints and projections while legacy Care coordinator behavior remains available during migration.

This isolation makes eventual legacy mutable-write shutdown independent of the immutable v2 schema.

## 3. Historical CareSchedule identity is not current lifecycle state

A referenced validated `CareSchedule` action can prove immutable identity fields such as:

- Hearth binding;
- assignee binding;
- schedule data present on that revision.

It cannot, by itself, prove the latest legacy lifecycle state because legacy `CareSchedule.status` and `completed_at` are mutable under original-author updates.

The v2 transition integrity foundation MUST NOT silently strengthen a historical CareSchedule action into evidence that the schedule is currently Active, Paused, or Completed.

### First transition-integrity tranche

The first isolated `CareCompletion` integrity subject may establish only:

- transition action author == `actor`;
- referenced CareSchedule exists and decodes canonically;
- schedule Hearth == transition Hearth;
- schedule assignee == transition assignee;
- current actor Hearth membership is fresh and Active through the membership authority proof stack;
- non-assignee actor has current guardian role;
- CareCompletion is immutable;
- transition index links are append-only indexes, not authority.

It makes **no claim** that a referenced legacy schedule revision proves latest legacy status.

### Client-rollout boundary

Broad `complete_task_v2` client rollout is blocked on #2016, which must define deterministic coexistence semantics for legacy Paused/Completed state and immutable v2 completion evidence.

Potential solutions include proving the latest legacy schedule revision, enrolling schedules into a frozen v2 lifecycle, or an explicit migration record. That choice is not made by this amendment.

## 4. Transition zome link namespace

Because v2 transitions have their own integrity zome, their link indexes are local to that new zome rather than appended to legacy Care LinkTypes.

Initial link type:

- `ScheduleToCompletions`: referenced legacy CareSchedule ActionHash -> immutable CareCompletion ActionHash.

Future swap-response link type:

- `SwapToResponses`: referenced legacy CareSwap ActionHash -> immutable CareSwapResponse ActionHash.

Within the new transition zome, new variants are append-only once published.

Link validation MUST bind target evidence to the supplied base and author where appropriate, but the link remains an index. The transition entry's validity is not strengthened by existence of a link.

## 5. Membership freshness theorem is unchanged

The original source-chain membership theorem remains normative:

- actor membership evidence must be canonical Kinship `HearthMembership`;
- membership actor/Hearth must match the transition;
- claimed membership must be Active;
- deterministic actor activity is anchored before the transition;
- the claimed membership must be the newest matching membership revision for that actor/Hearth;
- later Departed/Ancestral or newer Active revisions supersede an older Active claim;
- missing or unresolved deterministic evidence is never promoted to valid authority.

The pure proof kernel in #2010 and HDI evidence adapter in #2013 implement this separation.

## 6. Updated qualification implications

### Completion integrity foundation

Must reject:

- transition authored by someone other than `actor`;
- missing or malformed schedule reference;
- schedule Hearth mismatch;
- schedule assignee mismatch;
- stale, inactive, wrong-agent, wrong-Hearth, or missing membership evidence;
- non-assignee actor without current guardian role;
- any update/delete of immutable completion evidence;
- forged transition index links.

Must not claim:

- latest legacy schedule status;
- legacy/v2 digest reconciliation;
- general client readiness.

### Compatibility

Qualification must explicitly verify that introducing the transition crate/zome does not reorder historical Hearth zomes or alter legacy Care entry/link enums.

## 7. Relationship to existing subjects

- #2007 remains the original frozen contract record.
- #2010 isolates the pure membership freshness decision.
- #2013 isolates deterministic Holochain/Kinship evidence acquisition.
- #2005 now targets the isolated transition integrity zome first.
- #2016 owns legacy lifecycle coexistence semantics.

No queued or unexecuted CI run is treated as PASS by this amendment.
