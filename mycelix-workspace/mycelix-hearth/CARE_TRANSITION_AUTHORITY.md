# Hearth Care Transition Authority Contract

Status: design freeze for HEARTH-CARE-AUTH-001A

Parent issue: #1987
Implementation issue: #2005
Exact parent lineage when this contract was frozen: `f324b69aa89cb55a1c2f24823e60806fff23b1bc`

## 1. Problem

The current Care coordinator and Care integrity zome encode incompatible mutation authorities.

The coordinator permits an assignee or guardian to complete a CareSchedule and permits a swap responder or guardian to accept or decline a CareSwap. The integrity zome, however, protects updates with original-author matching. A non-author actor can therefore be authorized by the coordinator and still be rejected by integrity.

Removing the integrity author check would make cross-author mutation forgeable. The repair is therefore not to weaken validation. The repair is to stop representing these actions as mutation of somebody else's entry.

## 2. Governing theorem

Care lifecycle actions that may legitimately be performed by an actor other than the original entry author MUST be represented as new, immutable, actor-authored evidence.

The DHT must be able to verify:

- who acted;
- what object the action refers to;
- which Hearth governs the action;
- the relationship between actor and assignee/responder;
- that the actor had fresh Active Hearth membership at action time;
- any guardian-on-behalf authority independently of browser state;
- the transition timestamp from the Holochain action itself;
- conflict/replay semantics without last-write-wins mutation.

Presentation and coordinator checks may improve UX and fail early, but they do not substitute for integrity validation.

## 3. Compatibility rule: append, never renumber

Care currently has historical entry variants:

1. `CareSchedule`
2. `CareSwap`
3. `MealPlan`

New consensus entry variants MUST be appended after those variants. Existing variants MUST NOT be reordered.

The same rule applies to link types. New transition links MUST be appended after the existing Care link variants.

This avoids silently changing historical AppEntryDef or LinkType indexes.

## 4. Immutable completion evidence

The target v2 completion entry is conceptually:

```rust
pub struct CareCompletion {
    pub hearth_hash: ActionHash,
    pub schedule_hash: ActionHash,
    pub assignee: AgentPubKey,
    pub actor: AgentPubKey,
    pub actor_membership_hash: ActionHash,
    pub completed_at: Timestamp,
}
```

`actor` is the agent who authored the completion action.

`assignee` is the person to whom the CareSchedule belongs.

These fields MUST remain separate. A guardian completing on behalf of another member does not become the assignee merely because the guardian authored the completion event.

### 4.1 Completion integrity requirements

For a `Create` of `CareCompletion`, integrity MUST establish all of the following:

1. action author equals `actor`;
2. referenced `CareSchedule` exists;
3. referenced schedule's `hearth_hash` equals `CareCompletion.hearth_hash`;
4. referenced schedule's `assigned_to` equals `CareCompletion.assignee`;
5. `completed_at` equals the Holochain Create action timestamp;
6. the actor has a fresh Active membership proof for the same Hearth;
7. if `actor == assignee`, Active membership is sufficient for the actor relationship;
8. if `actor != assignee`, the actor's proven current membership role MUST be guardian-level;
9. the completion entry itself is immutable after creation.

Coordinator code SHOULD run equivalent checks before commit for fast feedback, but network validity is defined by integrity.

### 4.2 Duplicate and concurrent completion evidence

Multiple independently valid completion entries for one schedule are not mutable competing states. They are duplicate or concurrent attestations.

The derived schedule state is:

- zero valid completion events -> not completed by v2 evidence;
- one or more valid completion events -> completed.

Consumers MUST count a schedule once, not count completion-event cardinality as task cardinality.

All valid completion actors remain provenance. A projection MAY select a canonical effective completion timestamp for display by deterministic ordering, but MUST NOT discard the other valid evidence.

A recommended ordering key is `(completed_at, action_hash)`.

## 5. Immutable swap response evidence

The target v2 response entry is conceptually:

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
    pub responded_at: Timestamp,
}
```

The immutable `CareSwap` remains the proposal. `CareSwapResponse` represents an actor-authored response to that proposal.

### 5.1 Response integrity requirements

For a `Create` of `CareSwapResponse`, integrity MUST establish:

1. action author equals `actor`;
2. referenced `CareSwap` exists;
3. referenced swap's Hearth equals `CareSwapResponse.hearth_hash`;
4. referenced swap's responder equals `CareSwapResponse.responder`;
5. `responded_at` equals the Holochain Create action timestamp;
6. actor has fresh Active membership for that Hearth;
7. actor/responder policy is satisfied;
8. response is immutable after creation.

The actor/responder policy MUST be explicit. It MUST NOT be inferred from UI affordances.

## 6. Guardian-on-behalf swap policy and conflict safety

The current coordinator permits guardian fallback for swap response. An immutable event model exposes an important consequence that mutable last-write-wins state hides: multiple valid actors can disagree.

If guardian-on-behalf swap responses remain permitted, projection MUST surface disagreement explicitly.

Recommended derived resolution:

- no valid responses -> `Proposed`;
- all valid responses are `Accepted` -> `Accepted`;
- all valid responses are `Declined` -> `Declined`;
- valid responses contain both decisions -> `Contested`.

`Contested` is a projection state, not permission to delete or rewrite evidence.

A later policy may give direct responder evidence precedence over guardian evidence, but such precedence MUST be separately specified, tested, and integrity-safe. It is not assumed by this contract.

## 7. Fresh Active membership proof

A transition that merely references any historical Active `HearthMembership` is unsafe. After a member leaves, the old Active membership action remains part of history and can otherwise be replayed as stale authority.

Freshness is therefore part of the authority theorem.

### 7.1 Why source-chain proof is available

Current Hearth membership creation and departure are member-authored. Holochain 0.6 validation permits deterministic `must_get_agent_activity` calls, including from `validate`.

A Care transition Create action also exposes its `prev_action`. This allows integrity to examine the actor's own source-chain history before the transition.

### 7.2 Membership proof algorithm

Given:

- transition Create action `A`;
- actor `P`;
- Hearth `H`;
- claimed membership action `M`;

integrity SHOULD implement the following proof:

1. Require `A.author == P`.
2. Load `M` with `must_get_valid_record`.
3. Require the membership record author is `P`.
4. Decode the referenced entry as canonical `HearthMembership`.
5. Require `membership.agent == P`.
6. Require `membership.hearth_hash == H`.
7. Require `membership.status == MembershipStatus::Active`.
8. Capture the referenced membership action's AppEntryDef so later source-chain inspection is restricted to the same entry type.
9. Obtain `A.prev_action`; a normal Create transition must have one.
10. Request deterministic actor activity using `must_get_agent_activity(P, ChainFilter::until_hash(A.prev_action, M))`.
11. Inspect activity from newest to oldest.
12. For actions with the same AppEntryDef as `M`, load the record and decode HearthMembership.
13. Ignore memberships for other Hearths.
14. For the first matching `(agent=P, hearth_hash=H)` membership revision encountered, require its action hash is exactly `M`.
15. Return the role from that fresh Active membership as the transition's current authority role.

If a Departed or Ancestral revision exists after `M` and before the Care transition, it will be encountered first and the stale Active proof fails.

If the claimed membership action is not on the actor's chain before the transition, `until_hash` proof cannot establish the required history and validation fails or remains unresolved rather than strengthening incomplete evidence.

### 7.3 Unresolved dependencies

Integrity validation MUST preserve Holochain unresolved-dependency semantics. Missing deterministic activity or referenced records are not equivalent to valid authority.

The validator MUST NOT convert inability to obtain complete source-chain proof into success.

## 8. Membership proof performance

The proof span is the actor source-chain distance from the transition back to the latest membership revision for that Hearth.

This can become large for high-volume agents. Correctness comes before optimization.

If profiling demonstrates unacceptable cost, the preferred optimization is a separately validated Kinship authority checkpoint or lease that shortens the proof span while preserving equivalent revocation semantics.

An optimization MUST NOT turn membership freshness into a coordinator-only assertion.

## 9. Transition links

Append new link variants after all historical Care link variants.

Required v2 indexes:

- `ScheduleToCompletions`: CareSchedule action -> CareCompletion action;
- `SwapToResponses`: CareSwap action -> CareSwapResponse action.

Link validation MUST bind target entry fields to the link base and author where appropriate.

Links are indexes, not authority. A missing link cannot invalidate an otherwise valid transition entry unless the protocol explicitly defines the entry and link as one atomic required construction. A forged link cannot strengthen an invalid transition.

## 10. Read projections

V2 read APIs SHOULD return explicit derived state rather than pretending the original mutable entry changed.

Suggested projection shapes:

```rust
pub struct CareScheduleProjection {
    pub schedule: CareSchedule,
    pub schedule_hash: ActionHash,
    pub completions: Vec<CareCompletionEvidence>,
    pub effective_state: CareScheduleDerivedState,
}

pub enum CareScheduleDerivedState {
    Active,
    Completed,
    LegacyPaused,
    LegacyCompleted,
}
```

and:

```rust
pub enum CareSwapDerivedState {
    Proposed,
    Accepted,
    Declined,
    Contested,
    LegacyCompleted,
}
```

Exact public DTOs may differ, but raw immutable evidence and derived state MUST remain distinguishable.

## 11. Digest semantics

Care digests currently reason from mutable schedule completion fields.

Under v2:

- one schedule contributes at most one completed task to a digest interval;
- multiple valid completion attestations do not multiply task count;
- effective completion time is chosen deterministically from valid evidence;
- provenance retains every valid actor/event even when the aggregate count is one.

Digest migration must be qualified separately before legacy completion fields stop being written.

## 12. Legacy coexistence

The migration is deliberately staged.

### Stage A — contract/foundation

Freeze this authority contract. No production behavior switch.

### Stage B — immutable completion

Append `CareCompletion` and its link. Add a versioned completion endpoint and projection/query surfaces. Keep legacy `complete_task` available while clients migrate.

### Stage C — immutable swap response

Append `CareSwapResponse` and its link. Add versioned response/projection endpoints and explicit conflict semantics.

### Stage D — client migration

Move Care UI/composer to v2 projections and transition endpoints. Browser filtering remains advisory only.

### Stage E — legacy shutdown

After evidence demonstrates no required client depends on mutable status writes:

- stop coordinator writes that mutate CareSchedule/CareSwap lifecycle state;
- make legacy lifecycle updates invalid in integrity;
- retain historical entries and projections for compatibility/audit.

## 13. Qualification matrix

### Membership freshness

- valid current Active membership -> PASS;
- membership for another agent -> reject;
- membership for another Hearth -> reject;
- historical Active membership followed by Departed revision -> reject;
- membership hash not in actor chain -> reject/unresolved;
- malformed or wrong entry type -> reject.

### Completion

- active self-assignee completion -> PASS;
- active guardian acting on behalf -> PASS if policy retained;
- non-guardian non-assignee -> reject;
- departed former assignee -> reject;
- actor/assignee mismatch against referenced schedule -> reject;
- Hearth mismatch -> reject;
- forged timestamp -> reject;
- duplicate valid completion attestations -> deterministic completed projection, task counted once.

### Swap response

- active responder decision -> PASS;
- unauthorized unrelated member -> reject;
- departed responder -> reject;
- wrong swap/Hearth binding -> reject;
- forged responder -> reject;
- conflicting valid response evidence -> `Contested`, never silent last-write-wins.

### Compatibility

- historical CareSchedule/CareSwap/MealPlan entry indexes unchanged;
- historical link indexes unchanged;
- legacy records remain readable during migration.

## 14. Explicit non-claims

This contract does not establish that #1988 or any queued CI run has passed.

It does not solve the separate Paused/Completed swap eligibility policy in #1969.

It does not make browser state authoritative.

It does not weaken Care integrity update-author checks.

It does not claim that a referenced historical Active membership is sufficient without source-chain freshness proof.

It does not silently redefine `assignee` to mean `actor`.

## 15. Design precedent

Hearth Kinship already uses the same fundamental pattern for invitation resolution:

- invitation remains immutable;
- invitee publishes an immutable `InvitationResponse`;
- response author is bound to the claimed invitee;
- response is linked back to the invitation;
- updates to the response are invalid.

Care v2 generalizes that approach for lifecycle transitions that belong to actors other than the original object author, while adding an explicit membership-freshness theorem required by Care authority.
