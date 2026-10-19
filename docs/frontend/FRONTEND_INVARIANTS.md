# Mycelix Frontend Invariants v1

## Purpose

These invariants define what Mycelix user interfaces are allowed to imply. They apply regardless of visual design, domain, framework component, animation, optimistic update, cache state, or network condition.

The central rule is:

> Presentation may compress truth, but it must never strengthen it.

A UI may simplify a complex valid state for ordinary users. It may not display a stronger claim than the authoritative state supports.

## Truth-strengthening invariants

The following transitions are forbidden unless the underlying authoritative state supports them:

```text
unverified    → verified
pending       → executed
local         → replicated
connected     → healthy
cached        → current
submitted     → accepted
accepted      → authorized
authorized    → settled
revoked       → actionable
unknown       → successful
```

Animations, stale caches, optimistic rendering, missing fields, fallback content, and component failures must not violate these rules.

## Typed state over boolean soup

Consequential frontend states should use explicit enums or equivalent sum types instead of unrelated booleans whose combinations can become contradictory.

Preferred:

```rust
pub enum EvidenceState {
    Verified,
    Unverified,
    Invalid,
    Unknown,
}

pub enum FreshnessState {
    Live,
    Cached,
    Stale,
    Offline,
}

pub enum ExecutionState {
    Draft,
    Proposed,
    Authorized,
    Pending,
    Executed,
    Failed,
}
```

Avoid state models that permit combinations such as `verified = true` and `invalid = true`, or `settled = true` while `pending = true`, without an explicit higher-level state explaining the combination.

## Identity

1. An identity must not render as verified without current evidence supporting the verification state.
2. A display name, avatar, or cached profile is not identity proof.
3. Missing verification data renders as unknown/unverified, never verified by fallback.
4. Revocation must remove or disable actions whose authority depended on the revoked credential or capability.
5. Domain-specific badges may add context but must not overwrite canonical identity semantics.

## Authority and capabilities

1. UI visibility is not authorization.
2. Hiding an action is a usability feature, not an authority boundary.
3. Showing an action must not imply that execution will be authorized.
4. Capability state must be revalidated at the execution boundary when the domain requires it.
5. Expired, revoked, stale, or unverifiable capabilities must never render as currently actionable.
6. Delegated authority should expose its grantor and relevant scope through progressive disclosure.

## Execution

1. A proposal passing is not equivalent to execution authorization.
2. Authorization is not equivalent to successful execution.
3. Successful local submission is not equivalent to federation or settlement.
4. A queued action must remain visually distinguishable from a completed action.
5. A failed action must not disappear into a neutral state that implies success.
6. Retrying an action must preserve idempotency semantics where the underlying domain supports them.

## Connectivity, freshness, and offline state

The frontend must preserve distinctions between:

- connected
- healthy
- live
- cached
- stale
- offline
- locally saved
- queued for replication
- replicated
- conflicted
- unavailable

Rules:

1. `Connected` must not imply `healthy`.
2. `Cached` must not imply `current`.
3. `Saved locally` must not imply `replicated`.
4. Offline operation must not imply remote consensus or remote acknowledgement.
5. Freshness degradation must move presentation toward uncertainty, never toward confidence.
6. Reconnection must not silently overwrite unresolved local/remote conflicts.

## Provenance and evidence

1. Evidence summaries must identify whether evidence is verified, incomplete, stale, invalid, or unavailable.
2. A provenance trail may omit detail for readability but must preserve causal ordering.
3. If the UI says “why this happened,” the path shown must be grounded in actual evidence, not inferred decorative relationships.
4. Missing evidence must be represented as missing or unknown.
5. Human-readable summaries must link to inspectable evidence when that evidence is available.

## Optimistic UI

Optimistic interaction is encouraged for local-first responsiveness, subject to these constraints:

1. Optimistic state must never claim a stronger remote or authority result than is known.
2. Optimistic state must be reversible without destroying the user's local intent.
3. If replication or execution fails, the user must be able to distinguish the local action from the failed remote outcome.
4. Consequential destructive actions should not be visually finalized before the relevant authority boundary has succeeded.

Recommended vocabulary:

```text
Saved
Saved locally
Queued
Submitting
Awaiting approval
Authorized
Executing
Completed
Failed
```

## Error boundaries

1. A rendering failure must not mutate authoritative domain state.
2. A component panic/error must not convert unknown state into success state.
3. A failed optional visualization must fall back to a simpler representation when possible.
4. Error recovery must not repeat non-idempotent actions without explicit domain support.
5. Diagnostic information should be available without exposing secrets or capability material.

## Accessibility invariants

1. Critical state is never represented by color alone.
2. Interactive controls have programmatic names.
3. Keyboard focus order follows the logical interaction order.
4. Destructive and authority-changing actions remain understandable to screen-reader users.
5. Dynamic consequential state changes are announced through appropriate semantic mechanisms.
6. Reduced-motion preferences must not remove information conveyed only through motion.

## Calm-state invariant

Healthy systems should be quiet.

The default UI should compress multiple healthy subsystems into a simple trustworthy status. More detailed state should appear when:

- the user asks for it,
- action is required,
- uncertainty materially affects a decision, or
- the system is degraded.

This compression must still obey the truth-strengthening rule.

## Qualification requirements

Core frontend surfaces should eventually pass the following ladder:

```text
cargo check
  ↓
unit tests
  ↓
component tests
  ↓
semantic state tests
  ↓
browser E2E
  ↓
accessibility checks
  ↓
visual regression
  ↓
responsive matrix
  ↓
offline/degraded-state tests
  ↓
authority/truth-state tests
```

At minimum, automated coverage should prove:

- stale authority cannot render as current;
- unverified identity cannot render as verified;
- queued execution cannot render as settled;
- failed replication cannot erase local-save state;
- offline mode cannot imply remote consensus;
- revoked capability cannot remain actionable;
- fallback rendering cannot strengthen truth state.

## Review rule

Any frontend change that introduces a new consequential state should answer three questions in review:

1. What is the authoritative source of this state?
2. What weaker/unknown/degraded states can occur?
3. Can any rendering path accidentally display a stronger state than the source supports?

If those questions cannot be answered, the state model is not ready to ship.
