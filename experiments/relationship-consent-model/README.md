# Relationship Consent State Model

**Status:** exploratory, bounded research model; not production protocol code.

This isolated, dependency-free Rust crate makes pair-specific consent transitions explicit so they can be tested independently from recommendation scores. It is intentionally separate from Pulse runtime crates and stores no identities, profile fields, messages, or network state.

## Why this is a separate research lane

A fairer shortlist is not enough if a protocol leaks interest before consent, accepts a stale message, races an acceptance against expiration, or leaves a conversation enabled after blocking or withdrawal. The matching experiment (#5128) studies allocation; this crate studies a different question: whether the protocol state machine preserves its stated invariants under event orderings.

## Model

- A proposal is scoped to one pair and one monotonically increasing epoch. The proposer gives pair-specific consent to the proposed introduction, but the request must be neutral: no reciprocal-interest detail is disclosed by the proposal itself.
- The other party's distinct acceptance is required before the model records a connection, increments its reveal counter, or enables a shared conversation.
- Discovery opt-in is a prerequisite for starting an introduction, not acceptance of one. Turning off discovery closes a still-pending introduction but does not silently terminate an already connected conversation.
- Both parties can decline or withdraw; either party can block. Withdrawal and block close current conversation permission. Prior disclosure cannot be made unknown again; a product must not promise otherwise.
- Pending proposals expire at their deadline. Expiration is processed before any other event at that time, so acceptance exactly at the deadline is rejected.
- Decline, withdrawal, expiration, and blocking activate a symbolic pair-level no-recontact cooldown (RECONTACT_COOLDOWN_TICKS). A recent block keeps this cooldown even if the block is later lifted. This is a model placeholder, not a proposed wall-clock policy or sufficient anti-harassment control.
- Every actor-originated command carries a monotonically increasing per-actor sequence. Duplicate or older commands are ignored even if policy conditions change later; a fresh sequence is consumed even when its command is policy-rejected. The model still assumes authenticated actor identity and durable sequence storage, which must be implemented and tested separately.
- A new proposal gets a fresh epoch, preventing a delayed acceptance from a prior attempt from connecting it.

## Run

From this directory with Rust 2021:

- cargo test — deterministic unit and invariant tests, including deliberate invalid-state negative controls.
- cargo run -- [max-depth] [max-states] — bounded explicit-state exploration; defaults are depth 6 and 100,000 distinct states.

The CLI emits a single JSON report. A state cap or invariant violation is a non-zero exit; truncated exploration must not be represented as a pass. No claim is made here that the Rust source has compiled or that CI has passed until the exact commit has corresponding evidence.

## Initial acceptance criteria

These are proposed engineering gates for this model, not evidence about live-user outcomes:

1. All reachable states in the declared bounded exploration satisfy invariants, and the run completes without hitting its state cap.
2. A proposal, single acceptance, duplicate acceptance, or stale acceptance cannot reveal reciprocal interest or enable chat.
3. The only transition that first reveals reciprocal interest is the second distinct acceptance in the same active epoch.
4. Expiry, decline, withdrawal, discovery opt-out during a pending proposal, and either party's block produce the specified terminal state.
5. No stale epoch can mutate the current proposal; blocking prevents starting new proposals until unblocked.
6. Negative controls that forcibly enable chat while pending or connect with only one acceptance are rejected by the invariant checker.
7. Output and exploration order are deterministic for a fixed source version and command-line configuration.

## Important limitations

This is a small sequential state model, not a distributed implementation or a proof about Holochain, Pulse, concurrent transactions, message delivery, identity, authentication, or production storage. It does not model independent replicas, clock skew, request authentication, message signatures, rate limiting across accounts, appeals, abuse-report workflows, retention, or metadata side channels. Those require additional threat modeling and implementation-specific tests.

Before a live pilot, translate the invariants into tests against the actual command/authority boundary and persistence semantics; add race/replay tests for duplicate delivery and reordering; and independently review privacy, retention, blocking, account replacement, and safety operations. A bounded state-space result is useful evidence about this model only—not a security certification or evidence of relationship quality.
