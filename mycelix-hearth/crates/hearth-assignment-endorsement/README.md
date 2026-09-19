# hearth-assignment-endorsement

Pure Rust contract for the final human-authorization step before an accepted household work proposal may become an assignment transition.

Ordinary proposal responses remain negotiation evidence. A transition is executable only after every required responder explicitly endorses the exact transition intent derived from the proposal's state-bound assignment.

## Why a second step?

A transition must not become valid by cherry-picking a convenient subset of proposal responses. Final endorsements are positive, transition-specific commitments made by the exact affected members. A competing transition requires a separate set of endorsements and the assignment-transition chain still fails closed on forks.

## Binding

An endorsement is bound to:

- proposal ID
- assignment proposal ID
- deterministic transition ID
- endorser identity
- endorsement timestamp

The transition must match the proposal's schedule, exact assignment-state reference, current assignee, and proposed assignee.

## Scope

This contract does not perform Holochain calls. A DHT adapter must bind `endorser_id` to the source-chain action author and bind timestamps to action timestamps. It must include one valid endorsement from every required responder before creating transition evidence.
