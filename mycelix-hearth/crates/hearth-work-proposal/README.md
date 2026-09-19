# hearth-work-proposal

Pure Rust contract for turning Hearth's Care + Rhythms planner output into explicit, consent-bearing household work proposals.

## Boundary

The planner may recommend. This crate determines what must be accepted before a recommendation may become an authorized household change. It performs no Holochain calls and no Care mutation.

Each planner assignment is rebound to an authoritative `AssignmentStateSnapshot` captured when the proposal is created. The snapshot carries the source schedule reference, an exact assignment-state reference, and the current assignee. This prevents a stale planning result from silently becoming consent for a newer schedule state.

## Direct-consent theorem

For v1:

- the proposed assignee is always a required responder;
- if an assignment changes from one member to another, the current assignee is also required;
- silence is never acceptance;
- guardian status alone does not substitute for a required member response;
- each assignment resolves independently;
- a proposal can therefore be partly accepted without forcing unrelated assignments through.

Future delegated consent should be represented as explicit Autonomy evidence rather than weakening this default.

## Append-only responses

Responses are modeled as immutable records. Repeated identical record IDs are deduplicated. Multiple records by one responder with the same decision are agreeing duplicates; contradictory decisions are a conflict and can never produce `Accepted`.

Response decisions are:

- `Accept`
- `Decline`
- `RequestChanges`

`RequestChanges` requires a note. Responses outside the proposal validity window are invalid for this contract.

## Resolution

Assignment resolution is deterministic:

1. conflicting responses -> `Conflict`
2. any decline -> `Declined`
3. any change request -> `NeedsChanges`
4. every required responder accepted -> `Accepted`
5. missing responses before expiry -> `Pending`
6. missing responses at/after expiry -> `Expired`

No status mutates the underlying Care schedule. A later DHT/execution tranche must re-check the bound assignment state before applying an accepted proposal.
