# Creator Deal State Core (experimental)

A standard-library-only Rust prototype for evidence-backed commercial-deal lifecycle transitions. It is deliberately isolated from the root Holochain workspace and from payment execution.

## What it models

- Immutable, increasing versions of proposed terms.
- Explicit acceptance by both distinct local party IDs before work starts.
- Creator-authorized work start and deliverable submission.
- Sponsor-authorized deliverable acceptance or rejection, bound to the specific submitted deliverable ID.
- Multiple sequential deliverables, with an explicit final-deliverable flag.
- Dispute/cancellation terminal states.
- Event-ID idempotency and fail-closed conflict detection.
- Replay from event history to reconstruct and validate the current state without trusting a persisted status field.
- Required evidence references, chronological event timestamps, and atomic transitions.

## Crucial semantic boundary

**Accepted means the most recent final deliverable was accepted. It does not mean the creator was paid.** This crate has no invoice, receivable, bank transaction, legal-entitlement, or payment-custody model. Those require a separate reconciliation layer.

The caller supplies local party identifiers and evidence references. This crate does not authenticate identities, validate signatures, hash documents, or prove that referenced evidence is truthful. It validates state-transition invariants only.

## Run

From the repository root:

```sh
cargo test --manifest-path mycelix-workspace/research/creator-deal-state-core/Cargo.toml
```

The source currently contains 14 unit-test cases covering role authority, term changes, multi-deliverable sequencing, atomic rejection, event replay, and acceptance-vs-payment semantics. They have not yet been executed in this environment.

## Current limits

- One current terms version per deal; terms can be revised only before work starts, and a revision clears both parties' approvals.
- One deliverable may be pending at a time; deliverables are handled sequentially.
- A disputed deal is terminal in v1; an external resolution process should create a new, explicitly linked record rather than silently rewriting state.
- A cancellation records workflow status only and does not calculate cancellation compensation or payment liability.
- Evidence refs are opaque strings, not verified URLs or cryptographic attestations.
- Party IDs are application-supplied; authentication and authorization of those identities must happen outside this crate.
- The crate is research-only until compiled, formatted, reviewed, and tested in a Rust-enabled environment.
