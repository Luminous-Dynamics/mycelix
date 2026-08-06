# Refoundation Tranche 2 Implementation

**Status:** implemented in the core library; canonical API integration remains open.

## Purpose

Tranche 1 established signed append-only events but intentionally left four
critical authority questions unresolved:

1. Does the signing key belong to the actor named by the event?
2. Is that actor authorized to perform the scientific action?
3. Do reviews and replications represent independent sources rather than event
   volume?
4. Can the event history survive restart and rebuild deterministically?

Tranche 2 addresses those questions before expanding the product surface.

## Included

### Canonical event protocol

- Explicit protocol, protocol version, schema version, and codec identifiers.
- Domain-separated deterministic binary encoding independent of Rust enum layout.
- Ed25519 signatures over canonical bytes and BLAKE3 hashes over signed events.
- Actor-scoped idempotency keys, global event-ID lookup, event-hash lookup, and
  server-observed append receipts.
- Five-minute maximum future clock skew while preserving signer-claimed event
  time separately from receipt time.

### Identity and authorization

- Actor-key bindings with validity intervals and revocation time.
- Organization membership and acting-organization validation.
- Resolved scientific roles: contributor, reviewer, editor, institution, and
  service.
- A fail-closed governed event-log wrapper.
- Conservative action policy for claim proposal, evidence attachment,
  attestation, correction, withdrawal, supersession, and retraction.
- A valid signature from an unbound, expired, or revoked key is rejected.
- An arbitrary actor cannot retract or supersede another actor's claim.

### Independence-safe attestations

- Reproduction and replication attestations require evidence and a versioned
  protocol reference.
- One active attestation per actor, kind, and protocol; a new UUID cannot create
  another vote.
- Reviews count unique external actors.
- Independent replications exclude the creator and creator organization.
- Multiple actors from one acting organization count as one independent source.
- Attestations can be corrected or withdrawn without deleting history.
- Corrections preserve both claim and attestation identity.

### Deterministic assessment

- Evidence profiles expose factual dimensions rather than a confidence scalar.
- Assessment output includes a policy ID, policy version, maturity label,
  contestation flag, and deterministic reasons.
- Contestation can coexist with reproduction or replication maturity instead of
  erasing it.

### Durable reference storage

- Expanded event-log contract with head, pagination, event-ID lookup, event-hash
  lookup, append receipts, and explicit receipt time.
- Atomic single-process file adapter for deterministic local persistence.
- Startup verifies signatures, ordering, IDs, idempotency keys, hash continuity,
  and projection rebuildability.
- Persistence failure rolls back the in-memory mutation.
- Corrupt streams fail closed rather than loading a partial projection.

## Deliberate protocol reset

Tranche 1 used `bincode` over Rust data structures as the signed byte format.
That format was acceptable for an internal spike but unsafe as a public protocol
because ordinary Rust layout evolution could change signatures.

Tranche 2 introduces schema version 2 and an explicit canonical codec before the
canonical event API is publicly released. Experimental schema-version-1 events
must be reissued or imported as unassessed legacy history; they must not be
silently re-signed as if their authority had already been verified.

## Security boundary

The authoritative append sequence is:

```text
verify canonical signature
→ resolve actor at server receipt time
→ verify active key binding
→ verify acting-organization membership
→ rebuild current projection
→ authorize requested scientific action
→ verify stream and domain invariants
→ atomically persist event
→ return append receipt
```

Skipping the governed wrapper is suitable only for deterministic low-level store
tests. Production command handlers must use the governed append path.

## Known limitations

- The file adapter is a single-process reference implementation, not a
  cross-process transactional database.
- Holochain event persistence and startup projection rebuilding remain open.
- JWT subjects, DIDs, ORCID authentication, institutional credentials, and key
  status still need a production identity-resolver adapter.
- The canonical event API and CLI command surface are not yet wired.
- Legacy `DesciClaim` migration tooling remains open.
- Cross-language golden vectors for Rust, TypeScript, and Python remain open.
- Research-object and attestation aggregates still share a claim stream; a later
  tranche should split independently versioned aggregate streams.

## Required next tranche

1. Wire authenticated API commands exclusively through
   `GovernedScientificEventLog`.
2. Add a transactional database adapter and crash-injection recovery tests.
3. Import legacy claims through explicit `unassessed_legacy_state` events and
   discrepancy reports.
4. Publish cross-language canonical-codec vectors.
5. Split research-object, claim, attestation, and assessment aggregates.
6. Implement a Holochain adapter against the same event-log conformance suite.

## Validation status

The patchset includes deterministic unit tests for canonical signing, chain
continuity, idempotency, unique-source replication, correction and withdrawal,
file replay, future-time rejection, actor-key binding, key revocation, and
unauthorized retraction.

`git diff --check`, ordered patch replay, and cumulative patch dry-run are
validated by the patch builder. `cargo fmt`, `cargo test`, and `cargo clippy`
could not be executed in the patch environment because Rust/Cargo are absent and
the source archive references the unavailable sibling crate
`../crates/mycelix-zkp-core`.
