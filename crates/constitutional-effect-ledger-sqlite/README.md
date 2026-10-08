# constitutional-effect-ledger-sqlite

This crate is the first concrete durable implementation of the constitutional
same-action fence contract.

It is intentionally **host-side**. The Holochain zome layer must not treat DHT
links as a global lock. The deployment boundary is the shared SQLite store (or
another store with equivalent linearizable, durable, conflict-detecting
transactions).

## Transaction theorem

Every mutating operation uses:

`BEGIN IMMEDIATE -> validate persisted state -> mutate attempt/replay/fence -> COMMIT`

Admission installs all three roots in one transaction:

`native replay binding + attempt record + ActionKey fence`

Terminal resolution also commits as one unit:

`owner/token checks + exact evidence binding + attempt transition + fence close/release`

Any error before COMMIT rolls back the complete mutation set.

## Durable collision keys

The schema gives durable uniqueness to:

- `attempt_identity`
- `action_key_digest`
- `native_replay_identity`

These are deliberately different namespaces.

## Deployment boundary

All boundary instances that can reach the same effecting target must use the same
SQLite database for the property to hold. Two independent database files are two
independent collision domains. Filesystem locking and crash durability are therefore
deployment prerequisites.

This adapter does not authenticate terminal evidence or grant execution authority.
A separate authority/evidence verifier must do that before terminal resolution.

## Verifier affirmation

Terminal verification is represented as a typed `VerifiedTerminalOutcomeV1`
proof. The proof carries the exact attempt identity, operation identifier,
native replay identity, action-key digest, and verification purpose that were
evaluated. The effect boundary checks all of those bindings again before it
commits or releases the durable attempt/fence state.

The type is constructed from the evaluated attempt rather than from caller-
supplied identity fields. A verifier that returns a terminal outcome for a
different attempt, action, or purpose is therefore rejected at the boundary
and the occupied fence remains held.


## Final provider-entry claim

The boundary performs a durable read of DISPATCH_PENDING, then installs a
single-winner provider-entry claim in another BEGIN IMMEDIATE transaction before
calling the provider. Reconciliation refuses to advance a DISPATCH_PENDING
attempt while that claim exists, eliminating the read-to-provider-entry race
without holding a SQLite transaction across the remote provider call.

Provider adapters receive a ProviderEntryPermitV1 instead of a bare mutable
attempt. The permit carries a frozen attempt projection and a provider
idempotency key derived from native replay identity plus provider scope. The
operation identifier is deliberately not an input to this derivation.

A claim is consumed atomically with the transition to INVOKED. If the process
fails after claim creation, the claim remains durable and reconciliation cannot
guess that provider entry did not occur. An explicitly authorized claim
recovery moves the attempt to INDETERMINATE and removes the claim in one durable
transaction; there is no unsafe lease timeout.


## Idempotency affirmation

Terminal verification now receives the exact provider idempotency key that the
provider-entry permit exposed. The returned verification proof repeats that key,
and the boundary requires it to match before terminal evidence is constructed.

Terminal evidence uses the idempotency key in its v3 digest domain as well.
Consequently, changing the downstream idempotency identity changes the
terminal-evidence commitment rather than leaving it as an unbound adapter detail.


## Provider least privilege

The provider adapter no longer receives AttemptRecordV1 directly. It receives a
ProviderEntryPermitV1 for entry and a ProviderActionContextV1 for reconciliation.

The provider-facing context contains only the frozen material action/provider
fields needed downstream: action and target digests, operation and native replay
identity, provider-reference descriptors, provider environment/audience, adapter
identity, and the provider idempotency key.

Boundary custody material is intentionally absent from that type. In particular,
provider adapters cannot depend on or receive the owner token, durable lifecycle
state, reconciliation token, or terminal-evidence state.


## Persisted provider idempotency

The provider idempotency key is part of the durable attempt record, not just a
derived value in the host process. The core contract owns the domain-separated
derivation and validates the persisted value against that versioned algorithm.

The SQLite attempt row therefore preserves the exact downstream idempotency
identity across restart and prevents a later binary from silently changing the
key for an INDETERMINATE attempt. Changing the derivation requires a contract
version/profile change rather than an implicit behavioral change.


## Final provider-entry gate

Provider entry requires a final, host-side verifier immediately before the
durable single-winner claim is converted into a provider-entry permit. The
verifier must affirm the exact attempt, persisted provider idempotency key, and
current authorization/status snapshots through a bounded validity window.

The claim is held while this gate runs. A rejected or expired proof is atomically
converted to NotEntered and the action fence is released before any provider
call. If that release cannot be confirmed, the attempt remains held rather than
being reported as a clean refusal.


## Durable final-entry proof

The final authorization/status proof is recorded on the attempt before the
provider call. The provider-entry claim stays held while this proof is persisted,
and the provider permit requires the persisted proof digest to match the proof
that was just verified.

This gives restart/reconciliation a durable record of the exact final-entry
admission decision. A process crash after proof persistence but before provider
entry therefore cannot be reclassified as NotEntered merely because the
process disappeared.

An orphaned proof on a DISPATCH_PENDING attempt is treated as corrupted durable
state and fails closed on restart. A rejected final-entry check clears the proof,
claim, and action fence atomically as NotEntered; if that cleanup cannot be
confirmed, the attempt remains held.


## Constructor-bound trust root

Evidence verification and recovery authority are now bound to the host boundary
at construction rather than supplied by each dispatch or recovery call.

The trust root pins:
- initial admission authorizer + expected verifier identity
- provider adapter authorizer
- terminal outcome verifier + expected verifier identity
- final provider-entry verifier + expected verifier identity
- pre-entry recovery authority
- stranded provider-entry claim recovery authority

This prevents a caller that can reach the host API from selecting a weaker
verifier or recovery authority for one attempt while using a stronger verifier
for another. The provider adapter remains a separate deployment concern; this
trust root does not claim to authenticate executable code.


## Pinned provider adapter binding

Provider selection is now authorized by the constructor-bound trust root before
the durable dispatch transition is acquired. The attempt carries its declared
adapter identity, the concrete adapter must report the same identity, and the
pinned adapter-authorizer must permit that identity.

This is a deployment binding, not executable-code attestation. A deployment
that needs code-measurement guarantees must supply an adapter authorizer that
verifies its own platform-specific measurement or registry evidence.
