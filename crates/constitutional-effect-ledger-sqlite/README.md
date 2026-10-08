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

Evidence verification, admission authority, recovery authority, and provider
selection are fixed when the effect boundary is constructed rather than supplied
by each dispatch or recovery call.

The trust root pins:
- initial admission authorizer + expected verifier identity
- terminal outcome verifier + expected verifier identity
- final provider-entry verifier + expected verifier identity
- pre-entry recovery authority
- stranded provider-entry claim recovery authority

Provider selection is separately fixed by the constructor-pinned provider registry.
The trust root therefore controls the authorities that can authorize the boundary,
while the registry controls which concrete provider objects can execute within it.


## Constructor-pinned provider registry

Provider adapter selection is now an ownership property of the effect boundary.
The host is constructed with a PinnedProviderAdapterRegistry that owns the concrete
ProviderAdapter objects for the lifetime of the boundary.

An attempt records the adapter identity it requires. The boundary resolves that
identity only inside the pinned registry; dispatch and reconciliation expose no
caller-supplied provider parameter. An unregistered identity cannot reach provider
entry at all.

The registry pins the concrete adapter object, but the adapter identity itself is
not executable-code attestation. Deployments that require measured-code assurance
must construct the registry only from adapters whose provenance has already been
verified by their deployment trust root.


## Idempotency protocol semantics

The provider idempotency key is an internal, deterministic server-side identity,
not a client-controlled HTTP header. It is derived from the immutable native
replay identity, action-key digest, provider environment/audience, and adapter
identity, then frozen in durable attempt state.

Accordingly, the same native replay identity cannot retain the same downstream
key when the material action or provider scope changes. There is no independent
expiry policy for this internal key: the durable attempt/fence lifecycle is the
retention boundary, and any provider-facing retention policy must be explicitly
qualified by that provider adapter.


### Provider object ownership invariant

There is intentionally no `dispatch(..., &mut ProviderAdapter)` or
`reconcile(..., &mut ProviderAdapter)` API. The caller supplies only the action,
attempt identity, and ownership token. The boundary owns the provider object and
selects it from the constructor-pinned registry.

This closes the distinction between an allowed identity string and an allowed
executable object: a caller cannot inject an arbitrary object that merely
self-reports a permitted adapter identity.


## Durable authorization evidence receipt

The authorization proof is persisted in full, not only as a digest in the
attempt record. The receipt contains the exact action/attempt scope, provider
scope, authorization/policy/status snapshot commitments, verifier identity, and
its validity window.

The receipt is self-validating: on restart, SQLite recomputes its digest and
rejects any tampered preimage or receipt/attempt mismatch. The receipt table is
foreign-keyed to the attempt table, and the durable integrity audit rejects both
orphan receipts and attempts whose admission receipt is missing.

This is an evidence-preservation mechanism, not a second authorization system.
The deployment's admission authorizer remains responsible for producing the
proof in the first place.
