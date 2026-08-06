# Security Policy

## Project status

Mycelix-DeSci is experimental research infrastructure. The canonical event,
credential-registry, and authority-receipt layers are undergoing architectural
refoundation and have not completed an independent security audit. Do not treat
the current repository as a production trust anchor for safety-critical,
financial, clinical, or regulatory decisions.

## Supported versions

Security fixes are developed against the current main branch. Historical phase
snapshots and the legacy mutable claim API are not supported security boundaries.
Legacy mutation routes are disabled by default and should remain disabled outside
a bounded migration window.

## Reporting a vulnerability

Do not include secrets, private datasets, signing seeds, access tokens, or
personal information in a public issue.

Use the repository's private security-advisory mechanism when available. If no
private channel is available, open a minimal public issue requesting a private
contact channel without disclosing exploit details.

A useful report includes:

- affected commit and component;
- threat model and preconditions;
- reproducible steps or a minimal proof of concept;
- impact on integrity, confidentiality, availability, or scientific authority;
- whether credential, event, or receipt keys may be compromised;
- suggested containment or remediation, when known.

No response-time, bounty, disclosure-date, or payment commitment is made by this
repository.

## Canonical trust boundaries

Authoritative scientific writes require all of the following:

- a client-signed canonical scientific event;
- a JWT subject exactly matching the event actor;
- a signing key active for that actor in the append-only scientific credential
  registry;
- an event-specific authorization-policy decision;
- durable scientific-event storage;
- an independently signed receipt-time authority record;
- an authority receipt bound to the exact credential-registry revision used for
  acceptance.

A valid Ed25519 signature alone is not evidence that the signer was authorized,
independent, qualified, or scientifically correct.

## Key separation

Use separate keys for:

1. scientific actors;
2. credential-registry administrators;
3. the authority-receipt and credential-acceptance service;
4. the authority outbox delivery service;
5. the database-epoch promotion service;
6. the external authority-write lease issuer; and
7. independent transparency witnesses.

The implementation rejects reuse of the acceptance-service or outbox-delivery
key as an actor key, rejects receipt/delivery key overlap, and rejects lifetime
reuse of one actor key by another actor. Secret seed files
must be regular non-symbolic-link files containing 32 raw bytes or 64 hexadecimal
characters. On Unix, the API and administrative CLI require mode `0600` or
stricter.

Preserve historical public receipt keys during service-key rotation. Removing a
key still required by historical receipts or credential acceptance records makes
replay fail closed.

## Credential governance

Only bootstrap public keys are operator-configured. Actor registration, role
changes, organization memberships, key authorization, compromise notices, and
revocations are append-only signed credential events.

Every accepted credential event also records a server-observed acceptance time
signed by the independent authority service. Successor events sign the previous
record hash, so the credential-registry revision commits to both the actor event
and its acceptance time.

Non-genesis registry mutations are threshold governed. A proposal binds the
exact credential-registry head, snapshots a deterministic risk tier, requires
unique active-administrator approvals and—where configured—distinct administrator
organizations, waits through the tier-specific activation delay, and is
revalidated at execution.
An approval stops counting if its administrator loses the role or every active
key before execution. Stale and expired proposals fail closed.

Governance acceptance-service key authorization, revocation, and policy changes
use the same proposal path. Execution persists the fully signed acceptance
record before any credential side effect, so crash recovery does not depend on
retaining an old private service key. Governed external witnesses can sign locally generated transparency checkpoints;
production readiness should require multiple independently operated witness
organizations and externally retained checkpoint copies.

## Deployment requirements

- Use a high-entropy `JWT_SECRET` of at least 32 bytes.
- Configure exact `JWT_ISSUER` and `JWT_AUDIENCE` values.
- Use PostgreSQL schema v4 for scientific events, authority receipts, credential authority, threshold governance, database epochs, recovery reconciliation, mirror observations, the signed publication outbox, and write-fencing state. `memory` is simulation only; do not mix file and PostgreSQL authority backends in one deployment.
- Keep automatic schema migration disabled in production. Schema mutation requires a current trusted `schema_migration` lease bound to the connected database identity.
- Require short-lived externally signed write leases for every PostgreSQL authority mutation. The lease issuer must be independent of the database host and its key must be separated from all event, receipt, epoch, delivery, and witness identities.
- Keep `DESCI_ENABLE_LEGACY_MUTATIONS=false`.
- Treat HTTP 503 readiness as a deployment failure, not a warning to ignore.
- Back up PostgreSQL, the bootstrap trust file, and historical receipt/delivery public-key trust material as one recovery set.
- Never restore one of these stores independently without replay and consistency
  verification.
- PostgreSQL serializes scientific, credential, governance, receipt, and outbox commits across API replicas. File adapters remain a single-process reference path only.


## Database promotion and disaster recovery

A PostgreSQL primary is part of the authority boundary. Initial activation,
planned failover, and disaster recovery require a critical threshold-governance
proposal over the exact promotion intent. The signed epoch certificate commits
the database system identifier, timeline, LSN, checkpoint anchor, predecessor,
and exact credential, governance, scientific-event, and receipt heads measured
under an exclusive SQL barrier.

Disaster recovery additionally requires a bounded multi-person ceremony, a
recovery target no later than incident declaration, and exact signed recovery
reconciliation before readiness passes. Delivery acknowledgements must point to
immutable hash-bound HTTPS objects and are counted by independent governed
organization. Every cooperating authority mutation additionally requires a
short-lived signed lease bound to the deployment, primary, database system
identifier, timeline, monotonic generation, exact governed epoch, and operation
scope. Lease expiration is strict and is never extended by clock-skew grace.

This application-level fence is necessary but not sufficient: the independent
issuer must stop renewal to a former primary, and network/storage orchestration
must isolate that primary before a successor lease is issued.

## Transparency mirrors and compromise response

Mirror observations are signed assertions by governed transparency witnesses. They
do not prove that a URI remains reachable or that its bytes match the checkpoint;
independent verifiers must retrieve the document and recompute the canonical hash.

A critical governance action may record a witness-compromise interval. Checkpoint
and mirror quorum is recomputed by signing time, preserving evidence outside the
affected interval while removing compromised signatures from readiness counts.
Never delete the original signatures or observations during incident response.

The publication outbox provides at-least-once delivery. Every receiver must
verify the domain-separated delivery signature, pin an authenticated delivery
public key, deduplicate by delivery identifier, and authenticate the transport
endpoint independently.

## Cryptographic scope

The refoundation layer uses:

- Ed25519 signatures for scientific events, credential events, credential
  acceptance records, authority receipts, and outbox delivery envelopes;
- BLAKE3 domain-separated hashes for event, record, projection, and receipt
  commitments;
- explicit versioned canonical binary encodings for signed protocol objects.

The repository also contains experimental proof, privacy, trust, and federated
learning modules. Their presence does not imply that a production zero-knowledge
proof system, privacy guarantee, or Byzantine-security threshold has been
validated. Unfinished proof paths fail closed rather than emitting fabricated
success.

## Required validation before release

At minimum:

1. `cargo fmt --all -- --check`
2. `cargo clippy --workspace --all-targets --all-features -- -D warnings`
3. `cargo test --workspace --all-features`
4. corruption and crash-recovery tests for all durable stores
5. adversarial actor/key/role/revocation tests
6. cross-language golden vectors for every signed codec
7. deterministic replay from empty projections
8. dependency and secret scanning
9. deterministic lease-expiry, stale-primary, generation-fork, failover, and PITR tests
10. independent cryptographic and application-security review

See `docs/SCIENTIFIC_CREDENTIAL_REGISTRY.md`, `docs/AUTHORITY_RECEIPTS.md`,
`docs/RISK_TIERED_CREDENTIAL_GOVERNANCE.md`,
`docs/EXTERNAL_TRANSPARENCY_WITNESSES.md`,
`docs/CHECKPOINT_MIRRORS_AND_COMPROMISES.md`,
`docs/POSTGRES_AUTHORITY_BACKEND.md`,
`docs/TRANSACTIONAL_SQL_CREDENTIAL_GOVERNANCE.md`,
`docs/SIGNED_AUTHORITY_DELIVERY.md`, `docs/AUTHORITY_DATABASE_EPOCHS.md`,
`docs/POSTGRES_EPOCH_RECOVERY_RUNBOOK.md`, `docs/HARDWARE_BACKED_AUTHORITY_SIGNING.md`,
`docs/AUTHORITY_WRITE_FENCING.md`, `docs/POSTGRES_FAILOVER_PITR_VALIDATION.md`,
and `docs/POSTGRES_RECOVERY_RUNBOOK.md` for the current authority model.

---

Last updated: 2026-08-05
