# Scientific Credential Registry

The scientific credential registry is the authoritative source for actor keys, scientific roles, and organization memberships. Mutable startup actor profiles are no longer used for canonical authorization.

## Trust model

Operators configure only a small bootstrap trust set: public keys allowed to sign the registry genesis event. After genesis, every mutation must:

1. use the next sequence and previous recorded credential hash;
2. carry a valid Ed25519 signature over the canonical credential codec;
3. be signed by an actor who was a `registry_admin` with an active key at the server-observed acceptance time; and
4. leave at least one active registry administrator key with no scheduled expiration or revocation.

The private bootstrap key should remain offline after genesis. Removing its public key from the bootstrap trust file makes an existing registry fail startup verification; the trust file therefore belongs in protected, version-controlled deployment configuration.

## Bootstrap

Create a mode-0600 32-byte Ed25519 seed, then initialize both the registry and its public trust file:

```bash
cargo run --locked --release \
  --package mycelix-desci-core \
  --bin mycelix-desci -- \
  credential-registry-init \
  --registry ./data/scientific-credentials.json \
  --actor did:key:initial-registry-admin \
  --signing-key-file ./config/registry-admin.seed \
  --acceptance-signing-key-file ./config/authority-receipt-signing.key \
  --bootstrap-trust-output ./config/credential-bootstrap-trust.json
```

The command refuses to overwrite either output. The independent acceptance-service key signs the server-observed genesis acceptance time; it may be the same domain-separated service key used for scientific authority receipts, but it must never be an actor key.


Register a second administrator before initializing threshold governance. This direct command is a bootstrap-only operation and becomes permanently unavailable after threshold cutover:

```bash
cargo run --locked --release \
  --package mycelix-desci-core \
  --bin mycelix-desci -- \
  credential-actor-register \
  --registry ./data/scientific-credentials.json \
  --bootstrap-trust-file ./config/credential-bootstrap-trust.json \
  --administrator did:key:initial-registry-admin \
  --administrator-signing-key-file ./config/registry-admin.seed \
  --acceptance-signing-key-file ./config/authority-receipt-signing.key \
  --actor did:key:second-registry-admin \
  --public-key <64-hex-character-public-key> \
  --role registry_admin
```

## Credential event types

- `registry_initialized`
- `actor_registered`
- `actor_key_authorized`
- `actor_key_revoked`
- `actor_key_compromised`
- `role_granted`
- `role_revoked`
- `organization_membership_granted`
- `organization_membership_revoked`

Registry events are append-only. Corrections are expressed as later events; earlier signed statements remain auditable. A signing key is lifetime-unique to one actor and cannot be rebound to another actor after revocation.

## Roles

The current roles are:

- `contributor`
- `reviewer`
- `editor`
- `institution`
- `service`
- `migration_service`
- `registry_admin`

Only `registry_admin` actors may approve credential-registry mutations. After threshold-governance initialization, no single administrator can append a non-genesis transition directly. Scientific-event authorization remains event-specific: possession of a valid registry key alone does not authorize every scientific action.

## Revision binding

Every resolved actor profile carries the credential registry head hash. Authority-receipt protocol v2 signs this exact revision into both the receipt and its authority snapshot hash. Later role changes or key revocations therefore do not rewrite why an earlier scientific event was accepted.

The credential registry assigns `received_at` only after acquiring its write lock. This is its linearization point and gives concurrent credential revocation and scientific-event authorization a deterministic order. The independent acceptance service signs the event hash and `received_at`; the resulting record hash becomes the next event's signed predecessor and the authority-receipt registry revision.

## HTTP surface

| Method | Path | Purpose |
|---|---|---|
| `POST` | `/api/v1/scientific/authority/events` | Pre-governance compatibility append; returns HTTP 410 after threshold initialization |
| `POST` | `/api/v1/scientific/authority/governance/events` | Open, approve, cancel, or checkpoint a signed proposal |
| `POST` | `/api/v1/scientific/authority/governance/execute` | Execute a mature proposal atomically |
| `GET` | `/api/v1/scientific/authority` | Registry status, revision, and minimum administrator threshold |
| `GET` | `/api/v1/scientific/authority/events` | Paged recorded events including server acceptance times |
| `GET` | `/api/v1/scientific/authority/events/{id}` | One recorded event and canonical event hash |
| `GET` | `/api/v1/scientific/authority/actors/{actor}` | Current resolved profile and registry revision |

The API accepts complete signed protocol objects. It never accepts an unsigned “grant this role” convenience request. A credential event is embedded in a governance action, approved, delayed, and executed unchanged.

## Readiness

Readiness requires:

- an initialized durable registry;
- the genesis key present in bootstrap trust;
- initialized durable threshold governance;
- an active governed acceptance-service key and no pending execution recovery;
- at least `DESCI_MIN_CREDENTIAL_REGISTRY_ADMINS` active administrators, default `2`; and
- the other canonical event and authority-receipt checks documented by the API.

The registry itself also prevents removal of the final continuing administrator recovery key. This is stronger than the readiness count: it prevents a future expiration schedule from making all governance mutations impossible.

## Key rotation and compromise

Authorize a replacement key before revoking an old key. A compromise event may use the time at which compromise became effective, while the registry preserves the later server acceptance time. Replay checks that the event signer still held registry authority at that acceptance time.

Credential acceptance and scientific authority receipts use separate domain-separated signatures and may share an authority-service key only when an operator deliberately accepts that coupling. The signed publication outbox always uses its own dedicated key. No service key may be registered as a scientific actor key, and the outbox key must not overlap any trusted receipt key. During rotation, retain historical public keys in the corresponding trust set so old acceptances, receipts, and deliveries remain verifiable.

## Threshold governance

See [Threshold Credential Governance](THRESHOLD_CREDENTIAL_GOVERNANCE.md). The current protocol provides unique-actor approvals, delayed activation, stale-proposal rejection, cancellation, governed service-key rotation, crash recovery, and signed transparency checkpoints.

## Remaining work

Risk-tiered thresholds, organization-diverse approvals, governed external
witnesses, compromise intervals, hash-addressed mirror observations, and the
transactional PostgreSQL credential/governance repository are implemented.
Remaining work includes formal emergency-recovery ceremonies, external DID/ORCID
proof verification, hardware-backed service keys, and live failover/PITR exercises.
Those additions should extend these event streams rather than reintroduce mutable
operator profiles.
