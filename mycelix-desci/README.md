# Mycelix-DeSci

> Experimental infrastructure for signed, auditable scientific evidence

[![License: AGPL-3.0-or-later](https://img.shields.io/badge/License-AGPL--3.0--or--later-blue.svg)](LICENSE)
[![Rust](https://img.shields.io/badge/Rust-1.96-orange.svg)](rust-toolchain.toml)
[![Status: Refoundation](https://img.shields.io/badge/Status-Architectural%20Refoundation-orange.svg)]()

## Status

Mycelix-DeSci is undergoing an architectural refoundation. The repository contains a broad set of DeSci research modules, but the authoritative path is now deliberately narrower:

```text
client-signed scientific event
        ↓
JWT actor/session binding
        ↓
append-only credential-registry revision
        ↓
threshold proposal → unique approvals → activation delay
        ↓
actor ↔ key ↔ organization ↔ role resolution
        ↓
scientific authorization policy
        ↓
externally signed, short-lived database write lease
        ↓
signed receipt-time authority journal
        ↓
append-only durable event stream
        ↓
deterministic evidence projection
        ↓
versioned, explainable assessment
```

The older mutable `DesciClaim`, trust, query, and E0–E4 APIs remain only as a compatibility layer. They are not the canonical scientific record, and their mutation routes are disabled by default.

## What the canonical kernel provides

- Explicit research objects, atomic claims, evidence artifacts, and attestations.
- Ed25519 signatures over a versioned canonical binary codec.
- An append-only credential registry for actor registration, key rotation and compromise, organization membership, roles, and revocation.
- Threshold governance with unique-administrator approvals, delayed activation, stale-proposal rejection, emergency cancellation, and governed authority-service key rotation.
- Actor/key validity intervals, organization membership, and role authorization resolved at an exact registry revision.
- Per-stream hash chaining, optimistic concurrency, idempotency, event lookup, and append receipts.
- Independently signed receipt-time authority evidence with two-phase crash recovery and key-rotation trust sets.
- Durable file-backed reference replay plus a PostgreSQL multi-process authority backend.
- Serializable atomic commit of scientific events, authority receipts, credentials, governance execution, and publication outbox messages.
- Domain-separated signed delivery envelopes that remain verifiable across queue leasing, retries, and relays.
- Unique-actor and unique-organization evidence counting rather than raw event counts.
- Correctable and withdrawable attestations without deleting history.
- Separate evidence dimensions and versioned assessment reasons rather than one opaque confidence score.
- Explicit legacy import that preserves source history without treating old tiers or verification blobs as evidence.
- Signed HTTPS checkpoint-mirror observations and time-bounded witness-compromise classification.
- Threshold-governed PostgreSQL database epochs that bind primary identity, timeline, LSN, checkpoint anchor, and the exact authority-state heads observed under an exclusive SQL barrier.
- Multi-person emergency-recovery ceremonies, exact PITR reconciliation, and organization-diverse acknowledgement of immutable epoch publications.
- A pluggable authority-signer interface plus a fail-closed Unix-domain remote signer adapter for HSM, KMS, or threshold-signing agents.
- Short-lived externally signed PostgreSQL write leases bound to deployment, primary, system identifier, timeline, generation, exact epoch, and operation scopes.

## Build boundary

This archive is not a standalone workspace. The root manifest depends on the sibling crate:

```text
../crates/mycelix-zkp-core
```

Use the repository in its normal parent workspace layout, or provide that crate explicitly. A missing sibling dependency is a build error; the project does not substitute a mock proof implementation.

Required toolchain: Rust 1.96 with `rustfmt` and `clippy`, as pinned in `rust-toolchain.toml`.

## Canonical API configuration

Create a dedicated initial registry-administrator key and authority-receipt key. Secret key files must contain 32 raw bytes or 64 hexadecimal characters and use mode `0600` or stricter.

```bash
mkdir -p config data
chmod 600 config/registry-admin.seed config/authority-receipt-signing.key

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

The command refuses to overwrite either output. The generated trust file contains only the public genesis key; the private administrator key remains offline. Before readiness can pass, use a signed credential event to register a second administrator because the default `DESCI_MIN_CREDENTIAL_REGISTRY_ADMINS` is `2`.

Register the second administrator with a distinct public key:

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

The registry rejects public-key reuse across actors for the lifetime of the log, including keys that were later revoked.

Initialize threshold governance only after the second administrator exists:

```bash
cargo run --locked --release \
  --package mycelix-desci-core \
  --bin mycelix-desci -- \
  credential-governance-init \
  --registry ./data/scientific-credentials.json \
  --bootstrap-trust-file ./config/credential-bootstrap-trust.json \
  --acceptance-signing-key-file ./config/authority-receipt-signing.key \
  --governance ./data/scientific-credential-governance.json \
  --administrator did:key:initial-registry-admin \
  --administrator-signing-key-file ./config/registry-admin.seed \
  --approval-threshold 2 \
  --activation-delay-seconds 86400 \
  --proposal-ttl-seconds 604800
```

After initialization, direct credential mutation is disabled. Every non-genesis credential change must be proposed, approved by unique active administrators, survive the activation delay, and be executed against the same credential-registry head. Before production readiness can pass, govern a risk policy that assigns progressively stronger thresholds, organization diversity, and delays to routine, sensitive, and critical actions.

Canonical writes require:

- `JWT_SECRET` of at least 32 bytes;
- exact `JWT_ISSUER` and `JWT_AUDIENCE` matches;
- PostgreSQL-backed scientific events, authority receipts, credentials, threshold governance, and publication outbox;
- a current externally signed authority-write lease from a trusted issuer, bound to the connected database identity and exact governed epoch;
- a dedicated authority-delivery signing identity distinct from receipt, actor, epoch, witness, and lease-issuer keys;
- an initialized credential registry whose genesis signer is in the bootstrap trust file;
- initialized durable threshold credential governance with a governed risk policy and no pending crash-recovery record;
- at least the configured minimum number of active registry administrators;
- at least one administrator recovery key without scheduled expiry or revocation;
- a dedicated authority-receipt signing key;
- a reconciled authority journal with no pending receipts or unsafe receipt gaps;
- the configured number of independent organizations witnessing and mirroring the latest published checkpoint;
- a client-signed event whose actor equals the JWT subject;
- at least one threshold-governed database epoch for PostgreSQL deployments;
- exact recovery reconciliation after a disaster-recovery epoch; and
- the configured number of independent organizations acknowledging the latest immutable epoch publication.

Start from source:

```bash
cargo run --locked --release --package mycelix-desci-api --bin mycelix-api
```

Readiness:

```bash
curl --fail http://localhost:8080/api/v1/system/health
```

The endpoint returns HTTP 503 when PostgreSQL is unavailable, the externally signed write lease is absent, expired, untrusted, stale, or bound to another database identity or epoch, or the configured outbox backlog/age policy is exceeded; when no governed database epoch exists; when disaster recovery lacks exact reconciliation; when the latest epoch publication lacks the configured organization-diverse acknowledgements; when the registry is uninitialized, ephemeral, or below the administrator threshold; when risk-tiered governance is absent; when the latest checkpoint lacks the configured organization-diverse witnesses or currently valid mirrors; when receipt signing is absent; when receipt finalization is pending; or when authority history contains an unsafe gap.

See [PostgreSQL Authority Backend](docs/POSTGRES_AUTHORITY_BACKEND.md), [Transactional SQL Credential Governance](docs/TRANSACTIONAL_SQL_CREDENTIAL_GOVERNANCE.md), [Signed Authority Delivery](docs/SIGNED_AUTHORITY_DELIVERY.md), [Governed Database Epochs](docs/AUTHORITY_DATABASE_EPOCHS.md), [Epoch Recovery Runbook](docs/POSTGRES_EPOCH_RECOVERY_RUNBOOK.md), [Delivery Acknowledgements](docs/SIGNED_AUTHORITY_DELIVERY_ACKNOWLEDGEMENTS.md), [Hardware-Backed Signing](docs/HARDWARE_BACKED_AUTHORITY_SIGNING.md), [Authority-Write Fencing](docs/AUTHORITY_WRITE_FENCING.md), [Failover/PITR Validation](docs/POSTGRES_FAILOVER_PITR_VALIDATION.md), [PostgreSQL Recovery Runbook](docs/POSTGRES_RECOVERY_RUNBOOK.md), [Checkpoint Mirrors and Compromises](docs/CHECKPOINT_MIRRORS_AND_COMPROMISES.md), [Threshold Credential Governance](docs/THRESHOLD_CREDENTIAL_GOVERNANCE.md), [Risk-Tiered Governance](docs/RISK_TIERED_CREDENTIAL_GOVERNANCE.md), [External Transparency Witnesses](docs/EXTERNAL_TRANSPARENCY_WITNESSES.md), [Transactional File Storage](docs/TRANSACTIONAL_FILE_STORAGE.md), [Scientific Credential Registry](docs/SCIENTIFIC_CREDENTIAL_REGISTRY.md), and [Canonical Event API](docs/CANONICAL_EVENT_API.md).

## Docker

Docker builds use the sibling proof crate as a named BuildKit context. From this repository:

```bash
docker buildx build \
  --build-context zkp-core=../crates/mycelix-zkp-core \
  -t mycelix-desci-api .
```

For Compose, bootstrap and validate the credential and governance file journals offline, create distinct mode-0600 receipt and outbox signing keys, then atomically import the journals into an empty PostgreSQL authority schema:

```bash
cargo run --locked --release \
  --package mycelix-desci-core \
  --bin mycelix-desci -- \
  credential-authority-import-postgres \
  --database-url "$DESCI_POSTGRES_URL" \
  --registry ./data/scientific-credentials.json \
  --governance ./data/scientific-credential-governance.json \
  --bootstrap-trust-file ./config/credential-bootstrap-trust.json \
  --acceptance-signing-key-file ./config/authority-receipt-signing.key \
  --outbox-signing-key-file ./config/authority-outbox-signing.key \
  --deployment-id "$DESCI_AUTHORITY_DEPLOYMENT_ID" \
  --write-lease-file ./runtime/authority/current-write-lease.json \
  --write-lease-trust-file ./config/authority-write-lease-trust.json \
  --auto-migrate=false

docker compose up --build
```

Before import or startup, apply schema v4 under a valid `schema_migration` lease and atomically replace `runtime/authority/current-write-lease.json` with a current bootstrap or epoch lease. Compose mounts the containing directory read-only so issuer-side atomic replacement remains visible.

Compose uses PostgreSQL for scientific events, receipts, credential authority, threshold governance, checkpoint mirrors, and the signed publication outbox. Legacy mutations remain disabled, automatic schema migration is off by default, and authority writes remain fenced unless a current trusted lease matches the connected PostgreSQL identity.

## Canonical endpoints

| Method | Path | Purpose |
|---|---|---|
| `POST` | `/api/v1/scientific/authority/events` | Pre-governance compatibility append; returns 410 after threshold initialization |
| `GET` | `/api/v1/scientific/authority` | Return credential-registry status and revision |
| `POST` | `/api/v1/scientific/authority/governance/events` | Open, approve, cancel, or checkpoint a signed governance event |
| `POST` | `/api/v1/scientific/authority/governance/execute` | Atomically execute a mature proposal |
| `GET` | `/api/v1/scientific/authority/governance` | Return governance policy, proposals, service keys, and head |
| `GET` | `/api/v1/scientific/authority/governance/events/{id}` | Retrieve one independently accepted governance event |
| `GET` | `/api/v1/scientific/authority/governance/proposals/{id}` | Return proposal state and effective stale/expired status |
| `GET` | `/api/v1/scientific/authority/governance/checkpoint-candidate` | Build a transparency checkpoint candidate |
| `POST` | `/api/v1/scientific/authority/governance/checkpoint-mirrors` | Record a signed immutable checkpoint mirror observation |
| `GET` | `/api/v1/scientific/authority/governance/checkpoint-mirrors/{hash}` | Export mirror observations with current compromise-aware validity |
| `GET` | `/api/v1/scientific/authority/database-epochs` | Return the verified database-epoch chain summary and readiness evidence |
| `GET` | `/api/v1/scientific/authority/database-epochs/state-commitment` | Capture the current checkpoint-bound SQL authority-state commitment |
| `POST` | `/api/v1/scientific/authority/database-epochs` | Commit one threshold-authorized signed primary epoch and publication |
| `GET` | `/api/v1/scientific/authority/database-epochs/{epoch_number}` | Retrieve one epoch and its recovery reconciliations |
| `POST` | `/api/v1/scientific/authority/recovery-reconciliations` | Record exact signed PITR/disaster-recovery reconciliation |
| `POST` | `/api/v1/scientific/authority/delivery-acknowledgements` | Record an independent immutable-publication acknowledgement |
| `GET` | `/api/v1/scientific/authority/deliveries/{delivery_id}/acknowledgements` | Export acknowledgements with current compromise-aware validity |
| `GET` | `/api/v1/scientific/authority/events` | Export recorded credential events and server receipt times |
| `GET` | `/api/v1/scientific/authority/actors/{actor}` | Resolve an actor at the current registry revision |
| `POST` | `/api/v1/scientific/events` | Verify and append one client-signed canonical event |
| `GET` | `/api/v1/scientific/claims/{id}` | Return the deterministic projection and assessment |
| `GET` | `/api/v1/scientific/claims/{id}/events` | Page through the signed source stream |
| `GET` | `/api/v1/scientific/claims/{id}/authority-receipts` | Export the ordered receipt chain and unattested-event statuses |
| `GET` | `/api/v1/scientific/events/{id}` | Retrieve one signed event, hash, and authority status |
| `GET` | `/api/v1/scientific/events/{id}/authority-receipt` | Retrieve the signed receipt-time authorization evidence |

Public submission of `legacy_claim_imported` is forbidden. Migration is an offline operator action.

## Legacy migration

Legacy records are imported as visibly unassessed history. Old creator strings, tiers, verification counts, and provenance counts are retained only as historical metadata. They create no reviews, reproductions, replications, or maturity.

```bash
chmod 600 /secure/migration-ed25519.seed
chmod 600 /secure/authority-receipt-ed25519.seed

cargo run --locked --release \
  --package mycelix-desci-core \
  --bin mycelix-desci -- \
  migrate-legacy .mycelix/claims \
  --event-log ./data/scientific-events \
  --actor did:key:migration-service \
  --signing-key-file /secure/migration-ed25519.seed \
  --credential-registry ./data/scientific-credentials.json \
  --credential-bootstrap-trust-file ./config/credential-bootstrap-trust.json \
  --authority-audit ./data/scientific-authority \
  --receipt-signing-key-file /secure/authority-receipt-ed25519.seed \
  --report ./data/legacy-migration-report.json
```

The migration command is deterministic and idempotent, rejects symbolic-link key files, requires restrictive Unix key permissions, enforces distinct event-actor and receipt-service keys, supports historical receipt keys during rotation, and records non-sensitive logical source locators rather than host filesystem paths.

See [Legacy Migration](docs/LEGACY_MIGRATION.md).

## Repository map

```text
src/core/src/scientific_events.rs       canonical event protocol and projections
src/core/src/scientific_credentials.rs  governed actor/key/role registry
src/core/src/scientific_credential_governance.rs
                                        threshold proposals, approvals, execution, checkpoints
src/core/src/scientific_governance.rs   identity and authorization boundary
src/core/src/scientific_authority_audit.rs
                                        signed receipt-time authority journal
src/core/src/postgres_credential_authority.rs
                                        atomic SQL credential/governance repository
src/core/src/authority_delivery.rs       signed publication envelopes
src/core/src/authority_epoch.rs          governed database epochs and recovery evidence
src/core/src/postgres_authority_epoch.rs SQL epoch, reconciliation, and acknowledgement storage
src/core/src/authority_signing.rs        HSM/KMS-compatible signer capability
src/core/src/authority_fencing.rs        externally signed PostgreSQL write leases
src/core/src/legacy_migration.rs         conservative legacy importer
src/api/src/handlers/scientific.rs       authoritative HTTP event surface
src/api/src/state.rs                     durable backend and authority configuration
src/core/src/bin/commands/migrate_legacy.rs
                                        offline migration command
```

The remaining legacy and advanced modules are retained for migration, research, and future processor integration. Prediction markets, Bayesian inference, citation analytics, expertise, MATL, PoGQ, and semantic models should consume canonical events and publish versioned derived outputs; they should not directly mutate scientific truth.

## Current limitations

- Bootstrap trust remains a local public-key file; dynamic DID/ORCID verification and external credential-status resolution are not yet implemented.
- PostgreSQL schema v4 adds externally signed write fencing, but the repository does not ship the independent lease-issuer service or infrastructure automation that isolates and promotes database primaries. A real PostgreSQL integration, contention, lease-expiry, split-brain, backup/restore, failover, and PITR campaign is still required before production use.
- A contiguous prefix written before receipt journaling remains explicitly `legacy_unattested`; the system can acknowledge that cutover but cannot reconstruct missing receipt-time authority evidence. Any missing receipt at or after the first receipt in a stream is `unsafe_unattested` and always blocks readiness.
- Risk-tiered thresholds, external witnesses, and compromise intervals are implemented, but risk classification remains code-defined. A signed mirror observation is evidence of a witness assertion, not proof of continuing availability; external verifiers must fetch and hash the mirror document.
- Canonical client SDKs and cross-language golden vectors still need to be published.
- Legacy ownership adoption is not implemented; imported creator strings are not proof of current key control.
- Canonical endpoints are documented in Markdown but are not yet fully represented in generated OpenAPI schemas.
- The repository still depends on the external sibling `mycelix-zkp-core` crate.
- SQLx is declared but `Cargo.lock` could not be regenerated in this environment; regenerate it with the pinned toolchain before using `--locked`.
- A full Rust build, PostgreSQL integration test, formatting, and lint pass is required before merge.

## Validation order

Before claiming a release:

1. `cargo fmt --all -- --check`
2. `cargo clippy --workspace --all-targets --all-features -- -D warnings`
3. `cargo test --workspace --all-features`
4. Migration idempotency and corruption-replay tests
5. API authorization and optimistic-concurrency tests
6. Cross-language canonical codec vectors, including `scripts/verify-authority-write-lease.py`
7. Deterministic projection rebuild from an empty query store
8. Lease-expiry, stale-primary, generation-fork, failover, and PITR scenarios from `docs/POSTGRES_FAILOVER_PITR_VALIDATION.md`

## License

AGPL-3.0-or-later. See [LICENSE](LICENSE). No separate commercial license is granted by this repository distribution.

**Status:** experimental architectural refoundation

**Updated:** August 5, 2026
