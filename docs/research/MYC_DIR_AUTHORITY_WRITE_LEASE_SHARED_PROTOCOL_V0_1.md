# Mycelix shared authority-write lease protocol v0.1

**Profile:** MYCELIX-AUTHORITY-WRITE-LEASE-SHARED-PROTOCOL-0.1  
**Status:** standalone verifier crate added; adapter dependency and exact-target helper added; a canonical-byte golden vector is authored. Exact-head tests and a cross-implementation comparison against the existing DeSci type remain pending.  
**Related:** [#6525 — reuse external PostgreSQL write fencing](https://github.com/Luminous-Dynamics/mycelix/issues/6525), [#6552 — Recovery-Closed ACK](https://github.com/Luminous-Dynamics/mycelix/issues/6552)

## 1. Why a small shared protocol crate

The existing signed lease implementation is in the DeSci core. That package also carries application dependencies including SQLx, HTTP/configuration clients, CLI support and ZKP dependencies. Making a small Directory Fabric adapter depend on the entire application core would enlarge the build and supply-chain surface unnecessarily.

Added:

`mycelix-workspace/crates/mycelix-authority-write-lease`

The crate has a separate Cargo workspace and a deliberately narrow dependency set:
- serde;
- chrono;
- uuid;
- ed25519-dalek;
- blake3;
- serde_json for tests.

It contains no database client, Tokio runtime, HTTP client, application CLI or DeSci storage dependency. Its manifest is an isolated Cargo workspace and the outer monorepo workspace excludes it from the broad resolver. The outer Mycelix workspace excludes the crate from its broad resolver, while the NATS adapter consumes it through a path dependency.

## 2. Wire protocol preserved

The crate implements the current authority-write lease v1 envelope:

- protocol: `mycelix-desci-authority-write-lease`;
- protocol version: 1;
- schema version: 1;
- codec: `mycelix-canonical-binary-v1`;
- BLAKE3 ContentHash with transparent 32-byte serde representation;
- Ed25519 signature verification against explicitly supplied trusted public keys;
- bounded lease duration and clock-skew validation;
- exact deployment, primary, PostgreSQL system identifier, timeline, epoch number and epoch hash binding;
- explicit scope authorization.

The scope code assignments remain stable:

| Scope | Code |
|---|---:|
| ScientificEvent | 1 |
| CredentialRegistry | 2 |
| CredentialGovernance | 3 |
| DatabaseEpochPromotion | 4 |
| RecoveryReconciliation | 5 |
| DeliveryAcknowledgement | 6 |
| CheckpointMirror | 7 |
| OutboxDelivery | 8 |
| SchemaMigration | 9 |
| DirectoryFabricEffect | 10 |

The adapter exposes `verify_directory_fabric_write_lease`, which requires the Directory Fabric scope and checks the exact target identity and epoch. Its return value is the signed lease hash that a transaction-bound admission layer can subsequently record. The NATS adapter workflow now format-checks, tests and Clippy-checks this crate independently.

## 3. Critical trust boundary

This crate proves only that a supplied lease:
- has a supported protocol envelope;
- has a valid Ed25519 signature;
- chains to a caller-supplied trusted key;
- is within its signed validity period;
- grants the exact operation scope;
- binds the exact target deployment, primary, system identifier, timeline and database epoch.

It does **not**:
- install or lock the lease inside PostgreSQL;
- prove that every mutation entry point enforces the lease;
- make a client-side preflight check atomic with a later SQL commit;
- prove that a former primary is physically fenced;
- prove replica durability or recovery closure.

The next mutation-boundary increment must persist a verified lease identity/hash/generation in a database-side lease state, lock and validate that state in the same transaction as the strict effect function, and restrict application roles so they cannot bypass the lease-checked entry point. The trusted lease verifier/controller must be the only principal able to advance the database-side verified lease state. Its trust key set and control-plane identity need their own rotation and revocation tests.

## 4. Compatibility gate still required

The v1 canonical encoder and envelope were reproduced from the existing DeSci implementation, and the new crate includes deterministic signature, JSON roundtrip, scope and target-binding tests. Before switching DeSci core to re-export the shared types, add a golden cross-implementation compatibility test that serializes the existing core type and verifies that the shared crate produces identical:
- lease canonical bytes;
- signed-envelope signing bytes;
- signed lease hash;
- serde JSON field representation.

The shared crate now includes a fixed canonical-byte vector for a Directory Fabric lease (fixed UUID, deployment, database identity, timeline, epoch, generation, scope, timestamps and reason). This pins the protocol bytes independently of an implementation-generated expected value. It is not yet a comparison against the existing DeSci type. Do not claim wire compatibility has been independently qualified until that cross-implementation vector is checked on an exact head.

## 5. Qualification

The NATS adapter workflow now:
- formats the standalone lease crate;
- runs its unit tests;
- runs Clippy on all lease crate targets;
- builds the NATS adapter against the shared verifier.

This qualifies source/build behavior only after the exact-head workflow passes. It does not qualify PostgreSQL lease-state enforcement or physical fencing.

## 6. PostgreSQL mutation-boundary admission (migration 004)

Added `tools/myc-dir-fabric-reference/postgres/004_authority_write_lease_admission.sql` and its adversarial SQL test.

The migration introduces:
- a database-local mirror of deployment lease high-water (the external high-water receipt remains authoritative and must be outside the PG rollback domain);
- append-only per-effect-owner signed lease receipts containing the complete signed JSON envelope, signer key ID/public key/signature, exact target system/timeline, database epoch, resource owner epoch/generation, scopes, expiry and external high-water/fencing commitments;
- a mutable per-effect-owner current-lease pointer;
- append-only effect-to-lease attestations;
- append-only owner-epoch-transition-to-promotion-lease attestations.

The trusted controller installs a lease only after the shared Rust verifier has authenticated the Ed25519 signature, trust root, scope, expiry, deployment, primary, database system identifier, timeline, database epoch and external high-water proof. SQL checks the v1 envelope fields, key/signature encodings, server system identifier/timeline, current or next resource owner position, deployment lease-generation monotonicity, epoch monotonicity, same-epoch primary/timeline fork rejection, and expiry. SQL cannot itself recompute the BLAKE3 signed-lease hash or verify the external high-water signature; the installer is executable only by the dedicated NOLOGIN controller role, which is the explicit trust boundary.

New entry points:
- `install_effect_owner_authority_write_lease` — controller-only lease admission;
- `apply_owner_epoch_claim_fenced_balance_delta_with_authority_lease` — worker-only effect mutation, locking claim → owner fence → lease state, then invoking the prior strict function inside the same transaction and appending an effect/lease attestation;
- `advance_effect_owner_epoch_with_authority_lease` — controller-only epoch promotion, requiring the DatabaseEpochPromotion scope and exact physical-fencing receipt before invoking the prior transition function and appending a transition/lease attestation.

The worker role has no direct table INSERT privilege and no EXECUTE privilege on the legacy non-lease mutation or owner-epoch functions. It is granted only the lease-checked wrapper. The controller role is granted only lease installation and lease-checked promotion entry points. Deployment operators still need to grant these NOLOGIN roles to the correct authenticated service principals and keep the controller signing/trust configuration outside the database.

The wrapper rechecks the live PostgreSQL system identifier and timeline at mutation time. A lease installed on a previous timeline cannot authorize a protected operation after promotion to a different timeline.

**Important limitations remain:** a SECURITY DEFINER controller installation function trusts the controller's verified result; it is not a PostgreSQL-native Ed25519 verifier. The database-local high-water can roll back under PITR; the external controller must reject stale generations after restore. A lease does not physically fence the former primary. The current PostgreSQL/NATS receipt verifier still needs to join `effect_execution_authority_lease_attestations` and independently verify the stored signed envelope before creating its final ACK capability. The new migration and SQL tests require their own exact-head CI run.
