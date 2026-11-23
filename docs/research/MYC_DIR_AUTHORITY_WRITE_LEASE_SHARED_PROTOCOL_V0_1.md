# Mycelix shared authority-write lease protocol v0.1

**Profile:** MYCELIX-AUTHORITY-WRITE-LEASE-SHARED-PROTOCOL-0.1  
**Status:** standalone verifier crate added; adapter dependency and target-check helper added; cross-implementation golden-vector test and mutation-transaction integration remain pending  
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

It contains no database client, Tokio runtime, HTTP client, application CLI or DeSci storage dependency. The outer Mycelix workspace excludes the crate from its broad resolver, while the NATS adapter consumes it through a path dependency.

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

The adapter exposes `verify_directory_fabric_write_lease`, which requires the Directory Fabric scope and checks the exact target identity and epoch. Its return value is the signed lease hash that a transaction-bound admission layer can subsequently record.

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

Do not claim wire compatibility has been independently qualified until that cross-implementation vector is checked on an exact head.

## 5. Qualification

The NATS adapter workflow now:
- formats the standalone lease crate;
- runs its unit tests;
- runs Clippy on all lease crate targets;
- builds the NATS adapter against the shared verifier.

This qualifies source/build behavior only after the exact-head workflow passes. It does not qualify PostgreSQL lease-state enforcement or physical fencing.
