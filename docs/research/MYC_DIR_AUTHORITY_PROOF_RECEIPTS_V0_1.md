# Mycelix Authority Proof Receipts v0.1

**Profile:** MYCELIX-AUTHORITY-PROOF-RECEIPTS-0.1  
**Status:** standalone Rust protocol source and adversarial unit tests authored; exact-head formatting, tests and Clippy pending. Not yet integrated into PostgreSQL migration 004 or the ACK verifier.  
**Related:** [#6562 — proof-carrying high-water and physical fencing](https://github.com/Luminous-Dynamics/mycelix/issues/6562), [#6552 — Recovery-Closed ACK](https://github.com/Luminous-Dynamics/mycelix/issues/6552).

## 1. Why a second proof protocol

A signed AuthorityWriteLease says that a configured authority permits a scoped mutation for a bounded period and exact database target. It does not prove the historical facts needed for safe recovery:
- that the external lease high-water advanced from the exact previously accepted state;
- that an independent fencing actuator disabled the old primary;
- that a nominated successor contains the exact acknowledged transaction on the correct timeline.

The protocol separates these statements into three signed proof purposes. The payloads use distinct domain tags and a single issuer role is authorized for each purpose.

## 2. Implemented source

Added `mycelix-workspace/crates/mycelix-authority-proof`, with a standalone Cargo workspace and no SQLx, HTTP client, application framework or async runtime.

Protocol surface:
- `AuthorityProofPurpose::{LeaseHighWater, PhysicalFencing, PromotionEligibility}`;
- `AuthorityProofIssuerRole::{LeaseHighWaterAuthority, PhysicalFencingAttestor, PromotionController}`;
- `SignedAuthorityProofReceipt` with protocol version, key ID, Ed25519 public key and signature;
- canonical, purpose-domain-separated encoding and BLAKE3 digests;
- trust policy keyed by issuer key ID and public key;
- strict sequence advancement over an externally supplied prior sequence;
- bounded lifetime, clock-skew, exact-target and issuer-role checks.

The proof verifier returns a hash of the complete signed envelope, including issuer key identity and signature. Promotion receipts therefore reference the exact signed high-water and fencing receipts, not merely their unsigned payloads.

## 3. Three proof types

### 3.1 Lease high-water proof

It commits to:
- prior lease generation and prior lease hash;
- new lease generation, lease ID and lease hash;
- authority policy epoch;
- deployment, database system identifier, primary, timeline and owner epoch;
- issuer role, key ID, sequence and validity interval.

Verification requires the caller to supply the independently retained predecessor generation/hash and exact pending successor lease tuple. An old database backup cannot lower this predecessor state because the authoritative high-water must live outside the PostgreSQL rollback domain.

Generation zero is supported as a genesis predecessor only when paired with the all-zero prior hash; nonzero predecessors must carry a nonzero hash.

### 3.2 Physical-fencing proof

It commits to:
- exact old primary, system identifier, timeline and owner epoch;
- monotonic fence sequence;
- actuator mechanism, actuator ID and operation ID;
- exact successor primary;
- observation time and a digest of the underlying actuator evidence.

The verifier requires the outer signed target and inner old-primary identity to match. It requires the exact intended successor and the configured database-writer key, and rejects a fencing proof signed by that same database-writer key.

`NetworkPartitionOnly` is explicitly represented as an invalid mechanism. Network partition detection is not physical fencing. A valid production receipt still requires an authorized independent actuator/attestor and evidence from the real control mechanism (power, hypervisor, cloud API, storage lease, quorum lease or hardware watchdog). The signed digest alone cannot establish physical truth.

### 3.3 Promotion-eligibility proof

It commits to:
- candidate primary, database system identifier and timeline;
- exact target transaction end-LSN;
- candidate durable flush LSN and replay LSN;
- timeline-ancestry commitment;
- promotion-policy epoch;
- hashes of the exact signed high-water and physical-fencing proof receipts.

The verifier requires all three expected commitments from the independent caller. A candidate with a numerically sufficient LSN but different timeline ancestry must be rejected. Flush and replay coverage must both reach the exact transaction end-LSN before a candidate is eligible to accept writes.

## 4. Critical external trust contract

The proof crate verifies cryptographic structure, configured issuer role/key, sequence advancement, exact target and time constraints. The caller still must:
1. source trusted key IDs and public keys from an independently controlled trust configuration;
2. atomically compare-and-advance the sequence in an external monotonic store;
3. revoke and rotate key IDs without accepting stale keys;
4. obtain actuator evidence from a physically independent control plane;
5. verify timeline ancestry and transaction end-LSN from PostgreSQL recovery evidence;
6. refuse finality when proof-store or trust-root access is unavailable.

The minimum/last accepted sequence passed into the verifier is not itself durable state. It must be read and atomically advanced in a store that cannot be rolled back by restoring the database being qualified.

## 5. Adversarial tests authored

The source includes tests for:
- purpose and issuer-role separation;
- trusted issuer key ID/public-key binding;
- exact physical-fencing old-primary and successor binding;
- rejection when the fencing attestor key is the database writer key;
- rejection of network-partition-only fencing;
- exact high-water predecessor and successor lease binding;
- stale/replayed proof sequence rejection;
- exact promotion transaction end-LSN and proof-chain binding;
- wrong purpose/role rejection.

These tests are authored but are not yet CI-qualified at the current branch head.

## 6. Next integration increments

Migration 005 should add append-only tables for the full proof JSON, canonical digest, signed-envelope digest, issuer key ID, verified-at identity and target scope. Link proof IDs to:
- deployment lease high-water and lease receipt;
- owner epoch transition;
- effect execution lease attestation;
- Recovery Finality Certificate.

The SQL installer must require full proof envelopes, not only nonzero digests. SQL may enforce shape and row consistency, but the trusted Rust controller must verify the issuer signature and independent trust roots before installation.

The PostgreSQL receipt verifier must join and independently reverify all three proof classes before creating an ACK capability. Missing proof, stale external sequence, unavailable trust roots, wrong timeline, candidate lag or inconsistent hashes must yield `UNKNOWN` and no effect-success ACK.

## 7. Qualification and performance

The benchmark profile is `MYC_DIR_RECOVERY_CLOSED_ACK_V0_1.json` version 0.1.3, with 55 scenarios and 44 metrics. It now contains explicit proof-chain safety gates and aspirational performance targets:
- zero invalid proof acceptance;
- zero fencing proof accepted from the database writer key;
- zero stale/replayed high-water acceptance;
- zero promotion from a wrong timeline or below the effect end-LSN;
- zero ACK when the independent proof store is unavailable or rolled back;
- proof-verification p95 target of 3 ms;
- proof-bundle p95 target of 8 KiB per effect;
- valid-proof false-UNKNOWN target of at most 0.1%.

Those performance values are proposed goals, not measurements or claims of beating current products. Report safety first, then proof-validation latency, bytes, recovery time and valid-finality liveness under matched hardware/failure schedules.

## 8. Industry comparison boundaries

- PostgreSQL synchronous replication provides well-defined commit durability modes; its semantics depend on the per-transaction `synchronous_commit` setting and the active synchronous standby configuration.
- Patroni synchronous mode constrains promotion to eligible nodes and can trade write availability for durability. Strict mode prevents disabling synchronous replication when no eligible synchronous standby is available; it does not remove every possible failure mode.
- Kafka transactions coordinate offsets and output records inside Kafka's own transaction domain. External destination systems require their own cooperation.
- RabbitMQ quorum publisher confirms prove queue-level quorum acceptance, not a separately committed PostgreSQL business effect.
- Debezium's transactional outbox captures outbox rows committed with application state, but a downstream destination still needs idempotency/transactional handling.

The proposed Mycelix contribution is not “a better broker” or “zero-loss PostgreSQL” by assertion. It is a proof-carrying cross-system effect contract that joins the database effect, recovery position, promotion policy, signed authority, independent fencing and broker acknowledgement. Its advantage must be demonstrated in matched fault campaigns with hard zero safety limits and measured liveness/overhead.
