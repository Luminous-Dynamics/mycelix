# Mycelix Recovery Finality Certificate v0.1

**Profile:** MYC-DIR-RECOVERY-FINALITY-CERTIFICATE-0.1  
**Status:** design specification; not implemented or qualified  
**Related:** [#6552 Recovery-Closed ACK](https://github.com/Luminous-Dynamics/mycelix/issues/6552), [#6525 signed authority leases](https://github.com/Luminous-Dynamics/mycelix/issues/6525), [PR #6527 owner epoch and execution attestation](https://github.com/Luminous-Dynamics/mycelix/pull/6527)

## 1. The paradigm shift: certify a recoverable history, not a successful write

A current-primary read proves visibility on the queried server. A broker double-ack proves that the broker recorded the acknowledgement. Neither proves that the committed database effect is present in every recovery path the deployment is allowed to select.

The proposed primitive is a **Recovery Finality Certificate (RFC)**. It binds the actual effect to a WAL commit position and to a versioned, externally enforced recovery/promotion policy.

The target invariant is:

`ACK(E) => EffectCommitted(E) AND RecoveryClosed(E, PolicyEpoch)`

`RecoveryClosed` is not a PostgreSQL boolean. It is a proof obligation over the set of recovery histories that the external promotion authority currently permits. The policy must either:
- admit only candidates that contain the effect (or have durably retained the WAL required to replay it before accepting writes); or
- reject/quarantine that candidate and require reconciliation.

A policy epoch is part of the certificate. A later policy change must not silently broaden the permitted promotion set or weaken the required recovery point.

## 2. Proposed certificate envelope

A v0.1 certificate should include the following typed sections.

### 2.1 Effect identity

- profile and canonical encoding version;
- effect ID and canonical effect-receipt digest;
- execution-attestation ID and digest;
- realm/resource/action/effect-owner scope;
- logical operation and idempotency digest;
- owner epoch and accepted resource generation;
- exact NATS stream, consumer, stream sequence and delivery identity that will be acknowledged.

### 2.2 PostgreSQL origin identity

- cluster system identifier;
- deployment and primary identity;
- database identity where relevant;
- origin timeline ID;
- WAL commit LSN and transaction end LSN;
- transaction ID as supporting correlation only, never as a substitute for LSN/timeline identity;
- digest of the transactional effect marker and the decoded marker payload.

The transaction marker must be written in the same database transaction as the protected effect and execution attestation. The logical decoder must only produce a committed certificate from the transaction's commit callback. An aborted transaction must not produce an effect-finality certificate.

PostgreSQL 16 exposes **pg_logical_emit_message(true, prefix, content)** for transactional logical-decoding messages. The logical replication protocol's commit message exposes both commit LSN and transaction end LSN. This makes a marker-plus-commit-observer prototype viable, but the marker does not itself prove standby quorum, promotion eligibility or old-primary fencing.

### 2.3 Durability evidence

Use an explicit enum, not a boolean:

- **LOCAL_FLUSH:** local WAL flush only.
- **REMOTE_WRITE:** required synchronous standby(s) have written WAL to their operating-system buffers; weaker than durable remote flush.
- **REMOTE_FLUSH:** configured synchronous standby quorum has durably flushed WAL.
- **REMOTE_APPLY:** configured synchronous standby quorum has applied the commit.
- **CANDIDATE_REPLAY_VERIFIED:** a named candidate is independently shown to have replayed through the transaction end LSN.
- **PROMOTION_CLOSED:** external policy admits only candidates proven to contain the effect or retain/replay all WAL through its end LSN, and the old writer is fenced.
- **UNKNOWN:** insufficient, stale or contradictory evidence.

A transaction may receive LOCAL_FLUSH from synchronous_commit=on when synchronous_standby_names is empty. It must not be labelled remotely durable in that configuration. Per-transaction synchronous_commit may differ from server defaults, so the effective value at commit matters.

Do not infer the exact set of standbys that satisfied a particular commit from a later SHOW synchronous_standby_names or a later pg_stat_replication sample. Those are policy/current-state observations, not an immutable per-transaction receipt. If exact responder identity is unavailable, the certificate must use a safer promotion-set rule: all candidates that might be promoted must be independently checked against the transaction end LSN, and candidates that fail the check must be fenced from promotion.

### 2.4 Recovery-set evidence

- recovery policy epoch and canonical policy digest;
- allowed candidate-set commitment;
- for each allowed candidate: authenticated node identity, system identifier, timeline ancestry proof, durable WAL flush position and replay/apply position;
- the target transaction end LSN and the comparison result;
- archive continuity and timeline-history references for PITR;
- required logical failover-slot state where a logical observer must resume;
- slot name, failover, synced, temporary, invalidation_reason, slotsync_skip_reason, restart LSN and confirmed-flush LSN where available.

A numeric LSN comparison alone is insufficient across diverged timelines. The verifier must establish a shared timeline ancestry and the correct branch. A PITR target below the acknowledged end LSN is not a permitted recovery path; it must be rejected or restored into quarantine.

### 2.5 External authority and fencing

- signed lease ID/hash, key ID, protocol version and verified signer;
- exact deployment, system identifier, timeline, owner epoch, lease generation, scope and expiry;
- external owner/lease high-water proof;
- promotion-policy authority receipt;
- old-primary physical-fencing receipt;
- evidence that the successor is the sole writer before writes resume.

A digest of a lease or fencing receipt is a commitment to evidence, not signature verification or physical fencing. The verifier must authenticate the signer and trust root and validate target binding. The former primary must be fenced by an independent mechanism before a successor accepts writes.

### 2.6 Certificate signature and storage

- canonical certificate digest;
- signing identity and key ID;
- issue time and bounded validity;
- monotonic policy and lease generations;
- external durable certificate/high-water receipt.

The certificate store must be outside the rollback domain of the PostgreSQL cluster it qualifies. If the external high-water store is unavailable, finality is UNKNOWN; it must not silently fall back to a database-local epoch.

## 3. Transactional WAL marker prototype

The proposed first implementation path is:

1. The strict SQL mutation function writes the protected effect and execution attestation.
2. In that same transaction, it writes a canonical transactional WAL marker containing effect ID, attestation ID, receipt digest, scope, owner epoch and resource generation.
3. The transaction commits.
4. An independently operated logical-decoding observer consumes the marker and its transaction commit record.
5. The observer verifies the marker against the actual effect/attestation receipt and records commit LSN, end LSN, transaction identity and timeline identity.
6. The external recovery controller proves the required candidate/promotion-set predicate and old-primary fencing.
7. The certificate is stored in the independent monotonic evidence store.
8. Only then may the adapter create the finality-qualified ACK capability and issue AckSync for the exact JetStream delivery.

The observer must be idempotent across restart. It must fail closed on missing marker, marker/effect mismatch, aborted transaction, slot loss, required-WAL loss, archive gap, observer lag beyond policy, unknown timeline ancestry or rollback below its external high-water.

## 4. PostgreSQL major-version profiles

The current Directory Fabric workflow uses PostgreSQL 16.16. Keep it as the compatibility baseline. It does not qualify logical failover-slot continuity.

PostgreSQL 17 and 18 support failover logical replication slots and asynchronous synchronization to physical standbys. Slot presence alone is not readiness: the intended standby must have the needed synchronized, persistent, non-invalidated slot, and must be sufficiently caught up before promotion.

PostgreSQL 19 is the current major release as of 2026-11-23. Its profile should additionally exercise synchronized_standby_slots, sync_replication_slots, explicit slot synchronization recovery, and the pg_replication_slots readiness/invalidation fields. These are optional qualification profiles, not substitutions for the PG16 CI baseline.

Never combine metrics across different PostgreSQL major versions without reporting server version, slot state, replication configuration and topology.

## 5. Promotion and recovery closure algorithm

For each certificate:

1. Verify effect and execution-attestation identity and canonical receipt digest.
2. Verify the transactional marker's exact binding to those records.
3. Verify commit and end LSN from a committed logical-decoding transaction, plus cluster/timeline identity.
4. Verify the durability class actually demonstrated; do not infer remote durability from synchronous_commit alone.
5. Enumerate all candidates the current promotion policy could select.
6. Verify each candidate's system identity, timeline ancestry and WAL coverage. A candidate may be admitted if its durable WAL contains the transaction end LSN and the promotion procedure guarantees replay through that LSN before writes are enabled.
7. Verify archive/PITR policy rejects targets below the certificate's end LSN.
8. Verify external authority high-water, signed lease scope/generation/expiry and physical fencing of the old primary.
9. Store the certificate and policy epoch outside the PostgreSQL rollback domain.
10. Bind the certificate to the exact JetStream delivery before AckSync.

If any predicate is not established, return UNKNOWN and do not ACK as an effect success. A separately typed durable reconciliation handoff may be recorded, but it is not an effect-success acknowledgement.

## 6. Research risks and non-claims

- A transactional logical-decoding marker proves transaction association and commit position; it does not prove a synchronous standby quorum.
- A synchronous replication policy states which standbys may satisfy commit waits; a later status sample is not a durable per-transaction responder receipt.
- A synchronized logical failover slot is not equivalent to proving every database recovery/promotion candidate contains the protected effect.
- A WAL end LSN is meaningful only with system identity and timeline ancestry.
- A signed lease without physical fencing is not proof that an isolated former primary cannot write.
- A one-primary/one-NATS integration test cannot qualify failover, PITR, split-brain or multi-node durability.
- The certificate format is a research proposal. No implementation or production guarantee is claimed by this document.

## 7. Qualification and performance

Correctness precedes performance. Every safety metric in MYC_DIR_RECOVERY_CLOSED_ACK_V0_1.json has a hard zero limit. Add liveness measurements so that “never ACK” cannot win:

- percentage of effects that become eligible and receive an ACK within five seconds after valid finality evidence exists;
- false-UNKNOWN rate when all required evidence is valid;
- reconciliation completion rate and p50/p95/p99;
- Safety-Adjusted Throughput: verified effects per second, reported only when all safety gates are zero.

Compare at least:
- JetStream AckSync only;
- JetStream plus primary-only effect receipt;
- JetStream plus effect and execution attestation;
- PostgreSQL synchronous replication without external recovery closure;
- transactional outbox/Debezium;
- Kafka transactions within Kafka's transaction domain;
- Recovery-Closed ACK.

Use matched hardware, payload sizes, replication factor, fault schedule, retention and offered load. Report p50/p95/p99 and raw samples. Do not compare Kafka's internal input-offset/output-topic transaction with a PostgreSQL external side effect as though they were the same transaction domain. Kafka's own design documentation explicitly distinguishes atomic processing inside Kafka from writes to an external system, which require cooperation with that destination.

The benchmark remains preregistered until the multi-node fault campaign and independent review have been executed.

## 8. Primary references

- PostgreSQL 16 WAL durability modes: https://www.postgresql.org/docs/16/runtime-config-wal.html
- PostgreSQL 16 synchronous standby selection and quorum: https://www.postgresql.org/docs/16/runtime-config-replication.html
- PostgreSQL 16 synchronous replication behavior: https://www.postgresql.org/docs/16/warm-standby.html
- PostgreSQL 16 transactional logical-decoding marker: https://www.postgresql.org/docs/16/functions-admin.html
- PostgreSQL 16 logical replication commit and end-LSN message formats: https://www.postgresql.org/docs/16/protocol-logicalrep-message-formats.html
- PostgreSQL 17 logical replication failover and readiness: https://www.postgresql.org/docs/17/logical-replication-failover.html
- PostgreSQL 19 replication and synchronized standby slots: https://www.postgresql.org/docs/19/runtime-config-replication.html
- PostgreSQL 19 replication-slot synchronization/invalidation fields: https://www.postgresql.org/docs/19/view-pg-replication-slots.html
- PostgreSQL 19 timeline-aware PITR: https://www.postgresql.org/docs/19/continuous-archiving.html
- NATS JetStream double acknowledgement and redelivery: https://docs.nats.io/nats-concepts/jetstream/consumers
- Apache Kafka transaction scope: https://kafka.apache.org/44/design/design/
- Debezium transactional outbox: https://debezium.io/documentation/reference/stable/transformations/outbox-event-router.html

- Patroni 4.1.5 dynamic configuration and replication modes: https://patroni.readthedocs.io/en/latest/dynamic_configuration.html and https://patroni.readthedocs.io/en/latest/replication_modes.html
- RabbitMQ quorum queues and publisher confirms: https://www.rabbitmq.com/docs/quorum-queues

Patroni must be treated as a serious PostgreSQL HA comparator, not dismissed. Its synchronous mode constrains promotion to a node believed to contain transactions that were reported committed; strict mode can trade write availability for durability when no synchronous replica is eligible. The benchmark separates strict and non-strict profiles. RabbitMQ quorum publisher confirms establish queue-level quorum acceptance; they still do not establish the destination PostgreSQL business effect.

These references establish product behavior and protocol boundaries. They do not qualify the Mycelix implementation; that requires its own exact-head and multi-node fault evidence.

## 9. Refined industry comparison — guarantee boundaries

The comparison must distinguish the state machine being protected, not just products that expose an ACK API.

### PostgreSQL 19 synchronous commit

PostgreSQL 19 documents five `synchronous_commit` modes: `remote_apply`, `on`, `remote_write`, `local` and `off`. With no synchronous standby configured, the non-off modes provide local WAL flush rather than remote replication. With a synchronous standby, `on` waits for durable standby flush, `remote_write` waits for standby OS write, and `remote_apply` waits for apply/visibility. The effective value is per transaction at commit, so a later server configuration read is not proof of the durability mode used by an earlier effect.

This is a strong baseline, and Mycelix should not claim to replace it. The additional research question is whether an application effect can be tied to the exact transaction end-LSN, recovery-candidate set and external fencing policy that govern future promotion.

### Patroni 4.1.5

Patroni synchronous mode is a serious HA baseline: it restricts automatic promotion to nodes believed to contain acknowledged transactions. Strict mode prevents Patroni from disabling synchronous replication when no synchronous standby candidate is available, blocking client writes. Its documentation also describes caveats: per-transaction `synchronous_commit=off/local` can bypass synchronous durability, manual failover can override the guard, and simultaneous failures can still matter.

Therefore the fair comparison is not “Patroni is unsafe.” It is:
- Patroni owns the PostgreSQL HA/promotion state machine and is already strong at synchronous standby selection;
- Mycelix is researching a separately verifiable application-effect and cross-system finality certificate, including proof of the exact effect, external lease high-water, physical fencing and broker delivery;
- both must be measured with the same replica count, write mode, failure schedule, network, storage and RPO requirement.

### Kafka 4.2 transactions

Kafka's exactly-once processing model atomically commits consumed offsets and produced records in Kafka's own transactional domain (including Kafka Streams state/output behavior). Kafka's design documentation states that external destination systems require cooperation from those systems to coordinate output with consumed offsets.

Do not compare Kafka Streams exactly-once to a PostgreSQL side effect as if the destination were part of the Kafka transaction. Use a matched Kafka Connect/outbox or destination-coordinating sink profile and measure the actual database commit and offset recovery.

### RabbitMQ quorum queues

RabbitMQ publisher confirms for quorum queues are issued after quorum replicas accept the message. Consumer manual acknowledgements govern redelivery. Those are meaningful queue guarantees, but neither is a proof that a separate PostgreSQL mutation committed or that every permitted database recovery candidate can retain it.

### Debezium transactional outbox

The outbox pattern atomically couples application state with an outbox row in the same database transaction; Debezium's outbox router captures that row and emits it. This is a strong and directly relevant comparator, not a straw man. Mycelix must compare against it with idempotent destination writes, connector restarts, duplicate emission, WAL retention and failover-slot continuity. The remaining question is whether the destination and promotion authority can present independently verifiable finality evidence for the exact database effect and ACK.

## 10. Official product references

- PostgreSQL 19 write-ahead-log and synchronous-commit modes: https://www.postgresql.org/docs/19/runtime-config-wal.html
- PostgreSQL 19 synchronous and logical failover replication settings: https://www.postgresql.org/docs/19/runtime-config-replication.html
- PostgreSQL 19 logical replication commit/end-LSN wire format: https://www.postgresql.org/docs/19/protocol-logicalrep-message-formats.html
- PostgreSQL 19 replication slot readiness/invalidation fields: https://www.postgresql.org/docs/19/view-pg-replication-slots.html
- Patroni 4.1.5 synchronous, strict and quorum replication modes: https://patroni.readthedocs.io/en/latest/replication_modes.html
- Kafka 4.2 design and transaction boundary: https://kafka.apache.org/42/design/design/
- Kafka Streams processing guarantees: https://kafka.apache.org/42/streams/core-concepts/
- RabbitMQ publisher confirms: https://www.rabbitmq.com/docs/confirms
- RabbitMQ quorum queues: https://www.rabbitmq.com/docs/quorum-queues
- Debezium transactional outbox event router: https://debezium.io/documentation/reference/stable/transformations/outbox-event-router.html

These sources establish product-level behavior, not Mycelix qualification. Mycelix claims still require exact-head tests, multi-node fault injection, independent proof-root tests and reproducible performance data.
