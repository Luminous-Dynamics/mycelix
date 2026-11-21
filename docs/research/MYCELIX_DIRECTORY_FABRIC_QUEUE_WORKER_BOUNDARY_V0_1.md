# Mycelix Directory Fabric: queued command and worker effect boundary

**Status:** research contract and reference semantics; no broker adapter, queue runtime, worker sandbox or broker integration test is qualified here.

## 1. Core rule

A broker delivery is an invitation to attempt a piece of work, not a durable authorization decision and not proof of an effect.

The worker must revalidate the exact command at the point of work:
1. Broker namespace, destination, stable application message key and payload digest match the immutable command binding.
2. The exact decision receipt is independently authenticated and verified as Allow for the command's request, realm, policy, purpose, audience and effect owner.
3. The command remains inside the decision validity interval under the exact pinned clock profile and bounded uncertainty.
4. The resource owner compares the exact fence generation inside the same atomic boundary as the mutation.
5. The effect attempt/reconciliation record is durably committed.
6. Only after a durable disposition or durable reconciliation handoff may the broker delivery be acknowledged.

The Rust queue-delivery module implements only the first and part of the time-binding checks. Its result EligibleForOwnerRevalidation is deliberately not named Authorized. The effect-fence and receipt contracts still own decision verification, mutation-time generation comparison and effect truth.

## 2. Separate the four planes

| Plane | Evidence | Does not establish |
|---|---|---|
| Broker transport | publish confirm, stream offset, delivery attempt, redelivery count, consumer ack | identity, permission, unique physical effect |
| Worker assignment | consumer group, claim/lease, worker epoch, lease-renewal event | that the current worker is still authorized |
| Authorization | exact request-bound Allow receipt, dependency closure, clock profile and validity | that the effect occurred |
| Effect owner | atomic owner fence check, idempotency binding, effect receipt and exact reconciliation | completeness of other mutation routes |

Broker sequence numbers and redelivery counters are transport metadata. They must never be substituted for the Mycelix authority frontier, the resource fence generation, a logical operation ID or a physical attempt receipt.

## 3. Why broker exactly-once is not enough

The broker's guarantee must be measured at its exact boundary:

- **NATS JetStream:** durable streams and consumers support replay and at-least-once delivery. Unacknowledged messages are redelivered. Message-ID deduplication is a publisher/broker facility; it is not a resource-side effect fence and does not remove the need for effect-owner idempotency. See https://docs.nats.io/concepts/jetstream and https://docs.nats.io/using-nats/jetstream/model_deep_dive.
- **RabbitMQ 4.4 quorum queues:** Raft-replicated queues can issue publisher confirms after quorum replication. Consumers should use manual acknowledgements for redelivery. At-least-once dead-letter transfer can duplicate messages. See https://www.rabbitmq.com/docs/quorum-queues and https://www.rabbitmq.com/docs/confirms.
- **Apache Kafka 4.4:** a transactional producer can atomically write output records and consumed offsets within Kafka, with read-committed consumers. For an external destination, Kafka's own documentation says the destination generally must cooperate; storing the offset with the external output is one way to obtain the stronger guarantee. See https://kafka.apache.org/44/design/design/.

These systems solve difficult transport, durability and stream-processing problems. Mycelix should reuse those strengths, not compete with broker consensus or reimplement their delivery protocols. Its distinct job is to bind authority to the eventual effect owner and make the exact execution path revocable.

## 4. Durable handoff and acknowledgement

A common mistake is to choose between two bad extremes:
- acknowledge immediately after delivery, risking loss before any durable effect/recovery record exists;
- never acknowledge an Unknown result, causing repeated hot redelivery while an external effect is being reconciled.

The reference state machine distinguishes:
- Unstarted: no physical attempt has begun;
- AttemptUnknown: outcome is uncertain and no durable recovery task has been confirmed;
- UnknownWithDurableReconciliationTask: an immutable recovery task and exact attempt reference have been durably committed;
- Committed: the effect owner durably observed the commit;
- ConfirmedNoEffect: the exact attempt was reconciled as no-effect;
- RejectedBeforeAttempt: an expired, mismatched or fenced command was durably rejected.

The broker message may be acknowledged after Committed, ConfirmedNoEffect, RejectedBeforeAttempt, or UnknownWithDurableReconciliationTask. In the last case, the acknowledgement means only that the recovery obligation has been durably transferred; it does not mean success, completion or no-effect. The durable reconciliation service must remain independently observable and retryable.

A new physical attempt is allowed only for an unstarted operation or after an exact owner-side ConfirmedNoEffect reconciliation. AttemptUnknown blocks new attempts. The same logical operation and idempotency commitment survive redelivery, worker replacement and broker migration.

## 5. Worker claims are fenced too

Consumer-group assignment, lease renewal, visibility timeout and worker heartbeat are liveness/coordination mechanisms. A paused or partitioned worker may resume after another worker has taken over.

Every claim therefore carries a monotonically increasing claim epoch, but the epoch is not itself authorization. The Rust reference now has WorkerClaimToken and WorkerClaimState types. The token binds the logical operation, worker identity, epoch, clock profile, validity window and external proof commitment; the effect owner compares those values with its durable current claim. At the effect boundary the owner must reject a command whose worker-claim epoch is no longer the exact current claim where the resource uses worker-claim fencing. A stale worker cannot rely on a previous heartbeat or a broker delivery tag.

The resource fence generation and worker claim epoch have different scopes:
- resource generation: revocation state for a realm/resource/action/effect owner;
- worker claim epoch: current executor ownership for one logical command/partition;
- broker delivery count: diagnostic metadata about transport retries.

A claim lease may expire to make work available for recovery, but it cannot make an old worker safe. The atomic effect owner must check the current claim token and resource generation, or a separately proven equivalent.

## 6. The queue-to-effect flow

1. The authority evaluator creates one signed, short-lived decision receipt and command envelope.
2. The outbox commits the immutable envelope with its logical operation ID and idempotency key; the broker publisher retries the exact message key.
3. The broker acknowledges storage. This is recorded as transport evidence only.
4. A worker receives a delivery, validates the message key/body digest/route, obtains a current claim epoch and revalidates decision time.
5. The worker calls the actual effect owner. The owner independently authenticates the Allow, validates the exact generation and claim token, and performs idempotency and mutation atomically.
6. The owner stores the physical attempt outcome. On unknown outcome, it creates a durable reconciliation task and exact target-attempt reference in the same owner-controlled recovery path.
7. The worker acknowledges the broker delivery only after a durable terminal disposition or the durable reconciliation handoff.
8. A redelivery returns the existing immutable operation result or resumes reconciliation; it does not create a new operation identity.

A transactional outbox only solves the source-side state/message publication gap. It does not make the broker, worker and external effect owner one distributed transaction. Each boundary must expose its own durable receipt and recovery contract.

## 7. Adversarial qualification corpus

The queue worker corpus defines 27 provider-neutral and broker-specific scenarios spanning broker mismatch, payload substitution, backlog expiry, owner-fence revocation, stale worker claims, redelivery, broker dedup expiry, dead-letter duplication, lost acknowledgements, Unknown outcomes, reconciliation, and Kafka transaction-scope boundaries.

The broker adapters must be tested under equivalent semantic profiles:
- durable storage and replication settings declared;
- publisher confirms / acknowledgements and consumer acknowledgement modes pinned;
- delivery, redelivery, retry, dead-letter and retention settings recorded;
- duplicate delivery forced by lost acknowledgement and worker crash;
- broker deduplication window configured and then intentionally expired;
- real consumer rebalances/partition failover and delayed worker resumption;
- real downstream effect owner instrumented with its own fence and idempotency log.

Never compare exactly-once labels across products without comparing the actual source, processing and destination transaction boundaries.

## 8. Benchmark programme

Profiles:
- Small: 100 messages/s, 1 worker, 1,000 queued messages.
- Production: 10,000 messages/s, 32 workers, 1 million queued messages.
- Enterprise stress: 100,000 messages/s, 256 workers, 10 million queued messages.
- Revocation backlog: command ages 0 s / 1 min / 1 h / 24 h; revoke while commands are queued, delivered, in-flight, delayed and recovering.

Measure transport and effect layers separately:
- broker publish confirm and delivery latency;
- delivery-to-worker-revalidation latency;
- claim acquisition and rebalance recovery;
- effect-owner mutation latency and lock contention;
- queue depth, oldest-command age, redelivery and dead-letter volume;
- time from revocation request to last old-generation accepted effect;
- time to durable reconciliation and Unknown resolution;
- CPU, memory, storage write amplification and network bytes per accepted effect.

Non-compensable safety gates:
- 0 stale-generation effects after exact owner-fence acknowledgement;
- 0 stale-worker-claim effects where claim fencing is required;
- 0 expired-decision effects;
- 0 payload/message/route substitution accepts;
- 0 duplicate logical effects;
- 0 retry while the exact previous physical outcome remains Unknown;
- 0 broker acknowledgements before either durable terminal disposition or durable reconciliation handoff;
- 100% broker deliveries resolve to a durable terminal or recovery state;
- 100% strict revocation plans include every independently executable mutation route in the verified inventory.

Provisional latency objectives, not achieved results:
- worker-side envelope and time-binding p95 <= 250 microseconds for the pure reference check;
- worker delivery-to-owner-revalidation p95 <= 5 ms on a local healthy network;
- healthy-network revocation request to durable owner fence completion p95 <= 5 s and p99 <= 30 s;
- after owner fence acknowledgement, zero old-generation effects regardless of queue backlog age.

The benchmark harness must use open-loop offered load, at least 10 independent runs per configuration, raw samples, p50/p95/p99/max per run, offered and accepted rates, error rate, warm/cold conditions, overload and recovery phases. No averaged percentiles and no simulated-only result represented as a real broker test.

## 9. Claim ceiling and next integrations

This is a provider-neutral contract and deterministic Rust reference module, not a NATS, RabbitMQ or Kafka integration. No broker runtime is qualified. The worker-claim epoch must be implemented by the effect owner, not just by the queue adapter. External APIs that cannot provide durable idempotency and atomic fencing remain BestEffortOnly in the effect-surface inventory.

The next execution stages are:
1. NATS JetStream adapter and redelivery/ack-loss integration harness.
2. RabbitMQ quorum queue adapter with publisher confirms, manual acknowledgements and duplicated dead-letter delivery.
3. Kafka 4.4 consumer/producer transaction profile plus an external database effect owner that demonstrates the boundary of Kafka-only exactly-once.
4. PostgreSQL two-session race and crash-recovery tests using the existing Mycelix-DeSci authority/epoch mechanisms.

Sources:
- NATS JetStream: https://docs.nats.io/concepts/jetstream
- RabbitMQ Quorum Queues 4.4: https://www.rabbitmq.com/docs/quorum-queues
- RabbitMQ confirms: https://www.rabbitmq.com/docs/confirms
- Apache Kafka 4.4 delivery semantics: https://kafka.apache.org/44/design/design/
