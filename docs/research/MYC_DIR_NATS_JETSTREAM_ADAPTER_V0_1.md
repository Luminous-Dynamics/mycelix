# Mycelix Directory Fabric: concrete NATS JetStream adapter v0.1

**Status:** source implementation and ignored integration tests present; not yet compiled or executed in this environment  
**Branch:** `research/myc-dir-nats-async-adapter-v0-2`  
**Base:** corrected NATS contract PR #6507  
**Upstream pins:** `async-nats = 0.53.0`; NATS server `2.15.3`

## 1. Move from contract to actual SDK

The previous NATS increment defined only a dependency-free contract. This increment adds a concrete asynchronous JetStream client crate:

`tools/myc-dir-fabric-nats-adapter/`

The adapter is written against async-nats 0.53.0, with explicit use of the upstream APIs for custom PubAck headers, JetStream publishing, durable pull consumers, message delivery metadata and acknowledgement operations.

The server integration fixture pins NATS 2.15.3, the current stable release at the time this work was authored.

## 2. Real client API mapping

The implementation uses:
- `jetstream::Context::publish_message` with `PublishMessage::message_id` to set the reserved Nats-Msg-Id header and receive `PublishAck`;
- stream metadata to verify subjects, duplicate window and replica count;
- `stream::get_or_create_consumer` with explicit durable pull-consumer configuration;
- `Consumer::info` to verify AckPolicy, AckWait, MaxAckPending, MaxDeliver and filter subject;
- `Consumer::fetch` for bounded one-message pulls;
- `Message::info` for stream, consumer, stream sequence, consumer sequence and delivery count;
- `Message::split` to separate immutable payload/headers from the acknowledgement capability;
- `Acker::ack`, `Acker::ack_with(AckKind::Progress)` and `Acker::double_ack`.

The ACK types deliberately preserve the difference between sending +ACK and awaiting broker acknowledgement confirmation.

## 3. Wire-level hardening

The publisher computes the SHA-256 payload digest itself and serializes the typed command binding into a versioned application header envelope. The binding includes:
- account label, stream, subject and durable consumer;
- Nats-Msg-Id and separate application publication ID;
- logical operation and idempotency commitment;
- request and decision receipt digests;
- effect-owner ID and owner epoch/resource generation.

The adapter stores its own event identity in `Mycelix-Publication-Id`. It uses the reserved `Nats-Msg-Id` only for stream deduplication. It does not require the reserved header to survive broker storage unchanged.

Incoming commands must have exactly one value for every required header. Duplicate header values are rejected. The adapter recomputes the payload digest and canonical typed-binding digest before returning a message for owner revalidation.

These digests establish structural integrity only. They are not signatures, and they do not authenticate the decision, worker identity, claim or owner fence. Those remain separate verification/enforcement responsibilities.

## 4. Startup configuration is fail-closed

The adapter cannot publish or fetch until `verify_server_configuration()` succeeds. It checks:
- exact stream subject inclusion;
- stream duplicate window and replica count;
- durable consumer name;
- Explicit AckPolicy;
- AckWait;
- MaxAckPending;
- MaxDeliver;
- exact consumer subject filter.

A mismatch is returned as an error; existing server-side configurations are not silently edited. The verification result is a mandatory startup gate. The pull path rechecks the server-returned stream and consumer configuration before each delivery. Publishers must repeat the startup verification after control-plane changes; this adapter does not claim that the server cannot drift between checks.

## 5. ACK policy

The adapter delegates eligibility to the dependency-free Directory Fabric contract:
- EffectCommitted, TerminalRejected, owner-terminal TerminalNoEffect, or durable reconciliation handoff may permit ACK.
- InFlight and Unknown cannot be ACKed.
- Ack sends the consumer ACK but reports no server confirmation.
- AckSync uses `double_ack()` and reports confirmation only after the broker response.
- Progress extends the JetStream AckWait timer; it does not renew a Mycelix decision, worker claim or owner/resource generation.


## 5.1 Authenticated owner-disposition gate

A structurally valid disposition is not itself proof that an effect owner issued it. The adapter therefore requires an external `OwnerDispositionVerifier` to validate the durable owner receipt against the exact command binding. Only then can `VerifiedOwnerDisposition` be constructed. It stores a SHA-256 commitment to the canonical command binding, and `ReceivedDelivery::acknowledge` checks that commitment again before sending +ACK or requesting double-ACK confirmation.

The verifier must check the actual signed/attested receipt, owner identity, logical operation, idempotency commitment and terminal status. The integration harness uses a deliberately synthetic verifier to isolate broker transport semantics; it is not a production owner verifier.

## 6. Integration harness

Adds `.github/nats-jetstream-test.conf` and `.github/workflows/myc-dir-nats-adapter.yml`. The workflow starts an isolated `nats:2.15.3` server with JetStream enabled and a dedicated JetStream domain.

Two ignored integration tests cover:
1. PubAck publication, duplicate Nats-Msg-Id in the duplicate window, delivery metadata reconstruction, and AckSync after a durable owner effect receipt.
2. Dropping a delivery without ACK, waiting beyond AckWait, observing the same stream sequence with a higher delivery count, and ACKing only after a durable reconciliation handoff.

The crate also adds 13 adapter unit tests for config validation, hex digest parsing, and duplicate authority-header rejection. The separate contract crate now contributes 21 authored tests; the adapter adds 13 unit tests and two ignored real-server integration tests.

**Execution evidence (2026-11-22):** GitHub Actions run [#45000430944](https://github.com/Luminous-Dynamics/mycelix/actions/runs/45000430944) on exact head `1105375305704c019b7d2ccecda78a18dc0e7faf` passed formatting, 21/21 reference tests, 18/18 adapter unit tests, 2/2 live JetStream integration tests, and Clippy with warnings denied. The NATS server was `2.15.3`. This is single-node broker evidence only; the integration harness uses a synthetic owner-receipt verifier and does not qualify a production cryptographic verifier, PostgreSQL owner integration, clustered replica failover or performance.

## 7. Benchmark updates

This is still a single-node correctness harness, not a performance comparison. The follow-on clustered benchmark should preserve the existing NATS/RabbitMQ/Kafka matched-durability matrix and add:
- real adapter initialization/configuration verification time;
- publish-to-PubAck latency;
- consumer fetch/redelivery timing;
- owner revalidation and effect commit latency;
- ACK send vs AckSync confirmation latency;
- AckWait and MaxDeliver behaviour under worker pause/kill;
- leader and replica failover with lost PubAck/ACK responses;
- duplicate republish beyond the stream duplicate window;
- 10 independent open-loop repetitions per configuration.

The success criterion remains zero stale-claim and stale-generation effects, zero duplicate logical effects, and zero premature ACKs. Broker throughput is not an end-to-end effect-safety result.

## 8. Evidence boundary

- `async-nats 0.53.0` and `nats:2.15.3` are pinned directly in source/CI.
- This environment has not run Cargo, rustfmt, Clippy or an actual NATS server.
- The current CI workflow is the first real build/runtime qualification gate.
- The integration service is single-node. No clustered failover or production account/credential policy is qualified.
- No PostgreSQL effect owner is called here; tests use synthetic durable owner dispositions.
- No security signature, trust-root, or decision-receipt verifier is implemented in the NATS crate.

This increment advances the work from reference-only transport semantics to an actual SDK adapter and an automated broker test harness, while keeping the external effect-owner boundary intact.
