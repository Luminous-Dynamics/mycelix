# Mycelix NATS JetStream adapter

A concrete asynchronous NATS JetStream adapter for the Directory Fabric transport contract.

## Pinned implementation

- Rust client: `async-nats = 0.53.0`
- Server integration image: `nats:2.15.3`
- Adapter uses explicit durable pull consumers, Nats-Msg-Id publication IDs, a separate Mycelix publication ID header, stream/consumer delivery metadata, and Ack/AckSync/Progress operations.
- Direct Rust dependencies are version-pinned in Cargo.toml. Transitive dependencies are resolved by Cargo; release qualification should preserve the generated Cargo.lock from the tested environment.

The client API version was selected against the upstream async-nats 0.53.0 API, including `PublishMessage::message_id`, `Context::publish_message`, `Message::info`, `Message::split`, `Acker::ack`, `Acker::ack_with`, and `Acker::double_ack`.

## Startup

1. Connect with `JetStreamAdapter::connect`.
2. Call `verify_server_configuration()`.
3. Only after that check succeeds can `publish` and `fetch_one` be used.

The verification step checks exact subjects, duplicate window, replica count, file storage, ACK-enabled stream, non-destructive DiscardNew policy, retention policy, max-age, message and byte capacity; it then checks that the named durable consumer exists with Explicit AckPolicy, exact AckWait, MaxAckPending, MaxDeliver and filter subject. It never silently rewrites an existing mismatched consumer. Configuration must be reverified after any server-side reconfiguration; it is a startup guard, not proof that the server cannot drift later.

## Command wire binding

The publisher computes the payload SHA-256 itself and binds it to:
- the application account label, stream, subject and durable consumer;
- application publication ID and Nats-Msg-Id;
- logical-operation ID and owner-scoped idempotency digest;
- request and decision-receipt digests;
- effect-owner ID and owner epoch/resource generation.

The adapter writes a versioned header envelope. Required header names are unique: repeated values are rejected instead of silently selecting the first or last. The application publication ID is stored in `Mycelix-Publication-Id`; the reserved `Nats-Msg-Id` is used only for JetStream duplicate detection. This avoids relying on a broker-specific reserved-header persistence detail for the Mycelix event identity.

On receive, the adapter verifies payload digest, reconstructs the typed binding and checks its canonical binding digest before returning the delivery for owner revalidation. This is structural integrity, not a signature or authorization proof. The worker must authenticate the decision receipt and worker claim, and the effect owner must enforce the current claim and fence atomically with the protected mutation.

## ACK semantics

- `progress()` sends JetStream `AckKind::Progress` to extend AckWait only. It does not renew Mycelix decision or worker/resource authority.
- `acknowledge(..., AckMode::Ack)` sends +ACK after a durable owner disposition but does not claim a broker response confirmation.
- `acknowledge(..., AckMode::AckSync)` calls async-nats `double_ack()` and returns only after JetStream confirms receipt.
- InFlight and Unknown cannot be acknowledged.
- A durable reconciliation handoff can permit a broker ACK while a separate reconciliation owner continues resolving the external outcome. It is not effect success.

## Test

With a disposable JetStream service:

```sh
NATS_URL=nats://127.0.0.1:4222 cargo test \
  --manifest-path tools/myc-dir-fabric-nats-adapter/Cargo.toml

NATS_URL=nats://127.0.0.1:4222 cargo test \
  --manifest-path tools/myc-dir-fabric-nats-adapter/Cargo.toml \
  --test jetstream_integration -- --ignored --nocapture
```

The focused CI workflow starts a pinned `nats:2.15.3` server with JetStream enabled and runs the ignored integration tests. It exercises:
- publish and PubAck;
- duplicate Nats-Msg-Id within the stream duplicate window;
- durable pull delivery and exact binding reconstruction;
- AckSync / double_ack confirmation after a durable effect disposition;
- unacknowledged-message redelivery after AckWait;
- durable reconciliation-handoff acknowledgement following redelivery.

The integration suite uses a single-node File-storage server. It is not a clustered failover, replica-loss or production performance qualification.

## Explicit limitations

- The adapter does not authenticate/sign decision receipts or worker claims. It checks structural bindings and SHA-256 content commitments only.
- Account ID is an application label. Deployments should supply authenticated NATS credentials/TLS and pin the configured domain/account scope according to their account topology.
- Server configuration verification is an explicit startup gate; operators must reverify after control-plane changes.
- No PostgreSQL transaction or external effect is performed by this crate.
- No local Cargo compilation or NATS runtime result is claimed until the exact-head workflow completes.


The adapter crate adds 13 pure unit tests and two ignored JetStream integration tests. The reference contract currently has 21 unit tests. They remain uncompiled until the exact-head workflow completes.

## Authenticated owner-disposition gate

The adapter does not accept a raw public `DurableOwnerDisposition` as sufficient proof to ACK. Call `VerifiedOwnerDisposition::verify(binding, disposition, verifier)`. The external `OwnerDispositionVerifier` must authenticate the actual durable effect-owner receipt against the exact binding. The resulting token stores a digest of that exact canonical binding, and `ReceivedDelivery::acknowledge` rechecks the binding before sending ACK/AckSync.

The integration suite uses a clearly labelled synthetic verifier only to exercise transport behaviour. A production deployment must supply a verifier backed by the real owner receipt signature / attestation and durable receipt ledger.
