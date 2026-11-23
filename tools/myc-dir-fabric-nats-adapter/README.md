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

The focused CI workflow starts a pinned `nats:2.15.3` server with JetStream enabled and runs the ignored integration tests. This exact-head run passed both tests on a single-node server. It exercises:
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


The **earlier NATS-only** source head `1105375305704c019b7d2ccecda78a18dc0e7faf` passed GitHub Actions run [#45000430944](https://github.com/Luminous-Dynamics/mycelix/actions/runs/45000430944): formatting, 21/21 contract tests, 18/18 adapter tests, 2/2 JetStream integration tests and Clippy. This later owner-epoch branch has expanded the contract and adapter tests and is not covered by that earlier run.

## Authenticated owner-disposition gate

The adapter does not accept a raw public `DurableOwnerDisposition` as sufficient proof to ACK. Call `VerifiedOwnerDisposition::verify(binding, disposition, verifier)`. The external `OwnerDispositionVerifier` must authenticate the actual durable effect-owner receipt against the exact binding. The resulting token stores a digest of that exact canonical binding, and `ReceivedDelivery::acknowledge` rechecks the binding before sending ACK/AckSync.

The integration suite uses a clearly labelled synthetic verifier only to exercise transport behaviour. A production deployment must supply a verifier backed by the real owner receipt signature / attestation and durable receipt ledger. The CI integration profile is single-node; it does not qualify cluster failover or PostgreSQL effect-owner integration.


## PostgreSQL-backed effect receipt verification

The adapter now exposes `postgres::PostgresEffectCommitVerifier`. It connects with a caller-supplied CA file using rustls certificate and hostname verification, rejects recovery/read-only nodes, and queries the existing append-only `myc_effect_owner.effect_commits` table by realm/resource/action/owner/object/currency, idempotency key and logical operation. It then compares request, decision, payload and accepted-generation fields and recomputes a versioned canonical receipt digest from the database row.

The verifier only accepts `EffectCommitted`, because the reference SQL ledger has no durable terminal-rejection, terminal-no-effect or reconciliation-handoff tables. It returns no ACK capability when the effect or required execution attestation is missing or mismatched. Migration 003 extends the owner schema with the execution-attestation and epoch-transition ledgers; the owner-epoch branch changes the test path to call the strict epoch-aware mutation function and read all three records over CA-verified TLS before AckSyncing the exact NATS message.

This owner-epoch branch currently contains 23 reference-contract tests, 16 adapter-core unit tests, 4 PostgreSQL receipt/attestation unit tests, 3 ignored real-service integration tests, and a 20-assertion PostgreSQL SQL adversarial script. These are source counts only until this branch's own exact-head workflow passes.

The benchmark corpus is expanded to 52 scenarios and 37 metrics. It remains preregistered, not executed.

**Important boundary:** the existing receipt table records accepted resource generation, but not owner epoch. The verifier checks the former only; owner-epoch recovery remains a separate owner-state proof. A receipt visible on the writable primary does not, by itself, prove survival after asynchronous replica promotion. The integration transaction sets synchronous_commit=on, but production synchronous replication, failover and promotion policy must be qualified separately.


## Owner epoch and historical worker-claim execution attestation

The follow-on PostgreSQL owner-epoch branch adds migration 003 and the strict SQL entry point `apply_owner_epoch_claim_fenced_balance_delta`. It writes the accepted owner epoch, resource generation, worker identity, claim epoch, claim proof commitment, claim validity interval and PostgreSQL transaction ID into the append-only `effect_execution_attestations` table in the same transaction as the new effect.

The verifier now joins `effect_commits`, `effect_execution_attestations` and the corresponding `effect_owner_epoch_transitions` row. It includes the actual owner epoch, worker claim and external epoch/physical-fencing receipt commitments in the canonical receipt digest. It rejects missing attestations and non-initial owner epochs without a corresponding transition.

Migration and SQL tests are under `tools/myc-dir-fabric-reference/postgres/003_owner_epoch_execution_attestation.sql` and `tools/myc-dir-fabric-reference/postgres/tests/003_owner_epoch_execution_attestation.sql`. The workflow applies them and runs the SQL test before the Rust/NATS integration tests.

This is local durable evidence, not a complete distributed-fencing solution. The SQL function stores receipt commitments but cannot authenticate their signatures or prevent a former writable primary from accepting writes. Production must authenticate the epoch authority and physical-fencing receipts, physically fence the old primary, enforce a single-writer lease, and qualify synchronous replication/promotion. This branch's owner-epoch and PostgreSQL tests remain unqualified until its exact-head CI passes.


## Owner-epoch-bound worker claims

Migration 003 also adds `owner_epoch` to `worker_claim_state` and the explicit `install_worker_claim_for_owner_epoch` entry point. Claim installation and effect execution use the same lock order: worker-claim row first, then owner-fence row. A claim from an earlier owner epoch cannot execute after promotion. Crossing owner epochs requires a strictly higher worker claim epoch; the strict effect function rejects a claim whose recorded owner epoch differs from the current fence epoch.

The SQL adversarial test covers: stale claim reuse after owner-epoch transition, failure to increment claim epoch, successful epoch-2 claim installation, and later worker reassignment to a higher claim epoch. It also checks UPDATE/DELETE/TRUNCATE rejection on the append-only owner/attestation evidence tables.

The bounded state-machine companion is `specs/owner-epoch/OwnerEpochExecutionAttestation.tla` with configuration `OwnerEpochExecutionAttestation.cfg`. It is authored but not yet model-checked with TLC.
