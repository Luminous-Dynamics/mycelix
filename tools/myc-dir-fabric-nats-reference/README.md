# NATS JetStream transport contract reference

Dependency-free Rust reference types for the NATS JetStream portion of the Mycelix Directory Fabric queued-worker contract.

## Current behavior

- Binds NATS account, stream, subject, durable consumer identity and application Nats-Msg-Id to the exact payload digest.
- Keeps the Mycelix logical-operation/idempotency identity and owner epoch/generation distinct from NATS publication and delivery metadata.
- Validates exact delivery bindings before returning `EligibleForOwnerRevalidation`.
- Requires explicit consumer acknowledgement configuration.
- Allows ACK planning only after a durable owner disposition: committed effect, terminal rejection, terminal no-effect evidence or durable reconciliation handoff.
- Rejects ACK planning for InFlight and Unknown outcomes.
- Separates PubAck publication/deduplication results from effect receipts.
- Separates AckSync confirmation of a broker ACK from effect-owner evidence.
- Validates redelivery as the same stream message/payload with a higher delivery count.
- Treats duplicate-window expiry as a broker concern; owner idempotency remains stable across republish with a changed Nats-Msg-Id.

## Run

From the repository root:

```sh
cargo test --manifest-path tools/myc-dir-fabric-nats-reference/Cargo.toml
```

There are 23 unit tests in the current scope-bound command contract. The earlier NATS-only exact-head run passed 21 tests before the PostgreSQL-scope expansion. The 23-test scope-aware contract on the PostgreSQL-verifier branch is pending its own exact-head CI run. On exact PR head `1105375305704c019b7d2ccecda78a18dc0e7faf`, GitHub Actions run [#45000430944](https://github.com/Luminous-Dynamics/mycelix/actions/runs/45000430944) passed all 21 tests, formatting and Clippy.

## Explicit limits

This is a provider-specific **transport contract**, not a real NATS client adapter. It does not depend on async-nats, connect to a broker, create streams/consumers, publish, receive PubAck, issue Ack/AckSync, or exercise redelivery/failover.

The concrete client and PostgreSQL receipt verifier now live in the adjacent `myc-dir-fabric-nats-adapter` crate. The PostgreSQL-backed branch is awaiting its own exact-head CI qualification.