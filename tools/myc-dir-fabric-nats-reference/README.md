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

There are 21 unit tests. On exact PR head `1105375305704c019b7d2ccecda78a18dc0e7faf`, GitHub Actions run [#45000430944](https://github.com/Luminous-Dynamics/mycelix/actions/runs/45000430944) passed all 21 tests, formatting and Clippy.

## Explicit limits

This is a provider-specific **transport contract**, not a real NATS client adapter. It does not depend on async-nats, connect to a broker, create streams/consumers, publish, receive PubAck, issue Ack/AckSync, or exercise redelivery/failover.

The next increment must pin NATS server and async-nats client versions and add a real integration harness with forced AckWait redelivery, lost PubAck/ACK responses, late worker acknowledgements, duplicate republish after window expiry and the PostgreSQL effect owner.