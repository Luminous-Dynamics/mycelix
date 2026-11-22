# Mycelix Directory Fabric: PostgreSQL effect-receipt verifier v0.1

**Profile:** MYC-DIR-POSTGRES-RECEIPT-VERIFIER-0.1  
**Status:** source-present verifier and integration harness; not yet compiled or executed  
**Tracking:** [MYC-DIR-POSTGRES-RECEIPT-VERIFIER-001 / #6513](https://github.com/Luminous-Dynamics/mycelix/issues/6513)  
**Composes with:** [NATS adapter PR #6512](https://github.com/Luminous-Dynamics/mycelix/pull/6512) and the PostgreSQL effect-owner schema / worker claim fencing functions in `tools/myc-dir-fabric-reference/postgres/`.

## 1. Why broker ACK must follow a real owner receipt

The NATS adapter's first integration test used a synthetic `OwnerDispositionVerifier`. That was sufficient to test broker semantics, but it did not prove that a real external effect had committed.

This increment adds `PostgresEffectCommitVerifier`, which reads the existing append-only `myc_effect_owner.effect_commits` table. It does not create a parallel receipt ledger and does not treat a PubAck, worker log or consumer ACK as proof of an effect.

The ACK chain becomes:

`NATS delivery -> exact command binding -> PostgreSQL primary/read-only check -> exact append-only effect row -> canonical row digest -> VerifiedOwnerDisposition -> Ack/AckSync`.

Only an exact, already-committed effect receipt produces a capability that can pass the NATS ACK gate.

## 2. Exact resource scope in the command binding

The command now binds the actual PostgreSQL resource tuple:
- realm ID;
- resource ID;
- action ID;
- effect-owner ID;
- object ID;
- currency code.

It also binds:
- logical operation ID;
- owner-scoped idempotency digest;
- request digest;
- decision receipt digest;
- payload digest;
- resource generation;
- owner epoch;
- NATS publication and consumer identities.

The canonical binding bytes include the exact database resource tuple. The NATS wire envelope carries each scope component in a distinct, unique header. Duplicate copies of a required header remain a hard rejection.

The PostgreSQL ledger presently records the accepted **resource generation**, but it does not have an `owner_epoch` column. Accordingly, this verifier does not claim to prove owner-epoch continuity. Owner epoch/recovery remains a separate owner-state proof and the resource owner still enforces the current fence atomically at mutation time.

## 3. Existing append-only effect ledger

The verifier reads the existing `myc_effect_owner.effect_commits` row by the exact tuple:
- `realm_id`;
- `resource_id`;
- `action_id`;
- `effect_owner_id`;
- `object_id`;
- `currency_code`;
- `idempotency_key_digest`;
- `logical_operation_id`.

It then independently compares:
- `request_digest`;
- `decision_receipt_digest`;
- `payload_digest`;
- `accepted_generation`;
- immutable `effect_id`;
- committed delta and resulting balance;
- the full resource tuple and operation identity.

Missing rows reject ACK. Duplicate matches are a query error (the schema's uniqueness constraints should make this impossible). Wrong scope, wrong generation or digest substitution rejects ACK.

The verifier constructs a versioned canonical receipt encoding from the row values and recomputes its SHA-256 digest. The caller cannot submit an arbitrary receipt digest and have it accepted.

This is a content commitment to a row observed from the authenticated database connection. It is not a database signature or an independent attestation by itself.

## 4. TLS, primary and read-only checks

The verifier connects with `rustls-tokio-postgres 0.5.1` using `config_from_ca_cert` and `MakeRustlsConnect`. Certificate-chain and hostname verification remain enabled. It does not use `config_no_verify`.

The effect lookup statement itself returns:
- `pg_is_in_recovery()`;
- `current_setting('transaction_read_only')`;
- the exact matching effect row.

The query rejects a recovery/read replica and a read-only session. The primary-state check is in the same SQL statement as the receipt lookup, so it shares the statement snapshot with the row being verified. The verifier is expected to connect to the authoritative writer endpoint.

An unavailable CA, TLS handshake, database connection, query or primary check is verification failure and cannot construct a VerifiedOwnerDisposition.

## 5. Durability boundary

A committed row visible on the writable primary proves that the row committed on that server. It does **not**, by itself, prove that an asynchronous replica eligible for promotion contains the commit.

The integration fixture performs the effect inside an explicit transaction with `SET LOCAL synchronous_commit = on` before invoking the existing fenced effect function. The verifier rejects read-only/recovery nodes.

Production qualification must separately pin:
- PostgreSQL synchronous_commit policy;
- synchronous_standby_names and standby acknowledgement policy;
- replica topology and WAL durability;
- failover/promotion rules;
- connection routing after primary change;
- backup/restore high-water recovery for owner epoch and generation.

This verifier's successful SELECT is not a substitute for proving that the configured failover policy preserves the commit.

## 6. Supported dispositions

This first PostgreSQL verifier supports **EffectCommitted only**.

The current reference ledger does not store durable terminal rejection, terminal no-effect or reconciliation-handoff records. The verifier therefore rejects those disposition variants rather than allowing them through a generic receipt path. They need explicit append-only records and their own canonical binding/verification rules before they can permit ACK.

An already-committed effect can be safely resolved from its exact row and returned receipt. That resolution is not a new authorization to execute another mutation.

## 7. Integration test and CI

The real PostgreSQL/NATS integration test:
1. connects to a disposable PostgreSQL primary using a generated test CA and hostname-verified TLS;
2. creates a unique effect fence and protected balance;
3. installs a worker claim through the existing SQL function;
4. calls `apply_claim_fenced_balance_delta` inside a transaction with `synchronous_commit = on`;
5. commits the actual effect and append-only ledger row;
6. asks the PostgreSQL verifier to load the exact row;
7. checks negative controls for wrong resource, request, decision, payload and generation;
8. publishes the command to JetStream and receives it through the durable pull consumer;
9. uses the verified PostgreSQL receipt capability to request AckSync for the exact delivery.

The base PostgreSQL receipt-verifier workflow provisioned PostgreSQL 16.16 with TLS, applied migrations 001/002 and started NATS 2.15.3. The owner-epoch follow-on adds migration 003, executes its SQL adversarial script, and changes the PostgreSQL/NATS integration to require the resulting execution-attestation row. That follow-on still requires its own exact-head run.

## 8. Benchmarks

The expanded benchmark companion distinguishes PostgreSQL verification from broker latency. It measures:
- receipt lookup p50/p95/p99;
- database connection/pool wait;
- exact scope/index lookup latency;
- effect receipt digest recomputation cost;
- missing/mismatch/replica/readonly rejection time;
- rate of unverified owner dispositions that reach ACK (hard target zero);
- false accepted mismatched receipt rows (hard target zero).

The performance corpus remains preregistered until CI and a repeatable open-loop PostgreSQL benchmark run. The real integration test is a correctness check, not a throughput benchmark.

## 9. Explicit limitations

- No PostgreSQL-backed verifier result has been executed yet.
- No production PostgreSQL credentials, client certificate, CA rotation or service discovery policy is qualified.
- The test uses a generated CA and a superuser on a disposable database; production should use a dedicated read-only receipt-verifier role and tightly scoped database grants, while still routing to the authoritative primary.
- The pre-migration effect_commits row did not persist owner epoch or worker-claim history. Migration 003 adds effect_execution_attestations and the verifier follow-on joins it with effect_commits and the owner-epoch transition ledger. This owner-epoch implementation remains unqualified until its own exact-head workflow passes.
- Synchronous commit in the test does not qualify synchronous replication, failover or promotion.
- Terminal rejection, terminal no-effect and reconciliation handoff are unsupported until they have their own durable receipt tables.
- No clustered NATS, PostgreSQL failover, latency, throughput or SOTA result is claimed.

The next gates are the exact-head TLS/PostgreSQL + JetStream CI run, real PostgreSQL failover/replication schedules, and a production least-privilege verifier role.


## 10. Owner epoch and worker-claim execution attestations

The companion owner-epoch SQL branch `research/myc-dir-owner-epoch-sql-v0-1` adds migration 003. It adds a local monotonic owner_epoch to the effect fence domain, an append-only owner epoch transition ledger and an append-only effect_execution_attestations table.

The strict queue-backed SQL function `apply_owner_epoch_claim_fenced_balance_delta` locks the current worker claim and owner fence in a consistent order, checks the exact current owner epoch/resource generation, calls the existing claim-fenced effect function and writes the actual worker identity, claim epoch, claim proof commitment, claim clock profile/validity, owner epoch, generation, and transaction ID in the same transaction as the protected effect.

Exact replay returns the original effect and attestation rather than creating a second effect. The strict path rejects legacy effects that have no execution attestation. It also rejects a replay position newer than the current command position.

The PostgreSQL verifier now joins all three append-only records and includes owner epoch, worker claim coordinates, database transaction ID and the epoch-authority/physical-fencing receipt commitments in the canonical receipt digest. It requires a matching transition record for non-initial owner epochs. These receipt commitments are persisted, not cryptographically authenticated by SQL or by the verifier; the external authority must verify signatures and physically fence the prior primary.

The workflow applies migration 003 and executes the SQL adversarial test script before the Rust/NATS integration suite. The integration fixture advances to owner epoch 2 / generation 2 and then creates the protected effect using the strict function, so the live verifier test exercises a non-initial epoch transition.

**Status:** source and test harness are present; this owner-epoch branch has not yet completed its exact-head migration/Rust/NATS CI run. The earlier NATS-only green run does not qualify these changes. Split-brain fencing, synchronous replication/promotion, production role grants, and performance remain separate qualification gates.


## 11. Worker-claim epoch binding

The owner-epoch SQL extension also adds `worker_claim_state.owner_epoch` and `install_worker_claim_for_owner_epoch`. An existing worker claim cannot cross an owner-epoch transition with the same claim epoch: it must be reinstalled under the current owner epoch with a strictly higher claim epoch. The strict mutation function verifies the claim's recorded owner epoch equals the current fence owner epoch. The installer uses the same claim-row-first, owner-fence-row-second lock order as the effect path to avoid a lock-order inversion.
