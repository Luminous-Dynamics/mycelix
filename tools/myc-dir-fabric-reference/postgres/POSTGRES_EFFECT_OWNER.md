# PostgreSQL effect-owner reference adapter

**Status:** executable SQL source and transactional self-test authored; no PostgreSQL server was available here, so SQL parse, migration, integration, concurrency and failover tests have not run. This is not a production migration.

## Reuse the existing PostgreSQL control plane

Mycelix-DeSci already has a PostgreSQL authority backend with serializable event/receipt/outbox commits, externally signed database-write leases, governed database epochs, recovery reconciliation and an at-least-once publication outbox. Directory Fabric should reuse those controls and add a narrower resource-effect fence. It must not create another database authority, lease issuer, epoch chain or publication transport.

Relevant existing implementation:
- mycelix-workspace/mycelix-desci/src/core/src/postgres_authority.rs
- mycelix-workspace/mycelix-desci/docs/POSTGRES_AUTHORITY_BACKEND.md
- mycelix-workspace/mycelix-desci/docs/AUTHORITY_WRITE_FENCING.md
- mycelix-workspace/mycelix-desci/docs/AUTHORITY_DATABASE_EPOCHS.md

The existing deployment lease fences who may write to the PostgreSQL authority database. The new owner-local generation fences whether one exact resource mutation remains current. These are distinct scopes and both must hold where applicable.

## Transaction contract

For one resource-owner mutation:

1. The PEP verifies the signed decision, exact ALLOW outcome, request/realm/policy/cut binding, closure proof and effect-owner scope before entering SQL. PostgreSQL stores the decision-receipt digest; this reference SQL does not verify Ed25519 or infer that a digest means Allow.
2. Begin a short transaction and set synchronous_commit = on before committing any protected effect or fence advance.
3. Acquire transaction-scoped advisory locks for the idempotency-key digest and logical-operation identifier in sorted order. This prevents concurrent retries or operation-ID substitutions from racing. Hash collisions can only add contention.
4. Acquire SELECT ... FOR SHARE on the exact fence-domain row. Multiple effect transactions can hold this shared lock concurrently.
5. Look up exact idempotency/logical-operation history. An exact replay returns only the original immutable effect receipt identity; it does not mutate again or disclose the current protected object value.
6. For a new operation, require supplied generation = current durable generation.
7. Update the protected object, append its effect/idempotency receipt and commit all changes in the same SQL transaction.
8. Return success only after COMMIT is confirmed. If the connection is lost during COMMIT, outcome is unknown: reconnect to the authoritative primary/epoch and resolve by the same idempotency key. Never invent a new key or retry as a new logical operation.

The example applies an integer-minor-unit balance delta. It demonstrates a real database-local mutation and immutable operation receipt in one transaction. Replace that DML with application-specific tables only while keeping the same lock order and atomic boundary.

## Why shared lock for effects, exclusive lock for revocation?

A mutation holds FOR SHARE on the fence row until its transaction ends. Other mutations can acquire the same shared lock, so they do not serialize on the fence row. Revocation updates the generation under an exclusive row lock; it conflicts with in-flight shared locks and waits for earlier effect transactions to finish.

This creates a linearization point:

- If the effect obtains the shared lock first, it may commit before the revocation advances.
- If revocation obtains the exclusive lock first, the effect waits. Under PostgreSQL READ COMMITTED, SELECT FOR SHARE locks and returns the updated row version after the concurrent update commits, then rejects the old generation.
- The revocation acknowledgement is not emitted at request time or function return. It is emitted only after the fence transaction itself has committed durably.

PostgreSQL documents the relevant row-lock compatibility and updated-row behavior in the official explicit-locking and transaction-isolation manuals:
https://www.postgresql.org/docs/18/explicit-locking.html
https://www.postgresql.org/docs/current/transaction-iso.html

Use READ COMMITTED for the shown adapter path, or handle 40001 serialization failures by retrying the entire transaction with the same idempotency identifiers under higher isolation. Lock ordering must remain consistent: idempotency locks → fence row → protected object → append-only receipt. Do not call arbitrary external services or wait for user/network input while holding these locks.

## Revocation transaction and acknowledgement

    BEGIN;
    SET LOCAL synchronous_commit = on;

    SELECT myc_effect_owner.advance_effect_fence(
        :realm_id, :resource_id, :action_id, :effect_owner_id,
        :revocation_digest
    );

    COMMIT;
    -- Only after COMMIT success: sign and publish the owner acknowledgement.

The advance function is idempotent for an exact revocation digest. A retry returns its original installed generation. Acknowledgement must bind the exact scope, revocation digest, generation and durable-state commitment.

For a strict failover guarantee, local WAL durability alone is insufficient if the deployment may promote a lagging asynchronous replica. Configure synchronous replication and promotion rules so every node eligible to become primary preserves the acknowledged fence generation; recover under the existing governed database-epoch protocol. PostgreSQL documents that asynchronous commit may report success before WAL is durable and can lose recent transactions after a crash, while synchronous commit waits for WAL flush under its configured durability mode:
https://www.postgresql.org/docs/16/wal-async-commit.html
https://www.postgresql.org/docs/current/runtime-config-wal.html

## External effects and the outbox boundary

A PostgreSQL transaction can atomically fence a PostgreSQL-local mutation and insert an outbox row. It cannot atomically commit an arbitrary remote API side effect. Mycelix-DeSci's outbox is at-least-once; downstream consumers must deduplicate. For a strong remote revocation guarantee, the receiving service must itself enforce a monotonic owner-local fence and idempotency at its own mutation boundary. A delivery acknowledgement alone proves transport observation, not downstream effect or continued rejection of stale commands.

A non-fenceable API remains explicitly best-effort and cannot be counted as strongly revoked.

## Files and execution

- postgres/001_effect_owner.sql: schema, owner-local revocation advance, shared-lock effect function, append-only effect receipt.
- postgres/test_effect_owner.sql: transactional single-session self-tests. The test transaction rolls back.
- postgres/POSTGRES_EFFECT_OWNER.md: integration contract, crash semantics and manual concurrency schedule.

Apply and test on a disposable PostgreSQL database:

    psql "$TEST_DATABASE_URL" -v ON_ERROR_STOP=1 -f tools/myc-dir-fabric-reference/postgres/001_effect_owner.sql
    psql "$TEST_DATABASE_URL" -v ON_ERROR_STOP=1 -f tools/myc-dir-fabric-reference/postgres/test_effect_owner.sql

## Required two-session race tests (not covered by the single-session SQL test)

**Schedule A — effect wins:** Session A begins, invokes apply_balance_delta using generation N, then pauses before COMMIT. Session B invokes advance_effect_fence for the same domain. B must block. A commits. B advances to N+1 and commits. Only then may the owner acknowledgement be issued.

**Schedule B — revocation wins:** Session B advances and commits N+1 first. Session A submits a new command with N. A must observe N+1 after waiting and reject without modifying the protected object or appending an effect receipt.

**Schedule C — commit acknowledgement lost:** Commit an operation but drop the client connection before it receives the response. Retry the exact same idempotency key and payload. It must return the same effect ID and leave the balance unchanged. Repeat with a transaction killed before COMMIT: no effect or idempotency row may survive; if the token has since been fenced, retry must fail stale.

**Schedule D — concurrent independent writes:** Run different idempotency/logical-operation keys on different object_id rows within the same fence domain. They should overlap under compatible FOR SHARE locks. Measure lock wait and throughput; same-object balance updates serialize on their own row.

**Schedule E — crash/failover:** Crash the primary after fence transaction COMMIT but before acknowledgement publication; reconcile the exact generation on the promoted governed epoch before signing an acknowledgement. Repeat with an asynchronous-commit negative control: readiness must not claim durable revocation based on the server's early success response.

## Security and implementation limits

- This is an effect-owner exemplar, not a generic arbitrary-DML executor.
- The decision receipt and closure proof are verified by the trusted PEP, not by this SQL.
- The owner inventory must be independently complete. A missing resource route means no all-owner guarantee.
- Table/function grants must prevent application roles from directly updating protected balances, fence rows or effect history. After creating dedicated deployment roles, grant only:
  
      GRANT USAGE ON SCHEMA myc_effect_owner TO myc_effect_owner_pep, myc_effect_owner_revoker;
      GRANT EXECUTE ON FUNCTION myc_effect_owner.apply_claim_fenced_balance_delta(
        TEXT, TEXT, TEXT, TEXT, TEXT, TEXT, BIGINT, BYTEA, TEXT, BYTEA, BYTEA, BYTEA,
        BIGINT, TEXT, BIGINT, BYTEA, TEXT, TIMESTAMPTZ, TIMESTAMPTZ
      ) TO myc_effect_owner_pep;
      GRANT EXECUTE ON FUNCTION myc_effect_owner.install_worker_claim(
        TEXT, TEXT, TEXT, TEXT, TEXT, TEXT, BIGINT, BYTEA, TEXT, TIMESTAMPTZ, TIMESTAMPTZ
      ) TO myc_effect_owner_claim_installer;
      GRANT EXECUTE ON FUNCTION myc_effect_owner.advance_effect_fence(
        TEXT, TEXT, TEXT, TEXT, BYTEA
      ) TO myc_effect_owner_revoker;
  
  Do not grant table DML to either role. Keep migration/schema ownership separate from the PEP and revocation service identities.
- Function owners, migration roles, superusers and database operators remain trusted control-plane principals.
- Fence state must be initialized from a trusted bootstrap and recovered from a durable high-water mark. Never reset to generation 1 on restart or restore.
- This example does not implement cryptographic owner acknowledgements, a signature-verifier extension, backup/restore automation or multi-primary consensus.
- Remote effects require receiver-side fencing. The local SQL lock does not reach across a network.

Source-present is the current evidence state. Exact-head migration, SQL self-test, two-session race, crash recovery, synchronous-replica failover, privilege audit and production effect-owner integration are separate gates.


## Worker claim and decision-validity extension

Apply 002_worker_claim_fencing.sql after the base schema. It adds a durable worker_claim_state table keyed by exact realm/resource/action/effect-owner/logical-operation. Each row contains worker ID, monotonically increasing claim epoch, externally verified claim-proof digest, exact clock-profile ID and validity interval.

The trusted worker-claim issuer must authenticate a claim before the installer invokes install_worker_claim. Exact same-epoch replay is idempotent; lower epochs and same-epoch substitutions are rejected. The SQL installer does not verify the claim issuer's signature.

The effect path must call apply_claim_fenced_balance_delta, not the unwrapped apply_balance_delta. The wrapper:
1. Holds FOR SHARE on the exact logical operation's worker-claim row through transaction commit.
2. Rejects a worker/epoch/proof substitution.
3. Reads the effect owner's configured clock profile and maximum uncertainty.
4. Requires the worker claim and decision clock profiles to match the owner profile.
5. Requires the full owner-clock uncertainty interval to fall within both the worker-claim and Allow validity windows.
6. Calls apply_balance_delta under the same surrounding SQL transaction, retaining the claim and resource-fence locks until COMMIT.

Claim reassignment uses a strictly increasing epoch. It cannot commit while a previous effect transaction holds the old claim row FOR SHARE. Once reassignment commits, old claim tokens fail. This gives a durable ordering between current executor claim and PostgreSQL-local mutation. It does not fence a remote API or outbox consumer; those remain independent effect owners.

owner_clock_profile_id and max_clock_uncertainty_ms are added to effect_fence_state. The default clock profile and 25 ms uncertainty are illustrative bootstrap values, not universal safe settings. Operators must measure and qualify actual offset/uncertainty, monitor clock health, test restart/failover and disable time-bounded writes when the bound cannot be maintained. PostgreSQL clock_timestamp() is not a secure hardware clock.

The application PEP must still authenticate the exact Allow receipt and claim issuer before calling SQL. The SQL validates stored commitments and validity bindings; it does not derive policy or verify Ed25519. The PEP role should receive EXECUTE only on apply_claim_fenced_balance_delta; it must not have EXECUTE on the unwrapped apply_balance_delta or direct table DML. A separate trusted claim-installer role may execute install_worker_claim.

Run test_worker_claim_fencing.sql after applying both migrations. This is an authored single-session transactional self-test only; two-session claim-reassignment/effect races, clock drift, role-bypass, crash/recovery and synchronous failover remain runtime qualification requirements.


The base apply_balance_delta function accepts the exact decision clock profile, issued-at and expiry timestamps. It acquires advisory locks, the owner-fence row and the protected-object row before checking the full owner-clock uncertainty interval immediately before the write. Exact read-only idempotency replay remains available to reconcile an earlier commit; a new mutation outside the decision window is rejected. The queue-facing PEP role is granted only the worker-claim wrapper, not the unwrapped base mutation function.
