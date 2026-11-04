# PostgreSQL verifier challenge registry — schema slice

Tracks Mycelix identity issue #5746.

## Decision

PostgreSQL is the authoritative consume-once coordinator. Holochain/DHT records
may carry signed audit projections, but must not decide whether a challenge is
still consumable. An eventually consistent DHT lookup or replica read is not
an atomic one-time operation.

This migration adds a digest-only challenge table bound to verifier ID,
audience, transaction, request digest, holder-binding digest, issue time and
expiry computed by PostgreSQL from a bounded TTL; duplicate-digest rejection
during registration; a SECURITY DEFINER registration function; a single
conditional UPDATE as the consume serialization point; and public-role
revocation for direct table/function access.

The verifier generates 256 random bits from an OS CSPRNG, constructs the wire
challenge, computes a domain-separated SHA-256 digest over the exact challenge
bytes, and persists only the digest. The database enforces a maximum challenge
lifetime of ten minutes at both registration and row-constraint levels. The raw
challenge must never be logged or written to the database.

## Required call sequence

1. Call the native SQLx adapter's issue method with verifier ID, audience,
   transaction ID, exact request digest, holder-binding digest and a TTL from
   one through 600 seconds.
2. The adapter generates 256 unpredictable bits using the OS CSPRNG, computes
   a domain-separated SHA-256 digest, and registers that digest. PostgreSQL
   computes issued_at and expires_at from its own clock and the bounded TTL.
3. Return the raw challenge to the holder in the signed presentation request.
   The raw value remains in memory only and is redacted from Debug output.
4. Verify the holder signature and exact challenge/request/context equality
   before calling the consume method.
5. Call consume_and_commit on the authoritative primary. The adapter begins a
   transaction, invokes the atomic conditional UPDATE and commits only when
   the function returns true.
6. Accept authentication only after COMMIT is acknowledged. A false function
   result is a rejection. A COMMIT transport error is an unknown outcome and
   requires a fresh challenge; the old challenge is never retried.
7. Database outage, serialization failure, ambiguous commit, failover
   uncertainty or an unavailable primary is authentication failure/unknown.
   There is no equality-only or local-cache fallback.

The UPDATE itself is the atomic consume-once decision. Do not implement this as
a SELECT followed by an UPDATE. A retry after a lost verifier response must
not re-accept the consumed challenge; the verifier issues a fresh challenge.

## Native SQLx adapter

The separate native service crate lives at
`mycelix-identity/challenge-registry/service`. It is explicitly excluded from
the Holochain WASM workspace so SQLx and native TLS/runtime dependencies do not
leak into zome builds.

The adapter:
- uses the OS CSPRNG for 256-bit challenges;
- stores only SHA-256 over the versioned domain separator and exact 32-byte
  challenge material;
- keeps raw material in a non-serializable, redacted-Debug, zeroize-on-drop
  wrapper;
- validates verifier, audience, transaction ID, request digest, and
  holder-binding digest before database calls;
- restricts TTL to whole seconds from 1 to 600, with PostgreSQL computing the
  issue and expiry timestamps from its own clock;
- commits a successful consume in an explicit SQLx transaction before
  returning true;
- maps a commit transport failure to CommitOutcomeUnknown and never retries
  the same challenge.

The caller remains responsible for verifying the holder signature and exact
presentation bindings before invoking consume_and_commit. The adapter is not
a signature verifier and does not change the V1 presentation verifier by
itself.

The exact-head workflow runs against PostgreSQL 18.6. It applies the migration
as an administrator, then provisions a dedicated non-login function owner and
a separate runtime login. It asserts that the runtime has no direct table
privileges and runs all SQL/SQLx contract tests through the runtime role. It applies the migration,
runs the sequential SQL contract, starts the two-process psql race, runs the
Rust unit suite, runs an additional race across two independent SQLx pools,
and runs warnings-denied Clippy.

## Deployment requirements before high-assurance use

- Provision a dedicated non-login function-owner role that owns the schema and
  table, plus a separate runtime login role with schema USAGE and EXECUTE on
  the register/consume functions only. The runtime role must have no direct
  SELECT, INSERT, UPDATE, DELETE or TRUNCATE privilege on the table.
- Use exactly one writable primary; do not consume from read replicas.
- Configure failover so an acknowledged consume commit cannot be lost. An
  asynchronously replicated, potentially data-losing promotion is not an
  acceptable high-assurance failover mode.
- Prevent split-brain / dual-primary operation.
- Treat a registry timeout as an unknown consume result and fail closed.
- Keep digest tombstones. Expired or consumed challenge records must not be
  deleted if that would permit a previously seen digest to be registered again.
- Define an explicit retention/archive scheme that preserves uniqueness
  tombstones even if nonessential context fields are compacted.
- Audit only opaque challenge references and result reason codes, never raw
  challenges or presentation bearer material.

PostgreSQL atomicity is not a deployment proof. Use one writable primary and
consume at the primary only. At READ COMMITTED, the conditional UPDATE is the
serialization point for concurrent consumers of the same row; at stronger
isolation levels, serialization failures must fail closed. The service must
not return success until the transaction commit is acknowledged. If the
commit call times out, the caller must treat the result as unknown and require
a fresh challenge; it must not retry the old challenge expecting acceptance.

The actual service must test its primary/failover configuration, transaction
isolation, role permissions, backup/restore behavior, acknowledged-commit
survival and exactly-one-winner behavior across independent processes.

## Required tests

Run the sequential SQL contract test after applying the migration:

    psql "$DATABASE_URL" -v ON_ERROR_STOP=1 \
      -f mycelix-identity/challenge-registry/migrations/0001_verifier_challenges.sql
    psql "$DATABASE_URL" -v ON_ERROR_STOP=1 \
      -f mycelix-identity/challenge-registry/tests/consume_once.sql

Run the two-process race test with a disposable test database and the
migration applied:

    DATABASE_URL="$DATABASE_URL" \
      sh mycelix-identity/challenge-registry/tests/concurrent_consume.sh

The race script creates one random challenge digest and starts two separate
psql processes that contend on the same challenge. It requires exactly one
successful consume. It is an integration test of a running PostgreSQL primary,
not proof of behavior under failover or a split-brain deployment.

1. Valid challenge registers once; duplicate digest registration is rejected.
2. Correct context consumes once; sequential replay returns false.
3. Wrong verifier, audience, transaction, request digest or holder-binding
   digest returns false without consuming the row.
4. Expired, overlong-lifetime and unknown digests return false.
5. Two independent verifier processes racing on one digest produce exactly
   one true result.
6. A lost HTTP/API response after a successful commit does not make retry
   return true.
7. Restart and failover tests demonstrate that consumed tombstones survive.
8. Registry outage and ambiguous commit outcomes never fall back to local
   challenge equality.
9. Runtime role cannot update, delete, or reset consumed rows directly.
10. Logs, traces and errors contain no raw challenge material.

## Limits

This is a schema/stored-procedure contract plus a separately excluded native SQLx adapter. It does not add API authentication, deployment, or failover qualification. The PostgreSQL 18.6 exact-head workflow is the executable qualification gate; no database test is claimed before that workflow passes. Issue #5746 remains open until the service, cross-process race test, failover guarantees and V1 verifier integration are reviewed.
