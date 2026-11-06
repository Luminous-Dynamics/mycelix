# PostgreSQL verifier challenge registry — schema slice

Tracks Mycelix identity issue #5746.

## Decision

PostgreSQL is the authoritative consume-once coordinator. Holochain/DHT records
may carry signed audit projections, but must not decide whether a challenge is
still consumable. An eventually consistent DHT lookup or replica read is not
an atomic one-time operation.

This migration adds a digest-only challenge table bound to verifier ID,
audience, transaction, request digest, holder-binding digest, issue time and
expiry; duplicate-digest rejection during registration; a SECURITY DEFINER
registration function; a single conditional UPDATE as the consume
serialization point; and public-role revocation for direct table/function
access.

The verifier generates 256 random bits from an OS CSPRNG, constructs the wire
challenge, computes a domain-separated SHA-256 digest over the exact challenge
bytes, and persists only the digest. The database enforces a maximum challenge
lifetime of ten minutes at both registration and row-constraint levels. The raw
challenge must never be logged or written to the database.

## Required call sequence

1. Generate an unpredictable challenge with at least 256 random bits.
2. Bind it to verifier, audience, transaction/session, exact request digest,
   holder-binding digest and a short expiry.
3. Call register_verifier_challenge. A false result means collision,
   malformed input or invalid expiry; generate a new challenge and do not
   issue the failed value.
4. Return the raw challenge to the holder in the signed presentation request.
5. Verify the holder signature and exact challenge/request/context equality.
6. Call consume_verifier_challenge on the authoritative primary inside a
   database transaction.
7. Commit that transaction successfully before returning authentication
   acceptance to the caller. A TRUE function result inside an uncommitted
   transaction is not yet a durable consume event.
8. Accept authentication only if the transaction commit is confirmed. False,
   timeout, SQL error, serialization failure, ambiguous commit result, failover
   uncertainty or database outage is authentication failure/unknown; there is
   no equality-only fallback.

The UPDATE itself is the atomic consume-once decision. Do not implement this as
a SELECT followed by an UPDATE. A retry after a lost verifier response must
not re-accept the consumed challenge; the verifier issues a fresh challenge.

## Deployment requirements before high-assurance use

- Provision a dedicated runtime role with EXECUTE on the register/consume
  functions and no direct UPDATE or DELETE privilege on the table.
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

This is a schema and stored-procedure contract only. It does not add a running
service, SQLx adapter, API authentication, deployment, or independent
concurrency-test result. No Cargo or PostgreSQL test was run in the authoring
environment. Issue #5746 remains open until the service, cross-process race
test, failover guarantees and verifier integration are implemented and
reviewed.
