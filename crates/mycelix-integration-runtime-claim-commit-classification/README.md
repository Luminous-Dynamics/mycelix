# Runtime claim commit classification

This crate is the #756 Tier-A transaction owner for the legacy v2 runtime's combined expired-lease recovery + outbox claim commit.

It deliberately does **not** call `SqliteIntegrationStore::claim_outbox()`. Instead it opens the independently provisioned exact runtime store without schema creation/migration, starts one `IMMEDIATE` transaction, captures complete durable witnesses inside that transaction, performs the same bounded recovery/claim semantics, captures the exact candidate successor, and then commits.

If `COMMIT` returns an error, the writer is dropped before an independent read-only connection reopens the exact provisioned store. One deferred read snapshot must prove either the complete candidate successor or the complete predecessor; anything else is indeterminate.

## Capability boundary

A normal successful commit may return `ExecutionClaim` metadata, which the runtime already defines as non-authoritative claim metadata.

A post-error durable successor never reconstructs those claim objects. It returns only `RecoveredAttemptIdentity` values for recovery/reconciliation. An exact predecessor establishes persistence retry eligibility only; `ordinary_retry_authorized_here()` remains false.

## Witness scope

For every transaction-touched entry the witness binds the complete v2 outbox row plus execution-observation count, maximum observation ID, and the complete maximum observation row. The schema and relevant SQLite pragmas are exact-checked, and triggers on the two tables are rejected.

This covers the important same-transaction cases where an expired `AttemptPrepared` row is requeued and immediately reclaimed, while an expired `DispatchStarted` row becomes `Ambiguous` and appends one execution observation.

## Non-claims

This crate performs no provider/network I/O, does not grant execution authority, does not infer business outcomes, and does not make `DefinitelyNotCommitted` equivalent to permission to retry. Later runtime callers must still satisfy lease/currentness/authority policy.
