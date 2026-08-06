# PostgreSQL Authority Recovery Runbook

> **Schema-v4 note:** database promotion is a governed epoch transition and every authority mutation is externally lease-fenced. Follow `POSTGRES_EPOCH_RECOVERY_RUNBOOK.md` and `AUTHORITY_WRITE_FENCING.md` in addition to this database restore procedure.
>
> **v0.9 upgrade precondition:** drain or explicitly publish every pending schema-v1 outbox row before applying schema v2. Those rows predate signed delivery envelopes and cannot be retroactively signed without changing their historical acceptance semantics. Delivered v1 rows may remain as historical transport records.

## Backup requirements

Back up the entire authority database as one consistency domain. Do not restore
scientific events, credentials, governance, receipts, mirrors, or outbox tables
from different recovery points. Archive WAL continuously and record the exact
PostgreSQL timeline and LSN associated with each exported transparency
checkpoint.

## Restore validation

1. Restore into an isolated database with no API or publisher writers.
2. Apply only the repository migrations matching the deployed binary.
3. Run `migrations/verify_postgres_authority.sql` with `ON_ERROR_STOP=1`.
4. Keep all authority writers fenced. Apply migrations only under a current
   bootstrap or epoch-matched `schema_migration` lease.
5. Start one API replica with webhook delivery disabled and a read-only or
   narrowly scoped lease.
6. Require startup replay to pass; readiness should remain blocked until the
   restored primary has a governed epoch, exact reconciliation where required,
   and a normal active-epoch lease.
7. Compare the restored credential, governance, and scientific heads with the
   latest independently witnessed checkpoint at or before the target recovery
   point.
8. Inspect pending outbox rows. Subscribers must deduplicate replayed deliveries.
9. Govern and record the appropriate database epoch for the restored primary.
10. For disaster recovery, record exact signed recovery reconciliation.
11. Obtain the configured independent acknowledgements of the immutable epoch publication.
12. Issue and atomically distribute a higher-generation epoch-bound write lease. Only then admit writers and enable the outbox publisher.

## Failover invariant

A promoted replica must expose a database state from one prefix of the committed
WAL history. If credential and governance heads do not match the checkpoint
prefix expected for that timeline, keep readiness blocked. Never repair a gap by
editing hashes, JSON, sequence numbers, receipt status, lease generation, or
fencing state manually. A restored former primary must not receive a renewed
lease until promotion is governed and its exact timeline and epoch are verified.

## Point-in-time recovery

PITR may legitimately remove commits after the selected target. Publish a new
checkpoint after promotion and explicitly identify the prior checkpoint and
recovery target. Events observed externally but absent after PITR require a
new governed reconciliation process; they must not be silently recreated with
new receipt times.
