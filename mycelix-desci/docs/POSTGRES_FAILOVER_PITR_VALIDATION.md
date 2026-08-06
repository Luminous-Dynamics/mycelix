# Deterministic PostgreSQL Failover and PITR Validation

This campaign validates the combined database-epoch and external write-fencing
protocol. It is a required operator exercise, not evidence that the repository
has already passed live PostgreSQL testing.

## Required evidence capture

For every scenario retain:

- signed old and new write leases;
- database epoch and reconciliation records;
- PostgreSQL system identifier, timeline, and LSN;
- API readiness output before, during, and after the transition;
- attempted mutation request and exact rejection or acceptance;
- authority-outbox publication and acknowledgements; and
- database and API logs with synchronized UTC timestamps.

## Scenario 1 — Strict expiration

1. Start the active primary with a lease expiring in no more than 15 minutes.
2. Verify an allowed scientific mutation succeeds before expiration.
3. Stop lease renewal without stopping PostgreSQL or the API.
4. At `expires_at`, retry all mutation classes.
5. Confirm every mutation fails even when clock-skew allowance is configured.
6. Confirm read-only projection and audit exports remain available.

Pass criterion: no authority mutation commits at or after `expires_at`.

## Scenario 2 — Partitioned former primary

1. Begin with primary A and a governed epoch-bound lease.
2. Isolate A from the lease issuer and promotion control plane while keeping its
   local API and database reachable from a test client.
3. Promote B, creating a higher database timeline and governed epoch.
4. Issue B a higher-generation lease; do not renew A.
5. Attempt writes against A before and after its old lease expires.

Pass criterion: A may write only inside the explicitly bounded old lease window;
after expiration it rejects every authority scope. B accepts writes only after
its exact candidate and active-epoch leases are valid.

## Scenario 3 — Wrong timeline and system identity

Present otherwise valid leases with:

- the previous timeline;
- a different PostgreSQL system identifier;
- the correct epoch but wrong primary identifier; and
- the correct database identity but wrong deployment identifier.

Pass criterion: each lease is rejected before domain mutation.

## Scenario 4 — Generation rollback and fork

1. Persist generation N through a successful write.
2. Present generation N-1.
3. Present a different signed lease using generation N.
4. Present generation N+1 with correct bindings.

Pass criterion: N-1 and the forked N fail; N+1 succeeds.

## Scenario 5 — Scope isolation

Use leases that omit one scope at a time. Exercise scientific events,
credentials, governance, epoch promotion, recovery, acknowledgements, mirrors,
outbox delivery, and schema migration.

Pass criterion: only listed scopes commit. Runtime migration without
`schema_migration` fails before DDL.

## Scenario 6 — Planned failover

Execute the complete planned-failover ceremony from
[Epoch Promotion and Recovery Runbook](POSTGRES_EPOCH_RECOVERY_RUNBOOK.md).
Confirm the candidate promotion lease binds the exact future epoch hash, then
rotate to a normal active-epoch lease after commit.

Pass criterion: readiness never reports healthy with a lease bound to the old
primary, old timeline, or prior epoch.

## Scenario 7 — PITR and disaster recovery

1. Restore to the governed recovery target.
2. Confirm ordinary writes are blocked.
3. Execute the disaster-recovery epoch ceremony.
4. Record exact recovery reconciliation.
5. Issue the new epoch-bound lease.
6. Publish and independently acknowledge recovery evidence.

Pass criterion: no authority mutation occurs before promotion, reconciliation,
and correct lease activation. Restored state matches the signed checkpoint and
epoch commitment.

## Scenario 8 — Lease issuer and remote signer outage

Independently interrupt:

- the lease issuer or lease-file rotation;
- the Unix-domain authority signer; and
- the publication endpoint.

Pass criterion: writes stop at lease expiration, signature-dependent commits
fail closed immediately, and committed outbox records remain durable for later
at-least-once delivery.

## Validation order

1. Automated protocol and SQL integration tests.
2. Deterministic containerized failover and PITR scenarios.
3. Captured state/hash comparison and log review.
4. Owner-operated recovery drill.

Do not promote the backend based on static patch replay alone.
