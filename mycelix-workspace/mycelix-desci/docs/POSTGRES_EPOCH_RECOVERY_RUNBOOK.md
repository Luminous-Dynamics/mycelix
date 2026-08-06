# PostgreSQL Epoch Promotion and Recovery Runbook

This runbook supplements `POSTGRES_RECOVERY_RUNBOOK.md`. It describes the
governed authority steps around promotion; normal PostgreSQL backup, fencing,
replication, and WAL procedures still apply.

## Preconditions for every promotion

1. Stop lease renewal to the old writer and fence it at the infrastructure layer.
2. Keep the API outbox publisher and all authority writers quiesced; wait for or
   explicitly verify strict expiration of the old lease before admitting the
   successor.
3. Run `migrations/verify_postgres_authority.sql` against the candidate.
4. Confirm the candidate's PostgreSQL system identifier, timeline, and replay
   LSN from trusted administrative tooling.
5. Compare its credential and governance heads with the most recent witnessed
   checkpoint expected for this recovery point.
6. Retrieve a fresh authority state commitment while the authority barrier is
   exclusive.
7. Create a promotion intent using that checkpoint and the exact candidate
   metadata.
8. Execute the intent through critical threshold governance.
9. Sign the epoch certificate with the dedicated database-epoch key.
10. Obtain a higher-generation candidate lease authorizing only
    `database_epoch_promotion` and binding the exact candidate certificate,
    system identifier, timeline, epoch number, and epoch hash.
11. Submit the certificate before authority state changes.
12. After commit, replace the candidate lease with a higher-generation normal
    epoch lease for the new primary and required operation scopes.

A stale state commitment fails rather than being adapted or repaired.

## Initial SQL activation

- Import and validate the credential and governance histories.
- Verify all current authority streams and receipts.
- Publish and independently retain a transparency checkpoint.
- Govern `InitialActivation` for epoch one.
- Submit the signed certificate under the candidate promotion lease.
- Replace it with an active epoch-one lease for ordinary writes.
- Publish the linked outbox delivery under an `outbox_delivery` scope.
- Obtain the configured organization-diverse delivery acknowledgements.
- Admit writers only after readiness passes.

## Planned failover

- Record the previous epoch hash in the new intent.
- Use the candidate's actual system identifier, promoted timeline, and replay
  LSN.
- Ensure the epoch number is exactly one greater than the current epoch.
- Do not include an emergency ceremony or recovery target.
- After commit, issue and atomically distribute the successor's normal epoch
  lease, then confirm the epoch chain, lease identity, and signed publication
  through the read API before re-enabling traffic.
- Keep the old primary isolated until its last lease is strictly expired.

## Disaster recovery and PITR

1. Declare an incident and open a bounded emergency ceremony.
2. Name at least two governance participants.
3. Select an explicit PITR target at or before the incident declaration.
4. Restore into isolation and verify relational and cryptographic replay.
5. Compare the restored prefix with the latest independently witnessed
   checkpoint at or before the target.
6. Govern the exact `DisasterRecovery` intent. At least two named ceremony
   participants must also approve the proposal.
7. Issue a candidate promotion lease bound to the exact recovery certificate,
   then submit the signed epoch certificate during the ceremony window.
8. Replace it with the new active epoch lease, then create and sign an exact
   recovery reconciliation covering:
   - epoch hash and number;
   - operator;
   - primary and database system identifier;
   - PostgreSQL timeline and replay LSN;
   - requested recovery target; and
   - the complete observed authority state commitment.
9. Publish and independently acknowledge both the epoch and reconciliation
   deliveries.
10. Admit writers only after readiness confirms reconciliation.

Do not recreate missing pre-target events with new IDs or receipt times. Treat
externally observed but absent events as an explicit governed reconciliation
problem.

## Failure handling

- **State changed before epoch commit:** discard the certificate, obtain a new
  commitment, and repeat governance for the new exact intent.
- **Epoch delivery missing or altered:** keep readiness blocked; restore the
  database from a consistent backup rather than editing the row.
- **Epoch key suspected compromised:** fence writes, revoke operational access,
  preserve evidence, and govern a new promotion with a distinct key.
- **Acknowledgement witness compromised:** record the governed witness
  compromise interval. Do not delete historical signatures.
- **Two primaries claim the same successor epoch:** stop lease renewal,
  isolate both writers, and treat the event as a split-brain incident. Neither
  branch should be manually renumbered or hash-edited.
- **Lease issuer unavailable:** do not bypass fencing. Existing writes stop at
  strict expiry; retain read-only access while restoring issuer authority.

## Evidence to retain

Retain the governance proposal and approvals, epoch certificate, recovery
reconciliation, signed outbox deliveries, witness acknowledgements, PostgreSQL
system/timeline/LSN evidence, checkpoint documents, database verification
output, infrastructure fencing logs, and ceremony record as one incident
package.
