-- Read-only structural verification for Mycelix-DeSci PostgreSQL authority v4.
-- Run after restore, failover, or point-in-time recovery. Cryptographic replay
-- is still performed by API startup; this script catches relational damage
-- before the service is admitted.

BEGIN TRANSACTION READ ONLY;

DO $$
DECLARE
    bad_count BIGINT;
BEGIN
    IF NOT EXISTS (
        SELECT 1 FROM desci_authority_schema_migrations
        WHERE version = 4
          AND migration_id = 'mycelix-desci-authority-v4-2026-08-05'
    ) THEN
        RAISE EXCEPTION 'authority schema v4 marker is absent or incorrect';
    END IF;

    SELECT COUNT(*) INTO bad_count
    FROM (
        SELECT stream_id, sequence,
               lag(sequence) OVER (PARTITION BY stream_id ORDER BY sequence) AS prior
        FROM desci_scientific_events
    ) s
    WHERE (sequence = 0 AND prior IS NOT NULL)
       OR (sequence > 0 AND prior IS DISTINCT FROM sequence - 1);
    IF bad_count <> 0 THEN
        RAISE EXCEPTION 'scientific event stream sequence gaps: %', bad_count;
    END IF;

    SELECT COUNT(*) INTO bad_count
    FROM (
        SELECT sequence, lag(sequence) OVER (ORDER BY sequence) AS prior
        FROM desci_credential_events
    ) s
    WHERE (sequence = 0 AND prior IS NOT NULL)
       OR (sequence > 0 AND prior IS DISTINCT FROM sequence - 1);
    IF bad_count <> 0 THEN
        RAISE EXCEPTION 'credential sequence gaps: %', bad_count;
    END IF;

    SELECT COUNT(*) INTO bad_count
    FROM (
        SELECT sequence, lag(sequence) OVER (ORDER BY sequence) AS prior
        FROM desci_governance_events
    ) s
    WHERE (sequence = 0 AND prior IS NOT NULL)
       OR (sequence > 0 AND prior IS DISTINCT FROM sequence - 1);
    IF bad_count <> 0 THEN
        RAISE EXCEPTION 'governance sequence gaps: %', bad_count;
    END IF;

    SELECT COUNT(*) INTO bad_count
    FROM desci_authority_receipts r
    LEFT JOIN desci_scientific_events e ON e.event_id = r.event_id
    WHERE e.event_id IS NULL OR r.stream_id <> e.stream_id OR r.sequence <> e.sequence;
    IF bad_count <> 0 THEN
        RAISE EXCEPTION 'orphaned or misindexed authority receipts: %', bad_count;
    END IF;


    SELECT COUNT(*) INTO bad_count
    FROM (
        SELECT epoch_number, epoch_hash, previous_epoch_hash,
               lag(epoch_number) OVER (ORDER BY epoch_number) AS prior_number,
               lag(epoch_hash) OVER (ORDER BY epoch_number) AS prior_hash
        FROM desci_database_epochs
    ) e
    WHERE (epoch_number = 1 AND (prior_number IS NOT NULL OR previous_epoch_hash IS NOT NULL))
       OR (epoch_number > 1 AND (
            prior_number IS DISTINCT FROM epoch_number - 1
            OR previous_epoch_hash IS DISTINCT FROM prior_hash
       ));
    IF bad_count <> 0 THEN
        RAISE EXCEPTION 'database epoch chain gaps: %', bad_count;
    END IF;

    SELECT COUNT(*) INTO bad_count
    FROM desci_recovery_reconciliations r
    LEFT JOIN desci_database_epochs e ON e.epoch_hash = r.epoch_hash
    WHERE e.epoch_hash IS NULL;
    IF bad_count <> 0 THEN
        RAISE EXCEPTION 'orphaned recovery reconciliations: %', bad_count;
    END IF;

    SELECT COUNT(*) INTO bad_count
    FROM desci_database_epochs e
    LEFT JOIN desci_authority_outbox o ON o.id = e.publication_delivery_id
    WHERE e.accepted_at < e.promoted_at
       OR o.id IS NULL
       OR o.topic <> 'authority.database-epoch.recorded.v1'
       OR o.aggregate_id <> e.deployment_id
       OR o.aggregate_sequence <> e.epoch_number
       OR o.created_at <> e.accepted_at
       OR o.payload_hash IS NULL
       OR NOT (o.payload ? 'envelope' AND o.payload ? 'signer_public_key' AND o.payload ? 'signature');
    IF bad_count <> 0 THEN
        RAISE EXCEPTION 'missing or misindexed database epoch publications: %', bad_count;
    END IF;

    SELECT COUNT(*) INTO bad_count
    FROM desci_recovery_reconciliations r
    LEFT JOIN desci_database_epochs e ON e.epoch_hash = r.epoch_hash
    LEFT JOIN desci_authority_outbox o ON o.id = r.publication_delivery_id
    WHERE r.accepted_at < r.verified_at
       OR e.epoch_hash IS NULL
       OR o.id IS NULL
       OR o.topic <> 'authority.recovery-reconciliation.recorded.v1'
       OR o.aggregate_id <> encode(r.epoch_hash, 'hex')
       OR o.aggregate_sequence <> e.epoch_number
       OR o.created_at <> r.accepted_at
       OR o.payload_hash IS NULL
       OR NOT (o.payload ? 'envelope' AND o.payload ? 'signer_public_key' AND o.payload ? 'signature');
    IF bad_count <> 0 THEN
        RAISE EXCEPTION 'missing or misindexed recovery publications: %', bad_count;
    END IF;

    SELECT COUNT(*) INTO bad_count
    FROM desci_authority_delivery_acknowledgements a
    LEFT JOIN desci_authority_outbox o ON o.id = a.delivery_id
    WHERE a.accepted_at < a.acknowledged_at
       OR o.id IS NULL
       OR o.payload_hash <> a.delivery_hash
       OR o.published_at IS NULL;
    IF bad_count <> 0 THEN
        RAISE EXCEPTION 'orphaned or mismatched delivery acknowledgements: %', bad_count;
    END IF;

    SELECT COUNT(*) INTO bad_count
    FROM desci_authority_outbox
    WHERE payload IS NULL OR octet_length(payload_hash) <> 32;
    IF bad_count <> 0 THEN
        RAISE EXCEPTION 'invalid authority outbox rows: %', bad_count;
    END IF;

    SELECT COUNT(*) INTO bad_count
    FROM desci_authority_write_fencing_state f
    WHERE f.generation <= 0
       OR octet_length(f.lease_hash) <> 32
       OR octet_length(f.signer_public_key) <> 32
       OR f.postgres_timeline <= 0
       OR f.expires_at <= f.last_seen_at
       OR NOT (
            f.lease_json ? 'lease'
            AND f.lease_json ? 'signer_public_key'
            AND f.lease_json ? 'signature'
       )
       OR (f.phase = 'bootstrap' AND (f.epoch_number IS NOT NULL OR f.epoch_hash IS NOT NULL))
       OR (f.phase = 'epoch' AND (f.epoch_number IS NULL OR octet_length(f.epoch_hash) <> 32));
    IF bad_count <> 0 THEN
        RAISE EXCEPTION 'invalid authority write-fencing rows: %', bad_count;
    END IF;

    SELECT COUNT(*) INTO bad_count
    FROM desci_authority_write_fencing_state f
    JOIN LATERAL (
        SELECT epoch_number, epoch_hash, deployment_id, primary_id
        FROM desci_database_epochs
        ORDER BY epoch_number DESC
        LIMIT 1
    ) e ON TRUE
    WHERE f.phase <> 'epoch'
       OR f.deployment_id <> e.deployment_id
       OR f.primary_id <> e.primary_id
       OR f.epoch_number <> e.epoch_number
       OR f.epoch_hash <> e.epoch_hash;
    IF bad_count <> 0 THEN
        RAISE EXCEPTION 'authority write fence does not bind the latest database epoch: %', bad_count;
    END IF;
END
$$;

SELECT
    (SELECT COUNT(*) FROM desci_scientific_events) AS scientific_events,
    (SELECT COUNT(*) FROM desci_authority_receipts WHERE status = 'committed') AS committed_receipts,
    (SELECT COUNT(*) FROM desci_credential_events) AS credential_events,
    (SELECT COUNT(*) FROM desci_governance_events) AS governance_events,
    (SELECT COUNT(*) FROM desci_database_epochs) AS database_epochs,
    (SELECT COUNT(*) FROM desci_recovery_reconciliations) AS recovery_reconciliations,
    (SELECT COUNT(*) FROM desci_authority_delivery_acknowledgements) AS delivery_acknowledgements,
    (SELECT COUNT(*) FROM desci_authority_write_fencing_state) AS write_fencing_rows,
    (SELECT MAX(generation) FROM desci_authority_write_fencing_state) AS latest_write_fence_generation,
    (SELECT COUNT(*) FROM desci_authority_outbox WHERE published_at IS NULL) AS pending_outbox;

ROLLBACK;

-- All pending publications must use the signed authority-delivery envelope.
SELECT COUNT(*) AS unsigned_pending_outbox_rows
FROM desci_authority_outbox
WHERE published_at IS NULL
  AND NOT (
      payload ? 'envelope'
      AND payload ? 'signer_public_key'
      AND payload ? 'signature'
  );
