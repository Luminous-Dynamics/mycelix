-- Mycelix-DeSci credential/governance transactional authority schema v2
-- Apply after 0001_postgres_authority.sql. Runtime migration uses the same
-- advisory transaction lock and writes the schema marker only after all DDL.

BEGIN;
SELECT pg_advisory_xact_lock(730391904221);

DO $$
DECLARE
    existing_migration_id TEXT;
BEGIN
    SELECT migration_id INTO existing_migration_id
    FROM desci_authority_schema_migrations
    WHERE version = 2;

    IF existing_migration_id IS NOT NULL
       AND existing_migration_id <> 'mycelix-desci-authority-v2-2026-08-05' THEN
        RAISE EXCEPTION
            'authority schema version 2 is already owned by migration id %',
            existing_migration_id;
    END IF;
END
$$;

DO $$
DECLARE
    legacy_pending BIGINT;
BEGIN
    SELECT COUNT(*) INTO legacy_pending
    FROM desci_authority_outbox
    WHERE published_at IS NULL;

    IF legacy_pending > 0 THEN
        RAISE EXCEPTION
            'authority schema v2 requires draining % pending unsigned v1 outbox rows before migration',
            legacy_pending;
    END IF;
END
$$;

CREATE TABLE IF NOT EXISTS desci_credential_events (
    sequence BIGINT PRIMARY KEY CHECK (sequence >= 0),
    event_id UUID NOT NULL UNIQUE,
    event_hash BYTEA NOT NULL UNIQUE CHECK (octet_length(event_hash) = 32),
    record_hash BYTEA NOT NULL UNIQUE CHECK (octet_length(record_hash) = 32),
    previous_record_hash BYTEA CHECK (
        previous_record_hash IS NULL OR octet_length(previous_record_hash) = 32
    ),
    actor_id TEXT NOT NULL,
    idempotency_key TEXT,
    received_at TIMESTAMPTZ NOT NULL,
    record_json JSONB NOT NULL
);
CREATE UNIQUE INDEX IF NOT EXISTS desci_credential_idempotency_idx
ON desci_credential_events (actor_id, idempotency_key)
WHERE idempotency_key IS NOT NULL;

CREATE TABLE IF NOT EXISTS desci_governance_events (
    sequence BIGINT PRIMARY KEY CHECK (sequence >= 0),
    event_id UUID NOT NULL UNIQUE,
    event_hash BYTEA NOT NULL UNIQUE CHECK (octet_length(event_hash) = 32),
    record_hash BYTEA NOT NULL UNIQUE CHECK (octet_length(record_hash) = 32),
    previous_record_hash BYTEA CHECK (
        previous_record_hash IS NULL OR octet_length(previous_record_hash) = 32
    ),
    actor_id TEXT NOT NULL,
    idempotency_key TEXT NOT NULL,
    received_at TIMESTAMPTZ NOT NULL,
    record_json JSONB NOT NULL,
    UNIQUE (actor_id, idempotency_key)
);

INSERT INTO desci_authority_schema_migrations (version, migration_id)
VALUES (2, 'mycelix-desci-authority-v2-2026-08-05')
ON CONFLICT (version) DO NOTHING;

COMMIT;
