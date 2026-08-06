-- Mycelix-DeSci canonical authority schema v1
-- Keep synchronized with POSTGRES_AUTHORITY_MIGRATION in
-- src/core/src/postgres_authority.rs. Runtime migration is serialized under a
-- PostgreSQL advisory transaction lock.

BEGIN;
SELECT pg_advisory_xact_lock(730391904221);

CREATE TABLE IF NOT EXISTS desci_authority_schema_migrations (
    version BIGINT PRIMARY KEY CHECK (version > 0),
    migration_id TEXT NOT NULL UNIQUE,
    applied_at TIMESTAMPTZ NOT NULL DEFAULT clock_timestamp()
);
DO $$
DECLARE
    existing_migration_id TEXT;
BEGIN
    SELECT migration_id INTO existing_migration_id
    FROM desci_authority_schema_migrations
    WHERE version = 1;

    IF existing_migration_id IS NOT NULL
       AND existing_migration_id <> 'mycelix-desci-authority-v1-2026-08-05' THEN
        RAISE EXCEPTION
            'authority schema version 1 is already owned by migration id %',
            existing_migration_id;
    END IF;
END
$$;
CREATE TABLE IF NOT EXISTS desci_scientific_events (
    stream_id UUID NOT NULL,
    sequence BIGINT NOT NULL CHECK (sequence >= 0),
    event_id UUID NOT NULL UNIQUE,
    event_hash BYTEA NOT NULL UNIQUE CHECK (octet_length(event_hash) = 32),
    actor_id TEXT NOT NULL,
    idempotency_key TEXT,
    received_at TIMESTAMPTZ NOT NULL,
    event_json JSONB NOT NULL,
    PRIMARY KEY (stream_id, sequence)
);
CREATE UNIQUE INDEX IF NOT EXISTS desci_scientific_event_idempotency_idx
ON desci_scientific_events (actor_id, idempotency_key)
WHERE idempotency_key IS NOT NULL;

CREATE TABLE IF NOT EXISTS desci_authority_receipts (
    event_id UUID PRIMARY KEY REFERENCES desci_scientific_events(event_id)
        DEFERRABLE INITIALLY DEFERRED,
    stream_id UUID NOT NULL,
    sequence BIGINT NOT NULL CHECK (sequence >= 0),
    receipt_hash BYTEA NOT NULL UNIQUE CHECK (octet_length(receipt_hash) = 32),
    previous_receipt_hash BYTEA CHECK (
        previous_receipt_hash IS NULL OR octet_length(previous_receipt_hash) = 32
    ),
    status TEXT NOT NULL CHECK (status IN ('pending', 'committed')),
    receipt_json JSONB NOT NULL,
    prepared_at TIMESTAMPTZ NOT NULL,
    committed_at TIMESTAMPTZ,
    CHECK (
        (status = 'pending' AND committed_at IS NULL)
        OR (status = 'committed' AND committed_at IS NOT NULL)
    ),
    UNIQUE (stream_id, sequence),
    FOREIGN KEY (stream_id, sequence)
        REFERENCES desci_scientific_events(stream_id, sequence)
        DEFERRABLE INITIALLY DEFERRED
);

CREATE TABLE IF NOT EXISTS desci_authority_outbox (
    id UUID PRIMARY KEY,
    topic TEXT NOT NULL,
    aggregate_id TEXT NOT NULL,
    aggregate_sequence BIGINT NOT NULL,
    payload JSONB NOT NULL,
    payload_hash BYTEA NOT NULL CHECK (octet_length(payload_hash) = 32),
    created_at TIMESTAMPTZ NOT NULL,
    published_at TIMESTAMPTZ,
    attempts INTEGER NOT NULL DEFAULT 0 CHECK (attempts >= 0),
    lease_owner TEXT,
    lease_until TIMESTAMPTZ,
    last_error TEXT,
    UNIQUE (topic, aggregate_id, aggregate_sequence)
);
CREATE INDEX IF NOT EXISTS desci_authority_outbox_pending_idx
ON desci_authority_outbox (created_at, id)
WHERE published_at IS NULL;

-- Reserved import targets. The runtime does not yet claim that credential and
-- governance execution are transactionally cut over to PostgreSQL.
CREATE TABLE IF NOT EXISTS desci_credential_records (
    sequence BIGINT PRIMARY KEY CHECK (sequence >= 0),
    record_hash BYTEA NOT NULL UNIQUE CHECK (octet_length(record_hash) = 32),
    received_at TIMESTAMPTZ NOT NULL,
    record_json JSONB NOT NULL
);
CREATE TABLE IF NOT EXISTS desci_governance_records (
    sequence BIGINT PRIMARY KEY CHECK (sequence >= 0),
    record_hash BYTEA NOT NULL UNIQUE CHECK (octet_length(record_hash) = 32),
    received_at TIMESTAMPTZ NOT NULL,
    record_json JSONB NOT NULL
);

CREATE TABLE IF NOT EXISTS desci_checkpoint_mirror_observations (
    observation_id UUID PRIMARY KEY,
    checkpoint_hash BYTEA NOT NULL CHECK (octet_length(checkpoint_hash) = 32),
    mirror_actor TEXT NOT NULL,
    mirror_organization TEXT NOT NULL,
    mirror_uri TEXT NOT NULL,
    observed_at TIMESTAMPTZ NOT NULL,
    accepted_at TIMESTAMPTZ NOT NULL,
    observation_json JSONB NOT NULL,
    UNIQUE (checkpoint_hash, mirror_actor)
);

-- Record the schema marker only after all domain DDL above has succeeded.
INSERT INTO desci_authority_schema_migrations (version, migration_id)
VALUES (1, 'mycelix-desci-authority-v1-2026-08-05')
ON CONFLICT (version) DO NOTHING;

COMMIT;
