-- Mycelix-DeSci governed database epoch and recovery schema v3.
-- Apply after 0002_postgres_credential_authority.sql.

BEGIN;
SELECT pg_advisory_xact_lock(730391904221);

DO $$
DECLARE
    existing_migration_id TEXT;
BEGIN
    SELECT migration_id INTO existing_migration_id
    FROM desci_authority_schema_migrations
    WHERE version = 3;

    IF existing_migration_id IS NOT NULL
       AND existing_migration_id <> 'mycelix-desci-authority-v3-2026-08-05' THEN
        RAISE EXCEPTION
            'authority schema version 3 is already owned by migration id %',
            existing_migration_id;
    END IF;
END
$$;

CREATE TABLE IF NOT EXISTS desci_database_epochs (
    epoch_number BIGINT PRIMARY KEY CHECK (epoch_number > 0),
    epoch_id UUID NOT NULL UNIQUE,
    epoch_hash BYTEA NOT NULL UNIQUE CHECK (octet_length(epoch_hash) = 32),
    previous_epoch_hash BYTEA CHECK (
        previous_epoch_hash IS NULL OR octet_length(previous_epoch_hash) = 32
    ),
    deployment_id TEXT NOT NULL,
    primary_id TEXT NOT NULL,
    promoted_at TIMESTAMPTZ NOT NULL,
    accepted_at TIMESTAMPTZ NOT NULL CHECK (accepted_at >= promoted_at),
    state_commitment_hash BYTEA NOT NULL CHECK (
        octet_length(state_commitment_hash) = 32
    ),
    publication_delivery_id UUID NOT NULL UNIQUE REFERENCES desci_authority_outbox(id)
        DEFERRABLE INITIALLY DEFERRED,
    certificate_json JSONB NOT NULL,
    UNIQUE (deployment_id, epoch_number)
);

CREATE TABLE IF NOT EXISTS desci_recovery_reconciliations (
    reconciliation_id UUID PRIMARY KEY,
    epoch_hash BYTEA NOT NULL UNIQUE REFERENCES desci_database_epochs(epoch_hash),
    reconciliation_hash BYTEA NOT NULL UNIQUE CHECK (
        octet_length(reconciliation_hash) = 32
    ),
    verified_at TIMESTAMPTZ NOT NULL,
    accepted_at TIMESTAMPTZ NOT NULL CHECK (accepted_at >= verified_at),
    publication_delivery_id UUID NOT NULL UNIQUE REFERENCES desci_authority_outbox(id)
        DEFERRABLE INITIALLY DEFERRED,
    reconciliation_json JSONB NOT NULL
);

CREATE TABLE IF NOT EXISTS desci_authority_delivery_acknowledgements (
    acknowledgement_id UUID PRIMARY KEY,
    delivery_id UUID NOT NULL REFERENCES desci_authority_outbox(id),
    delivery_hash BYTEA NOT NULL CHECK (octet_length(delivery_hash) = 32),
    witness_actor TEXT NOT NULL,
    witness_organization TEXT NOT NULL,
    acknowledged_at TIMESTAMPTZ NOT NULL,
    accepted_at TIMESTAMPTZ NOT NULL CHECK (accepted_at >= acknowledged_at),
    acknowledgement_hash BYTEA NOT NULL UNIQUE CHECK (
        octet_length(acknowledgement_hash) = 32
    ),
    acknowledgement_json JSONB NOT NULL,
    UNIQUE (delivery_id, witness_actor)
);

CREATE INDEX IF NOT EXISTS desci_delivery_ack_delivery_idx
ON desci_authority_delivery_acknowledgements (delivery_id, accepted_at);

INSERT INTO desci_authority_schema_migrations (version, migration_id)
VALUES (3, 'mycelix-desci-authority-v3-2026-08-05')
ON CONFLICT (version) DO NOTHING;

COMMIT;
