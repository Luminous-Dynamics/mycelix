-- Mycelix-DeSci externally signed authority-write fencing schema v4.
-- Apply only after schema v3. The runtime uses the same migration identifier.
BEGIN;
SELECT pg_advisory_xact_lock(730391904221);

CREATE TABLE IF NOT EXISTS desci_authority_schema_migrations (
    version BIGINT PRIMARY KEY CHECK (version > 0),
    migration_id TEXT NOT NULL UNIQUE,
    applied_at TIMESTAMPTZ NOT NULL DEFAULT clock_timestamp()
);

DO $$
DECLARE
    existing_id TEXT;
BEGIN
    SELECT migration_id INTO existing_id
    FROM desci_authority_schema_migrations
    WHERE version = 4
    FOR UPDATE;
    IF existing_id IS NOT NULL
       AND existing_id <> 'mycelix-desci-authority-v4-2026-08-05' THEN
        RAISE EXCEPTION 'authority schema v4 is already owned by unexpected migration id %', existing_id;
    END IF;
END $$;

CREATE TABLE IF NOT EXISTS desci_authority_write_fencing_state (
    deployment_id TEXT PRIMARY KEY,
    generation BIGINT NOT NULL CHECK (generation > 0),
    lease_id UUID NOT NULL,
    lease_hash BYTEA NOT NULL CHECK (octet_length(lease_hash) = 32),
    signer_public_key BYTEA NOT NULL CHECK (octet_length(signer_public_key) = 32),
    phase TEXT NOT NULL CHECK (phase IN ('bootstrap', 'epoch')),
    epoch_number BIGINT CHECK (epoch_number IS NULL OR epoch_number > 0),
    epoch_hash BYTEA CHECK (epoch_hash IS NULL OR octet_length(epoch_hash) = 32),
    primary_id TEXT NOT NULL,
    database_system_identifier TEXT NOT NULL,
    postgres_timeline BIGINT NOT NULL CHECK (postgres_timeline > 0),
    expires_at TIMESTAMPTZ NOT NULL,
    last_scope TEXT NOT NULL,
    last_seen_at TIMESTAMPTZ NOT NULL,
    lease_json JSONB NOT NULL,
    CHECK (
        (phase = 'bootstrap' AND epoch_number IS NULL AND epoch_hash IS NULL)
        OR (phase = 'epoch' AND epoch_number IS NOT NULL AND epoch_hash IS NOT NULL)
    )
);

INSERT INTO desci_authority_schema_migrations (version, migration_id)
VALUES (4, 'mycelix-desci-authority-v4-2026-08-05')
ON CONFLICT (version) DO NOTHING;
COMMIT;
