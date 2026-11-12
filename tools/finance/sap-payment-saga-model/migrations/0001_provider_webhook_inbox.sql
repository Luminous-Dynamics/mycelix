-- Provider webhook inbox / object-reconciliation queue V1.
-- PostgreSQL durability boundary for authenticated provider notifications.
--
-- Verify HMAC over exact raw request bytes before calling
-- accept_provider_webhook_v1(). Encrypt the same raw bytes before storing them.
-- A webhook delivery is a change notification, NOT an ordered economic event
-- log and NOT a payment effect. Actual financial effects retain their own
-- saga/compensation effect IDs and authorization checks.

CREATE TABLE IF NOT EXISTS provider_webhook_inbox (
    inbox_id BIGSERIAL PRIMARY KEY,
    provider_id TEXT NOT NULL CHECK (length(provider_id) BETWEEN 1 AND 128),
    endpoint_id TEXT NOT NULL CHECK (length(endpoint_id) BETWEEN 1 AND 256),
    provider_account_scope TEXT NOT NULL CHECK (length(provider_account_scope) BETWEEN 1 AND 512),
    provider_region TEXT NOT NULL CHECK (length(provider_region) BETWEEN 1 AND 128),
    resource_account_scope TEXT NOT NULL CHECK (length(resource_account_scope) BETWEEN 1 AND 512),
    environment TEXT NOT NULL CHECK (environment IN ('test', 'live')),
    event_id TEXT NOT NULL CHECK (length(event_id) BETWEEN 1 AND 256),
    event_type TEXT NOT NULL CHECK (length(event_type) BETWEEN 1 AND 256),
    event_object_id TEXT NOT NULL CHECK (length(event_object_id) BETWEEN 1 AND 512),
    event_api_version TEXT,
    payload_kind TEXT NOT NULL CHECK (payload_kind IN ('snapshot', 'thin', 'provider-native')),
    -- Diagnostic provider event creation time. Never use as a total-order cursor.
    event_created_at TIMESTAMPTZ,
    signature_timestamp_unix_seconds BIGINT NOT NULL CHECK (signature_timestamp_unix_seconds > 0),
    signature_profile TEXT NOT NULL CHECK (length(signature_profile) BETWEEN 1 AND 128),
    signature_key_ref TEXT NOT NULL CHECK (length(signature_key_ref) BETWEEN 1 AND 256),
    verifier_revision TEXT NOT NULL CHECK (length(verifier_revision) BETWEEN 1 AND 128),
    provider_contract_digest TEXT NOT NULL
        CHECK (provider_contract_digest ~ '^sha256:[0-9a-f]{64}$'),
    raw_body_sha256 TEXT NOT NULL
        CHECK (raw_body_sha256 ~ '^sha256:[0-9a-f]{64}$'),
    raw_body_ciphertext BYTEA NOT NULL CHECK (octet_length(raw_body_ciphertext) > 0),
    encryption_key_id TEXT NOT NULL CHECK (length(encryption_key_id) BETWEEN 1 AND 256),
    verified_at TIMESTAMPTZ NOT NULL,
    received_at TIMESTAMPTZ NOT NULL DEFAULT clock_timestamp(),
    object_generation BIGINT CHECK (object_generation IS NULL OR object_generation > 0),
    reconciled_at TIMESTAMPTZ,
    reconciliation_digest TEXT
        CHECK (reconciliation_digest IS NULL OR reconciliation_digest ~ '^sha256:[0-9a-f]{64}$'),

    CONSTRAINT provider_webhook_inbox_event_scope_unique
        UNIQUE (provider_id, provider_account_scope, endpoint_id, environment, event_id)
);

CREATE INDEX IF NOT EXISTS provider_webhook_inbox_received_idx
    ON provider_webhook_inbox (received_at, inbox_id);

CREATE INDEX IF NOT EXISTS provider_webhook_inbox_object_idx
    ON provider_webhook_inbox (
        provider_id, provider_account_scope, provider_region, resource_account_scope,
        environment, event_object_id, object_generation
    );

CREATE TABLE IF NOT EXISTS provider_webhook_conflict (
    conflict_id BIGSERIAL PRIMARY KEY,
    provider_id TEXT NOT NULL,
    provider_account_scope TEXT NOT NULL,
    provider_region TEXT NOT NULL CHECK (length(provider_region) BETWEEN 1 AND 128),
    endpoint_id TEXT NOT NULL,
    environment TEXT NOT NULL CHECK (environment IN ('test', 'live')),
    event_id TEXT NOT NULL,
    existing_body_sha256 TEXT NOT NULL CHECK (existing_body_sha256 ~ '^sha256:[0-9a-f]{64}$'),
    incoming_body_sha256 TEXT NOT NULL CHECK (incoming_body_sha256 ~ '^sha256:[0-9a-f]{64}$'),
    existing_contract_digest TEXT NOT NULL CHECK (existing_contract_digest ~ '^sha256:[0-9a-f]{64}$'),
    incoming_contract_digest TEXT NOT NULL CHECK (incoming_contract_digest ~ '^sha256:[0-9a-f]{64}$'),
    incoming_ciphertext BYTEA NOT NULL CHECK (octet_length(incoming_ciphertext) > 0),
    encryption_key_id TEXT NOT NULL,
    observed_at TIMESTAMPTZ NOT NULL DEFAULT clock_timestamp(),
    conflict_reason TEXT NOT NULL CHECK (conflict_reason IN (
        'body_digest_changed',
        'contract_digest_changed',
        'provider_region_changed',
        'event_object_id_changed',
        'event_api_version_changed',
        'payload_kind_changed',
        'event_type_or_resource_scope_changed'
    ))
);

CREATE INDEX IF NOT EXISTS provider_webhook_conflict_scope_idx
    ON provider_webhook_conflict
       (provider_id, provider_account_scope, provider_region, endpoint_id, environment, event_id, observed_at);

-- One coalescing work row per provider object and resource account. Every newly
-- accepted event increments requested_generation. A worker snapshots that
-- generation when it claims the row; events arriving during the fetch increase
-- requested_generation and force another refresh after the current lease.
CREATE TABLE IF NOT EXISTS provider_webhook_object_reconciliation (
    provider_id TEXT NOT NULL,
    provider_account_scope TEXT NOT NULL,
    provider_region TEXT NOT NULL CHECK (length(provider_region) BETWEEN 1 AND 128),
    resource_account_scope TEXT NOT NULL,
    environment TEXT NOT NULL CHECK (environment IN ('test', 'live')),
    event_object_id TEXT NOT NULL CHECK (length(event_object_id) BETWEEN 1 AND 512),
    requested_generation BIGINT NOT NULL DEFAULT 0 CHECK (requested_generation >= 0),
    completed_generation BIGINT NOT NULL DEFAULT 0 CHECK (completed_generation >= 0),
    state TEXT NOT NULL DEFAULT 'pending' CHECK (state IN (
        'pending', 'leased', 'indeterminate', 'idle', 'quarantined'
    )),
    lease_token UUID,
    lease_owner TEXT,
    lease_generation BIGINT,
    lease_expires_at TIMESTAMPTZ,
    last_provider_state_digest TEXT
        CHECK (last_provider_state_digest IS NULL OR last_provider_state_digest ~ '^sha256:[0-9a-f]{64}$'),
    last_provider_api_version TEXT,
    last_reconciled_at TIMESTAMPTZ,
    last_error_code TEXT,
    created_at TIMESTAMPTZ NOT NULL DEFAULT clock_timestamp(),
    updated_at TIMESTAMPTZ NOT NULL DEFAULT clock_timestamp(),
    PRIMARY KEY (
        provider_id, provider_account_scope, provider_region, resource_account_scope,
        environment, event_object_id
    ),
    CHECK (completed_generation <= requested_generation),
    CHECK (
        (state = 'leased'
            AND lease_token IS NOT NULL
            AND lease_owner IS NOT NULL
            AND lease_generation IS NOT NULL
            AND lease_expires_at IS NOT NULL
            AND lease_generation <= requested_generation)
        OR
        (state <> 'leased'
            AND lease_token IS NULL
            AND lease_owner IS NULL
            AND lease_generation IS NULL
            AND lease_expires_at IS NULL)
    )
);

CREATE INDEX IF NOT EXISTS provider_webhook_object_pending_idx
    ON provider_webhook_object_reconciliation (updated_at, provider_id, provider_region, event_object_id)
    WHERE state IN ('pending', 'indeterminate');

CREATE INDEX IF NOT EXISTS provider_webhook_object_expired_lease_idx
    ON provider_webhook_object_reconciliation (lease_expires_at)
    WHERE state = 'leased';

-- The event outbox is only a durable, deduplicated processing receipt. work_id
-- is NOT a PSP idempotency key and NOT an economic effect ID. It can be derived
-- from the inbox identity and is consumed by the object-reconciliation worker.
CREATE TABLE IF NOT EXISTS provider_webhook_outbox (
    outbox_id BIGSERIAL PRIMARY KEY,
    inbox_id BIGINT NOT NULL UNIQUE
        REFERENCES provider_webhook_inbox(inbox_id) ON DELETE RESTRICT,
    work_id TEXT NOT NULL UNIQUE CHECK (length(work_id) BETWEEN 1 AND 512),
    object_generation BIGINT NOT NULL CHECK (object_generation > 0),
    state TEXT NOT NULL DEFAULT 'pending' CHECK (state IN (
        'pending', 'indeterminate', 'reconciled', 'quarantined'
    )),
    last_error_code TEXT,
    reconciliation_digest TEXT
        CHECK (reconciliation_digest IS NULL OR reconciliation_digest ~ '^sha256:[0-9a-f]{64}$'),
    reconciled_at TIMESTAMPTZ,
    created_at TIMESTAMPTZ NOT NULL DEFAULT clock_timestamp(),
    updated_at TIMESTAMPTZ NOT NULL DEFAULT clock_timestamp(),
    CHECK (
        (state = 'reconciled'
            AND reconciliation_digest IS NOT NULL
            AND reconciled_at IS NOT NULL)
        OR
        (state <> 'reconciled'
            AND reconciliation_digest IS NULL
            AND reconciled_at IS NULL)
    )
);

CREATE INDEX IF NOT EXISTS provider_webhook_outbox_pending_idx
    ON provider_webhook_outbox (outbox_id)
    WHERE state IN ('pending', 'indeterminate');

CREATE OR REPLACE FUNCTION accept_provider_webhook_v1(
    p_provider_id TEXT,
    p_endpoint_id TEXT,
    p_provider_account_scope TEXT,
    p_provider_region TEXT,
    p_resource_account_scope TEXT,
    p_environment TEXT,
    p_event_id TEXT,
    p_event_type TEXT,
    p_event_object_id TEXT,
    p_event_api_version TEXT,
    p_payload_kind TEXT,
    p_event_created_at TIMESTAMPTZ,
    p_signature_timestamp_unix_seconds BIGINT,
    p_signature_profile TEXT,
    p_signature_key_ref TEXT,
    p_verifier_revision TEXT,
    p_provider_contract_digest TEXT,
    p_raw_body_sha256 TEXT,
    p_raw_body_ciphertext BYTEA,
    p_encryption_key_id TEXT,
    p_verified_at TIMESTAMPTZ,
    p_work_id TEXT
) RETURNS TEXT
LANGUAGE plpgsql
AS $$
DECLARE
    v_inbox_id BIGINT;
    v_object_generation BIGINT;
    v_existing provider_webhook_inbox%ROWTYPE;
    v_conflict_reason TEXT;
BEGIN
    INSERT INTO provider_webhook_inbox (
        provider_id, endpoint_id, provider_account_scope, provider_region, resource_account_scope,
        environment, event_id, event_type, event_object_id, event_api_version,
        payload_kind, event_created_at, signature_timestamp_unix_seconds,
        signature_profile, signature_key_ref, verifier_revision,
        provider_contract_digest, raw_body_sha256, raw_body_ciphertext,
        encryption_key_id, verified_at
    ) VALUES (
        p_provider_id, p_endpoint_id, p_provider_account_scope, p_provider_region, p_resource_account_scope,
        p_environment, p_event_id, p_event_type, p_event_object_id, p_event_api_version,
        p_payload_kind, p_event_created_at, p_signature_timestamp_unix_seconds,
        p_signature_profile, p_signature_key_ref, p_verifier_revision,
        p_provider_contract_digest, p_raw_body_sha256, p_raw_body_ciphertext,
        p_encryption_key_id, p_verified_at
    )
    ON CONFLICT (
        provider_id, provider_account_scope, endpoint_id, environment, event_id
    ) DO NOTHING
    RETURNING inbox_id INTO v_inbox_id;

    IF v_inbox_id IS NULL THEN
        SELECT *
          INTO v_existing
          FROM provider_webhook_inbox
         WHERE provider_id = p_provider_id
           AND provider_account_scope = p_provider_account_scope
           AND endpoint_id = p_endpoint_id
           AND environment = p_environment
           AND event_id = p_event_id
         FOR SHARE;

        IF v_existing.raw_body_sha256 = p_raw_body_sha256
           AND v_existing.provider_contract_digest = p_provider_contract_digest
           AND v_existing.event_type = p_event_type
           AND v_existing.event_object_id = p_event_object_id
           AND v_existing.event_api_version IS NOT DISTINCT FROM p_event_api_version
           AND v_existing.payload_kind = p_payload_kind
           AND v_existing.resource_account_scope = p_resource_account_scope THEN
            RETURN 'duplicate';
        END IF;

        v_conflict_reason := CASE
            WHEN v_existing.raw_body_sha256 <> p_raw_body_sha256
                THEN 'body_digest_changed'
            WHEN v_existing.provider_contract_digest <> p_provider_contract_digest
                THEN 'contract_digest_changed'
            WHEN v_existing.provider_region <> p_provider_region
                THEN 'provider_region_changed'
            WHEN v_existing.event_object_id <> p_event_object_id
                THEN 'event_object_id_changed'
            WHEN v_existing.event_api_version IS DISTINCT FROM p_event_api_version
                THEN 'event_api_version_changed'
            WHEN v_existing.payload_kind <> p_payload_kind
                THEN 'payload_kind_changed'
            ELSE 'event_type_or_resource_scope_changed'
        END;

        INSERT INTO provider_webhook_conflict (
            provider_id, provider_account_scope, provider_region, endpoint_id, environment,
            event_id, existing_body_sha256, incoming_body_sha256,
            existing_contract_digest, incoming_contract_digest,
            incoming_ciphertext, encryption_key_id, conflict_reason
        ) VALUES (
            p_provider_id, p_provider_account_scope, p_provider_region, p_endpoint_id, p_environment,
            p_event_id, v_existing.raw_body_sha256, p_raw_body_sha256,
            v_existing.provider_contract_digest, p_provider_contract_digest,
            p_raw_body_ciphertext, p_encryption_key_id, v_conflict_reason
        );
        RETURN 'conflict';
    END IF;

    -- This is a change notification. Increment an object refresh generation,
    -- never a financial effect sequence. Preserve an active worker lease; it
    -- will observe requested_generation > lease_generation and refresh again.
    INSERT INTO provider_webhook_object_reconciliation (
        provider_id, provider_account_scope, provider_region, resource_account_scope,
        environment, event_object_id, requested_generation, state
    ) VALUES (
        p_provider_id, p_provider_account_scope, p_provider_region, p_resource_account_scope,
        p_environment, p_event_object_id, 1, 'pending'
    )
    ON CONFLICT (
        provider_id, provider_account_scope, provider_region, resource_account_scope,
        environment, event_object_id
    ) DO UPDATE
       SET requested_generation =
               provider_webhook_object_reconciliation.requested_generation + 1,
           state = CASE
               WHEN provider_webhook_object_reconciliation.state = 'leased' THEN 'leased'
               WHEN provider_webhook_object_reconciliation.state = 'quarantined' THEN 'quarantined'
               ELSE 'pending'
           END,
           updated_at = clock_timestamp()
    RETURNING requested_generation INTO v_object_generation;

    UPDATE provider_webhook_inbox
       SET object_generation = v_object_generation
     WHERE inbox_id = v_inbox_id;

    INSERT INTO provider_webhook_outbox (inbox_id, work_id, object_generation)
    VALUES (v_inbox_id, p_work_id, v_object_generation);

    RETURN 'accepted';
END;
$$;

-- Claim a provider-object refresh, not an external money movement. The GET
-- operation must be read-only and safely repeatable. Capture the generation
-- while holding the row lock.
--
-- BEGIN;
-- WITH candidate AS (
--     SELECT provider_id, provider_account_scope, provider_region, resource_account_scope,
--            environment, event_object_id
--       FROM provider_webhook_object_reconciliation
--      WHERE state IN ('pending', 'indeterminate')
--        AND requested_generation > completed_generation
--      ORDER BY updated_at, provider_id, event_object_id
--      FOR UPDATE SKIP LOCKED
--      LIMIT 1
-- )
-- UPDATE provider_webhook_object_reconciliation AS target
--    SET state = 'leased',
--        lease_token = gen_random_uuid(),
--        lease_owner = :worker_id,
--        lease_generation = target.requested_generation,
--        lease_expires_at = clock_timestamp() + interval '30 seconds',
--        updated_at = clock_timestamp()
--   FROM candidate
--  WHERE target.provider_id = candidate.provider_id
--    AND target.provider_account_scope = candidate.provider_account_scope
--    AND target.resource_account_scope = candidate.resource_account_scope
--    AND target.environment = candidate.environment
--    AND target.event_object_id = candidate.event_object_id
-- RETURNING target.*;
-- COMMIT;
--
-- After GET, signature/TLS/provider response authentication and schema
-- validation, commit only if lease_token still matches:
--
-- UPDATE provider_webhook_object_reconciliation
--    SET completed_generation = lease_generation,
--        last_provider_state_digest = :canonical_provider_state_digest,
--        last_provider_api_version = :response_api_version,
--        last_reconciled_at = clock_timestamp(),
--        state = CASE
--            WHEN requested_generation > lease_generation THEN 'pending'
--            ELSE 'idle'
--        END,
--        lease_token = NULL, lease_owner = NULL,
--        lease_generation = NULL, lease_expires_at = NULL,
--        last_error_code = NULL,
--        updated_at = clock_timestamp()
--    AND target.provider_region = :provider_region
--  WHERE ... remaining exact object key ...
--    AND state = 'leased'
--    AND lease_token = :lease_token;
--
-- In the SAME transaction, mark event work rows reconciled only through the
-- generation actually fetched; newer deliveries remain pending:
--
-- UPDATE provider_webhook_outbox AS work
--    SET state = 'reconciled',
--        reconciliation_digest = :canonical_provider_state_digest,
--        reconciled_at = clock_timestamp(),
--        updated_at = clock_timestamp()
--   FROM provider_webhook_inbox AS event
--  WHERE work.inbox_id = event.inbox_id
--    AND event.provider_id = :provider_id
--    AND event.provider_account_scope = :provider_account_scope
--    AND event.provider_region = :provider_region
--    AND event.resource_account_scope = :resource_account_scope
--    AND event.environment = :environment
--    AND event.event_object_id = :event_object_id
--    AND event.object_generation <= :lease_generation
--    AND work.state IN ('pending', 'indeterminate');
--
-- Lease expiry of a GET refresh is retryable because the read is side-effect
-- free. The sweeper records an indeterminate read outcome and releases only the
-- read-worker lease. A future worker may safely re-read the same provider object.
--
-- UPDATE provider_webhook_object_reconciliation
--    SET state = 'indeterminate',
--        lease_token = NULL, lease_owner = NULL,
--        lease_generation = NULL, lease_expires_at = NULL,
--        last_error_code = 'object_refresh_lease_expired',
--        updated_at = clock_timestamp()
--  WHERE state = 'leased' AND lease_expires_at <= clock_timestamp();
--
-- It MUST NOT reset or alter a payment effect. Payment/refund dispatch remains
-- in the compensation saga, using its own owner grant, liquidity reservation,
-- stable economic effect ID and provider retry permit.
