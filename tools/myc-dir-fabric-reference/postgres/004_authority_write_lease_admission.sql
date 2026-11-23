-- Mycelix Directory Fabric signed authority-write lease admission v0.1.
--
-- Migration 004 layers the existing DeSci v1 signed lease protocol over the
-- owner-epoch effect owner. The authority controller verifies the Ed25519
-- signature and trust root with the shared Rust verifier before calling the
-- installer. PostgreSQL then enforces exact target binding, monotonic lease
-- generation, expiry, local epoch/generation position and transaction-scoped
-- use. This SQL does NOT verify Ed25519 itself and does NOT physically fence a
-- disconnected former primary; those remain independent control-plane duties.

BEGIN;

DO $roles$
BEGIN
    IF NOT EXISTS (SELECT 1 FROM pg_catalog.pg_roles WHERE rolname = 'myc_dir_authority_lease_controller') THEN
        CREATE ROLE myc_dir_authority_lease_controller NOLOGIN;
    END IF;
    IF NOT EXISTS (SELECT 1 FROM pg_catalog.pg_roles WHERE rolname = 'myc_dir_effect_worker') THEN
        CREATE ROLE myc_dir_effect_worker NOLOGIN;
    END IF;
END
$roles$;

ALTER ROLE myc_dir_authority_lease_controller NOLOGIN;
ALTER ROLE myc_dir_effect_worker NOLOGIN;

CREATE TABLE IF NOT EXISTS myc_effect_owner.authority_write_lease_highwater (
    deployment_id TEXT PRIMARY KEY CHECK (btrim(deployment_id) <> ''),
    lease_generation BIGINT NOT NULL CHECK (lease_generation > 0),
    lease_id UUID NOT NULL,
    lease_hash BYTEA NOT NULL CHECK (
        octet_length(lease_hash) = 32
        AND lease_hash <> decode(repeat('00', 32), 'hex')
    ),
    owner_primary_id TEXT NOT NULL CHECK (btrim(owner_primary_id) <> ''),
    database_system_identifier TEXT NOT NULL CHECK (btrim(database_system_identifier) <> ''),
    postgres_timeline BIGINT NOT NULL CHECK (postgres_timeline > 0),
    database_epoch_number BIGINT,
    database_epoch_hash BYTEA CHECK (
        database_epoch_hash IS NULL OR octet_length(database_epoch_hash) = 32
    ),
    external_highwater_receipt_digest BYTEA NOT NULL CHECK (
        octet_length(external_highwater_receipt_digest) = 32
        AND external_highwater_receipt_digest <> decode(repeat('00', 32), 'hex')
    ),
    updated_at TIMESTAMPTZ NOT NULL DEFAULT clock_timestamp()
);

-- Immutable per-effect-owner receipt containing the complete signed envelope
-- and the controller-verified external high-water / fencing commitments.
CREATE TABLE IF NOT EXISTS myc_effect_owner.effect_owner_authority_write_lease_receipts (
    realm_id TEXT NOT NULL,
    resource_id TEXT NOT NULL,
    action_id TEXT NOT NULL,
    effect_owner_id TEXT NOT NULL,
    lease_id UUID NOT NULL,
    lease_hash BYTEA NOT NULL CHECK (
        octet_length(lease_hash) = 32
        AND lease_hash <> decode(repeat('00', 32), 'hex')
    ),
    lease_generation BIGINT NOT NULL CHECK (lease_generation > 0),
    deployment_id TEXT NOT NULL CHECK (btrim(deployment_id) <> ''),
    primary_id TEXT NOT NULL CHECK (btrim(primary_id) <> ''),
    database_system_identifier TEXT NOT NULL CHECK (btrim(database_system_identifier) <> ''),
    postgres_timeline BIGINT NOT NULL CHECK (postgres_timeline > 0),
    database_epoch_number BIGINT,
    database_epoch_hash BYTEA CHECK (
        database_epoch_hash IS NULL OR octet_length(database_epoch_hash) = 32
    ),
    resource_owner_epoch BIGINT NOT NULL CHECK (resource_owner_epoch > 0),
    resource_generation BIGINT NOT NULL CHECK (resource_generation > 0),
    lease_not_before TIMESTAMPTZ NOT NULL,
    lease_expires_at TIMESTAMPTZ NOT NULL,
    allowed_scopes JSONB NOT NULL CHECK (jsonb_typeof(allowed_scopes) = 'array'),
    signer_key_id TEXT NOT NULL CHECK (btrim(signer_key_id) <> ''),
    signer_public_key BYTEA NOT NULL CHECK (octet_length(signer_public_key) = 32),
    signature BYTEA NOT NULL CHECK (octet_length(signature) = 64),
    signed_lease_json JSONB NOT NULL CHECK (jsonb_typeof(signed_lease_json) = 'object'),
    external_highwater_receipt_digest BYTEA NOT NULL CHECK (
        octet_length(external_highwater_receipt_digest) = 32
        AND external_highwater_receipt_digest <> decode(repeat('00', 32), 'hex')
    ),
    physical_fencing_receipt_digest BYTEA CHECK (
        physical_fencing_receipt_digest IS NULL
        OR (
            octet_length(physical_fencing_receipt_digest) = 32
            AND physical_fencing_receipt_digest <> decode(repeat('00', 32), 'hex')
        )
    ),
    accepted_at TIMESTAMPTZ NOT NULL DEFAULT clock_timestamp(),
    PRIMARY KEY (realm_id, resource_id, action_id, effect_owner_id, lease_id),
    UNIQUE (realm_id, resource_id, action_id, effect_owner_id, lease_generation),
    FOREIGN KEY (realm_id, resource_id, action_id, effect_owner_id)
        REFERENCES myc_effect_owner.effect_fence_state (
            realm_id, resource_id, action_id, effect_owner_id
        ),
    CHECK (lease_not_before < lease_expires_at),
    CHECK (lease_expires_at - lease_not_before <= interval '15 minutes')
);

-- Mutable pointer to the exact lease currently admitted for a fence domain.
-- The referenced receipt is append-only and remains available for replay/audit.
CREATE TABLE IF NOT EXISTS myc_effect_owner.effect_owner_authority_write_lease_state (
    realm_id TEXT NOT NULL,
    resource_id TEXT NOT NULL,
    action_id TEXT NOT NULL,
    effect_owner_id TEXT NOT NULL,
    lease_id UUID NOT NULL,
    lease_hash BYTEA NOT NULL CHECK (octet_length(lease_hash) = 32),
    lease_generation BIGINT NOT NULL CHECK (lease_generation > 0),
    deployment_id TEXT NOT NULL,
    primary_id TEXT NOT NULL,
    database_system_identifier TEXT NOT NULL,
    postgres_timeline BIGINT NOT NULL CHECK (postgres_timeline > 0),
    database_epoch_number BIGINT,
    database_epoch_hash BYTEA,
    resource_owner_epoch BIGINT NOT NULL CHECK (resource_owner_epoch > 0),
    resource_generation BIGINT NOT NULL CHECK (resource_generation > 0),
    lease_not_before TIMESTAMPTZ NOT NULL,
    lease_expires_at TIMESTAMPTZ NOT NULL,
    allowed_scopes JSONB NOT NULL CHECK (jsonb_typeof(allowed_scopes) = 'array'),
    signer_key_id TEXT NOT NULL,
    signer_public_key BYTEA NOT NULL CHECK (octet_length(signer_public_key) = 32),
    signature BYTEA NOT NULL CHECK (octet_length(signature) = 64),
    signed_lease_json JSONB NOT NULL,
    external_highwater_receipt_digest BYTEA NOT NULL CHECK (
        octet_length(external_highwater_receipt_digest) = 32
    ),
    physical_fencing_receipt_digest BYTEA,
    updated_at TIMESTAMPTZ NOT NULL DEFAULT clock_timestamp(),
    PRIMARY KEY (realm_id, resource_id, action_id, effect_owner_id),
    FOREIGN KEY (realm_id, resource_id, action_id, effect_owner_id, lease_id)
        REFERENCES myc_effect_owner.effect_owner_authority_write_lease_receipts (
            realm_id, resource_id, action_id, effect_owner_id, lease_id
        )
);

CREATE TABLE IF NOT EXISTS myc_effect_owner.effect_execution_authority_lease_attestations (
    attestation_id UUID PRIMARY KEY
        REFERENCES myc_effect_owner.effect_execution_attestations(attestation_id),
    effect_id UUID NOT NULL UNIQUE
        REFERENCES myc_effect_owner.effect_commits(effect_id),
    realm_id TEXT NOT NULL,
    resource_id TEXT NOT NULL,
    action_id TEXT NOT NULL,
    effect_owner_id TEXT NOT NULL,
    lease_id UUID NOT NULL,
    lease_hash BYTEA NOT NULL CHECK (octet_length(lease_hash) = 32),
    lease_generation BIGINT NOT NULL CHECK (lease_generation > 0),
    deployment_id TEXT NOT NULL,
    primary_id TEXT NOT NULL,
    database_system_identifier TEXT NOT NULL,
    postgres_timeline BIGINT NOT NULL CHECK (postgres_timeline > 0),
    database_epoch_number BIGINT,
    database_epoch_hash BYTEA,
    resource_owner_epoch BIGINT NOT NULL,
    resource_generation BIGINT NOT NULL,
    signer_key_id TEXT NOT NULL,
    signer_public_key BYTEA NOT NULL CHECK (octet_length(signer_public_key) = 32),
    signature BYTEA NOT NULL CHECK (octet_length(signature) = 64),
    signed_lease_json JSONB NOT NULL,
    external_highwater_receipt_digest BYTEA NOT NULL CHECK (
        octet_length(external_highwater_receipt_digest) = 32
    ),
    physical_fencing_receipt_digest BYTEA,
    admitted_scope TEXT NOT NULL CHECK (admitted_scope = 'directory_fabric_effect'),
    attested_at TIMESTAMPTZ NOT NULL DEFAULT clock_timestamp(),
    FOREIGN KEY (realm_id, resource_id, action_id, effect_owner_id, lease_id)
        REFERENCES myc_effect_owner.effect_owner_authority_write_lease_receipts (
            realm_id, resource_id, action_id, effect_owner_id, lease_id
        )
);

CREATE TABLE IF NOT EXISTS myc_effect_owner.effect_owner_epoch_authority_lease_attestations (
    realm_id TEXT NOT NULL,
    resource_id TEXT NOT NULL,
    action_id TEXT NOT NULL,
    effect_owner_id TEXT NOT NULL,
    new_owner_epoch BIGINT NOT NULL CHECK (new_owner_epoch > 0),
    new_generation BIGINT NOT NULL CHECK (new_generation > 0),
    lease_id UUID NOT NULL,
    lease_hash BYTEA NOT NULL CHECK (octet_length(lease_hash) = 32),
    lease_generation BIGINT NOT NULL CHECK (lease_generation > 0),
    deployment_id TEXT NOT NULL,
    primary_id TEXT NOT NULL,
    database_system_identifier TEXT NOT NULL,
    postgres_timeline BIGINT NOT NULL CHECK (postgres_timeline > 0),
    database_epoch_number BIGINT,
    database_epoch_hash BYTEA,
    signer_key_id TEXT NOT NULL,
    signer_public_key BYTEA NOT NULL CHECK (octet_length(signer_public_key) = 32),
    signature BYTEA NOT NULL CHECK (octet_length(signature) = 64),
    signed_lease_json JSONB NOT NULL,
    external_highwater_receipt_digest BYTEA NOT NULL CHECK (
        octet_length(external_highwater_receipt_digest) = 32
    ),
    physical_fencing_receipt_digest BYTEA NOT NULL CHECK (
        octet_length(physical_fencing_receipt_digest) = 32
        AND physical_fencing_receipt_digest <> decode(repeat('00', 32), 'hex')
    ),
    attested_at TIMESTAMPTZ NOT NULL DEFAULT clock_timestamp(),
    PRIMARY KEY (realm_id, resource_id, action_id, effect_owner_id, new_owner_epoch),
    FOREIGN KEY (realm_id, resource_id, action_id, effect_owner_id, lease_id)
        REFERENCES myc_effect_owner.effect_owner_authority_write_lease_receipts (
            realm_id, resource_id, action_id, effect_owner_id, lease_id
        )
);

CREATE OR REPLACE FUNCTION myc_effect_owner.authority_write_lease_json_u8_array(p_values JSONB)
RETURNS BYTEA
LANGUAGE plpgsql
IMMUTABLE
STRICT
SET search_path = pg_catalog, myc_effect_owner
AS $function$
DECLARE
    v_result BYTEA := decode('', 'hex');
    v_item JSONB;
    v_byte INTEGER;
BEGIN
    IF jsonb_typeof(p_values) <> 'array' THEN
        RAISE EXCEPTION 'signed lease byte field must be a JSON array'
            USING ERRCODE = '22023';
    END IF;
    FOR v_item IN SELECT value FROM jsonb_array_elements(p_values) LOOP
        IF jsonb_typeof(v_item) <> 'number' THEN
            RAISE EXCEPTION 'signed lease byte array contains a non-number'
                USING ERRCODE = '22023';
        END IF;
        v_byte := (v_item #>> '{}')::INTEGER;
        IF v_byte < 0 OR v_byte > 255 THEN
            RAISE EXCEPTION 'signed lease byte array contains a value outside 0..255'
                USING ERRCODE = '22023';
        END IF;
        v_result := v_result || decode(lpad(to_hex(v_byte), 2, '0'), 'hex');
    END LOOP;
    RETURN v_result;
END
$function$;

CREATE OR REPLACE FUNCTION myc_effect_owner.reject_authority_write_lease_mutation()
RETURNS TRIGGER
LANGUAGE plpgsql
SET search_path = pg_catalog, myc_effect_owner
AS $function$
BEGIN
    RAISE EXCEPTION '% is append-only', TG_TABLE_NAME
        USING ERRCODE = '55000';
END
$function$;

DROP TRIGGER IF EXISTS authority_lease_receipts_no_update_delete
ON myc_effect_owner.effect_owner_authority_write_lease_receipts;
CREATE TRIGGER authority_lease_receipts_no_update_delete
BEFORE UPDATE OR DELETE ON myc_effect_owner.effect_owner_authority_write_lease_receipts
FOR EACH ROW EXECUTE FUNCTION myc_effect_owner.reject_authority_write_lease_mutation();

DROP TRIGGER IF EXISTS authority_lease_receipts_no_truncate
ON myc_effect_owner.effect_owner_authority_write_lease_receipts;
CREATE TRIGGER authority_lease_receipts_no_truncate
BEFORE TRUNCATE ON myc_effect_owner.effect_owner_authority_write_lease_receipts
FOR EACH STATEMENT EXECUTE FUNCTION myc_effect_owner.reject_authority_write_lease_mutation();

DROP TRIGGER IF EXISTS effect_execution_authority_lease_no_update_delete
ON myc_effect_owner.effect_execution_authority_lease_attestations;
CREATE TRIGGER effect_execution_authority_lease_no_update_delete
BEFORE UPDATE OR DELETE ON myc_effect_owner.effect_execution_authority_lease_attestations
FOR EACH ROW EXECUTE FUNCTION myc_effect_owner.reject_authority_write_lease_mutation();

DROP TRIGGER IF EXISTS effect_execution_authority_lease_no_truncate
ON myc_effect_owner.effect_execution_authority_lease_attestations;
CREATE TRIGGER effect_execution_authority_lease_no_truncate
BEFORE TRUNCATE ON myc_effect_owner.effect_execution_authority_lease_attestations
FOR EACH STATEMENT EXECUTE FUNCTION myc_effect_owner.reject_authority_write_lease_mutation();

DROP TRIGGER IF EXISTS effect_owner_epoch_authority_lease_no_update_delete
ON myc_effect_owner.effect_owner_epoch_authority_lease_attestations;
CREATE TRIGGER effect_owner_epoch_authority_lease_no_update_delete
BEFORE UPDATE OR DELETE ON myc_effect_owner.effect_owner_epoch_authority_lease_attestations
FOR EACH ROW EXECUTE FUNCTION myc_effect_owner.reject_authority_write_lease_mutation();

DROP TRIGGER IF EXISTS effect_owner_epoch_authority_lease_no_truncate
ON myc_effect_owner.effect_owner_epoch_authority_lease_attestations;
CREATE TRIGGER effect_owner_epoch_authority_lease_no_truncate
BEFORE TRUNCATE ON myc_effect_owner.effect_owner_epoch_authority_lease_attestations
FOR EACH STATEMENT EXECUTE FUNCTION myc_effect_owner.reject_authority_write_lease_mutation();

CREATE OR REPLACE FUNCTION myc_effect_owner.install_effect_owner_authority_write_lease(
    p_realm_id TEXT,
    p_resource_id TEXT,
    p_action_id TEXT,
    p_effect_owner_id TEXT,
    p_resource_owner_epoch BIGINT,
    p_resource_generation BIGINT,
    p_signed_lease_hash BYTEA,
    p_external_highwater_receipt_digest BYTEA,
    p_physical_fencing_receipt_digest BYTEA,
    p_signed_lease_json JSONB
)
RETURNS UUID
LANGUAGE plpgsql
SECURITY DEFINER
SET search_path = pg_catalog, myc_effect_owner
AS $function$
DECLARE
    v_lease JSONB;
    v_scopes JSONB;
    v_lease_id UUID;
    v_lease_hash BYTEA;
    v_lease_generation BIGINT;
    v_deployment_id TEXT;
    v_primary_id TEXT;
    v_database_system_identifier TEXT;
    v_postgres_timeline BIGINT;
    v_database_epoch_number BIGINT;
    v_database_epoch_hash BYTEA;
    v_phase TEXT;
    v_issued_at TIMESTAMPTZ;
    v_not_before TIMESTAMPTZ;
    v_expires_at TIMESTAMPTZ;
    v_signer_key_id TEXT;
    v_signer_public_key BYTEA;
    v_signature BYTEA;
    v_current_system_identifier TEXT;
    v_current_timeline BIGINT;
    v_current_owner_epoch BIGINT;
    v_current_generation BIGINT;
    v_hwm_generation BIGINT;
    v_hwm_lease_id UUID;
    v_hwm_lease_hash BYTEA;
    v_hwm_external_digest BYTEA;
    v_state_generation BIGINT;
    v_state_lease_id UUID;
    v_state_lease_hash BYTEA;
    v_state_owner_epoch BIGINT;
    v_state_resource_generation BIGINT;
    v_state_primary_id TEXT;
    v_state_system_identifier TEXT;
    v_state_timeline BIGINT;
    v_receipt RECORD;
    v_rows INTEGER;
    v_has_effect_scope BOOLEAN;
    v_has_promotion_scope BOOLEAN;
BEGIN
    IF p_signed_lease_json IS NULL OR jsonb_typeof(p_signed_lease_json) <> 'object' THEN
        RAISE EXCEPTION 'signed authority lease JSON must be an object'
            USING ERRCODE = '22023';
    END IF;
    IF p_signed_lease_hash IS NULL OR octet_length(p_signed_lease_hash) <> 32
       OR p_signed_lease_hash = decode(repeat('00', 32), 'hex')
       OR p_external_highwater_receipt_digest IS NULL
       OR octet_length(p_external_highwater_receipt_digest) <> 32
       OR p_external_highwater_receipt_digest = decode(repeat('00', 32), 'hex') THEN
        RAISE EXCEPTION 'signed lease hash and external high-water receipt must be nonzero 32-byte commitments'
            USING ERRCODE = '22023';
    END IF;

    IF p_realm_id IS NULL OR p_resource_id IS NULL OR p_action_id IS NULL
       OR p_effect_owner_id IS NULL OR btrim(p_realm_id) = ''
       OR btrim(p_resource_id) = '' OR btrim(p_action_id) = ''
       OR btrim(p_effect_owner_id) = '' THEN
        RAISE EXCEPTION 'lease target scope identifiers must be nonempty'
            USING ERRCODE = '22023';
    END IF;

    IF p_signed_lease_json->>'protocol' IS DISTINCT FROM 'mycelix-desci-authority-write-lease'
       OR p_signed_lease_json->>'protocol_version' IS DISTINCT FROM '1'
       OR p_signed_lease_json->>'codec' IS DISTINCT FROM 'mycelix-canonical-binary-v1'
       OR p_signed_lease_json->>'schema_version' IS DISTINCT FROM '1' THEN
        RAISE EXCEPTION 'unsupported signed authority lease protocol envelope'
            USING ERRCODE = '22023';
    END IF;

    v_lease := p_signed_lease_json->'lease';
    IF v_lease IS NULL OR jsonb_typeof(v_lease) <> 'object' THEN
        RAISE EXCEPTION 'signed authority lease payload is missing'
            USING ERRCODE = '22023';
    END IF;

    v_lease_id := (v_lease->>'lease_id')::UUID;
    v_lease_generation := (v_lease->>'generation')::BIGINT;
    v_deployment_id := v_lease->>'deployment_id';
    v_primary_id := v_lease->>'primary_id';
    v_database_system_identifier := v_lease->>'database_system_identifier';
    v_postgres_timeline := (v_lease->>'postgres_timeline')::BIGINT;
    v_phase := v_lease->>'phase';
    v_database_epoch_number := NULLIF(v_lease->>'epoch_number', '')::BIGINT;
    v_database_epoch_hash := CASE
        WHEN v_lease->'epoch_hash' IS NULL OR v_lease->'epoch_hash' = 'null'::JSONB
            THEN NULL
        ELSE myc_effect_owner.authority_write_lease_json_u8_array(v_lease->'epoch_hash')
    END;
    v_issued_at := (v_lease->>'issued_at')::TIMESTAMPTZ;
    v_not_before := (v_lease->>'not_before')::TIMESTAMPTZ;
    v_expires_at := (v_lease->>'expires_at')::TIMESTAMPTZ;
    v_scopes := v_lease->'allowed_scopes';
    v_signer_key_id := p_signed_lease_json->>'signer_key_id';
    v_signer_public_key := myc_effect_owner.authority_write_lease_json_u8_array(
        p_signed_lease_json->'signer_public_key'
    );
    v_signature := myc_effect_owner.authority_write_lease_json_u8_array(
        p_signed_lease_json->'signature'
    );
    v_lease_hash := p_signed_lease_hash;

    IF v_lease_generation IS NULL OR v_lease_generation <= 0
       OR v_deployment_id IS NULL OR btrim(v_deployment_id) = ''
       OR v_primary_id IS NULL OR btrim(v_primary_id) = ''
       OR v_database_system_identifier IS NULL OR btrim(v_database_system_identifier) = ''
       OR v_postgres_timeline IS NULL OR v_postgres_timeline <= 0
       OR v_signer_key_id IS NULL OR btrim(v_signer_key_id) = ''
       OR octet_length(v_signer_public_key) <> 32
       OR octet_length(v_signature) <> 64
       OR v_scopes IS NULL OR jsonb_typeof(v_scopes) <> 'array'
       OR jsonb_array_length(v_scopes) = 0 THEN
        RAISE EXCEPTION 'signed authority lease has incomplete identity, signer or scope fields'
            USING ERRCODE = '22023';
    END IF;

    IF v_phase = 'bootstrap' THEN
        IF v_database_epoch_number IS NOT NULL OR v_database_epoch_hash IS NOT NULL THEN
            RAISE EXCEPTION 'bootstrap lease must not carry a governed database epoch'
                USING ERRCODE = '22023';
        END IF;
    ELSIF v_phase = 'epoch' THEN
        IF v_database_epoch_number IS NULL OR v_database_epoch_number <= 0
           OR v_database_epoch_hash IS NULL
           OR octet_length(v_database_epoch_hash) <> 32 THEN
            RAISE EXCEPTION 'epoch lease must bind a positive database epoch and 32-byte epoch hash'
                USING ERRCODE = '22023';
        END IF;
    ELSE
        RAISE EXCEPTION 'unsupported authority lease phase'
            USING ERRCODE = '22023';
    END IF;

    IF v_not_before < v_issued_at
       OR v_expires_at <= v_not_before
       OR v_expires_at - v_not_before > interval '15 minutes'
       OR clock_timestamp() < v_not_before
       OR clock_timestamp() >= v_expires_at THEN
        RAISE EXCEPTION 'authority lease is outside its active bounded validity interval'
            USING ERRCODE = 'P0001';
    END IF;

    v_has_effect_scope := v_scopes ? 'directory_fabric_effect';
    v_has_promotion_scope := v_scopes ? 'database_epoch_promotion';
    IF NOT v_has_effect_scope AND NOT v_has_promotion_scope THEN
        RAISE EXCEPTION 'authority lease lacks a Directory Fabric effect or epoch-promotion scope'
            USING ERRCODE = 'P0001';
    END IF;

    SELECT system.system_identifier::TEXT, checkpoint.timeline_id::BIGINT
      INTO v_current_system_identifier, v_current_timeline
      FROM pg_catalog.pg_control_system() AS system
      CROSS JOIN pg_catalog.pg_control_checkpoint() AS checkpoint;
    IF v_current_system_identifier IS DISTINCT FROM v_database_system_identifier
       OR v_current_timeline IS DISTINCT FROM v_postgres_timeline THEN
        RAISE EXCEPTION 'authority lease database system identifier or PostgreSQL timeline does not match this server'
            USING ERRCODE = 'P0001';
    END IF;

    -- Lock order is fence domain -> deployment high-water -> per-domain lease
    -- state. The mutation wrappers use worker claim -> fence domain -> lease
    -- state, matching migration 002/003 and avoiding lease/fence lock inversion.
    SELECT owner_epoch, generation
      INTO v_current_owner_epoch, v_current_generation
      FROM myc_effect_owner.effect_fence_state
     WHERE realm_id = p_realm_id
       AND resource_id = p_resource_id
       AND action_id = p_action_id
       AND effect_owner_id = p_effect_owner_id
     FOR UPDATE;
    IF NOT FOUND THEN
        RAISE EXCEPTION 'authority lease target has no initialized effect fence domain'
            USING ERRCODE = 'P0001';
    END IF;

    IF p_resource_owner_epoch = v_current_owner_epoch
       AND p_resource_generation = v_current_generation THEN
        IF NOT v_has_effect_scope THEN
            RAISE EXCEPTION 'current-position lease must authorize directory_fabric_effect'
                USING ERRCODE = 'P0001';
        END IF;
    ELSIF p_resource_owner_epoch > v_current_owner_epoch
       AND p_resource_generation = v_current_generation + 1 THEN
        IF NOT v_has_promotion_scope
           OR p_physical_fencing_receipt_digest IS NULL
           OR octet_length(p_physical_fencing_receipt_digest) <> 32
           OR p_physical_fencing_receipt_digest = decode(repeat('00', 32), 'hex') THEN
            RAISE EXCEPTION 'future owner position requires database_epoch_promotion scope and physical fencing receipt'
                USING ERRCODE = 'P0001';
        END IF;
    ELSE
        RAISE EXCEPTION 'lease resource owner epoch/generation is neither the current effect position nor the next promoted position'
            USING ERRCODE = 'P0001';
    END IF;

    SELECT lease_generation, lease_id, lease_hash, external_highwater_receipt_digest
      INTO v_hwm_generation, v_hwm_lease_id, v_hwm_lease_hash, v_hwm_external_digest
      FROM myc_effect_owner.authority_write_lease_highwater
     WHERE deployment_id = v_deployment_id
     FOR UPDATE;

    IF FOUND THEN
        IF v_hwm_generation > v_lease_generation THEN
            RAISE EXCEPTION 'stale authority lease generation %, external database high-water is %',
                v_lease_generation, v_hwm_generation
                USING ERRCODE = 'P0001';
        END IF;
        IF v_hwm_generation = v_lease_generation
           AND (
               v_hwm_lease_id IS DISTINCT FROM v_lease_id
               OR v_hwm_lease_hash IS DISTINCT FROM v_lease_hash
               OR v_hwm_external_digest IS DISTINCT FROM p_external_highwater_receipt_digest
           ) THEN
            RAISE EXCEPTION 'authority lease generation is already bound to different signed bytes or high-water evidence'
                USING ERRCODE = 'P0001';
        END IF;
        IF v_hwm_generation < v_lease_generation THEN
            UPDATE myc_effect_owner.authority_write_lease_highwater
               SET lease_generation = v_lease_generation,
                   lease_id = v_lease_id,
                   lease_hash = v_lease_hash,
                   owner_primary_id = v_primary_id,
                   database_system_identifier = v_database_system_identifier,
                   postgres_timeline = v_postgres_timeline,
                   database_epoch_number = v_database_epoch_number,
                   database_epoch_hash = v_database_epoch_hash,
                   external_highwater_receipt_digest = p_external_highwater_receipt_digest,
                   updated_at = clock_timestamp()
             WHERE deployment_id = v_deployment_id;
        END IF;
    ELSE
        INSERT INTO myc_effect_owner.authority_write_lease_highwater (
            deployment_id, lease_generation, lease_id, lease_hash,
            owner_primary_id, database_system_identifier, postgres_timeline,
            database_epoch_number, database_epoch_hash, external_highwater_receipt_digest
        )
        VALUES (
            v_deployment_id, v_lease_generation, v_lease_id, v_lease_hash,
            v_primary_id, v_database_system_identifier, v_postgres_timeline,
            v_database_epoch_number, v_database_epoch_hash, p_external_highwater_receipt_digest
        );
    END IF;

    SELECT lease_generation, lease_id, lease_hash,
           resource_owner_epoch, resource_generation,
           primary_id, database_system_identifier, postgres_timeline
      INTO v_state_generation, v_state_lease_id, v_state_lease_hash,
           v_state_owner_epoch, v_state_resource_generation,
           v_state_primary_id, v_state_system_identifier, v_state_timeline
      FROM myc_effect_owner.effect_owner_authority_write_lease_state
     WHERE realm_id = p_realm_id
       AND resource_id = p_resource_id
       AND action_id = p_action_id
       AND effect_owner_id = p_effect_owner_id
     FOR UPDATE;

    IF FOUND THEN
        IF v_state_generation > v_lease_generation THEN
            RAISE EXCEPTION 'per-resource authority lease state is ahead of supplied lease generation'
                USING ERRCODE = 'P0001';
        END IF;
        IF v_state_owner_epoch > p_resource_owner_epoch THEN
            RAISE EXCEPTION 'authority lease attempts to roll back the resource owner epoch'
                USING ERRCODE = 'P0001';
        END IF;
        IF v_state_owner_epoch = p_resource_owner_epoch
           AND (
               v_state_primary_id IS DISTINCT FROM v_primary_id
               OR v_state_system_identifier IS DISTINCT FROM v_database_system_identifier
               OR v_state_timeline IS DISTINCT FROM v_postgres_timeline
           ) THEN
            RAISE EXCEPTION 'primary, database identity or timeline cannot change without a higher resource owner epoch'
                USING ERRCODE = 'P0001';
        END IF;
        IF v_state_owner_epoch < p_resource_owner_epoch
           AND (
               p_resource_generation <> v_state_resource_generation + 1
               OR NOT v_has_promotion_scope
               OR p_physical_fencing_receipt_digest IS NULL
           ) THEN
            RAISE EXCEPTION 'resource owner epoch advance requires next generation, promotion scope and physical fencing'
                USING ERRCODE = 'P0001';
        END IF;
        IF v_state_generation = v_lease_generation
           AND (v_state_lease_id IS DISTINCT FROM v_lease_id
                OR v_state_lease_hash IS DISTINCT FROM v_lease_hash) THEN
            RAISE EXCEPTION 'per-resource authority lease generation fork detected'
                USING ERRCODE = 'P0001';
        END IF;
    END IF;

    INSERT INTO myc_effect_owner.effect_owner_authority_write_lease_receipts (
        realm_id, resource_id, action_id, effect_owner_id,
        lease_id, lease_hash, lease_generation, deployment_id,
        primary_id, database_system_identifier, postgres_timeline,
        database_epoch_number, database_epoch_hash,
        resource_owner_epoch, resource_generation,
        lease_not_before, lease_expires_at, allowed_scopes,
        signer_key_id, signer_public_key, signature, signed_lease_json,
        external_highwater_receipt_digest, physical_fencing_receipt_digest
    )
    VALUES (
        p_realm_id, p_resource_id, p_action_id, p_effect_owner_id,
        v_lease_id, v_lease_hash, v_lease_generation, v_deployment_id,
        v_primary_id, v_database_system_identifier, v_postgres_timeline,
        v_database_epoch_number, v_database_epoch_hash,
        p_resource_owner_epoch, p_resource_generation,
        v_not_before, v_expires_at, v_scopes,
        v_signer_key_id, v_signer_public_key, v_signature, p_signed_lease_json,
        p_external_highwater_receipt_digest, p_physical_fencing_receipt_digest
    )
    ON CONFLICT (realm_id, resource_id, action_id, effect_owner_id, lease_id)
    DO NOTHING;
    GET DIAGNOSTICS v_rows = ROW_COUNT;
    IF v_rows = 0 THEN
        SELECT lease_hash, lease_generation, deployment_id, primary_id,
               database_system_identifier, postgres_timeline,
               database_epoch_number, database_epoch_hash,
               resource_owner_epoch, resource_generation,
               lease_not_before, lease_expires_at, allowed_scopes,
               signer_key_id, signer_public_key, signature, signed_lease_json,
               external_highwater_receipt_digest, physical_fencing_receipt_digest
          INTO v_receipt
          FROM myc_effect_owner.effect_owner_authority_write_lease_receipts
         WHERE realm_id = p_realm_id
           AND resource_id = p_resource_id
           AND action_id = p_action_id
           AND effect_owner_id = p_effect_owner_id
           AND lease_id = v_lease_id;
        IF NOT FOUND
           OR v_receipt.lease_hash IS DISTINCT FROM v_lease_hash
           OR v_receipt.lease_generation IS DISTINCT FROM v_lease_generation
           OR v_receipt.deployment_id IS DISTINCT FROM v_deployment_id
           OR v_receipt.primary_id IS DISTINCT FROM v_primary_id
           OR v_receipt.database_system_identifier IS DISTINCT FROM v_database_system_identifier
           OR v_receipt.postgres_timeline IS DISTINCT FROM v_postgres_timeline
           OR v_receipt.database_epoch_number IS DISTINCT FROM v_database_epoch_number
           OR v_receipt.database_epoch_hash IS DISTINCT FROM v_database_epoch_hash
           OR v_receipt.resource_owner_epoch IS DISTINCT FROM p_resource_owner_epoch
           OR v_receipt.resource_generation IS DISTINCT FROM p_resource_generation
           OR v_receipt.lease_not_before IS DISTINCT FROM v_not_before
           OR v_receipt.lease_expires_at IS DISTINCT FROM v_expires_at
           OR v_receipt.allowed_scopes IS DISTINCT FROM v_scopes
           OR v_receipt.signer_key_id IS DISTINCT FROM v_signer_key_id
           OR v_receipt.signer_public_key IS DISTINCT FROM v_signer_public_key
           OR v_receipt.signature IS DISTINCT FROM v_signature
           OR v_receipt.signed_lease_json IS DISTINCT FROM p_signed_lease_json
           OR v_receipt.external_highwater_receipt_digest IS DISTINCT FROM p_external_highwater_receipt_digest
           OR v_receipt.physical_fencing_receipt_digest IS DISTINCT FROM p_physical_fencing_receipt_digest THEN
            RAISE EXCEPTION 'same lease ID is being reused with different authority, target or signed-envelope fields'
                USING ERRCODE = 'P0001';
        END IF;
    END IF;

    INSERT INTO myc_effect_owner.effect_owner_authority_write_lease_state (
        realm_id, resource_id, action_id, effect_owner_id,
        lease_id, lease_hash, lease_generation, deployment_id,
        primary_id, database_system_identifier, postgres_timeline,
        database_epoch_number, database_epoch_hash,
        resource_owner_epoch, resource_generation,
        lease_not_before, lease_expires_at, allowed_scopes,
        signer_key_id, signer_public_key, signature, signed_lease_json,
        external_highwater_receipt_digest, physical_fencing_receipt_digest
    )
    VALUES (
        p_realm_id, p_resource_id, p_action_id, p_effect_owner_id,
        v_lease_id, v_lease_hash, v_lease_generation, v_deployment_id,
        v_primary_id, v_database_system_identifier, v_postgres_timeline,
        v_database_epoch_number, v_database_epoch_hash,
        p_resource_owner_epoch, p_resource_generation,
        v_not_before, v_expires_at, v_scopes,
        v_signer_key_id, v_signer_public_key, v_signature, p_signed_lease_json,
        p_external_highwater_receipt_digest, p_physical_fencing_receipt_digest
    )
    ON CONFLICT (realm_id, resource_id, action_id, effect_owner_id)
    DO UPDATE SET
        lease_id = EXCLUDED.lease_id,
        lease_hash = EXCLUDED.lease_hash,
        lease_generation = EXCLUDED.lease_generation,
        deployment_id = EXCLUDED.deployment_id,
        primary_id = EXCLUDED.primary_id,
        database_system_identifier = EXCLUDED.database_system_identifier,
        postgres_timeline = EXCLUDED.postgres_timeline,
        database_epoch_number = EXCLUDED.database_epoch_number,
        database_epoch_hash = EXCLUDED.database_epoch_hash,
        resource_owner_epoch = EXCLUDED.resource_owner_epoch,
        resource_generation = EXCLUDED.resource_generation,
        lease_not_before = EXCLUDED.lease_not_before,
        lease_expires_at = EXCLUDED.lease_expires_at,
        allowed_scopes = EXCLUDED.allowed_scopes,
        signer_key_id = EXCLUDED.signer_key_id,
        signer_public_key = EXCLUDED.signer_public_key,
        signature = EXCLUDED.signature,
        signed_lease_json = EXCLUDED.signed_lease_json,
        external_highwater_receipt_digest = EXCLUDED.external_highwater_receipt_digest,
        physical_fencing_receipt_digest = EXCLUDED.physical_fencing_receipt_digest,
        updated_at = clock_timestamp();

    RETURN v_lease_id;
END
$function$;

CREATE OR REPLACE FUNCTION myc_effect_owner.apply_owner_epoch_claim_fenced_balance_delta_with_authority_lease(
    p_realm_id TEXT,
    p_resource_id TEXT,
    p_action_id TEXT,
    p_effect_owner_id TEXT,
    p_object_id TEXT,
    p_currency_code TEXT,
    p_owner_epoch BIGINT,
    p_fence_generation BIGINT,
    p_idempotency_key_digest BYTEA,
    p_logical_operation_id TEXT,
    p_request_digest BYTEA,
    p_decision_receipt_digest BYTEA,
    p_payload_digest BYTEA,
    p_delta_minor BIGINT,
    p_worker_id TEXT,
    p_claim_epoch BIGINT,
    p_claim_proof_digest BYTEA,
    p_decision_clock_profile_id TEXT,
    p_decision_issued_at TIMESTAMPTZ,
    p_decision_expires_at TIMESTAMPTZ,
    p_lease_id UUID,
    p_lease_hash BYTEA,
    p_lease_generation BIGINT
)
RETURNS TABLE (
    replayed BOOLEAN,
    effect_id UUID,
    accepted_owner_epoch BIGINT,
    accepted_generation BIGINT,
    accepted_worker_id TEXT,
    accepted_claim_epoch BIGINT,
    execution_attestation_id UUID,
    authority_lease_id UUID,
    authority_lease_hash BYTEA,
    authority_lease_generation BIGINT
)
LANGUAGE plpgsql
SECURITY DEFINER
SET search_path = pg_catalog, myc_effect_owner
AS $function$
DECLARE
    v_result RECORD;
    v_claim myc_effect_owner.worker_claim_state%ROWTYPE;
    v_owner_epoch BIGINT;
    v_generation BIGINT;
    v_lease myc_effect_owner.effect_owner_authority_write_lease_state%ROWTYPE;
    v_historical myc_effect_owner.effect_execution_authority_lease_attestations%ROWTYPE;
BEGIN
    -- Preserve worker-claim -> owner-fence -> lease-state lock order.
    SELECT *
      INTO v_claim
      FROM myc_effect_owner.worker_claim_state
     WHERE realm_id = p_realm_id
       AND resource_id = p_resource_id
       AND action_id = p_action_id
       AND effect_owner_id = p_effect_owner_id
       AND logical_operation_id = p_logical_operation_id
       AND worker_id = p_worker_id
       AND claim_epoch = p_claim_epoch
       AND claim_proof_digest = p_claim_proof_digest
     FOR SHARE;
    IF NOT FOUND THEN
        RAISE EXCEPTION 'worker claim is missing or does not match exact authority-lease command'
            USING ERRCODE = 'P0001';
    END IF;

    SELECT owner_epoch, generation
      INTO v_owner_epoch, v_generation
      FROM myc_effect_owner.effect_fence_state
     WHERE realm_id = p_realm_id
       AND resource_id = p_resource_id
       AND action_id = p_action_id
       AND effect_owner_id = p_effect_owner_id
     FOR SHARE;
    IF NOT FOUND OR v_owner_epoch IS DISTINCT FROM p_owner_epoch
       OR v_generation IS DISTINCT FROM p_fence_generation THEN
        RAISE EXCEPTION 'authority-lease command is not at exact current owner epoch/resource generation'
            USING ERRCODE = 'P0001';
    END IF;

    SELECT *
      INTO v_lease
      FROM myc_effect_owner.effect_owner_authority_write_lease_state
     WHERE realm_id = p_realm_id
       AND resource_id = p_resource_id
       AND action_id = p_action_id
       AND effect_owner_id = p_effect_owner_id
     FOR SHARE;
    IF NOT FOUND
       OR v_lease.lease_id IS DISTINCT FROM p_lease_id
       OR v_lease.lease_hash IS DISTINCT FROM p_lease_hash
       OR v_lease.lease_generation IS DISTINCT FROM p_lease_generation
       OR v_lease.resource_owner_epoch IS DISTINCT FROM p_owner_epoch
       OR v_lease.resource_generation IS DISTINCT FROM p_fence_generation
       OR NOT (v_lease.allowed_scopes ? 'directory_fabric_effect')
       OR clock_timestamp() < v_lease.lease_not_before
       OR clock_timestamp() >= v_lease.lease_expires_at THEN
        RAISE EXCEPTION 'no active exact-target signed authority lease permits Directory Fabric effect mutation'
            USING ERRCODE = 'P0001';
    END IF;

    SELECT *
      INTO v_result
      FROM myc_effect_owner.apply_owner_epoch_claim_fenced_balance_delta(
        p_realm_id, p_resource_id, p_action_id, p_effect_owner_id,
        p_object_id, p_currency_code, p_owner_epoch, p_fence_generation,
        p_idempotency_key_digest, p_logical_operation_id,
        p_request_digest, p_decision_receipt_digest, p_payload_digest,
        p_delta_minor, p_worker_id, p_claim_epoch, p_claim_proof_digest,
        p_decision_clock_profile_id, p_decision_issued_at, p_decision_expires_at
      );

    IF v_result.replayed THEN
        SELECT *
          INTO v_historical
          FROM myc_effect_owner.effect_execution_authority_lease_attestations
         WHERE effect_id = v_result.effect_id;
        IF NOT FOUND THEN
            RAISE EXCEPTION 'replayed effect has no historical signed authority-lease attestation; reconciliation required'
                USING ERRCODE = 'P0001';
        END IF;
        RETURN QUERY SELECT
            v_result.replayed, v_result.effect_id,
            v_result.accepted_owner_epoch, v_result.accepted_generation,
            v_result.accepted_worker_id, v_result.accepted_claim_epoch,
            v_result.execution_attestation_id,
            v_historical.lease_id, v_historical.lease_hash,
            v_historical.lease_generation;
        RETURN;
    END IF;

    INSERT INTO myc_effect_owner.effect_execution_authority_lease_attestations (
        attestation_id, effect_id, realm_id, resource_id, action_id, effect_owner_id,
        lease_id, lease_hash, lease_generation, deployment_id, primary_id,
        database_system_identifier, postgres_timeline, database_epoch_number,
        database_epoch_hash, resource_owner_epoch, resource_generation,
        signer_key_id, signer_public_key, signature, signed_lease_json,
        external_highwater_receipt_digest, physical_fencing_receipt_digest, admitted_scope
    )
    VALUES (
        v_result.execution_attestation_id, v_result.effect_id,
        p_realm_id, p_resource_id, p_action_id, p_effect_owner_id,
        v_lease.lease_id, v_lease.lease_hash, v_lease.lease_generation,
        v_lease.deployment_id, v_lease.primary_id,
        v_lease.database_system_identifier, v_lease.postgres_timeline,
        v_lease.database_epoch_number, v_lease.database_epoch_hash,
        p_owner_epoch, p_fence_generation,
        v_lease.signer_key_id, v_lease.signer_public_key, v_lease.signature,
        v_lease.signed_lease_json, v_lease.external_highwater_receipt_digest,
        v_lease.physical_fencing_receipt_digest, 'directory_fabric_effect'
    );

    RETURN QUERY SELECT
        v_result.replayed, v_result.effect_id,
        v_result.accepted_owner_epoch, v_result.accepted_generation,
        v_result.accepted_worker_id, v_result.accepted_claim_epoch,
        v_result.execution_attestation_id,
        v_lease.lease_id, v_lease.lease_hash, v_lease.lease_generation;
END
$function$;

CREATE OR REPLACE FUNCTION myc_effect_owner.advance_effect_owner_epoch_with_authority_lease(
    p_realm_id TEXT,
    p_resource_id TEXT,
    p_action_id TEXT,
    p_effect_owner_id TEXT,
    p_new_owner_epoch BIGINT,
    p_epoch_authority_receipt_digest BYTEA,
    p_physical_fencing_receipt_digest BYTEA,
    p_lease_id UUID,
    p_lease_hash BYTEA,
    p_lease_generation BIGINT
)
RETURNS TABLE (
    accepted_owner_epoch BIGINT,
    accepted_generation BIGINT,
    authority_lease_id UUID,
    authority_lease_hash BYTEA,
    authority_lease_generation BIGINT
)
LANGUAGE plpgsql
SECURITY DEFINER
SET search_path = pg_catalog, myc_effect_owner
AS $function$
DECLARE
    v_current_owner_epoch BIGINT;
    v_current_generation BIGINT;
    v_lease myc_effect_owner.effect_owner_authority_write_lease_state%ROWTYPE;
    v_advanced RECORD;
    v_historical myc_effect_owner.effect_owner_epoch_authority_lease_attestations%ROWTYPE;
BEGIN
    SELECT owner_epoch, generation
      INTO v_current_owner_epoch, v_current_generation
      FROM myc_effect_owner.effect_fence_state
     WHERE realm_id = p_realm_id
       AND resource_id = p_resource_id
       AND action_id = p_action_id
       AND effect_owner_id = p_effect_owner_id
     FOR UPDATE;
    IF NOT FOUND THEN
        RAISE EXCEPTION 'owner epoch domain is not initialized'
            USING ERRCODE = 'P0001';
    END IF;

    IF v_current_owner_epoch = p_new_owner_epoch THEN
        SELECT *
          INTO v_historical
          FROM myc_effect_owner.effect_owner_epoch_authority_lease_attestations
         WHERE realm_id = p_realm_id
           AND resource_id = p_resource_id
           AND action_id = p_action_id
           AND effect_owner_id = p_effect_owner_id
           AND new_owner_epoch = p_new_owner_epoch;
        IF NOT FOUND
           OR v_historical.lease_id IS DISTINCT FROM p_lease_id
           OR v_historical.lease_hash IS DISTINCT FROM p_lease_hash
           OR v_historical.lease_generation IS DISTINCT FROM p_lease_generation THEN
            RAISE EXCEPTION 'replayed owner epoch transition does not match original authority lease attestation'
                USING ERRCODE = 'P0001';
        END IF;
        RETURN QUERY SELECT
            p_new_owner_epoch, v_historical.new_generation,
            v_historical.lease_id, v_historical.lease_hash,
            v_historical.lease_generation;
        RETURN;
    END IF;

    SELECT *
      INTO v_lease
      FROM myc_effect_owner.effect_owner_authority_write_lease_state
     WHERE realm_id = p_realm_id
       AND resource_id = p_resource_id
       AND action_id = p_action_id
       AND effect_owner_id = p_effect_owner_id
     FOR SHARE;

    IF NOT FOUND
       OR v_lease.lease_id IS DISTINCT FROM p_lease_id
       OR v_lease.lease_hash IS DISTINCT FROM p_lease_hash
       OR v_lease.lease_generation IS DISTINCT FROM p_lease_generation
       OR v_lease.resource_owner_epoch IS DISTINCT FROM p_new_owner_epoch
       OR v_lease.resource_generation IS DISTINCT FROM v_current_generation + 1
       OR NOT (v_lease.allowed_scopes ? 'database_epoch_promotion')
       OR v_lease.physical_fencing_receipt_digest IS DISTINCT FROM p_physical_fencing_receipt_digest
       OR clock_timestamp() < v_lease.lease_not_before
       OR clock_timestamp() >= v_lease.lease_expires_at THEN
        RAISE EXCEPTION 'no active exact-target promotion lease and physical-fencing evidence authorizes owner epoch transition'
            USING ERRCODE = 'P0001';
    END IF;

    SELECT *
      INTO v_advanced
      FROM myc_effect_owner.advance_effect_owner_epoch(
        p_realm_id, p_resource_id, p_action_id, p_effect_owner_id,
        p_new_owner_epoch, p_epoch_authority_receipt_digest,
        p_physical_fencing_receipt_digest
      );

    INSERT INTO myc_effect_owner.effect_owner_epoch_authority_lease_attestations (
        realm_id, resource_id, action_id, effect_owner_id,
        new_owner_epoch, new_generation,
        lease_id, lease_hash, lease_generation, deployment_id, primary_id,
        database_system_identifier, postgres_timeline, database_epoch_number,
        database_epoch_hash, signer_key_id, signer_public_key, signature,
        signed_lease_json, external_highwater_receipt_digest,
        physical_fencing_receipt_digest
    )
    VALUES (
        p_realm_id, p_resource_id, p_action_id, p_effect_owner_id,
        v_advanced.accepted_owner_epoch, v_advanced.accepted_generation,
        v_lease.lease_id, v_lease.lease_hash, v_lease.lease_generation,
        v_lease.deployment_id, v_lease.primary_id,
        v_lease.database_system_identifier, v_lease.postgres_timeline,
        v_lease.database_epoch_number, v_lease.database_epoch_hash,
        v_lease.signer_key_id, v_lease.signer_public_key, v_lease.signature,
        v_lease.signed_lease_json, v_lease.external_highwater_receipt_digest,
        v_lease.physical_fencing_receipt_digest
    );

    RETURN QUERY SELECT
        v_advanced.accepted_owner_epoch, v_advanced.accepted_generation,
        v_lease.lease_id, v_lease.lease_hash, v_lease.lease_generation;
END
$function$;

REVOKE ALL ON ALL TABLES IN SCHEMA myc_effect_owner
    FROM myc_dir_authority_lease_controller, myc_dir_effect_worker;
REVOKE ALL ON ALL FUNCTIONS IN SCHEMA myc_effect_owner
    FROM myc_dir_authority_lease_controller, myc_dir_effect_worker;
GRANT USAGE ON SCHEMA myc_effect_owner
    TO myc_dir_authority_lease_controller, myc_dir_effect_worker;

GRANT EXECUTE ON FUNCTION myc_effect_owner.install_effect_owner_authority_write_lease(
    TEXT, TEXT, TEXT, TEXT, BIGINT, BIGINT, BYTEA, BYTEA, BYTEA, JSONB
) TO myc_dir_authority_lease_controller;

GRANT EXECUTE ON FUNCTION myc_effect_owner.advance_effect_owner_epoch_with_authority_lease(
    TEXT, TEXT, TEXT, TEXT, BIGINT, BYTEA, BYTEA, UUID, BYTEA, BIGINT
) TO myc_dir_authority_lease_controller;

GRANT EXECUTE ON FUNCTION myc_effect_owner.apply_owner_epoch_claim_fenced_balance_delta_with_authority_lease(
    TEXT, TEXT, TEXT, TEXT, TEXT, TEXT, BIGINT, BIGINT, BYTEA, TEXT,
    BYTEA, BYTEA, BYTEA, BIGINT, TEXT, BIGINT, BYTEA, TEXT, TIMESTAMPTZ,
    TIMESTAMPTZ, UUID, BYTEA, BIGINT
) TO myc_dir_effect_worker;

REVOKE ALL ON ALL TABLES IN SCHEMA myc_effect_owner FROM PUBLIC;
REVOKE ALL ON ALL FUNCTIONS IN SCHEMA myc_effect_owner FROM PUBLIC;

COMMENT ON TABLE myc_effect_owner.authority_write_lease_highwater IS
'Database-local mirror of the external deployment lease high-water. The external signed high-water receipt remains authoritative and must be outside the PostgreSQL backup/PITR rollback domain.';
COMMENT ON TABLE myc_effect_owner.effect_owner_authority_write_lease_receipts IS
'Append-only exact signed authority lease envelopes admitted by the trusted lease controller for an effect-owner fence domain.';
COMMENT ON TABLE myc_effect_owner.effect_execution_authority_lease_attestations IS
'Append-only binding between each committed effect/execution attestation and the exact signed DirectoryFabricEffect lease accepted by the owner transaction.';
COMMENT ON TABLE myc_effect_owner.effect_owner_epoch_authority_lease_attestations IS
'Append-only binding between each owner epoch transition and the exact signed DatabaseEpochPromotion lease accepted by the transition transaction.';

COMMIT;
