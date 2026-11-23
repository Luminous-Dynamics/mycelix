-- Mycelix Directory Fabric external authority proof receipt store v0.1.
--
-- This migration persists full purpose-separated authority proof envelopes
-- and links them to the exact signed lease receipts. Cryptographic verification
-- and external anti-rollback checks are performed by the trusted Rust
-- authority-proof controller before insertion. PostgreSQL enforces envelope
-- shape, issuer/purpose mapping, monotonic local mirror, exact lease/payload
-- correspondence and append-only evidence links.
--
-- A local PostgreSQL row is NOT the external anti-rollback authority. After
-- PITR/restore the Rust controller must compare its independent proof sequence
-- store before any proof is admitted or any effect-success ACK is issued.

BEGIN;

CREATE TABLE IF NOT EXISTS myc_effect_owner.authority_proof_receipts (
    proof_id UUID PRIMARY KEY,
    purpose TEXT NOT NULL CHECK (purpose IN (
        'lease_high_water',
        'physical_fencing',
        'promotion_eligibility'
    )),
    issuer_role TEXT NOT NULL CHECK (issuer_role IN (
        'lease_high_water_authority',
        'physical_fencing_attestor',
        'promotion_controller'
    )),
    deployment_id TEXT NOT NULL CHECK (btrim(deployment_id) <> ''),
    database_system_identifier TEXT NOT NULL CHECK (btrim(database_system_identifier) <> ''),
    primary_id TEXT NOT NULL CHECK (btrim(primary_id) <> ''),
    postgres_timeline BIGINT NOT NULL CHECK (postgres_timeline > 0),
    owner_epoch BIGINT NOT NULL CHECK (owner_epoch > 0),
    proof_sequence BIGINT NOT NULL CHECK (proof_sequence > 0),
    issued_at TIMESTAMPTZ NOT NULL,
    not_before TIMESTAMPTZ NOT NULL,
    expires_at TIMESTAMPTZ NOT NULL,
    issuer_key_id TEXT NOT NULL CHECK (btrim(issuer_key_id) <> ''),
    issuer_public_key BYTEA NOT NULL CHECK (octet_length(issuer_public_key) = 32),
    signature BYTEA NOT NULL CHECK (octet_length(signature) = 64),
    canonical_proof_digest BYTEA NOT NULL CHECK (
        octet_length(canonical_proof_digest) = 32
        AND canonical_proof_digest <> decode(repeat('00', 32), 'hex')
    ),
    signed_receipt_hash BYTEA NOT NULL CHECK (
        octet_length(signed_receipt_hash) = 32
        AND signed_receipt_hash <> decode(repeat('00', 32), 'hex')
    ),
    signed_receipt_json JSONB NOT NULL CHECK (jsonb_typeof(signed_receipt_json) = 'object'),
    external_sequence_receipt_digest BYTEA NOT NULL CHECK (
        octet_length(external_sequence_receipt_digest) = 32
        AND external_sequence_receipt_digest <> decode(repeat('00', 32), 'hex')
    ),
    accepted_by_controller TEXT NOT NULL CHECK (btrim(accepted_by_controller) <> ''),
    accepted_at TIMESTAMPTZ NOT NULL DEFAULT clock_timestamp(),
    CHECK (not_before >= issued_at),
    CHECK (expires_at > not_before),
    CHECK (expires_at - not_before <= interval '10 minutes'),
    CHECK (
        (purpose = 'lease_high_water' AND issuer_role = 'lease_high_water_authority')
        OR (purpose = 'physical_fencing' AND issuer_role = 'physical_fencing_attestor')
        OR (purpose = 'promotion_eligibility' AND issuer_role = 'promotion_controller')
    ),
    UNIQUE (deployment_id, purpose, issuer_key_id, proof_sequence)
);

CREATE TABLE IF NOT EXISTS myc_effect_owner.authority_proof_sequence_highwater (
    deployment_id TEXT NOT NULL,
    purpose TEXT NOT NULL,
    issuer_key_id TEXT NOT NULL,
    last_accepted_sequence BIGINT NOT NULL CHECK (last_accepted_sequence > 0),
    last_proof_id UUID NOT NULL
        REFERENCES myc_effect_owner.authority_proof_receipts(proof_id),
    last_signed_receipt_hash BYTEA NOT NULL CHECK (octet_length(last_signed_receipt_hash) = 32),
    external_sequence_receipt_digest BYTEA NOT NULL CHECK (
        octet_length(external_sequence_receipt_digest) = 32
    ),
    updated_at TIMESTAMPTZ NOT NULL DEFAULT clock_timestamp(),
    PRIMARY KEY (deployment_id, purpose, issuer_key_id)
);

CREATE TABLE IF NOT EXISTS myc_effect_owner.effect_owner_authority_lease_proof_links (
    realm_id TEXT NOT NULL,
    resource_id TEXT NOT NULL,
    action_id TEXT NOT NULL,
    effect_owner_id TEXT NOT NULL,
    lease_id UUID NOT NULL,
    lease_highwater_proof_id UUID NOT NULL
        REFERENCES myc_effect_owner.authority_proof_receipts(proof_id),
    physical_fencing_proof_id UUID
        REFERENCES myc_effect_owner.authority_proof_receipts(proof_id),
    promotion_eligibility_proof_id UUID
        REFERENCES myc_effect_owner.authority_proof_receipts(proof_id),
    lease_hash BYTEA NOT NULL CHECK (octet_length(lease_hash) = 32),
    lease_generation BIGINT NOT NULL CHECK (lease_generation > 0),
    bound_at TIMESTAMPTZ NOT NULL DEFAULT clock_timestamp(),
    PRIMARY KEY (realm_id, resource_id, action_id, effect_owner_id, lease_id),
    FOREIGN KEY (realm_id, resource_id, action_id, effect_owner_id, lease_id)
        REFERENCES myc_effect_owner.effect_owner_authority_write_lease_receipts (
            realm_id, resource_id, action_id, effect_owner_id, lease_id
        ),
    CHECK (
        (physical_fencing_proof_id IS NULL AND promotion_eligibility_proof_id IS NULL)
        OR (physical_fencing_proof_id IS NOT NULL AND promotion_eligibility_proof_id IS NOT NULL)
    )
);

CREATE OR REPLACE FUNCTION myc_effect_owner.authority_proof_json_u8_array(p_values JSONB)
RETURNS BYTEA
LANGUAGE plpgsql
IMMUTABLE
STRICT
SET search_path = pg_catalog, myc_effect_owner
AS $function$
BEGIN
    RETURN myc_effect_owner.authority_write_lease_json_u8_array(p_values);
END
$function$;

DROP TRIGGER IF EXISTS authority_proof_receipts_no_update_delete
ON myc_effect_owner.authority_proof_receipts;
CREATE TRIGGER authority_proof_receipts_no_update_delete
BEFORE UPDATE OR DELETE ON myc_effect_owner.authority_proof_receipts
FOR EACH ROW EXECUTE FUNCTION myc_effect_owner.reject_authority_write_lease_mutation();

DROP TRIGGER IF EXISTS authority_proof_receipts_no_truncate
ON myc_effect_owner.authority_proof_receipts;
CREATE TRIGGER authority_proof_receipts_no_truncate
BEFORE TRUNCATE ON myc_effect_owner.authority_proof_receipts
FOR EACH STATEMENT EXECUTE FUNCTION myc_effect_owner.reject_authority_write_lease_mutation();

DROP TRIGGER IF EXISTS authority_proof_links_no_update_delete
ON myc_effect_owner.effect_owner_authority_lease_proof_links;
CREATE TRIGGER authority_proof_links_no_update_delete
BEFORE UPDATE OR DELETE ON myc_effect_owner.effect_owner_authority_lease_proof_links
FOR EACH ROW EXECUTE FUNCTION myc_effect_owner.reject_authority_write_lease_mutation();

DROP TRIGGER IF EXISTS authority_proof_links_no_truncate
ON myc_effect_owner.effect_owner_authority_lease_proof_links;
CREATE TRIGGER authority_proof_links_no_truncate
BEFORE TRUNCATE ON myc_effect_owner.effect_owner_authority_lease_proof_links
FOR EACH STATEMENT EXECUTE FUNCTION myc_effect_owner.reject_authority_write_lease_mutation();

CREATE OR REPLACE FUNCTION myc_effect_owner.install_authority_proof_receipt(
    p_signed_receipt_json JSONB,
    p_canonical_proof_digest BYTEA,
    p_signed_receipt_hash BYTEA,
    p_external_sequence_receipt_digest BYTEA,
    p_accepted_by_controller TEXT
)
RETURNS UUID
LANGUAGE plpgsql
SECURITY DEFINER
SET search_path = pg_catalog, myc_effect_owner
AS $function$
DECLARE
    v_receipt JSONB;
    v_payload JSONB;
    v_proof_id UUID;
    v_purpose TEXT;
    v_issuer_role TEXT;
    v_deployment_id TEXT;
    v_system_identifier TEXT;
    v_primary_id TEXT;
    v_timeline BIGINT;
    v_owner_epoch BIGINT;
    v_sequence BIGINT;
    v_issued_at TIMESTAMPTZ;
    v_not_before TIMESTAMPTZ;
    v_expires_at TIMESTAMPTZ;
    v_issuer_key_id TEXT;
    v_issuer_public_key BYTEA;
    v_signature BYTEA;
    v_existing_hash BYTEA;
    v_existing_canonical_digest BYTEA;
    v_existing_external_digest BYTEA;
    v_existing_json JSONB;
    v_existing_sequence BIGINT;
    v_existing_proof_id UUID;
    v_rows INTEGER;
BEGIN
    IF p_signed_receipt_json IS NULL
       OR jsonb_typeof(p_signed_receipt_json) <> 'object'
       OR p_canonical_proof_digest IS NULL
       OR octet_length(p_canonical_proof_digest) <> 32
       OR p_canonical_proof_digest = decode(repeat('00', 32), 'hex')
       OR p_signed_receipt_hash IS NULL
       OR octet_length(p_signed_receipt_hash) <> 32
       OR p_signed_receipt_hash = decode(repeat('00', 32), 'hex')
       OR p_external_sequence_receipt_digest IS NULL
       OR octet_length(p_external_sequence_receipt_digest) <> 32
       OR p_external_sequence_receipt_digest = decode(repeat('00', 32), 'hex')
       OR p_accepted_by_controller IS NULL
       OR btrim(p_accepted_by_controller) = '' THEN
        RAISE EXCEPTION 'authority proof envelope, hashes, external sequence receipt and controller identity must be complete'
            USING ERRCODE = '22023';
    END IF;

    IF p_signed_receipt_json->>'protocol' IS DISTINCT FROM 'mycelix-authority-proof'
       OR p_signed_receipt_json->>'version' IS DISTINCT FROM '1' THEN
        RAISE EXCEPTION 'unsupported authority proof protocol envelope'
            USING ERRCODE = '22023';
    END IF;

    v_receipt := p_signed_receipt_json->'receipt';
    IF v_receipt IS NULL OR jsonb_typeof(v_receipt) <> 'object' THEN
        RAISE EXCEPTION 'authority proof receipt payload is missing'
            USING ERRCODE = '22023';
    END IF;
    v_payload := v_receipt->'payload';
    IF v_payload IS NULL OR jsonb_typeof(v_payload) <> 'object' THEN
        RAISE EXCEPTION 'authority proof typed payload is missing'
            USING ERRCODE = '22023';
    END IF;

    v_proof_id := (v_receipt->>'proof_id')::UUID;
    v_purpose := v_receipt->>'purpose';
    v_issuer_role := v_receipt->>'issuer_role';
    v_deployment_id := v_receipt->>'deployment_id';
    v_system_identifier := v_receipt->>'database_system_identifier';
    v_primary_id := v_receipt->>'primary_id';
    v_timeline := (v_receipt->>'postgres_timeline')::BIGINT;
    v_owner_epoch := (v_receipt->>'owner_epoch')::BIGINT;
    v_sequence := (v_receipt->>'sequence')::BIGINT;
    v_issued_at := (v_receipt->>'issued_at')::TIMESTAMPTZ;
    v_not_before := (v_receipt->>'not_before')::TIMESTAMPTZ;
    v_expires_at := (v_receipt->>'expires_at')::TIMESTAMPTZ;
    v_issuer_key_id := p_signed_receipt_json->>'issuer_key_id';
    v_issuer_public_key := myc_effect_owner.authority_proof_json_u8_array(
        p_signed_receipt_json->'issuer_public_key'
    );
    v_signature := myc_effect_owner.authority_proof_json_u8_array(
        p_signed_receipt_json->'signature'
    );

    IF v_proof_id IS NULL
       OR v_purpose NOT IN ('lease_high_water', 'physical_fencing', 'promotion_eligibility')
       OR v_issuer_role IS NULL
       OR v_deployment_id IS NULL OR btrim(v_deployment_id) = ''
       OR v_system_identifier IS NULL OR btrim(v_system_identifier) = ''
       OR v_primary_id IS NULL OR btrim(v_primary_id) = ''
       OR v_timeline IS NULL OR v_timeline <= 0
       OR v_owner_epoch IS NULL OR v_owner_epoch <= 0
       OR v_sequence IS NULL OR v_sequence <= 0
       OR v_issuer_key_id IS NULL OR btrim(v_issuer_key_id) = ''
       OR octet_length(v_issuer_public_key) <> 32
       OR octet_length(v_signature) <> 64
       OR v_not_before < v_issued_at
       OR v_expires_at <= v_not_before
       OR v_expires_at - v_not_before > interval '10 minutes'
       OR clock_timestamp() < v_not_before
       OR clock_timestamp() >= v_expires_at THEN
        RAISE EXCEPTION 'authority proof identity, signer, sequence or validity fields are invalid'
            USING ERRCODE = 'P0001';
    END IF;

    IF (v_purpose = 'lease_high_water' AND v_issuer_role <> 'lease_high_water_authority')
       OR (v_purpose = 'physical_fencing' AND v_issuer_role <> 'physical_fencing_attestor')
       OR (v_purpose = 'promotion_eligibility' AND v_issuer_role <> 'promotion_controller') THEN
        RAISE EXCEPTION 'authority proof purpose and issuer role are inconsistent'
            USING ERRCODE = 'P0001';
    END IF;
    IF v_payload->>'kind' IS DISTINCT FROM v_purpose THEN
        RAISE EXCEPTION 'authority proof payload kind differs from the signed purpose'
            USING ERRCODE = 'P0001';
    END IF;

    -- Idempotent exact replay only. A proof ID cannot be rebound to different
    -- signed bytes or a different external sequence receipt.
    SELECT signed_receipt_hash, canonical_proof_digest,
           external_sequence_receipt_digest, signed_receipt_json
      INTO v_existing_hash, v_existing_canonical_digest,
           v_existing_external_digest, v_existing_json
      FROM myc_effect_owner.authority_proof_receipts
     WHERE proof_id = v_proof_id;
    IF FOUND THEN
        IF v_existing_hash IS DISTINCT FROM p_signed_receipt_hash
           OR v_existing_canonical_digest IS DISTINCT FROM p_canonical_proof_digest
           OR v_existing_external_digest IS DISTINCT FROM p_external_sequence_receipt_digest
           OR v_existing_json IS DISTINCT FROM p_signed_receipt_json THEN
            RAISE EXCEPTION 'authority proof ID is already bound to different signed bytes or external sequence evidence'
                USING ERRCODE = 'P0001';
        END IF;
        RETURN v_proof_id;
    END IF;

    -- Compare-and-advance the local mirror. The external sequence receipt is
    -- authoritative; this local row is a transactional duplicate-detection aid.
    INSERT INTO myc_effect_owner.authority_proof_receipts (
        proof_id, purpose, issuer_role, deployment_id, database_system_identifier,
        primary_id, postgres_timeline, owner_epoch, proof_sequence,
        issued_at, not_before, expires_at, issuer_key_id, issuer_public_key,
        signature, canonical_proof_digest, signed_receipt_hash, signed_receipt_json,
        external_sequence_receipt_digest, accepted_by_controller
    )
    VALUES (
        v_proof_id, v_purpose, v_issuer_role, v_deployment_id, v_system_identifier,
        v_primary_id, v_timeline, v_owner_epoch, v_sequence,
        v_issued_at, v_not_before, v_expires_at, v_issuer_key_id, v_issuer_public_key,
        v_signature, p_canonical_proof_digest, p_signed_receipt_hash, p_signed_receipt_json,
        p_external_sequence_receipt_digest, p_accepted_by_controller
    );
    INSERT INTO myc_effect_owner.authority_proof_sequence_highwater (
        deployment_id, purpose, issuer_key_id, last_accepted_sequence,
        last_proof_id, last_signed_receipt_hash, external_sequence_receipt_digest
    )
    VALUES (
        v_deployment_id, v_purpose, v_issuer_key_id, v_sequence,
        v_proof_id, p_signed_receipt_hash, p_external_sequence_receipt_digest
    )
    ON CONFLICT (deployment_id, purpose, issuer_key_id)
    DO UPDATE SET
        last_accepted_sequence = EXCLUDED.last_accepted_sequence,
        last_proof_id = EXCLUDED.last_proof_id,
        last_signed_receipt_hash = EXCLUDED.last_signed_receipt_hash,
        external_sequence_receipt_digest = EXCLUDED.external_sequence_receipt_digest,
        updated_at = clock_timestamp()
    WHERE myc_effect_owner.authority_proof_sequence_highwater.last_accepted_sequence
          < EXCLUDED.last_accepted_sequence;
    GET DIAGNOSTICS v_rows = ROW_COUNT;
    IF v_rows <> 1 THEN
        RAISE EXCEPTION 'authority proof sequence does not advance the stored high-water'
            USING ERRCODE = 'P0001';
    END IF;

    RETURN v_proof_id;
END
$function$;

CREATE OR REPLACE FUNCTION myc_effect_owner.bind_authority_write_lease_proofs(
    p_realm_id TEXT,
    p_resource_id TEXT,
    p_action_id TEXT,
    p_effect_owner_id TEXT,
    p_lease_id UUID,
    p_lease_highwater_proof_id UUID,
    p_physical_fencing_proof_id UUID,
    p_promotion_eligibility_proof_id UUID
)
RETURNS UUID
LANGUAGE plpgsql
SECURITY DEFINER
SET search_path = pg_catalog, myc_effect_owner
AS $function$
DECLARE
    v_lease myc_effect_owner.effect_owner_authority_write_lease_receipts%ROWTYPE;
    v_highwater myc_effect_owner.authority_proof_receipts%ROWTYPE;
    v_fencing myc_effect_owner.authority_proof_receipts%ROWTYPE;
    v_promotion myc_effect_owner.authority_proof_receipts%ROWTYPE;
    v_highwater_payload JSONB;
    v_fencing_payload JSONB;
    v_promotion_payload JSONB;
    v_scope_effect BOOLEAN;
    v_scope_promotion BOOLEAN;
    v_previous_generation BIGINT;
    v_new_generation BIGINT;
    v_new_lease_hash BYTEA;
BEGIN
    SELECT *
      INTO v_lease
      FROM myc_effect_owner.effect_owner_authority_write_lease_receipts
     WHERE realm_id = p_realm_id
       AND resource_id = p_resource_id
       AND action_id = p_action_id
       AND effect_owner_id = p_effect_owner_id
       AND lease_id = p_lease_id;
    IF NOT FOUND THEN
        RAISE EXCEPTION 'cannot bind proofs to a missing signed authority lease receipt'
            USING ERRCODE = 'P0001';
    END IF;

    v_scope_effect := v_lease.allowed_scopes ? 'directory_fabric_effect';
    v_scope_promotion := v_lease.allowed_scopes ? 'database_epoch_promotion';
    IF NOT v_scope_effect AND NOT v_scope_promotion THEN
        RAISE EXCEPTION 'lease proof binding requires an effect or promotion scope'
            USING ERRCODE = 'P0001';
    END IF;
    IF v_scope_promotion
       AND (p_physical_fencing_proof_id IS NULL OR p_promotion_eligibility_proof_id IS NULL) THEN
        RAISE EXCEPTION 'promotion lease requires high-water, physical-fencing and promotion-eligibility proof receipts'
            USING ERRCODE = 'P0001';
    END IF;
    IF v_scope_effect
       AND NOT v_scope_promotion
       AND (p_physical_fencing_proof_id IS NOT NULL OR p_promotion_eligibility_proof_id IS NOT NULL) THEN
        RAISE EXCEPTION 'effect-only lease cannot be linked to promotion-only proof receipts'
            USING ERRCODE = 'P0001';
    END IF;

    SELECT *
      INTO v_highwater
      FROM myc_effect_owner.authority_proof_receipts
     WHERE proof_id = p_lease_highwater_proof_id;
    IF NOT FOUND
       OR v_highwater.purpose <> 'lease_high_water'
       OR v_highwater.issuer_role <> 'lease_high_water_authority'
       OR v_highwater.deployment_id <> v_lease.deployment_id
       OR v_highwater.database_system_identifier <> v_lease.database_system_identifier
       OR v_highwater.primary_id <> v_lease.primary_id
       OR v_highwater.postgres_timeline <> v_lease.postgres_timeline
       OR v_highwater.owner_epoch <> v_lease.resource_owner_epoch
       OR v_highwater.external_sequence_receipt_digest <> v_lease.external_highwater_receipt_digest THEN
        RAISE EXCEPTION 'high-water proof receipt does not bind the exact signed lease target'
            USING ERRCODE = 'P0001';
    END IF;
    v_highwater_payload := v_highwater.signed_receipt_json #> '{receipt,payload}';
    v_previous_generation := (v_highwater_payload->>'previous_generation')::BIGINT;
    v_new_generation := (v_highwater_payload->>'new_generation')::BIGINT;
    v_new_lease_hash := myc_effect_owner.authority_proof_json_u8_array(
        v_highwater_payload->'new_lease_hash'
    );
    IF v_new_generation <> v_lease.lease_generation
       OR (v_highwater_payload->>'new_lease_id')::UUID <> v_lease.lease_id
       OR v_new_lease_hash <> v_lease.lease_hash
       OR v_new_generation <= v_previous_generation THEN
        RAISE EXCEPTION 'high-water proof payload does not authorize this exact lease ID/hash/generation'
            USING ERRCODE = 'P0001';
    END IF;

    IF v_scope_promotion THEN
        SELECT *
          INTO v_fencing
          FROM myc_effect_owner.authority_proof_receipts
         WHERE proof_id = p_physical_fencing_proof_id;
        IF NOT FOUND
           OR v_fencing.purpose <> 'physical_fencing'
           OR v_fencing.issuer_role <> 'physical_fencing_attestor'
           OR v_fencing.deployment_id <> v_lease.deployment_id
           OR v_fencing.database_system_identifier <> v_lease.database_system_identifier THEN
            RAISE EXCEPTION 'physical-fencing proof does not bind this promotion lease deployment/database'
                USING ERRCODE = 'P0001';
        END IF;
        v_fencing_payload := v_fencing.signed_receipt_json #> '{receipt,payload}';
        IF v_fencing_payload->>'successor_primary_id' IS DISTINCT FROM v_lease.primary_id
           OR v_fencing_payload->>'old_primary_id' IS NULL
           OR v_fencing_payload->>'old_primary_id' = v_lease.primary_id
           OR v_fencing_payload->>'old_primary_id' IS DISTINCT FROM v_fencing.primary_id
           OR (v_fencing_payload->>'old_system_identifier') IS DISTINCT FROM v_lease.database_system_identifier
           OR (v_fencing_payload->>'old_timeline')::BIGINT IS DISTINCT FROM v_fencing.postgres_timeline
           OR (v_fencing_payload->>'owner_epoch')::BIGINT IS DISTINCT FROM v_fencing.owner_epoch
           OR v_fencing_payload->>'mechanism' = 'network_partition_only'
           OR v_fencing.issuer_public_key = v_lease.signer_public_key THEN
            RAISE EXCEPTION 'physical-fencing proof does not bind the exact old-primary/successor pair or uses the lease signer as fence attestor'
                USING ERRCODE = 'P0001';
        END IF;

        SELECT *
          INTO v_promotion
          FROM myc_effect_owner.authority_proof_receipts
         WHERE proof_id = p_promotion_eligibility_proof_id;
        IF NOT FOUND
           OR v_promotion.purpose <> 'promotion_eligibility'
           OR v_promotion.issuer_role <> 'promotion_controller'
           OR v_promotion.deployment_id <> v_lease.deployment_id
           OR v_promotion.database_system_identifier <> v_lease.database_system_identifier
           OR v_promotion.primary_id <> v_lease.primary_id
           OR v_promotion.postgres_timeline <> v_lease.postgres_timeline
           OR v_promotion.owner_epoch <> v_lease.resource_owner_epoch THEN
            RAISE EXCEPTION 'promotion eligibility proof does not bind the exact lease successor/database/timeline/epoch'
                USING ERRCODE = 'P0001';
        END IF;
        v_promotion_payload := v_promotion.signed_receipt_json #> '{receipt,payload}';
        IF v_promotion_payload->>'candidate_primary_id' IS DISTINCT FROM v_lease.primary_id
           OR (v_promotion_payload->>'candidate_timeline')::BIGINT <> v_lease.postgres_timeline
           OR (v_promotion_payload->>'candidate_system_identifier') IS DISTINCT FROM v_lease.database_system_identifier
           OR myc_effect_owner.authority_proof_json_u8_array(v_promotion_payload->'lease_highwater_proof_hash')
                <> v_highwater.signed_receipt_hash
           OR myc_effect_owner.authority_proof_json_u8_array(v_promotion_payload->'physical_fencing_proof_hash')
                <> v_fencing.signed_receipt_hash THEN
            RAISE EXCEPTION 'promotion proof does not reference the exact signed high-water and fencing receipts'
                USING ERRCODE = 'P0001';
        END IF;
    END IF;

    INSERT INTO myc_effect_owner.effect_owner_authority_lease_proof_links (
        realm_id, resource_id, action_id, effect_owner_id, lease_id,
        lease_highwater_proof_id, physical_fencing_proof_id,
        promotion_eligibility_proof_id, lease_hash, lease_generation
    )
    VALUES (
        p_realm_id, p_resource_id, p_action_id, p_effect_owner_id, p_lease_id,
        p_lease_highwater_proof_id, p_physical_fencing_proof_id,
        p_promotion_eligibility_proof_id, v_lease.lease_hash, v_lease.lease_generation
    )
    ON CONFLICT (realm_id, resource_id, action_id, effect_owner_id, lease_id)
    DO NOTHING;

    IF NOT FOUND THEN
        IF NOT EXISTS (
            SELECT 1
              FROM myc_effect_owner.effect_owner_authority_lease_proof_links AS link
             WHERE link.realm_id = p_realm_id
               AND link.resource_id = p_resource_id
               AND link.action_id = p_action_id
               AND link.effect_owner_id = p_effect_owner_id
               AND link.lease_id = p_lease_id
               AND link.lease_highwater_proof_id = p_lease_highwater_proof_id
               AND link.physical_fencing_proof_id IS NOT DISTINCT FROM p_physical_fencing_proof_id
               AND link.promotion_eligibility_proof_id IS NOT DISTINCT FROM p_promotion_eligibility_proof_id
               AND link.lease_hash = v_lease.lease_hash
               AND link.lease_generation = v_lease.lease_generation
        ) THEN
            RAISE EXCEPTION 'same lease ID cannot be rebound to different authority proof receipts'
                USING ERRCODE = 'P0001';
        END IF;
    END IF;
    RETURN p_lease_id;
END
$function$;

CREATE OR REPLACE FUNCTION myc_effect_owner.apply_claim_fenced_balance_delta_with_proofs(
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
BEGIN
    IF NOT EXISTS (
        SELECT 1
          FROM myc_effect_owner.effect_owner_authority_lease_proof_links AS link
         WHERE link.realm_id = p_realm_id
           AND link.resource_id = p_resource_id
           AND link.action_id = p_action_id
           AND link.effect_owner_id = p_effect_owner_id
           AND link.lease_id = p_lease_id
           AND link.lease_hash = p_lease_hash
           AND link.lease_generation = p_lease_generation
           AND link.lease_highwater_proof_id IS NOT NULL
    ) THEN
        RAISE EXCEPTION 'effect lease lacks an independently verified external high-water proof link'
            USING ERRCODE = 'P0001';
    END IF;

    RETURN QUERY
    SELECT *
      FROM myc_effect_owner.apply_claim_fenced_balance_delta_with_lease(
        p_realm_id, p_resource_id, p_action_id, p_effect_owner_id,
        p_object_id, p_currency_code, p_owner_epoch, p_fence_generation,
        p_idempotency_key_digest, p_logical_operation_id,
        p_request_digest, p_decision_receipt_digest, p_payload_digest,
        p_delta_minor, p_worker_id, p_claim_epoch, p_claim_proof_digest,
        p_decision_clock_profile_id, p_decision_issued_at, p_decision_expires_at,
        p_lease_id, p_lease_hash, p_lease_generation
      );
END
$function$;

CREATE OR REPLACE FUNCTION myc_effect_owner.advance_owner_epoch_with_proofs(
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
BEGIN
    IF NOT EXISTS (
        SELECT 1
          FROM myc_effect_owner.effect_owner_authority_lease_proof_links AS link
         WHERE link.realm_id = p_realm_id
           AND link.resource_id = p_resource_id
           AND link.action_id = p_action_id
           AND link.effect_owner_id = p_effect_owner_id
           AND link.lease_id = p_lease_id
           AND link.lease_hash = p_lease_hash
           AND link.lease_generation = p_lease_generation
           AND link.lease_highwater_proof_id IS NOT NULL
           AND link.physical_fencing_proof_id IS NOT NULL
           AND link.promotion_eligibility_proof_id IS NOT NULL
    ) THEN
        RAISE EXCEPTION 'promotion lease lacks high-water, physical-fencing or promotion-eligibility proof links'
            USING ERRCODE = 'P0001';
    END IF;

    RETURN QUERY
    SELECT *
      FROM myc_effect_owner.advance_effect_owner_epoch_with_authority_lease(
        p_realm_id, p_resource_id, p_action_id, p_effect_owner_id,
        p_new_owner_epoch, p_epoch_authority_receipt_digest,
        p_physical_fencing_receipt_digest,
        p_lease_id, p_lease_hash, p_lease_generation
      );
END
$function$;

-- The worker must not bypass the proof-linked effect path. The controller
-- must not bypass proof-linked promotion. Lease installation remains
-- controller-only and proof receipt insertion/linking remains controller-only.
REVOKE EXECUTE ON FUNCTION myc_effect_owner.apply_claim_fenced_balance_delta_with_lease(
    TEXT, TEXT, TEXT, TEXT, TEXT, TEXT, BIGINT, BIGINT, BYTEA, TEXT,
    BYTEA, BYTEA, BYTEA, BIGINT, TEXT, BIGINT, BYTEA, TEXT, TIMESTAMPTZ,
    TIMESTAMPTZ, UUID, BYTEA, BIGINT
) FROM myc_dir_effect_worker;

REVOKE EXECUTE ON FUNCTION myc_effect_owner.advance_effect_owner_epoch_with_authority_lease(
    TEXT, TEXT, TEXT, TEXT, BIGINT, BYTEA, BYTEA, UUID, BYTEA, BIGINT
) FROM myc_dir_authority_lease_controller;

GRANT EXECUTE ON FUNCTION myc_effect_owner.install_authority_proof_receipt(
    JSONB, BYTEA, BYTEA, BYTEA, TEXT
) TO myc_dir_authority_lease_controller;

GRANT EXECUTE ON FUNCTION myc_effect_owner.bind_authority_write_lease_proofs(
    TEXT, TEXT, TEXT, TEXT, UUID, UUID, UUID, UUID
) TO myc_dir_authority_lease_controller;

GRANT EXECUTE ON FUNCTION myc_effect_owner.apply_claim_fenced_balance_delta_with_proofs(
    TEXT, TEXT, TEXT, TEXT, TEXT, TEXT, BIGINT, BIGINT, BYTEA, TEXT,
    BYTEA, BYTEA, BYTEA, BIGINT, TEXT, BIGINT, BYTEA, TEXT, TIMESTAMPTZ,
    TIMESTAMPTZ, UUID, BYTEA, BIGINT
) TO myc_dir_effect_worker;

GRANT EXECUTE ON FUNCTION myc_effect_owner.advance_owner_epoch_with_proofs(
    TEXT, TEXT, TEXT, TEXT, BIGINT, BYTEA, BYTEA, UUID, BYTEA, BIGINT
) TO myc_dir_authority_lease_controller;

REVOKE ALL ON ALL TABLES IN SCHEMA myc_effect_owner FROM PUBLIC;
REVOKE ALL ON ALL FUNCTIONS IN SCHEMA myc_effect_owner FROM PUBLIC;

COMMENT ON TABLE myc_effect_owner.authority_proof_receipts IS
'Append-only domain-separated signed proof receipts for lease high-water, physical fencing and promotion eligibility. Signature/root/actuator truth is verified by the independent Rust controller.';
COMMENT ON TABLE myc_effect_owner.authority_proof_sequence_highwater IS
'Database-local mirror of independent monotonic proof sequence state. Never authoritative after PITR/restore; verifier must consult the external anti-rollback store.';
COMMENT ON TABLE myc_effect_owner.effect_owner_authority_lease_proof_links IS
'Append-only binding from exact signed lease receipt to complete high-water and, for promotion, physical-fencing and promotion-eligibility proof receipts.';

COMMIT;
