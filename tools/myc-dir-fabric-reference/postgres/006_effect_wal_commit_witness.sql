-- Mycelix Directory Fabric transactional WAL marker and commit-position witness v0.1.
--
-- Every new proof-linked effect emits a transactional logical-decoding marker
-- in the same PostgreSQL transaction as the business effect and attestation.
-- An independent WAL observer reads that marker and the transaction commit
-- message, obtains commit/end-LSN, verifies timeline identity, signs a
-- wal_commit_position authority proof and records the proof ID below.
--
-- pg_logical_emit_message(true, ...) is transactional: aborted transactions
-- must not produce a committed logical marker. The observer is a separate
-- principal and the proof's Ed25519 signature/external sequence are verified
-- by the Rust authority-proof controller before this SQL installer is called.
-- This migration does not itself prove replica durability or recovery closure.

BEGIN;

DO $roles$
BEGIN
    IF NOT EXISTS (SELECT 1 FROM pg_catalog.pg_roles WHERE rolname = 'myc_dir_wal_observer') THEN
        CREATE ROLE myc_dir_wal_observer NOLOGIN;
    END IF;
END
$roles$;

ALTER ROLE myc_dir_wal_observer NOLOGIN;

CREATE TABLE IF NOT EXISTS myc_effect_owner.effect_wal_commit_positions (
    effect_id UUID PRIMARY KEY
        REFERENCES myc_effect_owner.effect_commits(effect_id),
    attestation_id UUID NOT NULL UNIQUE
        REFERENCES myc_effect_owner.effect_execution_attestations(attestation_id),
    proof_id UUID NOT NULL UNIQUE
        REFERENCES myc_effect_owner.authority_proof_receipts(proof_id),
    realm_id TEXT NOT NULL,
    resource_id TEXT NOT NULL,
    action_id TEXT NOT NULL,
    effect_owner_id TEXT NOT NULL,
    owner_epoch BIGINT NOT NULL CHECK (owner_epoch > 0),
    resource_generation BIGINT NOT NULL CHECK (resource_generation > 0),
    lease_id UUID NOT NULL,
    lease_hash BYTEA NOT NULL CHECK (octet_length(lease_hash) = 32),
    lease_generation BIGINT NOT NULL CHECK (lease_generation > 0),
    database_system_identifier TEXT NOT NULL CHECK (btrim(database_system_identifier) <> ''),
    primary_id TEXT NOT NULL CHECK (btrim(primary_id) <> ''),
    postgres_timeline BIGINT NOT NULL CHECK (postgres_timeline > 0),
    transaction_id TEXT NOT NULL CHECK (btrim(transaction_id) <> ''),
    commit_lsn NUMERIC(20, 0) NOT NULL CHECK (commit_lsn > 0),
    transaction_end_lsn NUMERIC(20, 0) NOT NULL CHECK (transaction_end_lsn >= commit_lsn),
    marker_digest BYTEA NOT NULL CHECK (
        octet_length(marker_digest) = 32
        AND marker_digest <> decode(repeat('00', 32), 'hex')
    ),
    marker_json JSONB NOT NULL CHECK (jsonb_typeof(marker_json) = 'object'),
    observer_key_id TEXT NOT NULL CHECK (btrim(observer_key_id) <> ''),
    observer_public_key BYTEA NOT NULL CHECK (octet_length(observer_public_key) = 32),
    observer_signed_receipt_hash BYTEA NOT NULL CHECK (
        octet_length(observer_signed_receipt_hash) = 32
        AND observer_signed_receipt_hash <> decode(repeat('00', 32), 'hex')
    ),
    external_sequence_receipt_digest BYTEA NOT NULL CHECK (
        octet_length(external_sequence_receipt_digest) = 32
        AND external_sequence_receipt_digest <> decode(repeat('00', 32), 'hex')
    ),
    observed_at TIMESTAMPTZ NOT NULL DEFAULT clock_timestamp(),
    FOREIGN KEY (realm_id, resource_id, action_id, effect_owner_id, lease_id)
        REFERENCES myc_effect_owner.effect_owner_authority_write_lease_receipts (
            realm_id, resource_id, action_id, effect_owner_id, lease_id
        )
);

CREATE OR REPLACE FUNCTION myc_effect_owner.reject_effect_wal_position_mutation()
RETURNS TRIGGER
LANGUAGE plpgsql
SET search_path = pg_catalog, myc_effect_owner
AS $function$
BEGIN
    RAISE EXCEPTION 'effect WAL commit positions are append-only'
        USING ERRCODE = '55000';
END
$function$;

DROP TRIGGER IF EXISTS effect_wal_positions_no_update_delete
ON myc_effect_owner.effect_wal_commit_positions;
CREATE TRIGGER effect_wal_positions_no_update_delete
BEFORE UPDATE OR DELETE ON myc_effect_owner.effect_wal_commit_positions
FOR EACH ROW EXECUTE FUNCTION myc_effect_owner.reject_effect_wal_position_mutation();

DROP TRIGGER IF EXISTS effect_wal_positions_no_truncate
ON myc_effect_owner.effect_wal_commit_positions;
CREATE TRIGGER effect_wal_positions_no_truncate
BEFORE TRUNCATE ON myc_effect_owner.effect_wal_commit_positions
FOR EACH STATEMENT EXECUTE FUNCTION myc_effect_owner.reject_effect_wal_position_mutation();

CREATE OR REPLACE FUNCTION myc_effect_owner.record_effect_wal_commit_position(
    p_effect_id UUID,
    p_attestation_id UUID,
    p_wal_proof_id UUID,
    p_marker_digest BYTEA,
    p_marker_json JSONB
)
RETURNS UUID
LANGUAGE plpgsql
SECURITY DEFINER
SET search_path = pg_catalog, myc_effect_owner
AS $function$
DECLARE
    v_effect myc_effect_owner.effect_commits%ROWTYPE;
    v_attestation myc_effect_owner.effect_execution_attestations%ROWTYPE;
    v_lease myc_effect_owner.effect_execution_authority_lease_attestations%ROWTYPE;
    v_proof myc_effect_owner.authority_proof_receipts%ROWTYPE;
    v_payload JSONB;
    v_effect_id UUID;
    v_attestation_id UUID;
    v_marker_digest BYTEA;
    v_transaction_id TEXT;
    v_commit_lsn NUMERIC(20, 0);
    v_end_lsn NUMERIC(20, 0);
    v_existing RECORD;
BEGIN
    IF p_effect_id IS NULL OR p_attestation_id IS NULL OR p_wal_proof_id IS NULL
       OR p_marker_digest IS NULL OR octet_length(p_marker_digest) <> 32
       OR p_marker_digest = decode(repeat('00', 32), 'hex')
       OR p_marker_json IS NULL OR jsonb_typeof(p_marker_json) <> 'object' THEN
        RAISE EXCEPTION 'WAL commit-position binding requires exact effect, attestation, proof, marker digest and marker JSON'
            USING ERRCODE = '22023';
    END IF;

    SELECT *
      INTO v_effect
      FROM myc_effect_owner.effect_commits
     WHERE effect_id = p_effect_id;
    IF NOT FOUND THEN
        RAISE EXCEPTION 'WAL commit-position proof names an unknown effect'
            USING ERRCODE = 'P0001';
    END IF;

    SELECT *
      INTO v_attestation
      FROM myc_effect_owner.effect_execution_attestations
     WHERE attestation_id = p_attestation_id
       AND effect_id = p_effect_id;
    IF NOT FOUND THEN
        RAISE EXCEPTION 'WAL commit-position proof does not bind the exact effect execution attestation'
            USING ERRCODE = 'P0001';
    END IF;

    SELECT *
      INTO v_lease
      FROM myc_effect_owner.effect_execution_authority_lease_attestations
     WHERE effect_id = p_effect_id
       AND attestation_id = p_attestation_id;
    IF NOT FOUND THEN
        RAISE EXCEPTION 'WAL commit-position proof has no matching signed effect lease attestation'
            USING ERRCODE = 'P0001';
    END IF;

    SELECT *
      INTO v_proof
      FROM myc_effect_owner.authority_proof_receipts
     WHERE proof_id = p_wal_proof_id;
    IF NOT FOUND
       OR v_proof.purpose <> 'wal_commit_position'
       OR v_proof.issuer_role <> 'wal_commit_observer'
       OR v_proof.deployment_id <> v_lease.deployment_id
       OR v_proof.database_system_identifier <> v_lease.database_system_identifier
       OR v_proof.primary_id <> v_lease.primary_id
       OR v_proof.postgres_timeline <> v_lease.postgres_timeline
       OR v_proof.owner_epoch <> v_attestation.owner_epoch
       OR v_proof.issuer_public_key = v_lease.signer_public_key THEN
        RAISE EXCEPTION 'WAL proof must come from an independent observer for the exact deployment, database, primary, timeline and owner epoch'
            USING ERRCODE = 'P0001';
    END IF;

    v_payload := v_proof.signed_receipt_json #> '{receipt,payload}';
    v_effect_id := (v_payload->>'effect_id')::UUID;
    v_attestation_id := (v_payload->>'execution_attestation_id')::UUID;
    v_marker_digest := myc_effect_owner.authority_proof_json_u8_array(
        v_payload->'marker_digest'
    );
    v_transaction_id := v_payload->>'transaction_id';
    v_commit_lsn := (v_payload->>'commit_lsn')::NUMERIC(20, 0);
    v_end_lsn := (v_payload->>'transaction_end_lsn')::NUMERIC(20, 0);

    IF v_effect_id IS DISTINCT FROM p_effect_id
       OR v_attestation_id IS DISTINCT FROM p_attestation_id
       OR v_marker_digest IS DISTINCT FROM p_marker_digest
       OR v_transaction_id IS NULL OR btrim(v_transaction_id) = ''
       OR v_commit_lsn IS NULL OR v_commit_lsn <= 0
       OR v_end_lsn IS NULL OR v_end_lsn < v_commit_lsn THEN
        RAISE EXCEPTION 'signed WAL proof payload does not bind the exact effect marker and commit/end-LSN chronology'
            USING ERRCODE = 'P0001';
    END IF;

    IF p_marker_json->>'profile' IS DISTINCT FROM 'MYC-DIR-EFFECT-WAL-MARKER-V1'
       OR p_marker_json->>'effect_id' IS DISTINCT FROM p_effect_id::TEXT
       OR p_marker_json->>'execution_attestation_id' IS DISTINCT FROM p_attestation_id::TEXT
       OR p_marker_json->>'realm_id' IS DISTINCT FROM v_effect.realm_id
       OR p_marker_json->>'resource_id' IS DISTINCT FROM v_effect.resource_id
       OR p_marker_json->>'action_id' IS DISTINCT FROM v_effect.action_id
       OR p_marker_json->>'effect_owner_id' IS DISTINCT FROM v_effect.effect_owner_id
       OR p_marker_json->>'object_id' IS DISTINCT FROM v_effect.object_id
       OR p_marker_json->>'currency_code' IS DISTINCT FROM v_effect.currency_code
       OR (p_marker_json->>'owner_epoch')::BIGINT IS DISTINCT FROM v_attestation.owner_epoch
       OR (p_marker_json->>'resource_generation')::BIGINT IS DISTINCT FROM v_attestation.accepted_generation
       OR p_marker_json->>'lease_id' IS DISTINCT FROM v_lease.lease_id::TEXT
       OR p_marker_json->>'lease_hash' IS DISTINCT FROM encode(v_lease.lease_hash, 'hex')
       OR (p_marker_json->>'lease_generation')::BIGINT IS DISTINCT FROM v_lease.lease_generation
       OR p_marker_json->>'logical_operation_id' IS DISTINCT FROM v_effect.logical_operation_id
       OR p_marker_json->>'idempotency_digest' IS DISTINCT FROM encode(v_effect.idempotency_key_digest, 'hex')
       OR p_marker_json->>'transaction_id' IS DISTINCT FROM v_attestation.database_xact_id
       OR p_marker_json->>'request_digest' IS DISTINCT FROM encode(v_effect.request_digest, 'hex')
       OR p_marker_json->>'decision_receipt_digest' IS DISTINCT FROM encode(v_effect.decision_receipt_digest, 'hex')
       OR p_marker_json->>'payload_digest' IS DISTINCT FROM encode(v_effect.payload_digest, 'hex') THEN
        RAISE EXCEPTION 'transactional WAL marker does not match the committed effect, execution attestation and authority lease'
            USING ERRCODE = 'P0001';
    END IF;

    SELECT *
      INTO v_existing
      FROM myc_effect_owner.effect_wal_commit_positions
     WHERE effect_id = p_effect_id;
    IF FOUND THEN
        IF v_existing.attestation_id IS DISTINCT FROM p_attestation_id
           OR v_existing.proof_id IS DISTINCT FROM p_wal_proof_id
           OR v_existing.marker_digest IS DISTINCT FROM p_marker_digest
           OR v_existing.marker_json IS DISTINCT FROM p_marker_json
           OR v_existing.transaction_id IS DISTINCT FROM v_transaction_id
           OR v_existing.commit_lsn IS DISTINCT FROM v_commit_lsn
           OR v_existing.transaction_end_lsn IS DISTINCT FROM v_end_lsn THEN
            RAISE EXCEPTION 'effect WAL position is already bound to different transaction or observer evidence'
                USING ERRCODE = 'P0001';
        END IF;
        RETURN p_effect_id;
    END IF;

    INSERT INTO myc_effect_owner.effect_wal_commit_positions (
        effect_id, attestation_id, proof_id, realm_id, resource_id,
        action_id, effect_owner_id, owner_epoch, resource_generation,
        lease_id, lease_hash, lease_generation,
        database_system_identifier, primary_id, postgres_timeline,
        transaction_id, commit_lsn, transaction_end_lsn,
        marker_digest, marker_json, observer_key_id, observer_public_key,
        observer_signed_receipt_hash, external_sequence_receipt_digest
    )
    VALUES (
        p_effect_id, p_attestation_id, p_wal_proof_id,
        v_effect.realm_id, v_effect.resource_id, v_effect.action_id,
        v_effect.effect_owner_id, v_attestation.owner_epoch,
        v_attestation.accepted_generation,
        v_lease.lease_id, v_lease.lease_hash, v_lease.lease_generation,
        v_lease.database_system_identifier, v_lease.primary_id,
        v_lease.postgres_timeline, v_transaction_id, v_commit_lsn, v_end_lsn,
        p_marker_digest, p_marker_json, v_proof.issuer_key_id,
        v_proof.issuer_public_key, v_proof.signed_receipt_hash,
        v_proof.external_sequence_receipt_digest
    );

    RETURN p_effect_id;
END
$function$;

CREATE OR REPLACE FUNCTION myc_effect_owner.apply_claim_fenced_balance_delta_with_wal_marker(
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
    v_marker JSONB;
BEGIN
    SELECT *
      INTO v_result
      FROM myc_effect_owner.apply_claim_fenced_balance_delta_with_proofs(
        p_realm_id, p_resource_id, p_action_id, p_effect_owner_id,
        p_object_id, p_currency_code, p_owner_epoch, p_fence_generation,
        p_idempotency_key_digest, p_logical_operation_id,
        p_request_digest, p_decision_receipt_digest, p_payload_digest,
        p_delta_minor, p_worker_id, p_claim_epoch, p_claim_proof_digest,
        p_decision_clock_profile_id, p_decision_issued_at, p_decision_expires_at,
        p_lease_id, p_lease_hash, p_lease_generation
      );

    IF NOT v_result.replayed THEN
        v_marker := jsonb_build_object(
            'profile', 'MYC-DIR-EFFECT-WAL-MARKER-V1',
            'effect_id', v_result.effect_id,
            'execution_attestation_id', v_result.execution_attestation_id,
            'realm_id', p_realm_id,
            'resource_id', p_resource_id,
            'action_id', p_action_id,
            'effect_owner_id', p_effect_owner_id,
            'object_id', p_object_id,
            'currency_code', p_currency_code,
            'owner_epoch', p_owner_epoch,
            'resource_generation', p_fence_generation,
            'logical_operation_id', p_logical_operation_id,
            'idempotency_digest', encode(p_idempotency_key_digest, 'hex'),
            'request_digest', encode(p_request_digest, 'hex'),
            'decision_receipt_digest', encode(p_decision_receipt_digest, 'hex'),
            'payload_digest', encode(p_payload_digest, 'hex'),
            'lease_id', v_result.authority_lease_id,
            'lease_hash', encode(v_result.authority_lease_hash, 'hex'),
            'lease_generation', v_result.authority_lease_generation
        );
        PERFORM pg_catalog.pg_logical_emit_message(
            TRUE,
            'mycelix.dir.effect-finality.v1',
            v_marker::TEXT
        );
    END IF;

    RETURN QUERY SELECT
        v_result.replayed, v_result.effect_id,
        v_result.accepted_owner_epoch, v_result.accepted_generation,
        v_result.accepted_worker_id, v_result.accepted_claim_epoch,
        v_result.execution_attestation_id,
        v_result.authority_lease_id, v_result.authority_lease_hash,
        v_result.authority_lease_generation;
END
$function$;

REVOKE EXECUTE ON FUNCTION myc_effect_owner.apply_claim_fenced_balance_delta_with_proofs(
    TEXT, TEXT, TEXT, TEXT, TEXT, TEXT, BIGINT, BIGINT, BYTEA, TEXT,
    BYTEA, BYTEA, BYTEA, BIGINT, TEXT, BIGINT, BYTEA, TEXT, TIMESTAMPTZ,
    TIMESTAMPTZ, UUID, BYTEA, BIGINT
) FROM myc_dir_effect_worker;

GRANT EXECUTE ON FUNCTION myc_effect_owner.apply_claim_fenced_balance_delta_with_wal_marker(
    TEXT, TEXT, TEXT, TEXT, TEXT, TEXT, BIGINT, BIGINT, BYTEA, TEXT,
    BYTEA, BYTEA, BYTEA, BIGINT, TEXT, BIGINT, BYTEA, TEXT, TIMESTAMPTZ,
    TIMESTAMPTZ, UUID, BYTEA, BIGINT
) TO myc_dir_effect_worker;

GRANT EXECUTE ON FUNCTION myc_effect_owner.record_effect_wal_commit_position(
    UUID, UUID, UUID, BYTEA, JSONB
) TO myc_dir_wal_observer;

REVOKE ALL ON ALL TABLES IN SCHEMA myc_effect_owner FROM PUBLIC;
REVOKE ALL ON ALL FUNCTIONS IN SCHEMA myc_effect_owner FROM PUBLIC;

COMMENT ON TABLE myc_effect_owner.effect_wal_commit_positions IS
'Append-only independently signed logical-decoding evidence binding a protected effect and transactional marker to exact PostgreSQL commit and transaction end-LSN.';
COMMENT ON FUNCTION myc_effect_owner.apply_claim_fenced_balance_delta_with_wal_marker(
    TEXT, TEXT, TEXT, TEXT, TEXT, TEXT, BIGINT, BIGINT, BYTEA, TEXT,
    BYTEA, BYTEA, BYTEA, BIGINT, TEXT, BIGINT, BYTEA, TEXT, TIMESTAMPTZ,
    TIMESTAMPTZ, UUID, BYTEA, BIGINT
) IS
'Executes the proof-linked protected effect and emits a transactional logical-decoding marker only for a newly committed effect; an independent observer must later certify the commit/end-LSN before effect-success acknowledgement.';

COMMIT;
