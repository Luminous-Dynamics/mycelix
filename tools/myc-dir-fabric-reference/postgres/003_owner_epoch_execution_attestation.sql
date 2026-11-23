-- Mycelix Directory Fabric PostgreSQL owner-epoch and execution-attestation extension v0.1
-- Apply after 001_effect_owner.sql and 002_worker_claim_fencing.sql.
--
-- This migration adds owner-local epoch monotonicity and records the exact
-- worker claim / owner epoch accepted by each new protected effect, atomically
-- with the effect transaction.
--
-- IMPORTANT DISTRIBUTED-SYSTEMS LIMIT:
-- PostgreSQL state and a nonzero receipt digest do not fence a former writable
-- primary or prove that a promotion candidate contains the last commit. The
-- trusted control plane MUST authenticate the external epoch/fencing receipts,
-- physically fence the previous primary, and enforce a single-writer lease
-- before calling advance_effect_owner_epoch. This SQL stores the commitments
-- and enforces local monotonicity; it does not verify external signatures.

BEGIN;

ALTER TABLE myc_effect_owner.effect_fence_state
    ADD COLUMN IF NOT EXISTS owner_epoch BIGINT NOT NULL DEFAULT 1
        CHECK (owner_epoch > 0);

ALTER TABLE myc_effect_owner.worker_claim_state
    ADD COLUMN IF NOT EXISTS owner_epoch BIGINT NOT NULL DEFAULT 1
        CHECK (owner_epoch > 0);

CREATE TABLE IF NOT EXISTS myc_effect_owner.effect_owner_epoch_transitions (
    realm_id TEXT NOT NULL,
    resource_id TEXT NOT NULL,
    action_id TEXT NOT NULL,
    effect_owner_id TEXT NOT NULL,
    previous_owner_epoch BIGINT NOT NULL CHECK (previous_owner_epoch > 0),
    new_owner_epoch BIGINT NOT NULL CHECK (new_owner_epoch > previous_owner_epoch),
    previous_generation BIGINT NOT NULL CHECK (previous_generation > 0),
    new_generation BIGINT NOT NULL CHECK (new_generation > previous_generation),
    epoch_authority_receipt_digest BYTEA NOT NULL CHECK (
        octet_length(epoch_authority_receipt_digest) = 32
        AND epoch_authority_receipt_digest <> decode(repeat('00', 32), 'hex')
    ),
    physical_fencing_receipt_digest BYTEA NOT NULL CHECK (
        octet_length(physical_fencing_receipt_digest) = 32
        AND physical_fencing_receipt_digest <> decode(repeat('00', 32), 'hex')
    ),
    transition_xact_id TEXT NOT NULL CHECK (btrim(transition_xact_id) <> ''),
    committed_at TIMESTAMPTZ NOT NULL DEFAULT clock_timestamp(),
    PRIMARY KEY (
        realm_id, resource_id, action_id, effect_owner_id, new_owner_epoch
    ),
    UNIQUE (
        realm_id, resource_id, action_id, effect_owner_id, new_generation
    ),
    FOREIGN KEY (realm_id, resource_id, action_id, effect_owner_id)
        REFERENCES myc_effect_owner.effect_fence_state (
            realm_id, resource_id, action_id, effect_owner_id
        )
);

-- Historical claim and owner coordinates are written once, in the same
-- transaction as the effect_commits row. This does not retrofit invented
-- values onto effects created before this migration.
CREATE TABLE IF NOT EXISTS myc_effect_owner.effect_execution_attestations (
    attestation_id UUID PRIMARY KEY,
    effect_id UUID NOT NULL UNIQUE
        REFERENCES myc_effect_owner.effect_commits(effect_id),
    realm_id TEXT NOT NULL,
    resource_id TEXT NOT NULL,
    action_id TEXT NOT NULL,
    effect_owner_id TEXT NOT NULL,
    object_id TEXT NOT NULL,
    currency_code TEXT NOT NULL CHECK (currency_code ~ '^[A-Z]{3}$'),
    logical_operation_id TEXT NOT NULL CHECK (
        btrim(logical_operation_id) <> '' AND octet_length(logical_operation_id) <= 512
    ),
    idempotency_key_digest BYTEA NOT NULL CHECK (
        octet_length(idempotency_key_digest) = 32
        AND idempotency_key_digest <> decode(repeat('00', 32), 'hex')
    ),
    request_digest BYTEA NOT NULL CHECK (
        octet_length(request_digest) = 32
        AND request_digest <> decode(repeat('00', 32), 'hex')
    ),
    decision_receipt_digest BYTEA NOT NULL CHECK (
        octet_length(decision_receipt_digest) = 32
        AND decision_receipt_digest <> decode(repeat('00', 32), 'hex')
    ),
    payload_digest BYTEA NOT NULL CHECK (
        octet_length(payload_digest) = 32
        AND payload_digest <> decode(repeat('00', 32), 'hex')
    ),
    owner_epoch BIGINT NOT NULL CHECK (owner_epoch > 0),
    accepted_generation BIGINT NOT NULL CHECK (accepted_generation > 0),
    worker_id TEXT NOT NULL CHECK (
        btrim(worker_id) <> '' AND octet_length(worker_id) <= 512
    ),
    claim_epoch BIGINT NOT NULL CHECK (claim_epoch > 0),
    claim_proof_digest BYTEA NOT NULL CHECK (
        octet_length(claim_proof_digest) = 32
        AND claim_proof_digest <> decode(repeat('00', 32), 'hex')
    ),
    claim_clock_profile_id TEXT NOT NULL CHECK (btrim(claim_clock_profile_id) <> ''),
    claim_issued_at TIMESTAMPTZ NOT NULL,
    claim_expires_at TIMESTAMPTZ NOT NULL,
    database_xact_id TEXT NOT NULL CHECK (btrim(database_xact_id) <> ''),
    attested_at TIMESTAMPTZ NOT NULL DEFAULT clock_timestamp(),
    CHECK (claim_issued_at < claim_expires_at),
    FOREIGN KEY (realm_id, resource_id, action_id, effect_owner_id)
        REFERENCES myc_effect_owner.effect_fence_state (
            realm_id, resource_id, action_id, effect_owner_id
        )
);

CREATE INDEX IF NOT EXISTS effect_execution_attestations_by_scope
ON myc_effect_owner.effect_execution_attestations (
    realm_id, resource_id, action_id, effect_owner_id,
    owner_epoch, accepted_generation, attested_at
);

CREATE OR REPLACE FUNCTION myc_effect_owner.reject_execution_attestation_mutation()
RETURNS TRIGGER
LANGUAGE plpgsql
SET search_path = pg_catalog, myc_effect_owner
AS $function$
BEGIN
    RAISE EXCEPTION '% is append-only', TG_TABLE_NAME
        USING ERRCODE = '55000';
END
$function$;

DROP TRIGGER IF EXISTS epoch_transitions_no_update_delete
ON myc_effect_owner.effect_owner_epoch_transitions;

CREATE TRIGGER epoch_transitions_no_update_delete
BEFORE UPDATE OR DELETE ON myc_effect_owner.effect_owner_epoch_transitions
FOR EACH ROW EXECUTE FUNCTION myc_effect_owner.reject_execution_attestation_mutation();

DROP TRIGGER IF EXISTS epoch_transitions_no_truncate
ON myc_effect_owner.effect_owner_epoch_transitions;

CREATE TRIGGER epoch_transitions_no_truncate
BEFORE TRUNCATE ON myc_effect_owner.effect_owner_epoch_transitions
FOR EACH STATEMENT EXECUTE FUNCTION myc_effect_owner.reject_execution_attestation_mutation();

DROP TRIGGER IF EXISTS execution_attestations_no_update_delete
ON myc_effect_owner.effect_execution_attestations;

CREATE TRIGGER execution_attestations_no_update_delete
BEFORE UPDATE OR DELETE ON myc_effect_owner.effect_execution_attestations
FOR EACH ROW EXECUTE FUNCTION myc_effect_owner.reject_execution_attestation_mutation();

DROP TRIGGER IF EXISTS execution_attestations_no_truncate
ON myc_effect_owner.effect_execution_attestations;

CREATE TRIGGER execution_attestations_no_truncate
BEFORE TRUNCATE ON myc_effect_owner.effect_execution_attestations
FOR EACH STATEMENT EXECUTE FUNCTION myc_effect_owner.reject_execution_attestation_mutation();

DROP TRIGGER IF EXISTS effect_commits_no_truncate
ON myc_effect_owner.effect_commits;

CREATE TRIGGER effect_commits_no_truncate
BEFORE TRUNCATE ON myc_effect_owner.effect_commits
FOR EACH STATEMENT EXECUTE FUNCTION myc_effect_owner.reject_effect_commit_mutation();

-- New epoch-aware claim installer. A claim from an earlier owner epoch
-- cannot simply carry forward after promotion/recovery: installing a claim for
-- a newer owner epoch requires a strictly higher worker claim epoch.
CREATE OR REPLACE FUNCTION myc_effect_owner.install_worker_claim_for_owner_epoch(
    p_realm_id TEXT,
    p_resource_id TEXT,
    p_action_id TEXT,
    p_effect_owner_id TEXT,
    p_owner_epoch BIGINT,
    p_logical_operation_id TEXT,
    p_worker_id TEXT,
    p_claim_epoch BIGINT,
    p_claim_proof_digest BYTEA,
    p_clock_profile_id TEXT,
    p_issued_at TIMESTAMPTZ,
    p_expires_at TIMESTAMPTZ
)
RETURNS BIGINT
LANGUAGE plpgsql
SECURITY DEFINER
SET search_path = pg_catalog, myc_effect_owner
AS $function$
DECLARE
    v_current_owner_epoch BIGINT;
    v_existing_owner_epoch BIGINT;
    v_existing_claim_epoch BIGINT;
    v_installed_claim_epoch BIGINT;
BEGIN
    IF p_owner_epoch IS NULL OR p_owner_epoch <= 0 THEN
        RAISE EXCEPTION 'worker claim owner epoch must be positive'
            USING ERRCODE = '22023';
    END IF;

    -- Lock order must match queue-backed execution:
    -- worker_claim_state first, then effect_fence_state.
    SELECT owner_epoch, claim_epoch
      INTO v_existing_owner_epoch, v_existing_claim_epoch
      FROM myc_effect_owner.worker_claim_state
     WHERE realm_id = p_realm_id
       AND resource_id = p_resource_id
       AND action_id = p_action_id
       AND effect_owner_id = p_effect_owner_id
       AND logical_operation_id = p_logical_operation_id
     FOR UPDATE;

    SELECT owner_epoch
      INTO v_current_owner_epoch
      FROM myc_effect_owner.effect_fence_state
     WHERE realm_id = p_realm_id
       AND resource_id = p_resource_id
       AND action_id = p_action_id
       AND effect_owner_id = p_effect_owner_id
     FOR SHARE;

    IF NOT FOUND THEN
        RAISE EXCEPTION 'owner epoch domain is not initialized for worker claim installation'
            USING ERRCODE = 'P0001';
    END IF;
    IF p_owner_epoch <> v_current_owner_epoch THEN
        RAISE EXCEPTION 'worker claim owner epoch is stale: supplied %, current %',
            p_owner_epoch, v_current_owner_epoch
            USING ERRCODE = 'P0001';
    END IF;

    IF v_existing_owner_epoch IS NOT NULL THEN
        IF v_existing_owner_epoch > p_owner_epoch THEN
            RAISE EXCEPTION 'existing worker claim belongs to a newer owner epoch'
                USING ERRCODE = 'P0001';
        END IF;
        IF v_existing_owner_epoch < p_owner_epoch
           AND p_claim_epoch <= v_existing_claim_epoch THEN
            RAISE EXCEPTION 'worker claim epoch must strictly advance when owner epoch changes'
                USING ERRCODE = 'P0001';
        END IF;
    END IF;

    SELECT myc_effect_owner.install_worker_claim(
        p_realm_id,
        p_resource_id,
        p_action_id,
        p_effect_owner_id,
        p_logical_operation_id,
        p_worker_id,
        p_claim_epoch,
        p_claim_proof_digest,
        p_clock_profile_id,
        p_issued_at,
        p_expires_at
    ) INTO v_installed_claim_epoch;

    UPDATE myc_effect_owner.worker_claim_state
       SET owner_epoch = p_owner_epoch
     WHERE realm_id = p_realm_id
       AND resource_id = p_resource_id
       AND action_id = p_action_id
       AND effect_owner_id = p_effect_owner_id
       AND logical_operation_id = p_logical_operation_id
       AND worker_id = p_worker_id
       AND claim_epoch = v_installed_claim_epoch
       AND claim_proof_digest = p_claim_proof_digest
       AND clock_profile_id = p_clock_profile_id
       AND issued_at = p_issued_at
       AND expires_at = p_expires_at;

    IF NOT FOUND THEN
        RAISE EXCEPTION 'epoch-aware worker claim install did not persist the exact claim'
            USING ERRCODE = 'P0001';
    END IF;

    RETURN v_installed_claim_epoch;
END
$function$;

-- Installs a strictly newer owner epoch and advances the resource generation
-- in the same row lock/transaction. The two receipt digests must have been
-- authenticated by an external epoch authority and physical fencing service.
-- The external service must fence the old primary before this function runs.
CREATE OR REPLACE FUNCTION myc_effect_owner.advance_effect_owner_epoch(
    p_realm_id TEXT,
    p_resource_id TEXT,
    p_action_id TEXT,
    p_effect_owner_id TEXT,
    p_new_owner_epoch BIGINT,
    p_epoch_authority_receipt_digest BYTEA,
    p_physical_fencing_receipt_digest BYTEA
)
RETURNS TABLE (
    accepted_owner_epoch BIGINT,
    accepted_generation BIGINT
)
LANGUAGE plpgsql
SECURITY DEFINER
SET search_path = pg_catalog, myc_effect_owner
AS $function$
DECLARE
    v_current_owner_epoch BIGINT;
    v_current_generation BIGINT;
    v_new_generation BIGINT;
BEGIN
    IF p_realm_id IS NULL OR p_resource_id IS NULL OR p_action_id IS NULL
       OR p_effect_owner_id IS NULL
       OR btrim(p_realm_id) = '' OR btrim(p_resource_id) = ''
       OR btrim(p_action_id) = '' OR btrim(p_effect_owner_id) = '' THEN
        RAISE EXCEPTION 'owner epoch scope identifiers must be nonempty'
            USING ERRCODE = '22023';
    END IF;
    IF p_new_owner_epoch IS NULL OR p_new_owner_epoch <= 0 THEN
        RAISE EXCEPTION 'new owner epoch must be positive'
            USING ERRCODE = '22023';
    END IF;
    IF p_epoch_authority_receipt_digest IS NULL
       OR octet_length(p_epoch_authority_receipt_digest) <> 32
       OR p_epoch_authority_receipt_digest = decode(repeat('00', 32), 'hex')
       OR p_physical_fencing_receipt_digest IS NULL
       OR octet_length(p_physical_fencing_receipt_digest) <> 32
       OR p_physical_fencing_receipt_digest = decode(repeat('00', 32), 'hex') THEN
        RAISE EXCEPTION 'epoch authority and physical fencing receipts must be nonzero 32-byte commitments'
            USING ERRCODE = '22023';
    END IF;

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
    IF p_new_owner_epoch <= v_current_owner_epoch THEN
        RAISE EXCEPTION 'owner epoch must strictly advance; supplied %, current %',
            p_new_owner_epoch, v_current_owner_epoch
            USING ERRCODE = 'P0001';
    END IF;
    IF v_current_generation = 9223372036854775807 THEN
        RAISE EXCEPTION 'resource generation exhausted during owner epoch transition'
            USING ERRCODE = '22003';
    END IF;

    v_new_generation := v_current_generation + 1;

    UPDATE myc_effect_owner.effect_fence_state
       SET owner_epoch = p_new_owner_epoch,
           generation = v_new_generation,
           updated_at = clock_timestamp()
     WHERE realm_id = p_realm_id
       AND resource_id = p_resource_id
       AND action_id = p_action_id
       AND effect_owner_id = p_effect_owner_id;

    INSERT INTO myc_effect_owner.effect_owner_epoch_transitions (
        realm_id, resource_id, action_id, effect_owner_id,
        previous_owner_epoch, new_owner_epoch,
        previous_generation, new_generation,
        epoch_authority_receipt_digest,
        physical_fencing_receipt_digest,
        transition_xact_id
    )
    VALUES (
        p_realm_id, p_resource_id, p_action_id, p_effect_owner_id,
        v_current_owner_epoch, p_new_owner_epoch,
        v_current_generation, v_new_generation,
        p_epoch_authority_receipt_digest,
        p_physical_fencing_receipt_digest,
        pg_current_xact_id()::TEXT
    );

    RETURN QUERY SELECT p_new_owner_epoch, v_new_generation;
END
$function$;

-- Strong entry point for new queue-backed effects. It locks and validates the
-- exact current claim and owner epoch before calling the existing claim-fenced
-- mutation, then writes the historical execution attestation before COMMIT.
--
-- Lock order matches 002_worker_claim_fencing.sql:
--   1. worker claim row FOR SHARE;
--   2. owner fence row FOR SHARE;
--   3. the inner effect function reuses those locks;
--   4. append-only execution attestation.
--
-- Exact replay is allowed only when the effect has an existing execution
-- attestation and its (owner_epoch, generation) is not newer than the current
-- command position. Legacy effect rows without attestations fail closed.
CREATE OR REPLACE FUNCTION myc_effect_owner.apply_owner_epoch_claim_fenced_balance_delta(
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
    p_decision_expires_at TIMESTAMPTZ
)
RETURNS TABLE (
    replayed BOOLEAN,
    effect_id UUID,
    accepted_owner_epoch BIGINT,
    accepted_generation BIGINT,
    accepted_worker_id TEXT,
    accepted_claim_epoch BIGINT,
    execution_attestation_id UUID
)
LANGUAGE plpgsql
SECURITY DEFINER
SET search_path = pg_catalog, myc_effect_owner
AS $function$
DECLARE
    v_claim myc_effect_owner.worker_claim_state%ROWTYPE;
    v_current_owner_epoch BIGINT;
    v_current_generation BIGINT;
    v_effect_result RECORD;
    v_existing_effect myc_effect_owner.effect_commits%ROWTYPE;
    v_attestation myc_effect_owner.effect_execution_attestations%ROWTYPE;
    v_attestation_id UUID;
    v_idempotency_lock BIGINT;
    v_operation_lock BIGINT;
    v_first_lock BIGINT;
    v_second_lock BIGINT;
BEGIN
    IF p_owner_epoch IS NULL OR p_owner_epoch <= 0 THEN
        RAISE EXCEPTION 'owner epoch must be positive'
            USING ERRCODE = '22023';
    END IF;

    -- Match the lock order of the existing worker-claim wrapper.
    SELECT *
      INTO v_claim
      FROM myc_effect_owner.worker_claim_state
     WHERE realm_id = p_realm_id
       AND resource_id = p_resource_id
       AND action_id = p_action_id
       AND effect_owner_id = p_effect_owner_id
       AND logical_operation_id = p_logical_operation_id
     FOR SHARE;

    IF NOT FOUND THEN
        RAISE EXCEPTION 'no current worker claim exists for this logical operation'
            USING ERRCODE = 'P0001';
    END IF;

    IF v_claim.worker_id IS DISTINCT FROM p_worker_id
       OR v_claim.claim_epoch IS DISTINCT FROM p_claim_epoch
       OR v_claim.claim_proof_digest IS DISTINCT FROM p_claim_proof_digest THEN
        RAISE EXCEPTION 'worker claim is stale or its proof commitment does not match'
            USING ERRCODE = 'P0001';
    END IF;

    -- Preserve the lock order used by 002_worker_claim_fencing.sql:
    -- worker claim -> idempotency/operation advisory locks -> owner fence.
    -- The locks serialize a concurrent exact replay against the first commit.
    v_idempotency_lock := pg_catalog.hashtextextended(
        'myc-effect-idempotency-v1:' ||
        pg_catalog.encode(p_idempotency_key_digest, 'hex'),
        761112
    );
    v_operation_lock := pg_catalog.hashtextextended(
        'myc-effect-operation-v1:' ||
        octet_length(p_realm_id)::TEXT || ':' || p_realm_id ||
        octet_length(p_resource_id)::TEXT || ':' || p_resource_id ||
        octet_length(p_action_id)::TEXT || ':' || p_action_id ||
        octet_length(p_effect_owner_id)::TEXT || ':' || p_effect_owner_id ||
        octet_length(p_logical_operation_id)::TEXT || ':' || p_logical_operation_id,
        761112
    );

    IF v_idempotency_lock <= v_operation_lock THEN
        v_first_lock := v_idempotency_lock;
        v_second_lock := v_operation_lock;
    ELSE
        v_first_lock := v_operation_lock;
        v_second_lock := v_idempotency_lock;
    END IF;
    PERFORM pg_catalog.pg_advisory_xact_lock(v_first_lock);
    IF v_second_lock <> v_first_lock THEN
        PERFORM pg_catalog.pg_advisory_xact_lock(v_second_lock);
    END IF;

    SELECT owner_epoch, generation
      INTO v_current_owner_epoch, v_current_generation
      FROM myc_effect_owner.effect_fence_state
     WHERE realm_id = p_realm_id
       AND resource_id = p_resource_id
       AND action_id = p_action_id
       AND effect_owner_id = p_effect_owner_id
     FOR SHARE;

    IF NOT FOUND THEN
        RAISE EXCEPTION 'owner epoch/fence domain is not initialized'
            USING ERRCODE = 'P0001';
    END IF;

    IF p_owner_epoch <> v_current_owner_epoch THEN
        RAISE EXCEPTION 'stale owner epoch: supplied %, current %',
            p_owner_epoch, v_current_owner_epoch
            USING ERRCODE = 'P0001';
    END IF;

    IF v_claim.owner_epoch IS DISTINCT FROM p_owner_epoch THEN
        RAISE EXCEPTION 'worker claim belongs to a different owner epoch; a fresh epoch-aware claim is required'
            USING ERRCODE = 'P0001';
    END IF;

    IF p_fence_generation IS NULL
       OR p_fence_generation <> v_current_generation THEN
        RAISE EXCEPTION 'strict effect admission requires the exact current resource generation'
            USING ERRCODE = 'P0001';
    END IF;

    -- A replay is a new delivery of an already committed logical operation,
    -- not a second mutation under its original generation. Verify the current
    -- owner/claim admission above, then return the immutable original effect
    -- and attestation without passing the current generation to the legacy
    -- mutation function (which correctly requires exact generation equality
    -- for an effect's original commit).
    SELECT *
      INTO v_existing_effect
      FROM myc_effect_owner.effect_commits
     WHERE realm_id = p_realm_id
       AND resource_id = p_resource_id
       AND action_id = p_action_id
       AND effect_owner_id = p_effect_owner_id
       AND idempotency_key_digest = p_idempotency_key_digest;

    IF FOUND THEN
        IF v_existing_effect.logical_operation_id IS DISTINCT FROM p_logical_operation_id
           OR v_existing_effect.object_id IS DISTINCT FROM p_object_id
           OR v_existing_effect.currency_code IS DISTINCT FROM p_currency_code
           OR v_existing_effect.request_digest IS DISTINCT FROM p_request_digest
           OR v_existing_effect.decision_receipt_digest IS DISTINCT FROM p_decision_receipt_digest
           OR v_existing_effect.payload_digest IS DISTINCT FROM p_payload_digest
           OR v_existing_effect.delta_minor IS DISTINCT FROM p_delta_minor
           OR v_existing_effect.accepted_generation > v_current_generation THEN
            RAISE EXCEPTION 'idempotency key reused with different command bindings or future effect position'
                USING ERRCODE = 'P0001';
        END IF;

        SELECT *
          INTO v_attestation
          FROM myc_effect_owner.effect_execution_attestations AS attestation
         WHERE attestation.effect_id = v_existing_effect.effect_id;

        IF NOT FOUND THEN
            RAISE EXCEPTION 'legacy effect has no execution attestation; reconciliation is required'
                USING ERRCODE = 'P0001';
        END IF;

        IF v_attestation.effect_id IS DISTINCT FROM v_existing_effect.effect_id
           OR v_attestation.realm_id IS DISTINCT FROM p_realm_id
           OR v_attestation.resource_id IS DISTINCT FROM p_resource_id
           OR v_attestation.action_id IS DISTINCT FROM p_action_id
           OR v_attestation.effect_owner_id IS DISTINCT FROM p_effect_owner_id
           OR v_attestation.object_id IS DISTINCT FROM p_object_id
           OR v_attestation.currency_code IS DISTINCT FROM p_currency_code
           OR v_attestation.logical_operation_id IS DISTINCT FROM p_logical_operation_id
           OR v_attestation.idempotency_key_digest IS DISTINCT FROM p_idempotency_key_digest
           OR v_attestation.request_digest IS DISTINCT FROM p_request_digest
           OR v_attestation.decision_receipt_digest IS DISTINCT FROM p_decision_receipt_digest
           OR v_attestation.payload_digest IS DISTINCT FROM p_payload_digest
           OR v_attestation.accepted_generation IS DISTINCT FROM v_existing_effect.accepted_generation
           OR v_attestation.owner_epoch > p_owner_epoch
           OR v_attestation.accepted_generation > p_fence_generation THEN
            RAISE EXCEPTION 'existing execution attestation differs from exact replay command or effect receipt'
                USING ERRCODE = 'P0001';
        END IF;

        IF v_attestation.owner_epoch > 1
           AND NOT EXISTS (
               SELECT 1
                 FROM myc_effect_owner.effect_owner_epoch_transitions t
                WHERE t.realm_id = p_realm_id
                  AND t.resource_id = p_resource_id
                  AND t.action_id = p_action_id
                  AND t.effect_owner_id = p_effect_owner_id
                  AND t.new_owner_epoch = v_attestation.owner_epoch
                  AND t.new_generation <= v_attestation.accepted_generation
           ) THEN
            RAISE EXCEPTION 'execution attestation has no matching historical owner-epoch transition'
                USING ERRCODE = 'P0001';
        END IF;

        RETURN QUERY SELECT
            TRUE,
            v_attestation.effect_id,
            v_attestation.owner_epoch,
            v_attestation.accepted_generation,
            v_attestation.worker_id,
            v_attestation.claim_epoch,
            v_attestation.attestation_id;
        RETURN;
    END IF;

    SELECT *
      INTO v_effect_result
      FROM myc_effect_owner.apply_claim_fenced_balance_delta(
        p_realm_id, p_resource_id, p_action_id, p_effect_owner_id,
        p_object_id, p_currency_code, p_fence_generation,
        p_idempotency_key_digest, p_logical_operation_id,
        p_request_digest, p_decision_receipt_digest, p_payload_digest,
        p_delta_minor, p_worker_id, p_claim_epoch, p_claim_proof_digest,
        p_decision_clock_profile_id, p_decision_issued_at, p_decision_expires_at
    );

    IF v_effect_result.replayed THEN
        SELECT *
          INTO v_attestation
          FROM myc_effect_owner.effect_execution_attestations AS attestation
         WHERE attestation.effect_id = v_effect_result.effect_id;

        IF NOT FOUND THEN
            RAISE EXCEPTION 'legacy effect has no execution attestation; reconciliation is required'
                USING ERRCODE = 'P0001';
        END IF;

        IF v_attestation.realm_id IS DISTINCT FROM p_realm_id
           OR v_attestation.resource_id IS DISTINCT FROM p_resource_id
           OR v_attestation.action_id IS DISTINCT FROM p_action_id
           OR v_attestation.effect_owner_id IS DISTINCT FROM p_effect_owner_id
           OR v_attestation.object_id IS DISTINCT FROM p_object_id
           OR v_attestation.currency_code IS DISTINCT FROM p_currency_code
           OR v_attestation.logical_operation_id IS DISTINCT FROM p_logical_operation_id
           OR v_attestation.idempotency_key_digest IS DISTINCT FROM p_idempotency_key_digest
           OR v_attestation.request_digest IS DISTINCT FROM p_request_digest
           OR v_attestation.decision_receipt_digest IS DISTINCT FROM p_decision_receipt_digest
           OR v_attestation.payload_digest IS DISTINCT FROM p_payload_digest THEN
            RAISE EXCEPTION 'existing execution attestation differs from exact replay command'
                USING ERRCODE = 'P0001';
        END IF;

        IF v_attestation.owner_epoch > p_owner_epoch
           OR v_attestation.accepted_generation > p_fence_generation THEN
            RAISE EXCEPTION 'existing effect position is newer than replay command position'
                USING ERRCODE = 'P0001';
        END IF;

        RETURN QUERY SELECT
            TRUE,
            v_attestation.effect_id,
            v_attestation.owner_epoch,
            v_attestation.accepted_generation,
            v_attestation.worker_id,
            v_attestation.claim_epoch,
            v_attestation.attestation_id;
        RETURN;
    END IF;

    IF v_current_generation <> p_fence_generation THEN
        RAISE EXCEPTION 'new effect requires the exact current resource generation'
            USING ERRCODE = 'P0001';
    END IF;

    INSERT INTO myc_effect_owner.effect_execution_attestations (
        attestation_id, effect_id,
        realm_id, resource_id, action_id, effect_owner_id,
        object_id, currency_code, logical_operation_id,
        idempotency_key_digest, request_digest, decision_receipt_digest, payload_digest,
        owner_epoch, accepted_generation,
        worker_id, claim_epoch, claim_proof_digest,
        claim_clock_profile_id, claim_issued_at, claim_expires_at,
        database_xact_id
    )
    VALUES (
        pg_catalog.gen_random_uuid(), v_effect_result.effect_id,
        p_realm_id, p_resource_id, p_action_id, p_effect_owner_id,
        p_object_id, p_currency_code, p_logical_operation_id,
        p_idempotency_key_digest, p_request_digest, p_decision_receipt_digest, p_payload_digest,
        p_owner_epoch, v_effect_result.accepted_generation,
        v_claim.worker_id, v_claim.claim_epoch, v_claim.claim_proof_digest,
        v_claim.clock_profile_id, v_claim.issued_at, v_claim.expires_at,
        pg_current_xact_id()::TEXT
    )
    RETURNING attestation_id INTO v_attestation_id;

    RETURN QUERY SELECT
        FALSE,
        v_effect_result.effect_id,
        p_owner_epoch,
        v_effect_result.accepted_generation,
        v_claim.worker_id,
        v_claim.claim_epoch,
        v_attestation_id;
END
$function$;

REVOKE ALL ON TABLE myc_effect_owner.effect_owner_epoch_transitions FROM PUBLIC;
REVOKE ALL ON TABLE myc_effect_owner.effect_execution_attestations FROM PUBLIC;
REVOKE ALL ON ALL FUNCTIONS IN SCHEMA myc_effect_owner FROM PUBLIC;

COMMENT ON COLUMN myc_effect_owner.effect_fence_state.owner_epoch IS
'Owner-local monotonic epoch. It is not a substitute for externally authenticated single-writer fencing or promotion high-water evidence.';

COMMENT ON TABLE myc_effect_owner.effect_owner_epoch_transitions IS
'Append-only owner epoch transition evidence. Receipt digests must be authenticated by an external epoch authority and physical fencing service before invocation.';

COMMENT ON TABLE myc_effect_owner.effect_execution_attestations IS
'Append-only historical execution coordinates written in the same transaction as effect_commits: owner epoch/generation, exact worker claim and command commitments. Does not itself authenticate external claim/epoch signatures.';

COMMENT ON COLUMN myc_effect_owner.worker_claim_state.owner_epoch IS
'Owner epoch under which this worker claim was installed. A claim from an earlier owner epoch cannot execute after an epoch transition.';

COMMENT ON FUNCTION myc_effect_owner.install_worker_claim_for_owner_epoch(
    TEXT, TEXT, TEXT, TEXT, BIGINT, TEXT, TEXT, BIGINT, BYTEA, TEXT, TIMESTAMPTZ, TIMESTAMPTZ
) IS
'Installs a worker claim bound to the current owner epoch. Crossing owner epochs requires a strictly higher claim epoch. External claim authentication remains the caller responsibility.';

COMMENT ON FUNCTION myc_effect_owner.advance_effect_owner_epoch(
    TEXT, TEXT, TEXT, TEXT, BIGINT, BYTEA, BYTEA
) IS
'Advances owner epoch and resource generation under an exclusive owner-fence row lock. Caller must first authenticate external epoch authority and physically fence the previous primary. SQL stores proof commitments only.';

COMMENT ON FUNCTION myc_effect_owner.apply_owner_epoch_claim_fenced_balance_delta(
    TEXT, TEXT, TEXT, TEXT, TEXT, TEXT, BIGINT, BIGINT, BYTEA, TEXT, BYTEA, BYTEA, BYTEA, BIGINT, TEXT, BIGINT, BYTEA, TEXT, TIMESTAMPTZ, TIMESTAMPTZ
) IS
'Strict queue-backed entry point: checks exact current owner epoch and worker claim, executes the existing fenced effect, and appends immutable execution coordinates atomically with the committed effect.';

COMMIT;
