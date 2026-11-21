-- Mycelix Directory Fabric PostgreSQL worker-claim fencing extension v0.1
-- Apply after 001_effect_owner.sql on the same database.
-- Claim issuer authentication is external; this stores the verified claim
-- commitment and makes the current claim comparison transactional with effect.

BEGIN;

CREATE TABLE IF NOT EXISTS myc_effect_owner.worker_claim_state (
    realm_id TEXT NOT NULL,
    resource_id TEXT NOT NULL,
    action_id TEXT NOT NULL,
    effect_owner_id TEXT NOT NULL,
    logical_operation_id TEXT NOT NULL CHECK (
        btrim(logical_operation_id) <> '' AND octet_length(logical_operation_id) <= 512
    ),
    worker_id TEXT NOT NULL CHECK (
        btrim(worker_id) <> '' AND octet_length(worker_id) <= 512
    ),
    claim_epoch BIGINT NOT NULL CHECK (claim_epoch > 0),
    claim_proof_digest BYTEA NOT NULL CHECK (
        octet_length(claim_proof_digest) = 32
        AND claim_proof_digest <> decode(repeat('00', 32), 'hex')
    ),
    issued_at TIMESTAMPTZ NOT NULL,
    expires_at TIMESTAMPTZ NOT NULL,
    installed_at TIMESTAMPTZ NOT NULL DEFAULT clock_timestamp(),
    PRIMARY KEY (
        realm_id, resource_id, action_id, effect_owner_id, logical_operation_id
    ),
    FOREIGN KEY (realm_id, resource_id, action_id, effect_owner_id)
        REFERENCES myc_effect_owner.effect_fence_state (
            realm_id, resource_id, action_id, effect_owner_id
        ),
    CHECK (issued_at < expires_at)
);

-- The trusted claim issuer must authenticate the claim before this function
-- is called. The proof digest is a commitment, not self-authenticating proof.
-- Only strictly increasing epochs can replace an existing claim. An exact
-- duplicate install is idempotent; an epoch collision with different content
-- or a lower epoch fails closed.
CREATE OR REPLACE FUNCTION myc_effect_owner.install_worker_claim(
    p_realm_id TEXT,
    p_resource_id TEXT,
    p_action_id TEXT,
    p_effect_owner_id TEXT,
    p_logical_operation_id TEXT,
    p_worker_id TEXT,
    p_claim_epoch BIGINT,
    p_claim_proof_digest BYTEA,
    p_issued_at TIMESTAMPTZ,
    p_expires_at TIMESTAMPTZ
)
RETURNS BIGINT
LANGUAGE plpgsql
SECURITY DEFINER
SET search_path = pg_catalog, myc_effect_owner
AS $function$
DECLARE
    v_installed_epoch BIGINT;
    v_current myc_effect_owner.worker_claim_state%ROWTYPE;
BEGIN
    IF p_realm_id IS NULL OR p_resource_id IS NULL OR p_action_id IS NULL
       OR p_effect_owner_id IS NULL OR p_logical_operation_id IS NULL
       OR p_worker_id IS NULL
       OR btrim(p_realm_id) = '' OR btrim(p_resource_id) = ''
       OR btrim(p_action_id) = '' OR btrim(p_effect_owner_id) = ''
       OR btrim(p_logical_operation_id) = '' OR btrim(p_worker_id) = ''
       OR octet_length(p_logical_operation_id) > 512
       OR octet_length(p_worker_id) > 512 THEN
        RAISE EXCEPTION 'claim scope, operation and worker identifiers must be bounded and nonempty'
            USING ERRCODE = '22023';
    END IF;
    IF p_claim_epoch IS NULL OR p_claim_epoch <= 0 THEN
        RAISE EXCEPTION 'claim epoch must be positive'
            USING ERRCODE = '22023';
    END IF;
    IF p_claim_proof_digest IS NULL
       OR octet_length(p_claim_proof_digest) <> 32
       OR p_claim_proof_digest = decode(repeat('00', 32), 'hex')
       OR p_issued_at IS NULL OR p_expires_at IS NULL
       OR p_issued_at >= p_expires_at THEN
        RAISE EXCEPTION 'claim proof and validity interval are invalid'
            USING ERRCODE = '22023';
    END IF;

    INSERT INTO myc_effect_owner.worker_claim_state (
        realm_id, resource_id, action_id, effect_owner_id,
        logical_operation_id, worker_id, claim_epoch, claim_proof_digest,
        issued_at, expires_at
    )
    VALUES (
        p_realm_id, p_resource_id, p_action_id, p_effect_owner_id,
        p_logical_operation_id, p_worker_id, p_claim_epoch, p_claim_proof_digest,
        p_issued_at, p_expires_at
    )
    ON CONFLICT (
        realm_id, resource_id, action_id, effect_owner_id, logical_operation_id
    ) DO UPDATE
       SET worker_id = EXCLUDED.worker_id,
           claim_epoch = EXCLUDED.claim_epoch,
           claim_proof_digest = EXCLUDED.claim_proof_digest,
           issued_at = EXCLUDED.issued_at,
           expires_at = EXCLUDED.expires_at,
           installed_at = clock_timestamp()
     WHERE myc_effect_owner.worker_claim_state.claim_epoch < EXCLUDED.claim_epoch
    RETURNING claim_epoch INTO v_installed_epoch;

    IF FOUND THEN
        RETURN v_installed_epoch;
    END IF;

    -- ON CONFLICT did not update: permit only an exact idempotent replay.
    SELECT *
      INTO v_current
      FROM myc_effect_owner.worker_claim_state
     WHERE realm_id = p_realm_id
       AND resource_id = p_resource_id
       AND action_id = p_action_id
       AND effect_owner_id = p_effect_owner_id
       AND logical_operation_id = p_logical_operation_id
     FOR UPDATE;

    IF FOUND
       AND v_current.worker_id = p_worker_id
       AND v_current.claim_epoch = p_claim_epoch
       AND v_current.claim_proof_digest = p_claim_proof_digest
       AND v_current.issued_at = p_issued_at
       AND v_current.expires_at = p_expires_at THEN
        RETURN v_current.claim_epoch;
    END IF;

    RAISE EXCEPTION 'worker claim epoch must strictly advance; equal epoch must be an exact replay'
        USING ERRCODE = 'P0001';
END
$function$;

-- This wrapper holds FOR SHARE on the exact current claim row until the
-- enclosing transaction commits. Claim reassignment/update therefore waits
-- for in-flight effects; a delayed old worker cannot pass after the newer
-- claim install transaction commits.
--
-- The original effect function performs the resource-generation check and
-- protected mutation under its own shared owner-fence lock in this same
-- surrounding SQL transaction. All locks are retained until COMMIT.
CREATE OR REPLACE FUNCTION myc_effect_owner.apply_claim_fenced_balance_delta(
    p_realm_id TEXT,
    p_resource_id TEXT,
    p_action_id TEXT,
    p_effect_owner_id TEXT,
    p_object_id TEXT,
    p_currency_code TEXT,
    p_fence_generation BIGINT,
    p_idempotency_key_digest BYTEA,
    p_logical_operation_id TEXT,
    p_request_digest BYTEA,
    p_decision_receipt_digest BYTEA,
    p_payload_digest BYTEA,
    p_delta_minor BIGINT,
    p_worker_id TEXT,
    p_claim_epoch BIGINT,
    p_claim_proof_digest BYTEA
)
RETURNS TABLE (
    replayed BOOLEAN,
    effect_id UUID,
    accepted_generation BIGINT
)
LANGUAGE plpgsql
SECURITY DEFINER
SET search_path = pg_catalog, myc_effect_owner
AS $function$
DECLARE
    v_claim myc_effect_owner.worker_claim_state%ROWTYPE;
    v_now TIMESTAMPTZ;
BEGIN
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

    v_now := clock_timestamp();
    IF v_now < v_claim.issued_at OR v_now >= v_claim.expires_at THEN
        RAISE EXCEPTION 'worker claim is not currently valid'
            USING ERRCODE = 'P0001';
    END IF;

    RETURN QUERY
    SELECT *
      FROM myc_effect_owner.apply_balance_delta(
        p_realm_id, p_resource_id, p_action_id, p_effect_owner_id,
        p_object_id, p_currency_code, p_fence_generation,
        p_idempotency_key_digest, p_logical_operation_id,
        p_request_digest, p_decision_receipt_digest, p_payload_digest,
        p_delta_minor
    );
END
$function$;

REVOKE ALL ON TABLE myc_effect_owner.worker_claim_state FROM PUBLIC;
REVOKE ALL ON ALL FUNCTIONS IN SCHEMA myc_effect_owner FROM PUBLIC;

COMMENT ON TABLE myc_effect_owner.worker_claim_state IS
'Durable latest worker claim for a logical operation; old epochs cannot execute after reassignment commits.';
COMMENT ON FUNCTION myc_effect_owner.install_worker_claim(
    TEXT, TEXT, TEXT, TEXT, TEXT, TEXT, BIGINT, BYTEA, TIMESTAMPTZ, TIMESTAMPTZ
) IS
'Installs strictly increasing claim epochs; exact same-epoch replay is idempotent. Claim proof authentication is external.';
COMMENT ON FUNCTION myc_effect_owner.apply_claim_fenced_balance_delta(
    TEXT, TEXT, TEXT, TEXT, TEXT, TEXT, BIGINT, BYTEA, TEXT, BYTEA, BYTEA, BYTEA, BIGINT, TEXT, BIGINT, BYTEA
) IS
'Holds the exact current worker-claim row FOR SHARE, then calls the existing resource-fenced mutation in the same SQL transaction. Requires caller to authenticate the signed claim and decision.';

COMMIT;
