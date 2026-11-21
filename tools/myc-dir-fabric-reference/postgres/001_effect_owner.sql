-- Mycelix Directory Fabric PostgreSQL effect-owner reference schema v0.1
-- Reference adapter only; not an automatic Mycelix-DeSci production migration.
-- PostgreSQL 15+ recommended. No extension dependency is required.

BEGIN;

CREATE SCHEMA IF NOT EXISTS myc_effect_owner;

CREATE TABLE IF NOT EXISTS myc_effect_owner.effect_fence_state (
    realm_id TEXT NOT NULL CHECK (btrim(realm_id) <> ''),
    resource_id TEXT NOT NULL CHECK (btrim(resource_id) <> ''),
    action_id TEXT NOT NULL CHECK (btrim(action_id) <> ''),
    effect_owner_id TEXT NOT NULL CHECK (btrim(effect_owner_id) <> ''),
    generation BIGINT NOT NULL CHECK (generation > 0),
    owner_clock_profile_id TEXT NOT NULL DEFAULT 'postgres-db-clock-v1'
        CHECK (btrim(owner_clock_profile_id) <> ''),
    max_clock_uncertainty_ms BIGINT NOT NULL DEFAULT 25
        CHECK (max_clock_uncertainty_ms >= 0),
    updated_at TIMESTAMPTZ NOT NULL DEFAULT clock_timestamp(),
    PRIMARY KEY (realm_id, resource_id, action_id, effect_owner_id)
);

CREATE TABLE IF NOT EXISTS myc_effect_owner.effect_fence_revocations (
    realm_id TEXT NOT NULL,
    resource_id TEXT NOT NULL,
    action_id TEXT NOT NULL,
    effect_owner_id TEXT NOT NULL,
    revocation_digest BYTEA NOT NULL CHECK (
        octet_length(revocation_digest) = 32
        AND revocation_digest <> decode(repeat('00', 32), 'hex')
    ),
    installed_generation BIGINT NOT NULL CHECK (installed_generation > 1),
    committed_at TIMESTAMPTZ NOT NULL DEFAULT clock_timestamp(),
    PRIMARY KEY (
        realm_id, resource_id, action_id, effect_owner_id, revocation_digest
    ),
    UNIQUE (
        realm_id, resource_id, action_id, effect_owner_id, installed_generation
    ),
    FOREIGN KEY (realm_id, resource_id, action_id, effect_owner_id)
        REFERENCES myc_effect_owner.effect_fence_state (
            realm_id, resource_id, action_id, effect_owner_id
        )
);

-- Example protected object: integer minor-unit balances. Multiple object_id
-- values share one fence domain and can mutate under compatible shared locks.
CREATE TABLE IF NOT EXISTS myc_effect_owner.protected_balance (
    realm_id TEXT NOT NULL,
    resource_id TEXT NOT NULL,
    action_id TEXT NOT NULL,
    effect_owner_id TEXT NOT NULL,
    object_id TEXT NOT NULL CHECK (btrim(object_id) <> ''),
    currency_code TEXT NOT NULL CHECK (currency_code ~ '^[A-Z]{3}$'),
    balance_minor BIGINT NOT NULL,
    PRIMARY KEY (
        realm_id, resource_id, action_id, effect_owner_id, object_id, currency_code
    ),
    FOREIGN KEY (realm_id, resource_id, action_id, effect_owner_id)
        REFERENCES myc_effect_owner.effect_fence_state (
            realm_id, resource_id, action_id, effect_owner_id
        )
);

-- Append-only effect log and idempotency ledger.
CREATE TABLE IF NOT EXISTS myc_effect_owner.effect_commits (
    effect_id UUID PRIMARY KEY,
    realm_id TEXT NOT NULL,
    resource_id TEXT NOT NULL,
    action_id TEXT NOT NULL,
    effect_owner_id TEXT NOT NULL,
    object_id TEXT NOT NULL,
    currency_code TEXT NOT NULL CHECK (currency_code ~ '^[A-Z]{3}$'),
    idempotency_key_digest BYTEA NOT NULL CHECK (
        octet_length(idempotency_key_digest) = 32
        AND idempotency_key_digest <> decode(repeat('00', 32), 'hex')
    ),
    logical_operation_id TEXT NOT NULL CHECK (
        btrim(logical_operation_id) <> '' AND octet_length(logical_operation_id) <= 512
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
    accepted_generation BIGINT NOT NULL CHECK (accepted_generation > 0),
    delta_minor BIGINT NOT NULL CHECK (delta_minor <> 0),
    balance_after_minor BIGINT NOT NULL,
    committed_at TIMESTAMPTZ NOT NULL DEFAULT clock_timestamp(),
    UNIQUE (
        realm_id, resource_id, action_id, effect_owner_id, idempotency_key_digest
    ),
    UNIQUE (
        realm_id, resource_id, action_id, effect_owner_id, logical_operation_id
    ),
    FOREIGN KEY (realm_id, resource_id, action_id, effect_owner_id)
        REFERENCES myc_effect_owner.effect_fence_state (
            realm_id, resource_id, action_id, effect_owner_id
        )
);

CREATE INDEX IF NOT EXISTS effect_commits_by_object
ON myc_effect_owner.effect_commits (
    realm_id, resource_id, action_id, effect_owner_id, object_id, committed_at
);

CREATE OR REPLACE FUNCTION myc_effect_owner.reject_effect_commit_mutation()
RETURNS TRIGGER
LANGUAGE plpgsql
SET search_path = pg_catalog, myc_effect_owner
AS $function$
BEGIN
    RAISE EXCEPTION 'effect_commits is append-only'
        USING ERRCODE = '55000';
END
$function$;

DROP TRIGGER IF EXISTS effect_commits_no_update_delete
ON myc_effect_owner.effect_commits;

CREATE TRIGGER effect_commits_no_update_delete
BEFORE UPDATE OR DELETE ON myc_effect_owner.effect_commits
FOR EACH ROW EXECUTE FUNCTION myc_effect_owner.reject_effect_commit_mutation();

-- Owner-local generation advance. The enclosing transaction must COMMIT before
-- the owner signs or publishes its durable-fence acknowledgement.
CREATE OR REPLACE FUNCTION myc_effect_owner.advance_effect_fence(
    p_realm_id TEXT,
    p_resource_id TEXT,
    p_action_id TEXT,
    p_effect_owner_id TEXT,
    p_revocation_digest BYTEA
)
RETURNS BIGINT
LANGUAGE plpgsql
SECURITY DEFINER
SET search_path = pg_catalog, myc_effect_owner
AS $function$
DECLARE
    v_current_generation BIGINT;
    v_existing_generation BIGINT;
    v_next_generation BIGINT;
BEGIN
    IF p_realm_id IS NULL OR p_resource_id IS NULL OR p_action_id IS NULL
       OR p_effect_owner_id IS NULL
       OR btrim(p_realm_id) = '' OR btrim(p_resource_id) = ''
       OR btrim(p_action_id) = '' OR btrim(p_effect_owner_id) = '' THEN
        RAISE EXCEPTION 'fence scope identifiers must be nonempty'
            USING ERRCODE = '22023';
    END IF;
    IF p_revocation_digest IS NULL
       OR octet_length(p_revocation_digest) <> 32
       OR p_revocation_digest = decode(repeat('00', 32), 'hex') THEN
        RAISE EXCEPTION 'revocation digest must be a nonzero 32-byte value'
            USING ERRCODE = '22023';
    END IF;

    SELECT generation
      INTO v_current_generation
      FROM myc_effect_owner.effect_fence_state
     WHERE realm_id = p_realm_id
       AND resource_id = p_resource_id
       AND action_id = p_action_id
       AND effect_owner_id = p_effect_owner_id
     FOR UPDATE;

    IF NOT FOUND THEN
        RAISE EXCEPTION 'fence domain is not initialized'
            USING ERRCODE = 'P0001';
    END IF;

    SELECT installed_generation
      INTO v_existing_generation
      FROM myc_effect_owner.effect_fence_revocations
     WHERE realm_id = p_realm_id
       AND resource_id = p_resource_id
       AND action_id = p_action_id
       AND effect_owner_id = p_effect_owner_id
       AND revocation_digest = p_revocation_digest;

    IF FOUND THEN
        RETURN v_existing_generation;
    END IF;

    IF v_current_generation = 9223372036854775807 THEN
        RAISE EXCEPTION 'fence generation exhausted'
            USING ERRCODE = '22003';
    END IF;

    v_next_generation := v_current_generation + 1;

    UPDATE myc_effect_owner.effect_fence_state
       SET generation = v_next_generation,
           updated_at = clock_timestamp()
     WHERE realm_id = p_realm_id
       AND resource_id = p_resource_id
       AND action_id = p_action_id
       AND effect_owner_id = p_effect_owner_id;

    INSERT INTO myc_effect_owner.effect_fence_revocations (
        realm_id, resource_id, action_id, effect_owner_id,
        revocation_digest, installed_generation
    )
    VALUES (
        p_realm_id, p_resource_id, p_action_id, p_effect_owner_id,
        p_revocation_digest, v_next_generation
    );

    RETURN v_next_generation;
END
$function$;

-- The PEP MUST already verify the signed decision receipt, exact Allow outcome,
-- request/realm/policy/cut binding and requested effect scope. SQL records the
-- digest only; it does not verify signatures or infer Allow from a digest.
--
-- Lock order:
--   1. transaction-scoped advisory locks for idempotency + operation identity;
--   2. shared fence-row lock;
--   3. protected-object update;
--   4. immutable effect/idempotency receipt.
--
-- Hash collisions add contention only; database unique constraints are the
-- final duplicate guard.
CREATE OR REPLACE FUNCTION myc_effect_owner.apply_balance_delta(
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
    p_decision_clock_profile_id TEXT,
    p_decision_issued_at TIMESTAMPTZ,
    p_decision_expires_at TIMESTAMPTZ
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
    v_idempotency_lock BIGINT;
    v_operation_lock BIGINT;
    v_first_lock BIGINT;
    v_second_lock BIGINT;
    v_current_generation BIGINT;
    v_owner_clock_profile_id TEXT;
    v_max_clock_uncertainty_ms BIGINT;
    v_now TIMESTAMPTZ;
    v_lower_bound TIMESTAMPTZ;
    v_upper_bound TIMESTAMPTZ;
    v_uncertainty INTERVAL;
    v_existing myc_effect_owner.effect_commits%ROWTYPE;
    v_balance_after BIGINT;
    v_effect_id UUID;
BEGIN
    IF p_realm_id IS NULL OR p_resource_id IS NULL OR p_action_id IS NULL
       OR p_effect_owner_id IS NULL OR p_object_id IS NULL
       OR p_logical_operation_id IS NULL
       OR btrim(p_realm_id) = '' OR btrim(p_resource_id) = ''
       OR btrim(p_action_id) = '' OR btrim(p_effect_owner_id) = ''
       OR btrim(p_object_id) = '' OR btrim(p_logical_operation_id) = ''
       OR octet_length(p_logical_operation_id) > 512 THEN
        RAISE EXCEPTION 'scope, object and logical operation identifiers must be nonempty and bounded'
            USING ERRCODE = '22023';
    END IF;
    IF p_currency_code IS NULL OR p_delta_minor IS NULL
       OR p_currency_code !~ '^[A-Z]{3}$' OR p_delta_minor = 0 THEN
        RAISE EXCEPTION 'currency must be three uppercase letters and delta must be nonzero'
            USING ERRCODE = '22023';
    END IF;
    IF p_fence_generation IS NULL OR p_fence_generation <= 0 THEN
        RAISE EXCEPTION 'fence generation must be positive'
            USING ERRCODE = '22023';
    END IF;
    IF p_idempotency_key_digest IS NULL
       OR octet_length(p_idempotency_key_digest) <> 32
       OR p_request_digest IS NULL OR octet_length(p_request_digest) <> 32
       OR p_decision_receipt_digest IS NULL
       OR octet_length(p_decision_receipt_digest) <> 32
       OR p_payload_digest IS NULL OR octet_length(p_payload_digest) <> 32
       OR p_idempotency_key_digest = decode(repeat('00', 32), 'hex')
       OR p_request_digest = decode(repeat('00', 32), 'hex')
       OR p_decision_receipt_digest = decode(repeat('00', 32), 'hex')
       OR p_payload_digest = decode(repeat('00', 32), 'hex') THEN
        RAISE EXCEPTION 'all command digests must be nonzero 32-byte values'
            USING ERRCODE = '22023';
    END IF;

    -- Serialize same-key and same-operation calls, acquiring advisory lock
    -- keys in a deterministic order to avoid lock-order inversion.
    v_idempotency_lock := pg_catalog.hashtextextended(
        'myc-effect-idempotency-v1:' || pg_catalog.encode(p_idempotency_key_digest, 'hex'),
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

    -- Compatible shared locks allow independent effects in one fence domain
    -- to proceed in parallel. Revocation's exclusive row lock conflicts.
    SELECT generation, owner_clock_profile_id, max_clock_uncertainty_ms
      INTO v_current_generation, v_owner_clock_profile_id, v_max_clock_uncertainty_ms
      FROM myc_effect_owner.effect_fence_state
     WHERE realm_id = p_realm_id
       AND resource_id = p_resource_id
       AND action_id = p_action_id
       AND effect_owner_id = p_effect_owner_id
     FOR SHARE;

    IF NOT FOUND THEN
        RAISE EXCEPTION 'fence domain is not initialized'
            USING ERRCODE = 'P0001';
    END IF;

    -- An exact replay is read-only, even after a later revocation. Return only
    -- immutable receipt identity, never the current protected-object value.
    SELECT *
      INTO v_existing
      FROM myc_effect_owner.effect_commits
     WHERE realm_id = p_realm_id
       AND resource_id = p_resource_id
       AND action_id = p_action_id
       AND effect_owner_id = p_effect_owner_id
       AND idempotency_key_digest = p_idempotency_key_digest;

    IF FOUND THEN
        IF v_existing.logical_operation_id <> p_logical_operation_id
           OR v_existing.object_id <> p_object_id
           OR v_existing.currency_code <> p_currency_code
           OR v_existing.request_digest <> p_request_digest
           OR v_existing.decision_receipt_digest <> p_decision_receipt_digest
           OR v_existing.payload_digest <> p_payload_digest
           OR v_existing.accepted_generation <> p_fence_generation
           OR v_existing.delta_minor <> p_delta_minor THEN
            RAISE EXCEPTION 'idempotency key reused with different command bindings'
                USING ERRCODE = 'P0001';
        END IF;

        RETURN QUERY SELECT TRUE, v_existing.effect_id, v_existing.accepted_generation;
        RETURN;
    END IF;

    -- A logical operation may not silently acquire a new idempotency key.
    SELECT *
      INTO v_existing
      FROM myc_effect_owner.effect_commits
     WHERE realm_id = p_realm_id
       AND resource_id = p_resource_id
       AND action_id = p_action_id
       AND effect_owner_id = p_effect_owner_id
       AND logical_operation_id = p_logical_operation_id;

    IF FOUND THEN
        RAISE EXCEPTION 'logical operation already exists under another idempotency key'
            USING ERRCODE = 'P0001';
    END IF;

    IF p_fence_generation <> v_current_generation THEN
        RAISE EXCEPTION 'stale or unissued fence generation: supplied %, current %',
            p_fence_generation, v_current_generation
            USING ERRCODE = 'P0001';
    END IF;

    IF p_decision_clock_profile_id IS NULL
       OR p_decision_clock_profile_id IS DISTINCT FROM v_owner_clock_profile_id THEN
        RAISE EXCEPTION 'decision clock profile differs from the effect owner profile'
            USING ERRCODE = 'P0001';
    END IF;
    IF p_decision_issued_at IS NULL OR p_decision_expires_at IS NULL
       OR p_decision_issued_at >= p_decision_expires_at THEN
        RAISE EXCEPTION 'decision validity interval is invalid'
            USING ERRCODE = '22023';
    END IF;

    -- Acquire the protected-object row before the final time check. The next
    -- UPDATE therefore does not queue behind another writer after validation.
    PERFORM 1
      FROM myc_effect_owner.protected_balance
     WHERE realm_id = p_realm_id
       AND resource_id = p_resource_id
       AND action_id = p_action_id
       AND effect_owner_id = p_effect_owner_id
       AND object_id = p_object_id
       AND currency_code = p_currency_code
     FOR UPDATE;

    IF NOT FOUND THEN
        RAISE EXCEPTION 'protected balance object does not exist'
            USING ERRCODE = 'P0001';
    END IF;

    v_now := clock_timestamp();
    v_uncertainty := make_interval(
        secs => v_max_clock_uncertainty_ms::double precision / 1000.0
    );
    v_lower_bound := v_now - v_uncertainty;
    v_upper_bound := v_now + v_uncertainty;

    IF v_lower_bound < p_decision_issued_at
       OR v_upper_bound >= p_decision_expires_at THEN
        RAISE EXCEPTION 'Allow decision is not valid for the full owner clock uncertainty interval'
            USING ERRCODE = 'P0001';
    END IF;

    UPDATE myc_effect_owner.protected_balance
       SET balance_minor = balance_minor + p_delta_minor
     WHERE realm_id = p_realm_id
       AND resource_id = p_resource_id
       AND action_id = p_action_id
       AND effect_owner_id = p_effect_owner_id
       AND object_id = p_object_id
       AND currency_code = p_currency_code
    RETURNING balance_minor INTO v_balance_after;

    IF NOT FOUND THEN
        RAISE EXCEPTION 'protected balance object does not exist'
            USING ERRCODE = 'P0001';
    END IF;

    v_effect_id := pg_catalog.gen_random_uuid();

    INSERT INTO myc_effect_owner.effect_commits (
        effect_id, realm_id, resource_id, action_id, effect_owner_id,
        object_id, currency_code, idempotency_key_digest, logical_operation_id,
        request_digest, decision_receipt_digest, payload_digest,
        accepted_generation, delta_minor, balance_after_minor
    )
    VALUES (
        v_effect_id, p_realm_id, p_resource_id, p_action_id, p_effect_owner_id,
        p_object_id, p_currency_code, p_idempotency_key_digest, p_logical_operation_id,
        p_request_digest, p_decision_receipt_digest, p_payload_digest,
        p_fence_generation, p_delta_minor, v_balance_after
    );

    RETURN QUERY SELECT FALSE, v_effect_id, p_fence_generation;
END
$function$;

REVOKE ALL ON SCHEMA myc_effect_owner FROM PUBLIC;
REVOKE ALL ON ALL TABLES IN SCHEMA myc_effect_owner FROM PUBLIC;
REVOKE ALL ON ALL FUNCTIONS IN SCHEMA myc_effect_owner FROM PUBLIC;

COMMENT ON TABLE myc_effect_owner.effect_fence_state IS
'Owner-local durable fence high-water marks. Mutations hold FOR SHARE until commit; revocation advances under an exclusive row lock.';
COMMENT ON TABLE myc_effect_owner.effect_commits IS
'Append-only DB-local effect journal and idempotency ledger. It does not prove delivery to a remote API.';
COMMENT ON FUNCTION myc_effect_owner.apply_balance_delta(
    TEXT, TEXT, TEXT, TEXT, TEXT, TEXT, BIGINT, BYTEA, TEXT, BYTEA, BYTEA, BYTEA, BIGINT, TEXT, TIMESTAMPTZ, TIMESTAMPTZ
) IS
'Assumes the caller independently verified an exact Allow decision. Rechecks the full decision validity interval after operation, fence and protected-object locks are acquired, then atomically mutates and records the effect.';
COMMENT ON FUNCTION myc_effect_owner.advance_effect_fence(TEXT, TEXT, TEXT, TEXT, BYTEA) IS
'Advances an owner-local generation. Publish the owner acknowledgement only after the enclosing transaction durably commits.';

COMMIT;
