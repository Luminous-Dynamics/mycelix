\set ON_ERROR_STOP on

-- Disposable PostgreSQL test for 005_authority_proof_receipt_store.sql.
-- The synthetic signatures are not expected to verify cryptographically:
-- signature verification is exercised by the Rust proof crate. This SQL
-- integration test validates trusted-controller admission, row consistency,
-- external sequence mirroring, append-only links and role separation.

BEGIN;
SET LOCAL synchronous_commit = on;

DO $test$
DECLARE
    v_realm TEXT := 'authority-proof-test-realm';
    v_resource TEXT := 'authority-proof-test-resource';
    v_action TEXT := 'balance-adjust';
    v_owner TEXT := 'authority-proof-test-owner';
    v_deployment TEXT := 'authority-proof-test-deployment';
    v_primary TEXT := 'postgres-proof-primary';
    v_lease_id UUID := '30000000-0000-4000-8000-000000000001';
    v_proof_id UUID := '40000000-0000-4000-8000-000000000001';
    v_wrong_proof_id UUID := '40000000-0000-4000-8000-000000000002';
    v_lease_hash BYTEA := decode(repeat('aa', 32), 'hex');
    v_external_sequence_digest BYTEA := decode(repeat('dd', 32), 'hex');
    v_canonical_digest BYTEA := decode(repeat('bb', 32), 'hex');
    v_signed_receipt_hash BYTEA := decode(repeat('cc', 32), 'hex');
    v_system_id TEXT;
    v_timeline BIGINT;
    v_now TIMESTAMPTZ;
    v_lease_json JSONB;
    v_proof_json JSONB;
    v_wrong_proof_json JSONB;
    v_installed UUID;
    v_rejected BOOLEAN;
BEGIN
    INSERT INTO myc_effect_owner.effect_fence_state (
        realm_id, resource_id, action_id, effect_owner_id,
        generation, owner_epoch, owner_clock_profile_id, max_clock_uncertainty_ms
    )
    VALUES (
        v_realm, v_resource, v_action, v_owner,
        1, 1, 'postgres-db-clock-v1', 25
    );

    SELECT system.system_identifier::TEXT, checkpoint.timeline_id::BIGINT
      INTO v_system_id, v_timeline
      FROM pg_catalog.pg_control_system() AS system
      CROSS JOIN pg_catalog.pg_control_checkpoint() AS checkpoint;

    v_now := clock_timestamp();
    v_lease_json := jsonb_build_object(
        'protocol', 'mycelix-desci-authority-write-lease',
        'protocol_version', 1,
        'codec', 'mycelix-canonical-binary-v1',
        'schema_version', 1,
        'lease', jsonb_build_object(
            'lease_id', v_lease_id,
            'deployment_id', v_deployment,
            'primary_id', v_primary,
            'database_system_identifier', v_system_id,
            'postgres_timeline', v_timeline,
            'generation', 7,
            'phase', 'bootstrap',
            'epoch_number', NULL,
            'epoch_hash', NULL,
            'allowed_scopes', jsonb_build_array('directory_fabric_effect'),
            'issued_at', v_now - interval '1 minute',
            'not_before', v_now - interval '30 seconds',
            'expires_at', v_now + interval '5 minutes',
            'reason', 'synthetic authority proof SQL test'
        ),
        'signer_key_id', 'lease-controller-test',
        'signer_public_key', to_jsonb(array_fill(84, ARRAY[32])),
        'signature', to_jsonb(array_fill(85, ARRAY[64]))
    );

    SELECT myc_effect_owner.install_effect_owner_authority_write_lease(
        v_realm, v_resource, v_action, v_owner,
        1, 1, v_lease_hash, v_external_sequence_digest, NULL, v_lease_json
    ) INTO v_installed;
    IF v_installed <> v_lease_id THEN
        RAISE EXCEPTION 'base signed lease did not install';
    END IF;

    v_proof_json := jsonb_build_object(
        'protocol', 'mycelix-authority-proof',
        'version', 1,
        'receipt', jsonb_build_object(
            'proof_id', v_proof_id,
            'purpose', 'lease_high_water',
            'issuer_role', 'lease_high_water_authority',
            'deployment_id', v_deployment,
            'database_system_identifier', v_system_id,
            'primary_id', v_primary,
            'postgres_timeline', v_timeline,
            'owner_epoch', 1,
            'sequence', 1,
            'issued_at', v_now - interval '1 minute',
            'not_before', v_now - interval '30 seconds',
            'expires_at', v_now + interval '5 minutes',
            'payload', jsonb_build_object(
                'kind', 'lease_high_water',
                'previous_generation', 6,
                'previous_lease_hash', to_jsonb(array_fill(102, ARRAY[32])),
                'new_generation', 7,
                'new_lease_id', v_lease_id,
                'new_lease_hash', to_jsonb(array_fill(170, ARRAY[32])),
                'authority_policy_epoch', 1
            )
        ),
        'issuer_key_id', 'highwater-test-key',
        'issuer_public_key', to_jsonb(array_fill(86, ARRAY[32])),
        'signature', to_jsonb(array_fill(87, ARRAY[64]))
    );

    SELECT myc_effect_owner.install_authority_proof_receipt(
        v_proof_json,
        v_canonical_digest,
        v_signed_receipt_hash,
        v_external_sequence_digest,
        'test-proof-controller'
    ) INTO v_installed;
    IF v_installed <> v_proof_id THEN
        RAISE EXCEPTION 'authority proof installer did not return the exact proof ID';
    END IF;

    -- Exact receipt replay is idempotent.
    SELECT myc_effect_owner.install_authority_proof_receipt(
        v_proof_json,
        v_canonical_digest,
        v_signed_receipt_hash,
        v_external_sequence_digest,
        'test-proof-controller'
    ) INTO v_installed;
    IF v_installed <> v_proof_id THEN
        RAISE EXCEPTION 'exact authority proof replay changed proof identity';
    END IF;

    -- Proof row and external sequence mirror are persisted as one transaction.
    IF NOT EXISTS (
        SELECT 1
          FROM myc_effect_owner.authority_proof_receipts
         WHERE proof_id = v_proof_id
           AND purpose = 'lease_high_water'
           AND issuer_role = 'lease_high_water_authority'
           AND signed_receipt_hash = v_signed_receipt_hash
           AND external_sequence_receipt_digest = v_external_sequence_digest
    ) OR NOT EXISTS (
        SELECT 1
          FROM myc_effect_owner.authority_proof_sequence_highwater
         WHERE deployment_id = v_deployment
           AND purpose = 'lease_high_water'
           AND issuer_key_id = 'highwater-test-key'
           AND last_accepted_sequence = 1
           AND last_proof_id = v_proof_id
    ) THEN
        RAISE EXCEPTION 'authority proof receipt or monotonic sequence mirror is missing';
    END IF;

    SELECT myc_effect_owner.bind_authority_write_lease_proofs(
        v_realm, v_resource, v_action, v_owner,
        v_lease_id, v_proof_id, NULL, NULL
    ) INTO v_installed;
    IF v_installed <> v_lease_id THEN
        RAISE EXCEPTION 'effect lease proof binding did not return the lease ID';
    END IF;

    IF NOT EXISTS (
        SELECT 1
          FROM myc_effect_owner.effect_owner_authority_lease_proof_links
         WHERE realm_id = v_realm
           AND resource_id = v_resource
           AND action_id = v_action
           AND effect_owner_id = v_owner
           AND lease_id = v_lease_id
           AND lease_highwater_proof_id = v_proof_id
           AND physical_fencing_proof_id IS NULL
           AND promotion_eligibility_proof_id IS NULL
           AND lease_hash = v_lease_hash
           AND lease_generation = 7
    ) THEN
        RAISE EXCEPTION 'append-only lease-to-high-water proof link is missing';
    END IF;

    -- Same proof ID but changed external sequence evidence must not be treated
    -- as an idempotent replay.
    v_rejected := FALSE;
    BEGIN
        PERFORM myc_effect_owner.install_authority_proof_receipt(
            v_proof_json,
            v_canonical_digest,
            v_signed_receipt_hash,
            decode(repeat('ee', 32), 'hex'),
            'test-proof-controller'
        );
    EXCEPTION WHEN SQLSTATE 'P0001' THEN
        v_rejected := TRUE;
    END;
    IF NOT v_rejected THEN
        RAISE EXCEPTION 'same proof ID with changed external sequence receipt was accepted';
    END IF;

    -- A second proof cannot reuse an already consumed sequence.
    v_wrong_proof_json := jsonb_set(
        v_proof_json,
        '{receipt,proof_id}',
        to_jsonb(v_wrong_proof_id),
        FALSE
    );
    v_rejected := FALSE;
    BEGIN
        PERFORM myc_effect_owner.install_authority_proof_receipt(
            v_wrong_proof_json,
            decode(repeat('be', 32), 'hex'),
            decode(repeat('ce', 32), 'hex'),
            v_external_sequence_digest,
            'test-proof-controller'
        );
    EXCEPTION WHEN unique_violation OR SQLSTATE 'P0001' THEN
        v_rejected := TRUE;
    END;
    IF NOT v_rejected THEN
        RAISE EXCEPTION 'replayed authority proof sequence was accepted under a different proof ID';
    END IF;

    -- Purpose and issuer role are a strict pair.
    v_wrong_proof_json := jsonb_set(
        v_proof_json,
        '{receipt,issuer_role}',
        to_jsonb('physical_fencing_attestor'::TEXT),
        FALSE
    );
    v_rejected := FALSE;
    BEGIN
        PERFORM myc_effect_owner.install_authority_proof_receipt(
            v_wrong_proof_json,
            decode(repeat('bf', 32), 'hex'),
            decode(repeat('cf', 32), 'hex'),
            decode(repeat('df', 32), 'hex'),
            'test-proof-controller'
        );
    EXCEPTION WHEN SQLSTATE 'P0001' THEN
        v_rejected := TRUE;
    END;
    IF NOT v_rejected THEN
        RAISE EXCEPTION 'mismatched authority proof purpose/issuer role was accepted';
    END IF;

    -- A high-water receipt that points to a different lease hash cannot be
    -- linked even when its envelope and sequence are otherwise well-formed.
    v_wrong_proof_json := jsonb_set(
        v_proof_json,
        '{receipt,payload,new_lease_hash}',
        to_jsonb(array_fill(153, ARRAY[32])),
        FALSE
    );
    v_wrong_proof_json := jsonb_set(
        v_wrong_proof_json,
        '{receipt,proof_id}',
        to_jsonb('40000000-0000-4000-8000-000000000003'::UUID),
        FALSE
    );
    v_wrong_proof_json := jsonb_set(
        v_wrong_proof_json,
        '{receipt,sequence}',
        to_jsonb(2),
        FALSE
    );
    PERFORM myc_effect_owner.install_authority_proof_receipt(
        v_wrong_proof_json,
        decode(repeat('b1', 32), 'hex'),
        decode(repeat('c1', 32), 'hex'),
        v_external_sequence_digest,
        'test-proof-controller'
    );

    v_rejected := FALSE;
    BEGIN
        PERFORM myc_effect_owner.bind_authority_write_lease_proofs(
            v_realm, v_resource, v_action, v_owner,
            v_lease_id,
            '40000000-0000-4000-8000-000000000003'::UUID,
            NULL,
            NULL
        );
    EXCEPTION WHEN SQLSTATE 'P0001' THEN
        v_rejected := TRUE;
    END;
    IF NOT v_rejected THEN
        RAISE EXCEPTION 'lease was linked to a high-water proof for another signed lease hash';
    END IF;

    -- The worker can execute only the proof-linked effect endpoint; the
    -- controller can execute only the proof-linked promotion endpoint.
    IF has_function_privilege(
        'myc_dir_effect_worker',
        'myc_effect_owner.apply_claim_fenced_balance_delta_with_lease(text,text,text,text,text,text,bigint,bigint,bytea,text,bytea,bytea,bytea,bigint,text,bigint,bytea,text,timestamp with time zone,timestamp with time zone,uuid,bytea,bigint)'::REGPROCEDURE,
        'EXECUTE'
    ) THEN
        RAISE EXCEPTION 'effect worker can bypass proof-linked lease mutation path';
    END IF;
    IF NOT has_function_privilege(
        'myc_dir_effect_worker',
        'myc_effect_owner.apply_claim_fenced_balance_delta_with_proofs(text,text,text,text,text,text,bigint,bigint,bytea,text,bytea,bytea,bytea,bigint,text,bigint,bytea,text,timestamp with time zone,timestamp with time zone,uuid,bytea,bigint)'::REGPROCEDURE,
        'EXECUTE'
    ) THEN
        RAISE EXCEPTION 'effect worker lacks proof-linked mutation entry point';
    END IF;
    IF has_function_privilege(
        'myc_dir_authority_lease_controller',
        'myc_effect_owner.advance_effect_owner_epoch_with_authority_lease(text,text,text,text,bigint,bytea,bytea,uuid,bytea,bigint)'::REGPROCEDURE,
        'EXECUTE'
    ) THEN
        RAISE EXCEPTION 'lease controller can bypass proof-linked promotion entry point';
    END IF;
    IF NOT has_function_privilege(
        'myc_dir_authority_lease_controller',
        'myc_effect_owner.advance_owner_epoch_with_proofs(text,text,text,text,bigint,bytea,bytea,uuid,bytea,bigint)'::REGPROCEDURE,
        'EXECUTE'
    ) THEN
        RAISE EXCEPTION 'lease controller lacks proof-linked promotion entry point';
    END IF;
END
$test$;

ROLLBACK;
