\set ON_ERROR_STOP on

-- Disposable PostgreSQL test for 004_authority_write_lease_admission.sql.
-- Signed JSON fixtures are intentionally synthetic: this SQL test validates
-- the database-side trust boundary/atomicity. Ed25519 trust-root validation is
-- tested separately by mycelix-authority-write-lease.

BEGIN;
SET LOCAL synchronous_commit = on;

DO $test$
DECLARE
    v_realm TEXT := 'authority-lease-test-realm';
    v_resource TEXT := 'authority-lease-test-resource';
    v_action TEXT := 'balance-adjust';
    v_owner TEXT := 'authority-lease-owner';
    v_object TEXT := 'account-lease-test';
    v_currency TEXT := 'USD';
    v_operation_a TEXT := 'authority-lease-operation-a';
    v_operation_b TEXT := 'authority-lease-operation-b';
    v_worker_a TEXT := 'authority-worker-a';
    v_worker_b TEXT := 'authority-worker-b';
    v_claim_a BYTEA := decode(repeat('11', 32), 'hex');
    v_claim_b BYTEA := decode(repeat('22', 32), 'hex');
    v_idem_a BYTEA := decode(repeat('33', 32), 'hex');
    v_idem_b BYTEA := decode(repeat('44', 32), 'hex');
    v_request_a BYTEA := decode(repeat('55', 32), 'hex');
    v_request_b BYTEA := decode(repeat('66', 32), 'hex');
    v_decision BYTEA := decode(repeat('77', 32), 'hex');
    v_payload_a BYTEA := decode(repeat('88', 32), 'hex');
    v_payload_b BYTEA := decode(repeat('99', 32), 'hex');
    v_lease_hash_a BYTEA := decode(repeat('aa', 32), 'hex');
    v_lease_hash_b BYTEA := decode(repeat('bb', 32), 'hex');
    v_lease_hash_c BYTEA := decode(repeat('cc', 32), 'hex');
    v_highwater_a BYTEA := decode(repeat('dd', 32), 'hex');
    v_highwater_b BYTEA := decode(repeat('ee', 32), 'hex');
    v_highwater_c BYTEA := decode(repeat('ab', 32), 'hex');
    v_physical_fence BYTEA := decode(repeat('bc', 32), 'hex');
    v_lease_a UUID := '10000000-0000-4000-8000-000000000001';
    v_lease_b UUID := '10000000-0000-4000-8000-000000000002';
    v_lease_c UUID := '10000000-0000-4000-8000-000000000003';
    v_system_id TEXT;
    v_timeline BIGINT;
    v_now TIMESTAMPTZ;
    v_signed_a JSONB;
    v_signed_b JSONB;
    v_signed_c JSONB;
    v_installed UUID;
    v_result RECORD;
    v_replay RECORD;
    v_advanced RECORD;
    v_attestation myc_effect_owner.effect_execution_attestations%ROWTYPE;
    v_balance BIGINT;
    v_effect_count BIGINT;
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

    INSERT INTO myc_effect_owner.protected_balance (
        realm_id, resource_id, action_id, effect_owner_id,
        object_id, currency_code, balance_minor
    )
    VALUES (
        v_realm, v_resource, v_action, v_owner,
        v_object, v_currency, 1000
    );

    SELECT system.system_identifier::TEXT, checkpoint.timeline_id::BIGINT
      INTO v_system_id, v_timeline
      FROM pg_catalog.pg_control_system() AS system
      CROSS JOIN pg_catalog.pg_control_checkpoint() AS checkpoint;

    v_now := clock_timestamp();
    v_signed_a := jsonb_build_object(
        'protocol', 'mycelix-desci-authority-write-lease',
        'protocol_version', 1,
        'codec', 'mycelix-canonical-binary-v1',
        'schema_version', 1,
        'lease', jsonb_build_object(
            'lease_id', v_lease_a,
            'deployment_id', 'authority-lease-test-deployment',
            'primary_id', 'postgres-test-primary',
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
            'reason', 'synthetic SQL integration fixture'
        ),
        'signer_key_id', 'test-ed25519-key',
        'signer_public_key', to_jsonb(array_fill(84, ARRAY[32])),
        'signature', to_jsonb(array_fill(85, ARRAY[64]))
    );

    SELECT myc_effect_owner.install_effect_owner_authority_write_lease(
        v_realm, v_resource, v_action, v_owner,
        1, 1, v_lease_hash_a, v_highwater_a, NULL, v_signed_a
    ) INTO v_installed;
    IF v_installed <> v_lease_a THEN
        RAISE EXCEPTION 'lease installer did not return the exact accepted lease ID';
    END IF;

    -- An exact install replay is idempotent.
    SELECT myc_effect_owner.install_effect_owner_authority_write_lease(
        v_realm, v_resource, v_action, v_owner,
        1, 1, v_lease_hash_a, v_highwater_a, NULL, v_signed_a
    ) INTO v_installed;
    IF v_installed <> v_lease_a THEN
        RAISE EXCEPTION 'exact signed lease install replay changed lease identity';
    END IF;

    PERFORM myc_effect_owner.install_worker_claim_for_owner_epoch(
        v_realm, v_resource, v_action, v_owner, 1,
        v_operation_a, v_worker_a, 1, v_claim_a,
        'postgres-db-clock-v1',
        clock_timestamp() - interval '1 minute',
        clock_timestamp() + interval '1 hour'
    );

    SELECT * INTO v_result
      FROM myc_effect_owner.apply_claim_fenced_balance_delta_with_lease(
        v_realm, v_resource, v_action, v_owner, v_object, v_currency,
        1, 1, v_idem_a, v_operation_a,
        v_request_a, v_decision, v_payload_a, 100,
        v_worker_a, 1, v_claim_a,
        'postgres-db-clock-v1',
        clock_timestamp() - interval '1 minute',
        clock_timestamp() + interval '1 hour',
        v_lease_a, v_lease_hash_a, 7
    );

    IF v_result.replayed
       OR v_result.accepted_owner_epoch <> 1
       OR v_result.accepted_generation <> 1
       OR v_result.authority_lease_id <> v_lease_a
       OR v_result.authority_lease_hash <> v_lease_hash_a
       OR v_result.authority_lease_generation <> 7 THEN
        RAISE EXCEPTION 'lease-fenced first effect did not return exact accepted lease coordinates';
    END IF;

    IF NOT EXISTS (
        SELECT 1
          FROM myc_effect_owner.effect_execution_authority_lease_attestations
         WHERE effect_id = v_result.effect_id
           AND attestation_id = v_result.execution_attestation_id
           AND lease_id = v_lease_a
           AND lease_hash = v_lease_hash_a
           AND lease_generation = 7
           AND admitted_scope = 'directory_fabric_effect'
    ) THEN
        RAISE EXCEPTION 'new effect lacks append-only authority lease attestation';
    END IF;

    -- Acknowledgement/replay resolution returns the original lease evidence.
    SELECT * INTO v_replay
      FROM myc_effect_owner.apply_claim_fenced_balance_delta_with_lease(
        v_realm, v_resource, v_action, v_owner, v_object, v_currency,
        1, 1, v_idem_a, v_operation_a,
        v_request_a, v_decision, v_payload_a, 100,
        v_worker_a, 1, v_claim_a,
        'postgres-db-clock-v1',
        clock_timestamp() - interval '1 minute',
        clock_timestamp() + interval '1 hour',
        v_lease_a, v_lease_hash_a, 7
    );
    IF NOT v_replay.replayed
       OR v_replay.effect_id <> v_result.effect_id
       OR v_replay.authority_lease_id <> v_lease_a
       OR v_replay.authority_lease_hash <> v_lease_hash_a
       OR v_replay.authority_lease_generation <> 7 THEN
        RAISE EXCEPTION 'exact effect replay did not preserve original authority lease evidence';
    END IF;

    SELECT balance_minor INTO v_balance
      FROM myc_effect_owner.protected_balance
     WHERE realm_id = v_realm
       AND resource_id = v_resource
       AND action_id = v_action
       AND effect_owner_id = v_owner
       AND object_id = v_object
       AND currency_code = v_currency;
    SELECT COUNT(*) INTO v_effect_count
      FROM myc_effect_owner.effect_commits
     WHERE realm_id = v_realm AND resource_id = v_resource
       AND action_id = v_action AND effect_owner_id = v_owner;
    IF v_balance <> 1100 OR v_effect_count <> 1 THEN
        RAISE EXCEPTION 'authority lease replay duplicated or changed the protected effect';
    END IF;

    -- Wrong lease ID/hash/generation cannot reach the effect owner.
    v_rejected := FALSE;
    BEGIN
        PERFORM *
          FROM myc_effect_owner.apply_claim_fenced_balance_delta_with_lease(
            v_realm, v_resource, v_action, v_owner, v_object, v_currency,
            1, 1, v_idem_a, v_operation_a,
            v_request_a, v_decision, v_payload_a, 100,
            v_worker_a, 1, v_claim_a,
            'postgres-db-clock-v1',
            clock_timestamp() - interval '1 minute',
            clock_timestamp() + interval '1 hour',
            v_lease_a, v_lease_hash_b, 7
        );
    EXCEPTION WHEN SQLSTATE 'P0001' THEN
        v_rejected := TRUE;
    END;
    IF NOT v_rejected THEN
        RAISE EXCEPTION 'mismatched authority lease hash unexpectedly permitted command';
    END IF;

    -- Same generation with different signed bytes is a deployment-level fork.
    v_signed_b := jsonb_set(
        v_signed_a,
        '{lease,lease_id}',
        to_jsonb(v_lease_b),
        FALSE
    );
    v_rejected := FALSE;
    BEGIN
        PERFORM myc_effect_owner.install_effect_owner_authority_write_lease(
            v_realm, v_resource, v_action, v_owner,
            1, 1, v_lease_hash_b, v_highwater_a, NULL, v_signed_b
        );
    EXCEPTION WHEN SQLSTATE 'P0001' THEN
        v_rejected := TRUE;
    END;
    IF NOT v_rejected THEN
        RAISE EXCEPTION 'same-generation different-lease fork unexpectedly installed';
    END IF;

    -- Expired lease is rejected before high-water or per-resource state changes.
    v_signed_b := jsonb_set(
        v_signed_a,
        '{lease,lease_id}',
        to_jsonb(v_lease_b),
        FALSE
    );
    v_signed_b := jsonb_set(
        v_signed_b,
        '{lease,generation}',
        to_jsonb(8),
        FALSE
    );
    v_signed_b := jsonb_set(
        v_signed_b,
        '{lease,issued_at}',
        to_jsonb(v_now - interval '10 minutes'),
        FALSE
    );
    v_signed_b := jsonb_set(
        v_signed_b,
        '{lease,not_before}',
        to_jsonb(v_now - interval '9 minutes'),
        FALSE
    );
    v_signed_b := jsonb_set(
        v_signed_b,
        '{lease,expires_at}',
        to_jsonb(v_now - interval '1 minute'),
        FALSE
    );
    v_rejected := FALSE;
    BEGIN
        PERFORM myc_effect_owner.install_effect_owner_authority_write_lease(
            v_realm, v_resource, v_action, v_owner,
            1, 1, v_lease_hash_b, v_highwater_b, NULL, v_signed_b
        );
    EXCEPTION WHEN SQLSTATE 'P0001' THEN
        v_rejected := TRUE;
    END;
    IF NOT v_rejected THEN
        RAISE EXCEPTION 'expired authority lease unexpectedly installed';
    END IF;

    -- A primary/timeline switch cannot be admitted at the same resource epoch.
    v_signed_b := jsonb_set(
        v_signed_a,
        '{lease,lease_id}',
        to_jsonb(v_lease_b),
        FALSE
    );
    v_signed_b := jsonb_set(
        v_signed_b,
        '{lease,generation}',
        to_jsonb(8),
        FALSE
    );
    v_signed_b := jsonb_set(
        v_signed_b,
        '{lease,primary_id}',
        to_jsonb('postgres-other-primary'::TEXT),
        FALSE
    );
    v_rejected := FALSE;
    BEGIN
        PERFORM myc_effect_owner.install_effect_owner_authority_write_lease(
            v_realm, v_resource, v_action, v_owner,
            1, 1, v_lease_hash_b, v_highwater_b, NULL, v_signed_b
        );
    EXCEPTION WHEN SQLSTATE 'P0001' THEN
        v_rejected := TRUE;
    END;
    IF NOT v_rejected THEN
        RAISE EXCEPTION 'same-epoch primary identity change unexpectedly installed';
    END IF;

    -- A future owner position must be installed under DatabaseEpochPromotion
    -- and a nonzero physical-fencing receipt.
    v_now := clock_timestamp();
    v_signed_c := jsonb_build_object(
        'protocol', 'mycelix-desci-authority-write-lease',
        'protocol_version', 1,
        'codec', 'mycelix-canonical-binary-v1',
        'schema_version', 1,
        'lease', jsonb_build_object(
            'lease_id', v_lease_c,
            'deployment_id', 'authority-lease-test-deployment',
            'primary_id', 'postgres-test-primary',
            'database_system_identifier', v_system_id,
            'postgres_timeline', v_timeline,
            'generation', 8,
            'phase', 'epoch',
            'epoch_number', 2,
            'epoch_hash', to_jsonb(array_fill(3, ARRAY[32])),
            'allowed_scopes', jsonb_build_array('database_epoch_promotion'),
            'issued_at', v_now - interval '1 minute',
            'not_before', v_now - interval '30 seconds',
            'expires_at', v_now + interval '5 minutes',
            'reason', 'synthetic SQL promotion fixture'
        ),
        'signer_key_id', 'test-ed25519-key',
        'signer_public_key', to_jsonb(array_fill(84, ARRAY[32])),
        'signature', to_jsonb(array_fill(86, ARRAY[64]))
    );

    SELECT myc_effect_owner.install_effect_owner_authority_write_lease(
        v_realm, v_resource, v_action, v_owner,
        2, 2, v_lease_hash_c, v_highwater_c, v_physical_fence, v_signed_c
    ) INTO v_installed;
    IF v_installed <> v_lease_c THEN
        RAISE EXCEPTION 'promotion lease did not install at next owner epoch/generation';
    END IF;

    SELECT * INTO v_advanced
      FROM myc_effect_owner.advance_effect_owner_epoch_with_authority_lease(
        v_realm, v_resource, v_action, v_owner,
        2, v_highwater_c, v_physical_fence,
        v_lease_c, v_lease_hash_c, 8
    );
    IF v_advanced.accepted_owner_epoch <> 2
       OR v_advanced.accepted_generation <> 2
       OR v_advanced.authority_lease_id <> v_lease_c
       OR v_advanced.authority_lease_generation <> 8 THEN
        RAISE EXCEPTION 'owner epoch promotion did not preserve the signed authority lease';
    END IF;

    IF NOT EXISTS (
        SELECT 1
          FROM myc_effect_owner.effect_owner_epoch_authority_lease_attestations
         WHERE realm_id = v_realm
           AND resource_id = v_resource
           AND action_id = v_action
           AND effect_owner_id = v_owner
           AND new_owner_epoch = 2
           AND lease_id = v_lease_c
           AND lease_hash = v_lease_hash_c
           AND physical_fencing_receipt_digest = v_physical_fence
    ) THEN
        RAISE EXCEPTION 'owner epoch transition lacks append-only promotion lease attestation';
    END IF;

    -- An effect lease for the new resource position must separately authorize
    -- DirectoryFabricEffect; a promotion-only lease cannot authorize mutations.
    v_signed_b := jsonb_set(v_signed_c, '{lease,lease_id}', to_jsonb(v_lease_b), FALSE);
    v_signed_b := jsonb_set(v_signed_b, '{lease,generation}', to_jsonb(9), FALSE);
    v_signed_b := jsonb_set(
        v_signed_b,
        '{lease,allowed_scopes}',
        jsonb_build_array('directory_fabric_effect'),
        FALSE
    );
    v_signed_b := jsonb_set(v_signed_b, '{lease,reason}', to_jsonb('synthetic post-promotion effect fixture'::TEXT), FALSE);
    SELECT myc_effect_owner.install_effect_owner_authority_write_lease(
        v_realm, v_resource, v_action, v_owner,
        2, 2, v_lease_hash_b, v_highwater_b, NULL, v_signed_b
    ) INTO v_installed;
    IF v_installed <> v_lease_b THEN
        RAISE EXCEPTION 'post-promotion effect lease did not install at current resource position';
    END IF;

    SELECT myc_effect_owner.install_worker_claim_for_owner_epoch(
        v_realm, v_resource, v_action, v_owner, 2,
        v_operation_b, v_worker_b, 1, v_claim_b,
        'postgres-db-clock-v1',
        clock_timestamp() - interval '1 minute',
        clock_timestamp() + interval '1 hour'
    );

    SELECT * INTO v_result
      FROM myc_effect_owner.apply_claim_fenced_balance_delta_with_lease(
        v_realm, v_resource, v_action, v_owner, v_object, v_currency,
        2, 2, v_idem_b, v_operation_b,
        v_request_b, v_decision, v_payload_b, 25,
        v_worker_b, 1, v_claim_b,
        'postgres-db-clock-v1',
        clock_timestamp() - interval '1 minute',
        clock_timestamp() + interval '1 hour',
        v_lease_b, v_lease_hash_b, 9
    );
    IF v_result.replayed
       OR v_result.accepted_owner_epoch <> 2
       OR v_result.accepted_generation <> 2
       OR v_result.authority_lease_id <> v_lease_b
       OR v_result.authority_lease_generation <> 9 THEN
        RAISE EXCEPTION 'post-promotion effect failed to bind to the DirectoryFabricEffect lease';
    END IF;

    -- A promotion-only lease cannot authorize an effect mutation.
    v_rejected := FALSE;
    BEGIN
        PERFORM *
          FROM myc_effect_owner.apply_claim_fenced_balance_delta_with_lease(
            v_realm, v_resource, v_action, v_owner, v_object, v_currency,
            2, 2, v_idem_b, v_operation_b,
            v_request_b, v_decision, v_payload_b, 25,
            v_worker_b, 1, v_claim_b,
            'postgres-db-clock-v1',
            clock_timestamp() - interval '1 minute',
            clock_timestamp() + interval '1 hour',
            v_lease_c, v_lease_hash_c, 8
        );
    EXCEPTION WHEN SQLSTATE 'P0001' THEN
        v_rejected := TRUE;
    END;
    IF NOT v_rejected THEN
        RAISE EXCEPTION 'promotion-only lease unexpectedly authorized effect mutation';
    END IF;

    -- Application worker has no direct path around the lease-checked wrapper.
    IF has_function_privilege(
        'myc_dir_effect_worker',
        'myc_effect_owner.apply_owner_epoch_claim_fenced_balance_delta(text,text,text,text,text,text,bigint,bigint,bytea,text,bytea,bytea,bytea,bigint,text,bigint,bytea,text,timestamp with time zone,timestamp with time zone)'::REGPROCEDURE,
        'EXECUTE'
    ) THEN
        RAISE EXCEPTION 'effect worker role can execute the legacy non-lease mutation entry point';
    END IF;
    IF NOT has_function_privilege(
        'myc_dir_effect_worker',
        'myc_effect_owner.apply_claim_fenced_balance_delta_with_lease(text,text,text,text,text,text,bigint,bigint,bytea,text,bytea,bytea,bytea,bigint,text,bigint,bytea,text,timestamp with time zone,timestamp with time zone,uuid,bytea,bigint)'::REGPROCEDURE,
        'EXECUTE'
    ) THEN
        RAISE EXCEPTION 'effect worker role lacks the lease-checked mutation entry point';
    END IF;
    IF has_table_privilege(
        'myc_dir_effect_worker',
        'myc_effect_owner.effect_commits',
        'INSERT'
    ) OR has_table_privilege(
        'myc_dir_effect_worker',
        'myc_effect_owner.effect_execution_authority_lease_attestations',
        'INSERT'
    ) THEN
        RAISE EXCEPTION 'effect worker role can bypass the SECURITY DEFINER effect path with direct INSERT';
    END IF;
END
$test$;

ROLLBACK;
