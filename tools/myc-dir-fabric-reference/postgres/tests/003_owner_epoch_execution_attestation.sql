\set ON_ERROR_STOP on

-- Disposable PostgreSQL integration test for 003_owner_epoch_execution_attestation.sql.
-- The outer transaction is rolled back so the script is repeatable.

BEGIN;
SET LOCAL synchronous_commit = on;

DO $test$
DECLARE
    v_realm TEXT := 'owner-epoch-test-realm';
    v_resource TEXT := 'owner-epoch-test-resource';
    v_action TEXT := 'balance-adjust';
    v_owner TEXT := 'postgres-owner-test';
    v_object TEXT := 'account-test';
    v_currency TEXT := 'USD';
    v_operation_a TEXT := 'owner-epoch-test-operation-a';
    v_operation_b TEXT := 'owner-epoch-test-operation-b';
    v_worker_a TEXT := 'worker-a';
    v_worker_b TEXT := 'worker-b';
    v_worker_c TEXT := 'worker-c';
    v_claim_a BYTEA := decode(repeat('11', 32), 'hex');
    v_claim_b BYTEA := decode(repeat('22', 32), 'hex');
    v_claim_c BYTEA := decode(repeat('33', 32), 'hex');
    v_idem_a BYTEA := decode(repeat('44', 32), 'hex');
    v_idem_b BYTEA := decode(repeat('55', 32), 'hex');
    v_request_a BYTEA := decode(repeat('66', 32), 'hex');
    v_request_b BYTEA := decode(repeat('77', 32), 'hex');
    v_decision BYTEA := decode(repeat('88', 32), 'hex');
    v_payload_a BYTEA := decode(repeat('99', 32), 'hex');
    v_payload_b BYTEA := decode(repeat('aa', 32), 'hex');
    v_epoch_authority BYTEA := decode(repeat('ab', 32), 'hex');
    v_physical_fence BYTEA := decode(repeat('bc', 32), 'hex');
    v_result RECORD;
    v_replay RECORD;
    v_advanced RECORD;
    v_attestation myc_effect_owner.effect_execution_attestations%ROWTYPE;
    v_current_claim_epoch BIGINT;
    v_current_claim_owner_epoch BIGINT;
    v_balance_after BIGINT;
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

    SELECT myc_effect_owner.install_worker_claim_for_owner_epoch(
        v_realm, v_resource, v_action, v_owner, 1,
        v_operation_a, v_worker_a, 1, v_claim_a,
        'postgres-db-clock-v1',
        clock_timestamp() - interval '1 minute',
        clock_timestamp() + interval '1 hour'
    ) INTO v_current_claim_epoch;
    IF v_current_claim_epoch <> 1 THEN
        RAISE EXCEPTION 'initial worker claim epoch should be 1';
    END IF;

    SELECT * INTO v_result
      FROM myc_effect_owner.apply_owner_epoch_claim_fenced_balance_delta(
        v_realm, v_resource, v_action, v_owner, v_object, v_currency,
        1, 1, v_idem_a, v_operation_a,
        v_request_a, v_decision, v_payload_a, 100,
        v_worker_a, 1, v_claim_a,
        'postgres-db-clock-v1',
        clock_timestamp() - interval '1 minute',
        clock_timestamp() + interval '1 hour'
    );

    IF v_result.replayed
       OR v_result.accepted_owner_epoch <> 1
       OR v_result.accepted_generation <> 1
       OR v_result.accepted_worker_id <> v_worker_a
       OR v_result.accepted_claim_epoch <> 1
       OR v_result.execution_attestation_id IS NULL THEN
        RAISE EXCEPTION 'first effect did not return the exact execution attestation';
    END IF;

    SELECT * INTO v_attestation
      FROM myc_effect_owner.effect_execution_attestations
     WHERE effect_id = v_result.effect_id;

    IF NOT FOUND
       OR v_attestation.owner_epoch <> 1
       OR v_attestation.accepted_generation <> 1
       OR v_attestation.worker_id <> v_worker_a
       OR v_attestation.claim_epoch <> 1
       OR v_attestation.claim_proof_digest <> v_claim_a
       OR v_attestation.database_xact_id = '' THEN
        RAISE EXCEPTION 'execution attestation row does not preserve accepted worker/owner coordinates';
    END IF;

    -- A stale owner epoch must not execute a new logical operation.
    PERFORM myc_effect_owner.install_worker_claim_for_owner_epoch(
        v_realm, v_resource, v_action, v_owner, 1,
        v_operation_b, v_worker_b, 1, v_claim_b,
        'postgres-db-clock-v1',
        clock_timestamp() - interval '1 minute',
        clock_timestamp() + interval '1 hour'
    );

    v_rejected := FALSE;
    BEGIN
        PERFORM *
          FROM myc_effect_owner.apply_owner_epoch_claim_fenced_balance_delta(
            v_realm, v_resource, v_action, v_owner, v_object, v_currency,
            2, 1, v_idem_b, v_operation_b,
            v_request_b, v_decision, v_payload_b, 50,
            v_worker_b, 1, v_claim_b,
            'postgres-db-clock-v1',
            clock_timestamp() - interval '1 minute',
            clock_timestamp() + interval '1 hour'
        );
    EXCEPTION WHEN SQLSTATE 'P0001' THEN
        v_rejected := TRUE;
    END;
    IF NOT v_rejected THEN
        RAISE EXCEPTION 'new effect with a stale owner epoch unexpectedly succeeded';
    END IF;

    -- The trusted epoch authority / physical fencing service is represented by
    -- nonzero receipt commitments in this SQL-only test. Signature validation
    -- and actual old-primary fencing are external qualification requirements.
    SELECT * INTO v_advanced
      FROM myc_effect_owner.advance_effect_owner_epoch(
        v_realm, v_resource, v_action, v_owner,
        2, v_epoch_authority, v_physical_fence
    );

    IF v_advanced.accepted_owner_epoch <> 2
       OR v_advanced.accepted_generation <> 2 THEN
        RAISE EXCEPTION 'owner epoch advance must also advance resource generation';
    END IF;

    -- A new owner epoch must not reuse the previous worker claim epoch.
    v_rejected := FALSE;
    BEGIN
        PERFORM myc_effect_owner.install_worker_claim_for_owner_epoch(
            v_realm, v_resource, v_action, v_owner, 2,
            v_operation_a, v_worker_b, 1, v_claim_b,
            'postgres-db-clock-v1',
            clock_timestamp() - interval '1 minute',
            clock_timestamp() + interval '1 hour'
        );
    EXCEPTION WHEN SQLSTATE 'P0001' THEN
        v_rejected := TRUE;
    END;
    IF NOT v_rejected THEN
        RAISE EXCEPTION 'worker claim epoch did not strictly advance across owner epoch transition';
    END IF;

    -- Claim reassignment is strictly increasing and the old worker must fail.
    SELECT myc_effect_owner.install_worker_claim_for_owner_epoch(
        v_realm, v_resource, v_action, v_owner, 2,
        v_operation_a, v_worker_b, 2, v_claim_b,
        'postgres-db-clock-v1',
        clock_timestamp() - interval '1 minute',
        clock_timestamp() + interval '1 hour'
    ) INTO v_current_claim_epoch;
    IF v_current_claim_epoch <> 2 THEN
        RAISE EXCEPTION 'worker claim epoch should advance to 2';
    END IF;

    v_rejected := FALSE;
    BEGIN
        PERFORM *
          FROM myc_effect_owner.apply_owner_epoch_claim_fenced_balance_delta(
            v_realm, v_resource, v_action, v_owner, v_object, v_currency,
            1, 1, v_idem_b, v_operation_b,
            v_request_b, v_decision, v_payload_b, 50,
            v_worker_b, 1, v_claim_b,
            'postgres-db-clock-v1',
            clock_timestamp() - interval '1 minute',
            clock_timestamp() + interval '1 hour'
        );
    EXCEPTION WHEN SQLSTATE 'P0001' THEN
        v_rejected := TRUE;
    END;
    IF NOT v_rejected THEN
        RAISE EXCEPTION 'old owner epoch / resource generation unexpectedly succeeded';
    END IF;

    -- Exact replay after an owner-epoch transition is read-only and must return
    -- the original effect's execution coordinates, not rewrite the attestation.
    SELECT * INTO v_replay
      FROM myc_effect_owner.apply_owner_epoch_claim_fenced_balance_delta(
        v_realm, v_resource, v_action, v_owner, v_object, v_currency,
        2, 2, v_idem_a, v_operation_a,
        v_request_a, v_decision, v_payload_a, 100,
        v_worker_b, 2, v_claim_b,
        'postgres-db-clock-v1',
        clock_timestamp() - interval '1 minute',
        clock_timestamp() + interval '1 hour'
    );

    IF NOT v_replay.replayed
       OR v_replay.effect_id <> v_result.effect_id
       OR v_replay.accepted_owner_epoch <> 1
       OR v_replay.accepted_generation <> 1
       OR v_replay.accepted_worker_id <> v_worker_a
       OR v_replay.accepted_claim_epoch <> 1
       OR v_replay.execution_attestation_id <> v_result.execution_attestation_id THEN
        RAISE EXCEPTION 'exact replay did not preserve the original execution attestation';
    END IF;

    SELECT balance_minor
      INTO v_balance_after
      FROM myc_effect_owner.protected_balance
     WHERE realm_id = v_realm
       AND resource_id = v_resource
       AND action_id = v_action
       AND effect_owner_id = v_owner
       AND object_id = v_object
       AND currency_code = v_currency;

    SELECT COUNT(*)
      INTO v_effect_count
      FROM myc_effect_owner.effect_commits
     WHERE realm_id = v_realm
       AND resource_id = v_resource
       AND action_id = v_action
       AND effect_owner_id = v_owner
       AND object_id = v_object
       AND currency_code = v_currency
       AND idempotency_key_digest = v_idem_a
       AND logical_operation_id = v_operation_a;

    IF v_balance_after <> 1100 OR v_effect_count <> 1 THEN
        RAISE EXCEPTION 'exact replay changed protected balance or created another effect';
    END IF;

    -- Even when the command presents the current owner epoch and generation,
    -- an old worker claim from epoch 1 is invalid until a fresh epoch-2 claim is
    -- explicitly installed.
    v_rejected := FALSE;
    BEGIN
        PERFORM *
          FROM myc_effect_owner.apply_owner_epoch_claim_fenced_balance_delta(
            v_realm, v_resource, v_action, v_owner, v_object, v_currency,
            2, 2, v_idem_b, v_operation_b,
            v_request_b, v_decision, v_payload_b, 50,
            v_worker_b, 1, v_claim_b,
            'postgres-db-clock-v1',
            clock_timestamp() - interval '1 minute',
            clock_timestamp() + interval '1 hour'
        );
    EXCEPTION WHEN SQLSTATE 'P0001' THEN
        v_rejected := TRUE;
    END;
    IF NOT v_rejected THEN
        RAISE EXCEPTION 'worker claim from the previous owner epoch unexpectedly executed';
    END IF;

    -- A new effect under epoch 2 must produce a fresh attestation with the new
    -- owner epoch, current generation and the currently installed worker claim.
    SELECT myc_effect_owner.install_worker_claim_for_owner_epoch(
        v_realm, v_resource, v_action, v_owner, 2,
        v_operation_b, v_worker_b, 2, v_claim_b,
        'postgres-db-clock-v1',
        clock_timestamp() - interval '1 minute',
        clock_timestamp() + interval '1 hour'
    ) INTO v_current_claim_epoch;

    SELECT owner_epoch, claim_epoch
      INTO v_current_claim_owner_epoch, v_current_claim_epoch
      FROM myc_effect_owner.worker_claim_state
     WHERE realm_id = v_realm
       AND resource_id = v_resource
       AND action_id = v_action
       AND effect_owner_id = v_owner
       AND logical_operation_id = v_operation_b;
    IF v_current_claim_owner_epoch <> 2 OR v_current_claim_epoch <> 2 THEN
        RAISE EXCEPTION 'worker claim row does not preserve the current owner epoch / renewed claim epoch';
    END IF;

    SELECT * INTO v_result
      FROM myc_effect_owner.apply_owner_epoch_claim_fenced_balance_delta(
        v_realm, v_resource, v_action, v_owner, v_object, v_currency,
        2, 2, v_idem_b, v_operation_b,
        v_request_b, v_decision, v_payload_b, 50,
        v_worker_b, 2, v_claim_b,
        'postgres-db-clock-v1',
        clock_timestamp() - interval '1 minute',
        clock_timestamp() + interval '1 hour'
    );

    IF v_result.replayed
       OR v_result.accepted_owner_epoch <> 2
       OR v_result.accepted_generation <> 2
       OR v_result.accepted_worker_id <> v_worker_b
       OR v_result.accepted_claim_epoch <> 1 THEN
        RAISE EXCEPTION 'new effect after epoch advance did not record the current coordinates';
    END IF;

    -- A later claim epoch makes the prior worker claim stale.
    PERFORM myc_effect_owner.install_worker_claim_for_owner_epoch(
        v_realm, v_resource, v_action, v_owner, 2,
        v_operation_b, v_worker_c, 3, v_claim_c,
        'postgres-db-clock-v1',
        clock_timestamp() - interval '1 minute',
        clock_timestamp() + interval '1 hour'
    );

    v_rejected := FALSE;
    BEGIN
        PERFORM *
          FROM myc_effect_owner.apply_owner_epoch_claim_fenced_balance_delta(
            v_realm, v_resource, v_action, v_owner, v_object, v_currency,
            2, 2, v_idem_b, v_operation_b,
            v_request_b, v_decision, v_payload_b, 50,
            v_worker_b, 2, v_claim_b,
            'postgres-db-clock-v1',
            clock_timestamp() - interval '1 minute',
            clock_timestamp() + interval '1 hour'
        );
    EXCEPTION WHEN SQLSTATE 'P0001' THEN
        v_rejected := TRUE;
    END;
    IF NOT v_rejected THEN
        RAISE EXCEPTION 'stale worker claim unexpectedly succeeded';
    END IF;

    -- Owner epoch may not move backwards or repeat the current value.
    v_rejected := FALSE;
    BEGIN
        PERFORM * FROM myc_effect_owner.advance_effect_owner_epoch(
            v_realm, v_resource, v_action, v_owner,
            2, v_epoch_authority, v_physical_fence
        );
    EXCEPTION WHEN SQLSTATE 'P0001' THEN
        v_rejected := TRUE;
    END;
    IF NOT v_rejected THEN
        RAISE EXCEPTION 'non-advancing owner epoch unexpectedly succeeded';
    END IF;

    -- Both historical evidence tables must be append-only.
    v_rejected := FALSE;
    BEGIN
        UPDATE myc_effect_owner.effect_execution_attestations
           SET worker_id = 'tampered'
         WHERE effect_id = v_result.effect_id;
    EXCEPTION WHEN SQLSTATE '55000' THEN
        v_rejected := TRUE;
    END;
    IF NOT v_rejected THEN
        RAISE EXCEPTION 'execution attestation update unexpectedly succeeded';
    END IF;

    v_rejected := FALSE;
    BEGIN
        DELETE FROM myc_effect_owner.effect_owner_epoch_transitions
         WHERE realm_id = v_realm
           AND resource_id = v_resource
           AND action_id = v_action
           AND effect_owner_id = v_owner
           AND new_owner_epoch = 2;
    EXCEPTION WHEN SQLSTATE '55000' THEN
        v_rejected := TRUE;
    END;
    IF NOT v_rejected THEN
        RAISE EXCEPTION 'owner epoch transition deletion unexpectedly succeeded';
    END IF;

    v_rejected := FALSE;
    BEGIN
        EXECUTE 'TRUNCATE TABLE myc_effect_owner.effect_execution_attestations';
    EXCEPTION WHEN SQLSTATE '55000' THEN
        v_rejected := TRUE;
    END;
    IF NOT v_rejected THEN
        RAISE EXCEPTION 'execution attestation TRUNCATE unexpectedly succeeded';
    END IF;

    v_rejected := FALSE;
    BEGIN
        EXECUTE 'TRUNCATE TABLE myc_effect_owner.effect_owner_epoch_transitions';
    EXCEPTION WHEN SQLSTATE '55000' THEN
        v_rejected := TRUE;
    END;
    IF NOT v_rejected THEN
        RAISE EXCEPTION 'owner epoch transition TRUNCATE unexpectedly succeeded';
    END IF;

    v_rejected := FALSE;
    BEGIN
        EXECUTE 'TRUNCATE TABLE myc_effect_owner.effect_commits';
    EXCEPTION WHEN SQLSTATE '55000' THEN
        v_rejected := TRUE;
    END;
    IF NOT v_rejected THEN
        RAISE EXCEPTION 'effect ledger TRUNCATE unexpectedly succeeded';
    END IF;
END
$test$;

ROLLBACK;
