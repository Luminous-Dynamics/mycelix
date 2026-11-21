-- PostgreSQL worker-claim fencing transactional self-tests.
-- Apply 001_effect_owner.sql and 002_worker_claim_fencing.sql first.
-- Run: psql -v ON_ERROR_STOP=1 -f test_worker_claim_fencing.sql
-- Test rows are rolled back. Single-session only; not the required race test.

BEGIN;
SET LOCAL synchronous_commit = on;

INSERT INTO myc_effect_owner.effect_fence_state (
    realm_id, resource_id, action_id, effect_owner_id, generation
) VALUES ('claim-test-realm','claim-test-resource','claim-test-action','claim-test-db',1);

INSERT INTO myc_effect_owner.protected_balance (
    realm_id, resource_id, action_id, effect_owner_id, object_id,
    currency_code, balance_minor
) VALUES
    ('claim-test-realm','claim-test-resource','claim-test-action','claim-test-db','account-1','USD',1000),
    ('claim-test-realm','claim-test-resource','claim-test-action','claim-test-db','account-2','USD',2000),
    ('claim-test-realm','claim-test-resource','claim-test-action','claim-test-db','account-3','USD',3000);

DO $test$
DECLARE
    v_issued TIMESTAMPTZ := clock_timestamp() - interval '1 second';
    v_expires TIMESTAMPTZ := clock_timestamp() + interval '1 hour';
    v_result RECORD;
    v_failed BOOLEAN;
    v_balance BIGINT;
    v_epoch BIGINT;
BEGIN
    v_epoch := myc_effect_owner.install_worker_claim(
        'claim-test-realm','claim-test-resource','claim-test-action','claim-test-db',
        'claim-logical-op-1','worker-a',1,decode(repeat('11',32),'hex'),
        'postgres-db-clock-v1',v_issued,v_expires
    );
    IF v_epoch <> 1 THEN RAISE EXCEPTION 'initial claim epoch was not installed'; END IF;

    v_epoch := myc_effect_owner.install_worker_claim(
        'claim-test-realm','claim-test-resource','claim-test-action','claim-test-db',
        'claim-logical-op-1','worker-b',2,decode(repeat('22',32),'hex'),
        'postgres-db-clock-v1',v_issued,v_expires
    );
    IF v_epoch <> 2 THEN RAISE EXCEPTION 'worker reassignment did not advance claim epoch'; END IF;

    IF myc_effect_owner.install_worker_claim(
        'claim-test-realm','claim-test-resource','claim-test-action','claim-test-db',
        'claim-logical-op-1','worker-b',2,decode(repeat('22',32),'hex'),
        'postgres-db-clock-v1',v_issued,v_expires
    ) <> 2 THEN RAISE EXCEPTION 'exact worker claim replay was not idempotent'; END IF;

    v_failed := FALSE;
    BEGIN
        PERFORM * FROM myc_effect_owner.apply_claim_fenced_balance_delta(
            'claim-test-realm','claim-test-resource','claim-test-action','claim-test-db',
            'account-1','USD',1,decode(repeat('31',32),'hex'),'claim-logical-op-1',
            decode(repeat('41',32),'hex'),decode(repeat('51',32),'hex'),decode(repeat('61',32),'hex'),100,
            'worker-a',1,decode(repeat('11',32),'hex'),
            'postgres-db-clock-v1',v_issued,v_expires
        );
    EXCEPTION WHEN SQLSTATE 'P0001' THEN v_failed := TRUE;
    END;
    IF NOT v_failed THEN RAISE EXCEPTION 'superseded worker claim was allowed to mutate'; END IF;

    SELECT * INTO v_result FROM myc_effect_owner.apply_claim_fenced_balance_delta(
        'claim-test-realm','claim-test-resource','claim-test-action','claim-test-db',
        'account-1','USD',1,decode(repeat('31',32),'hex'),'claim-logical-op-1',
        decode(repeat('41',32),'hex'),decode(repeat('51',32),'hex'),decode(repeat('61',32),'hex'),100,
        'worker-b',2,decode(repeat('22',32),'hex'),
        'postgres-db-clock-v1',v_issued,v_expires
    );
    IF v_result.replayed OR v_result.accepted_generation <> 1 THEN
        RAISE EXCEPTION 'current worker claim did not commit as a new effect';
    END IF;

    SELECT * INTO v_result FROM myc_effect_owner.apply_claim_fenced_balance_delta(
        'claim-test-realm','claim-test-resource','claim-test-action','claim-test-db',
        'account-1','USD',1,decode(repeat('31',32),'hex'),'claim-logical-op-1',
        decode(repeat('41',32),'hex'),decode(repeat('51',32),'hex'),decode(repeat('61',32),'hex'),100,
        'worker-b',2,decode(repeat('22',32),'hex'),
        'postgres-db-clock-v1',v_issued,v_expires
    );
    IF NOT v_result.replayed THEN RAISE EXCEPTION 'exact worker retry did not replay prior effect'; END IF;

    v_failed := FALSE;
    BEGIN
        PERFORM * FROM myc_effect_owner.apply_claim_fenced_balance_delta(
            'claim-test-realm','claim-test-resource','claim-test-action','claim-test-db',
            'account-2','USD',1,decode(repeat('32',32),'hex'),'claim-logical-op-2',
            decode(repeat('42',32),'hex'),decode(repeat('52',32),'hex'),decode(repeat('62',32),'hex'),100,
            'worker-c',1,decode(repeat('23',32),'hex'),
            'postgres-db-clock-v1',clock_timestamp() - interval '1 minute',
            clock_timestamp() - interval '30 seconds'
        );
    EXCEPTION WHEN SQLSTATE 'P0001' THEN v_failed := TRUE;
    END;
    IF NOT v_failed THEN RAISE EXCEPTION 'expired Allow decision was accepted'; END IF;

    PERFORM myc_effect_owner.install_worker_claim(
        'claim-test-realm','claim-test-resource','claim-test-action','claim-test-db',
        'claim-logical-op-3','worker-c',1,decode(repeat('33',32),'hex'),
        'postgres-db-clock-v1',clock_timestamp() - interval '1 hour',
        clock_timestamp() - interval '30 seconds'
    );
    v_failed := FALSE;
    BEGIN
        PERFORM * FROM myc_effect_owner.apply_claim_fenced_balance_delta(
            'claim-test-realm','claim-test-resource','claim-test-action','claim-test-db',
            'account-3','USD',1,decode(repeat('33',32),'hex'),'claim-logical-op-3',
            decode(repeat('43',32),'hex'),decode(repeat('53',32),'hex'),decode(repeat('63',32),'hex'),100,
            'worker-c',1,decode(repeat('33',32),'hex'),
            'postgres-db-clock-v1',clock_timestamp() - interval '1 minute',
            clock_timestamp() + interval '1 minute'
        );
    EXCEPTION WHEN SQLSTATE 'P0001' THEN v_failed := TRUE;
    END;
    IF NOT v_failed THEN RAISE EXCEPTION 'expired worker claim was accepted'; END IF;

    SELECT balance_minor INTO v_balance
      FROM myc_effect_owner.protected_balance
     WHERE realm_id='claim-test-realm' AND resource_id='claim-test-resource'
       AND action_id='claim-test-action' AND effect_owner_id='claim-test-db'
       AND object_id='account-1' AND currency_code='USD';
    IF v_balance <> 1100 THEN RAISE EXCEPTION 'duplicate or rejected effect changed protected balance'; END IF;

    IF (SELECT count(*) FROM myc_effect_owner.effect_commits
        WHERE realm_id='claim-test-realm' AND resource_id='claim-test-resource'
          AND action_id='claim-test-action' AND effect_owner_id='claim-test-db') <> 1 THEN
        RAISE EXCEPTION 'unexpected effect records committed';
    END IF;
END
$test$;

ROLLBACK;
