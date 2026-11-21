-- Transactional self-tests for 001_effect_owner.sql.
-- Run with psql -v ON_ERROR_STOP=1 -f this_file.sql after applying the schema.
-- All test rows are rolled back. This file does not test two-session races.

BEGIN;
SET LOCAL synchronous_commit = on;

INSERT INTO myc_effect_owner.effect_fence_state (
    realm_id, resource_id, action_id, effect_owner_id, generation
) VALUES ('test-realm', 'test-resource', 'balance-adjust', 'test-db-owner', 1);

INSERT INTO myc_effect_owner.protected_balance (
    realm_id, resource_id, action_id, effect_owner_id, object_id,
    currency_code, balance_minor
) VALUES
    ('test-realm', 'test-resource', 'balance-adjust', 'test-db-owner', 'account-a', 'USD', 10000),
    ('test-realm', 'test-resource', 'balance-adjust', 'test-db-owner', 'account-b', 'USD', 20000);

DO $test$
DECLARE
    v_first RECORD;
    v_replay RECORD;
    v_generation BIGINT;
    v_failed BOOLEAN;
    v_balance BIGINT;
BEGIN
    SELECT * INTO v_first
      FROM myc_effect_owner.apply_balance_delta(
        'test-realm', 'test-resource', 'balance-adjust', 'test-db-owner',
        'account-a', 'USD', 1,
        decode(repeat('11', 32), 'hex'), 'logical-op-1',
        decode(repeat('22', 32), 'hex'), decode(repeat('33', 32), 'hex'),
        decode(repeat('44', 32), 'hex'), 125
    , 'postgres-db-clock-v1',
        clock_timestamp() - interval '1 minute',
        clock_timestamp() + interval '1 minute');

    IF v_first.replayed OR v_first.accepted_generation <> 1 THEN
        RAISE EXCEPTION 'first effect was not recorded as a new generation-1 effect';
    END IF;

    SELECT * INTO v_replay
      FROM myc_effect_owner.apply_balance_delta(
        'test-realm', 'test-resource', 'balance-adjust', 'test-db-owner',
        'account-a', 'USD', 1,
        decode(repeat('11', 32), 'hex'), 'logical-op-1',
        decode(repeat('22', 32), 'hex'), decode(repeat('33', 32), 'hex'),
        decode(repeat('44', 32), 'hex'), 125
    , 'postgres-db-clock-v1',
        clock_timestamp() - interval '1 minute',
        clock_timestamp() + interval '1 minute');

    IF NOT v_replay.replayed OR v_replay.effect_id <> v_first.effect_id THEN
        RAISE EXCEPTION 'exact retry did not return the original effect receipt';
    END IF;

    v_failed := FALSE;
    BEGIN
        PERFORM *
          FROM myc_effect_owner.apply_balance_delta(
            'test-realm', 'test-resource', 'balance-adjust', 'test-db-owner',
            'account-a', 'USD', 1,
            decode(repeat('11', 32), 'hex'), 'logical-op-1',
            decode(repeat('22', 32), 'hex'), decode(repeat('33', 32), 'hex'),
            decode(repeat('55', 32), 'hex'), 125
        , 'postgres-db-clock-v1',
        clock_timestamp() - interval '1 minute',
        clock_timestamp() + interval '1 minute');
    EXCEPTION WHEN SQLSTATE 'P0001' THEN
        v_failed := TRUE;
    END;
    IF NOT v_failed THEN
        RAISE EXCEPTION 'idempotency-key substitution was not rejected';
    END IF;

    v_failed := FALSE;
    BEGIN
        PERFORM *
          FROM myc_effect_owner.apply_balance_delta(
            'test-realm', 'test-resource', 'balance-adjust', 'test-db-owner',
            'account-a', 'USD', 1,
            decode(repeat('66', 32), 'hex'), 'logical-op-1',
            decode(repeat('77', 32), 'hex'), decode(repeat('33', 32), 'hex'),
            decode(repeat('88', 32), 'hex'), 500
        , 'postgres-db-clock-v1',
        clock_timestamp() - interval '1 minute',
        clock_timestamp() + interval '1 minute');
    EXCEPTION WHEN SQLSTATE 'P0001' THEN
        v_failed := TRUE;
    END;
    IF NOT v_failed THEN
        RAISE EXCEPTION 'logical-operation substitution was not rejected';
    END IF;

    v_generation := myc_effect_owner.advance_effect_fence(
        'test-realm', 'test-resource', 'balance-adjust', 'test-db-owner',
        decode(repeat('99', 32), 'hex')
    );
    IF v_generation <> 2 THEN
        RAISE EXCEPTION 'revocation did not advance exactly one generation';
    END IF;

    IF myc_effect_owner.advance_effect_fence(
        'test-realm', 'test-resource', 'balance-adjust', 'test-db-owner',
        decode(repeat('99', 32), 'hex')
    ) <> 2 THEN
        RAISE EXCEPTION 'exact revocation retry was not idempotent';
    END IF;

    -- Exact replay remains a read-only way to resolve an earlier commit.
    SELECT * INTO v_replay
      FROM myc_effect_owner.apply_balance_delta(
        'test-realm', 'test-resource', 'balance-adjust', 'test-db-owner',
        'account-a', 'USD', 1,
        decode(repeat('11', 32), 'hex'), 'logical-op-1',
        decode(repeat('22', 32), 'hex'), decode(repeat('33', 32), 'hex'),
        decode(repeat('44', 32), 'hex'), 125
    , 'postgres-db-clock-v1',
        clock_timestamp() - interval '1 minute',
        clock_timestamp() + interval '1 minute');
    IF NOT v_replay.replayed OR v_replay.effect_id <> v_first.effect_id THEN
        RAISE EXCEPTION 'stale-token exact replay failed to resolve original receipt';
    END IF;

    v_failed := FALSE;
    BEGIN
        PERFORM *
          FROM myc_effect_owner.apply_balance_delta(
            'test-realm', 'test-resource', 'balance-adjust', 'test-db-owner',
            'account-b', 'USD', 1,
            decode(repeat('aa', 32), 'hex'), 'logical-op-2',
            decode(repeat('bb', 32), 'hex'), decode(repeat('cc', 32), 'hex'),
            decode(repeat('dd', 32), 'hex'), 500
        , 'postgres-db-clock-v1',
        clock_timestamp() - interval '1 minute',
        clock_timestamp() + interval '1 minute');
    EXCEPTION WHEN SQLSTATE 'P0001' THEN
        v_failed := TRUE;
    END;
    IF NOT v_failed THEN
        RAISE EXCEPTION 'stale generation was allowed to create a new effect';
    END IF;

    SELECT balance_minor INTO v_balance
      FROM myc_effect_owner.protected_balance
     WHERE realm_id = 'test-realm'
       AND resource_id = 'test-resource'
       AND action_id = 'balance-adjust'
       AND effect_owner_id = 'test-db-owner'
       AND object_id = 'account-a'
       AND currency_code = 'USD';
    IF v_balance <> 10125 THEN
        RAISE EXCEPTION 'duplicate or rejected effect changed the protected balance';
    END IF;

    IF (SELECT count(*) FROM myc_effect_owner.effect_commits
         WHERE realm_id = 'test-realm' AND resource_id = 'test-resource'
           AND action_id = 'balance-adjust' AND effect_owner_id = 'test-db-owner') <> 1 THEN
        RAISE EXCEPTION 'effect journal contains unexpected committed records';
    END IF;
END
$test$;

ROLLBACK;
