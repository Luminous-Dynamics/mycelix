-- SPDX-License-Identifier: AGPL-3.0-or-later
-- Sequential consume-once contract test. Runs in a transaction and rolls
-- back all fixtures. Apply migration 0001 before running this script.

BEGIN;

DO $challenge_registry_test$
DECLARE
    v_digest BYTEA := decode(repeat('a', 64), 'hex');
    v_expired_digest BYTEA := decode(repeat('b', 64), 'hex');
    v_long_ttl_digest BYTEA := decode(repeat('c', 64), 'hex');
    v_consume_expiry_digest BYTEA := decode(repeat('d', 64), 'hex');
    v_short_digest BYTEA := decode('aa', 'hex');
    v_request_digest BYTEA := decode(repeat('1', 64), 'hex');
    v_holder_digest BYTEA := decode(repeat('2', 64), 'hex');
    v_transaction_id UUID := 'c2a1dfb6-2a15-4ac7-8e6a-6d21f6b8a001';
BEGIN
    IF NOT identity_security.register_verifier_challenge(
        v_digest, 'test-verifier', 'urn:mycelix:test-audience',
        v_transaction_id, v_request_digest, v_holder_digest,
        clock_timestamp() + INTERVAL '5 minutes'
    ) THEN
        RAISE EXCEPTION 'valid challenge registration failed';
    END IF;

    IF identity_security.register_verifier_challenge(
        v_digest, 'test-verifier', 'urn:mycelix:test-audience',
        v_transaction_id, v_request_digest, v_holder_digest,
        clock_timestamp() + INTERVAL '5 minutes'
    ) THEN
        RAISE EXCEPTION 'duplicate challenge digest was accepted';
    END IF;

    IF identity_security.consume_verifier_challenge(
        v_digest, 'wrong-verifier', 'urn:mycelix:test-audience',
        v_transaction_id, v_request_digest, v_holder_digest
    ) THEN
        RAISE EXCEPTION 'wrong verifier identity consumed challenge';
    END IF;

    IF identity_security.consume_verifier_challenge(
        v_digest, 'test-verifier', 'urn:mycelix:wrong-audience',
        v_transaction_id, v_request_digest, v_holder_digest
    ) THEN
        RAISE EXCEPTION 'wrong audience consumed challenge';
    END IF;

    IF identity_security.consume_verifier_challenge(
        v_digest, 'test-verifier', 'urn:mycelix:test-audience',
        v_transaction_id, decode(repeat('3', 64), 'hex'), v_holder_digest
    ) THEN
        RAISE EXCEPTION 'wrong request digest consumed challenge';
    END IF;

    IF identity_security.consume_verifier_challenge(
        v_digest, 'test-verifier', 'urn:mycelix:test-audience',
        'c2a1dfb6-2a15-4ac7-8e6a-6d21f6b8a099'::uuid,
        v_request_digest, v_holder_digest
    ) THEN
        RAISE EXCEPTION 'wrong transaction ID consumed challenge';
    END IF;

    IF identity_security.consume_verifier_challenge(
        v_digest, 'test-verifier', 'urn:mycelix:test-audience',
        v_transaction_id, v_request_digest, decode(repeat('4', 64), 'hex')
    ) THEN
        RAISE EXCEPTION 'wrong holder binding consumed challenge';
    END IF;

    IF identity_security.consume_verifier_challenge(
        v_expired_digest, 'test-verifier', 'urn:mycelix:test-audience',
        v_transaction_id, v_request_digest, v_holder_digest
    ) THEN
        RAISE EXCEPTION 'unknown challenge digest was consumed';
    END IF;

    IF NOT identity_security.consume_verifier_challenge(
        v_digest, 'test-verifier', 'urn:mycelix:test-audience',
        v_transaction_id, v_request_digest, v_holder_digest
    ) THEN
        RAISE EXCEPTION 'correct context did not consume challenge';
    END IF;

    IF identity_security.consume_verifier_challenge(
        v_digest, 'test-verifier', 'urn:mycelix:test-audience',
        v_transaction_id, v_request_digest, v_holder_digest
    ) THEN
        RAISE EXCEPTION 'sequential replay consumed challenge twice';
    END IF;

    IF identity_security.register_verifier_challenge(
        v_digest, 'test-verifier', 'urn:mycelix:test-audience',
        v_transaction_id, v_request_digest, v_holder_digest,
        clock_timestamp() + INTERVAL '5 minutes'
    ) THEN
        RAISE EXCEPTION 'consumed challenge digest was registered again';
    END IF;

    IF NOT identity_security.register_verifier_challenge(
        v_consume_expiry_digest, 'test-verifier', 'urn:mycelix:test-audience',
        v_transaction_id, v_request_digest, v_holder_digest,
        clock_timestamp() + INTERVAL '2 seconds'
    ) THEN
        RAISE EXCEPTION 'short-lived consume-expiry challenge registration failed';
    END IF;

    PERFORM pg_sleep(2.1);

    IF identity_security.consume_verifier_challenge(
        v_consume_expiry_digest, 'test-verifier', 'urn:mycelix:test-audience',
        v_transaction_id, v_request_digest, v_holder_digest
    ) THEN
        RAISE EXCEPTION 'challenge expired after registration was consumed';
    END IF;

    IF identity_security.register_verifier_challenge(
        v_consume_expiry_digest, 'test-verifier', 'urn:mycelix:test-audience',
        v_transaction_id, v_request_digest, v_holder_digest,
        clock_timestamp() + INTERVAL '5 minutes'
    ) THEN
        RAISE EXCEPTION 'expired challenge digest was re-registered instead of retained as a tombstone';
    END IF;

    IF identity_security.register_verifier_challenge(
        v_expired_digest, 'test-verifier', 'urn:mycelix:test-audience',
        v_transaction_id, v_request_digest, v_holder_digest,
        clock_timestamp() - INTERVAL '1 second'
    ) THEN
        RAISE EXCEPTION 'expired challenge registration was accepted';
    END IF;

    IF identity_security.register_verifier_challenge(
        v_long_ttl_digest, 'test-verifier', 'urn:mycelix:test-audience',
        v_transaction_id, v_request_digest, v_holder_digest,
        clock_timestamp() + INTERVAL '11 minutes'
    ) THEN
        RAISE EXCEPTION 'challenge lifetime greater than ten minutes was accepted';
    END IF;

    IF identity_security.register_verifier_challenge(
        v_short_digest, 'test-verifier', 'urn:mycelix:test-audience',
        v_transaction_id, v_request_digest, v_holder_digest,
        clock_timestamp() + INTERVAL '5 minutes'
    ) THEN
        RAISE EXCEPTION 'malformed challenge digest length was accepted';
    END IF;

    RAISE NOTICE 'PASS: challenge registration, context binding, expiry, one-time consumption and replay rejection';
END;
$challenge_registry_test$;

ROLLBACK;
