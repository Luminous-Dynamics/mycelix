-- SPDX-License-Identifier: AGPL-3.0-or-later
-- CI-only role provisioning. Do not reuse the static test credential in a deployment.
-- Run as the disposable PostgreSQL administrator after migration 0001.

CREATE ROLE verifier_challenge_registry_owner
    NOLOGIN NOINHERIT NOSUPERUSER NOCREATEDB NOCREATEROLE NOREPLICATION;

CREATE ROLE verifier_challenge_runtime
    LOGIN INHERIT NOSUPERUSER NOCREATEDB NOCREATEROLE NOREPLICATION
    PASSWORD 'ci-only-verifier-challenge-runtime';

-- SECURITY DEFINER functions execute as a dedicated non-login owner that owns
-- both the schema and table. PostgreSQL requires the new schema owner to
-- hold CREATE on the containing database during ownership transfer. Revoke it
-- immediately after the transfer so the function owner has no database DDL
-- privilege during test execution.
GRANT CREATE ON DATABASE challenge_registry_test
    TO verifier_challenge_registry_owner;

ALTER SCHEMA identity_security
    OWNER TO verifier_challenge_registry_owner;
ALTER TABLE identity_security.verifier_challenges
    OWNER TO verifier_challenge_registry_owner;
ALTER FUNCTION identity_security.register_verifier_challenge(
    BYTEA, TEXT, TEXT, UUID, BYTEA, BYTEA, INTEGER
) OWNER TO verifier_challenge_registry_owner;
ALTER FUNCTION identity_security.consume_verifier_challenge(
    BYTEA, TEXT, TEXT, UUID, BYTEA, BYTEA
) OWNER TO verifier_challenge_registry_owner;

REVOKE CREATE ON DATABASE challenge_registry_test
    FROM verifier_challenge_registry_owner;

REVOKE ALL ON SCHEMA identity_security FROM PUBLIC;
REVOKE ALL ON TABLE identity_security.verifier_challenges FROM PUBLIC;
REVOKE ALL ON TABLE identity_security.verifier_challenges
    FROM verifier_challenge_runtime;
REVOKE ALL ON FUNCTION identity_security.register_verifier_challenge(
    BYTEA, TEXT, TEXT, UUID, BYTEA, BYTEA, INTEGER
) FROM PUBLIC;
REVOKE ALL ON FUNCTION identity_security.consume_verifier_challenge(
    BYTEA, TEXT, TEXT, UUID, BYTEA, BYTEA
) FROM PUBLIC;

GRANT USAGE ON SCHEMA identity_security TO verifier_challenge_runtime;
GRANT EXECUTE ON FUNCTION identity_security.register_verifier_challenge(
    BYTEA, TEXT, TEXT, UUID, BYTEA, BYTEA, INTEGER
) TO verifier_challenge_runtime;
GRANT EXECUTE ON FUNCTION identity_security.consume_verifier_challenge(
    BYTEA, TEXT, TEXT, UUID, BYTEA, BYTEA
) TO verifier_challenge_runtime;

DO $least_privilege_assertions$
BEGIN
    IF NOT has_schema_privilege(
        'verifier_challenge_runtime', 'identity_security', 'USAGE'
    ) THEN
        RAISE EXCEPTION 'runtime role is missing schema USAGE';
    END IF;

    IF has_table_privilege(
        'verifier_challenge_runtime',
        'identity_security.verifier_challenges',
        'SELECT'
    ) OR has_table_privilege(
        'verifier_challenge_runtime',
        'identity_security.verifier_challenges',
        'INSERT'
    ) OR has_table_privilege(
        'verifier_challenge_runtime',
        'identity_security.verifier_challenges',
        'UPDATE'
    ) OR has_table_privilege(
        'verifier_challenge_runtime',
        'identity_security.verifier_challenges',
        'DELETE'
    ) OR has_table_privilege(
        'verifier_challenge_runtime',
        'identity_security.verifier_challenges',
        'TRUNCATE'
    ) THEN
        RAISE EXCEPTION 'runtime role has direct table privileges';
    END IF;

    IF NOT has_function_privilege(
        'verifier_challenge_runtime',
        'identity_security.register_verifier_challenge(bytea,text,text,uuid,bytea,bytea,integer)',
        'EXECUTE'
    ) OR NOT has_function_privilege(
        'verifier_challenge_runtime',
        'identity_security.consume_verifier_challenge(bytea,text,text,uuid,bytea,bytea)',
        'EXECUTE'
    ) THEN
        RAISE EXCEPTION 'runtime role is missing challenge function EXECUTE';
    END IF;
END;
$least_privilege_assertions$;
