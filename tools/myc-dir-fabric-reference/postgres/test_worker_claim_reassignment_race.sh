#!/usr/bin/env bash
# Two-session PostgreSQL worker-claim reassignment race.
#
# Requires the reference migrations 001_effect_owner.sql and
# 002_worker_claim_fencing.sql to be applied to a DISPOSABLE test database.
# The caller must have permission to create a temporary test trigger/function.
#
# Run:
#   TEST_DATABASE_URL='postgresql://...' bash test_worker_claim_reassignment_race.sh
#
# This script is a runtime qualification harness. Source presence is not a
# passing result; preserve both session logs and the exact PostgreSQL version.

set -euo pipefail

: "${TEST_DATABASE_URL:?Set TEST_DATABASE_URL to a disposable PostgreSQL database}"
command -v psql >/dev/null 2>&1 || {
  printf '%s\n' "psql is required" >&2
  exit 127
}

tmp_dir="$(mktemp -d)"
realm="myc-claim-race-$$"
resource="race-resource"
action="balance-adjust"
owner="postgres-race-owner"
object="race-account"
logical_operation="race-logical-operation"

cleanup() {
  if [[ -n "${A_PID:-}" ]] && kill -0 "$A_PID" 2>/dev/null; then
    kill "$A_PID" 2>/dev/null || true
  fi
  if [[ -n "${B_PID:-}" ]] && kill -0 "$B_PID" 2>/dev/null; then
    kill "$B_PID" 2>/dev/null || true
  fi
  rm -rf "$tmp_dir"
}
trap cleanup EXIT INT TERM

psql_file() {
  psql "$TEST_DATABASE_URL" -X -v ON_ERROR_STOP=1 -v realm="$realm" "$@"
}

psql_scalar() {
  psql "$TEST_DATABASE_URL" -X -A -t -v ON_ERROR_STOP=1 -v realm="$realm" "$@"
}

printf '%s\n' "PostgreSQL version:"
psql_file -c 'SELECT version();'

# Isolated test state. Use a unique realm so this script does not collide with
# previous runs. The append-only effect ledger is intentionally retained.
psql_file <<'SQL'
BEGIN;
SET LOCAL synchronous_commit = on;

INSERT INTO myc_effect_owner.effect_fence_state (
    realm_id, resource_id, action_id, effect_owner_id, generation
) VALUES (
    :'realm', 'race-resource', 'balance-adjust', 'postgres-race-owner', 1
);

INSERT INTO myc_effect_owner.protected_balance (
    realm_id, resource_id, action_id, effect_owner_id, object_id,
    currency_code, balance_minor
) VALUES (
    :'realm', 'race-resource', 'balance-adjust', 'postgres-race-owner',
    'race-account', 'USD', 1000
);

SELECT myc_effect_owner.install_worker_claim(
    :'realm', 'race-resource', 'balance-adjust', 'postgres-race-owner',
    'race-logical-operation', 'worker-a', 1, decode(repeat('55', 32), 'hex'),
    'postgres-db-clock-v1',
    clock_timestamp() - interval '1 minute',
    clock_timestamp() + interval '1 hour'
);

COMMIT;
SQL

# Deliberately pause the protected-row UPDATE for this one unique realm.
# The effect function has already acquired FOR SHARE on worker_claim_state;
# a concurrent claim reassignment must block until Session A commits.
psql_file <<'SQL'
CREATE OR REPLACE FUNCTION myc_effect_owner.test_pause_claim_race()
RETURNS trigger
LANGUAGE plpgsql
SET search_path = pg_catalog, myc_effect_owner
AS $function$
BEGIN
    IF NEW.realm_id = current_setting('myc.dir.claim_race_realm', true) THEN
        PERFORM pg_catalog.pg_sleep(5);
    END IF;
    RETURN NEW;
END
$function$;

DROP TRIGGER IF EXISTS test_pause_claim_race
ON myc_effect_owner.protected_balance;

CREATE TRIGGER test_pause_claim_race
BEFORE UPDATE OF balance_minor ON myc_effect_owner.protected_balance
FOR EACH ROW EXECUTE FUNCTION myc_effect_owner.test_pause_claim_race();
SQL

# Session A acquires and holds the current worker claim row FOR SHARE while
# the trigger pauses the protected mutation.
psql_file >"$tmp_dir/session-a.log" 2>&1 <<'SQL' &
BEGIN;
SET LOCAL synchronous_commit = on;
SELECT set_config('myc.dir.claim_race_realm', :'realm', false);

SELECT *
FROM myc_effect_owner.apply_claim_fenced_balance_delta(
    :'realm', 'race-resource', 'balance-adjust', 'postgres-race-owner',
    'race-account', 'USD', 1,
    decode(repeat('11', 32), 'hex'),
    'race-logical-operation',
    decode(repeat('22', 32), 'hex'),
    decode(repeat('33', 32), 'hex'),
    decode(repeat('44', 32), 'hex'),
    100,
    'worker-a', 1, decode(repeat('55', 32), 'hex'),
    'postgres-db-clock-v1',
    clock_timestamp() - interval '1 minute',
    clock_timestamp() + interval '1 hour'
);

COMMIT;
SQL
A_PID=$!

# Let Session A enter the trigger, then attempt a higher-epoch reassignment.
# The reassignment must wait for the transaction that holds the old claim.
sleep 1
B_START=$SECONDS
psql_file >"$tmp_dir/session-b.log" 2>&1 <<'SQL' &
BEGIN;
SET LOCAL synchronous_commit = on;

SELECT myc_effect_owner.install_worker_claim(
    :'realm', 'race-resource', 'balance-adjust', 'postgres-race-owner',
    'race-logical-operation', 'worker-b', 2, decode(repeat('66', 32), 'hex'),
    'postgres-db-clock-v1',
    clock_timestamp() - interval '1 minute',
    clock_timestamp() + interval '1 hour'
);

COMMIT;
SQL
B_PID=$!

wait "$A_PID" || {
  cat "$tmp_dir/session-a.log" >&2
  exit 1
}
wait "$B_PID" || {
  cat "$tmp_dir/session-b.log" >&2
  exit 1
}
B_WAIT_SECONDS=$((SECONDS - B_START))

if (( B_WAIT_SECONDS < 3 )); then
  printf 'FAIL: claim reassignment did not demonstrate the expected lock wait (observed %ss)\n' "$B_WAIT_SECONDS" >&2
  cat "$tmp_dir/session-a.log" "$tmp_dir/session-b.log" >&2
  exit 1
fi

printf 'PASS: higher claim epoch waited %ss for the in-flight effect transaction\n' "$B_WAIT_SECONDS"

# The old worker must now be rejected, even when it tries an exact replay.
# Claim validation occurs before returning the old effect receipt.
if psql_file >"$tmp_dir/stale-worker.log" 2>&1 <<'SQL'
SELECT *
FROM myc_effect_owner.apply_claim_fenced_balance_delta(
    :'realm', 'race-resource', 'balance-adjust', 'postgres-race-owner',
    'race-account', 'USD', 1,
    decode(repeat('11', 32), 'hex'),
    'race-logical-operation',
    decode(repeat('22', 32), 'hex'),
    decode(repeat('33', 32), 'hex'),
    decode(repeat('44', 32), 'hex'),
    100,
    'worker-a', 1, decode(repeat('55', 32), 'hex'),
    'postgres-db-clock-v1',
    clock_timestamp() - interval '1 minute',
    clock_timestamp() + interval '1 hour'
);
SQL
then
  printf '%s\n' "FAIL: superseded worker-a claim was accepted" >&2
  exit 1
fi

grep -Fq 'worker claim is stale' "$tmp_dir/stale-worker.log" || {
  printf '%s\n' "FAIL: worker-a was rejected for an unexpected reason" >&2
  cat "$tmp_dir/stale-worker.log" >&2
  exit 1
}
printf '%s\n' "PASS: superseded worker-a claim is rejected after epoch 2 commits"

# The new worker may resolve the exact prior operation, but it must receive a
# read-only replay of the existing effect receipt.
replay_result="$(psql_scalar -c "
SELECT replayed::text || ':' || effect_id::text
FROM myc_effect_owner.apply_claim_fenced_balance_delta(
    '$realm', 'race-resource', 'balance-adjust', 'postgres-race-owner',
    'race-account', 'USD', 1,
    decode(repeat('11', 32), 'hex'),
    'race-logical-operation',
    decode(repeat('22', 32), 'hex'),
    decode(repeat('33', 32), 'hex'),
    decode(repeat('44', 32), 'hex'),
    100,
    'worker-b', 2, decode(repeat('66', 32), 'hex'),
    'postgres-db-clock-v1',
    clock_timestamp() - interval '1 minute',
    clock_timestamp() + interval '1 hour'
);")"

case "$replay_result" in
  t:*) printf '%s\n' "PASS: current worker-b receives the exact prior read-only effect receipt" ;;
  *) printf 'FAIL: current worker did not receive an exact replay: %s\n' "$replay_result" >&2; exit 1 ;;
esac

effect_count="$(psql_scalar -c "SELECT count(*) FROM myc_effect_owner.effect_commits WHERE realm_id = '$realm';")"
balance="$(psql_scalar -c "SELECT balance_minor FROM myc_effect_owner.protected_balance WHERE realm_id = '$realm' AND resource_id = 'race-resource' AND action_id = 'balance-adjust' AND effect_owner_id = 'postgres-race-owner' AND object_id = 'race-account' AND currency_code = 'USD';")"

[[ "$effect_count" == "1" ]] || {
  printf 'FAIL: expected exactly one effect record, got %s\n' "$effect_count" >&2
  exit 1
}
[[ "$balance" == "1100" ]] || {
  printf 'FAIL: expected balance 1100, got %s\n' "$balance" >&2
  exit 1
}
printf '%s\n' "PASS: exactly one logical effect exists; protected balance is 1100"

psql_file <<'SQL'
DROP TRIGGER IF EXISTS test_pause_claim_race
ON myc_effect_owner.protected_balance;
DROP FUNCTION IF EXISTS myc_effect_owner.test_pause_claim_race();
SQL

printf '%s\n' "RESULT: all worker-claim reassignment race assertions passed"
printf 'realm_id=%s\n' "$realm"
printf 'session_a_log=%s\n' "$tmp_dir/session-a.log"
printf 'session_b_log=%s\n' "$tmp_dir/session-b.log"
