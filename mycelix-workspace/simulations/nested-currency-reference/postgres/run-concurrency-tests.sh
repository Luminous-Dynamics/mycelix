#!/usr/bin/env bash
set -euo pipefail

: "${DATABASE_URL:?Set DATABASE_URL to a disposable PostgreSQL test database}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

call_admit() {
  local request_id="$1" payload_hex="$2" work_hex="$3" amount="$4"
  local principal="$5" verifier="$6" out="$7"
  local sql
  sql="SELECT ncf.admit_obligation(
    '$request_id'::uuid,
    decode(repeat('$payload_hex',64),'hex'),
    '$principal','community-a','NCF-RACE-UNIT',1,$amount,
    decode(repeat('$work_hex',64),'hex'),
    'aa000000-0000-0000-0000-000000000002'::uuid,'a_to_b',
    'bb000000-0000-0000-0000-000000000002'::uuid,'shard-race',1,
    'pg-concurrency-verifier-v1',decode(repeat('$verifier',64),'hex')
  )->>'decision';"
  psql "$DATABASE_URL" -XAt -v ON_ERROR_STOP=1 -c "$sql" > "$TMP/$out"
}

assert_pair() {
  local left="$1" right="$2" expected="$3" label="$4"
  local actual
  actual="$(cat "$TMP/$left" "$TMP/$right" | sort | paste -sd, -)"
  if [[ "$actual" != "$expected" ]]; then
    echo "FAIL $label: expected $expected; got $actual" >&2
    echo "$left=$(cat "$TMP/$left") $right=$(cat "$TMP/$right")" >&2
    exit 1
  fi
  echo "PASS $label: $actual"
}

# Case 1: distinct 60-unit work items compete for a 100-unit shared cap/slice.
call_admit "20000000-0000-0000-0000-000000000001" a a 60 cap-racer-a 1 distinct_a &
pid1=$!
call_admit "20000000-0000-0000-0000-000000000002" b b 60 cap-racer-b 2 distinct_b &
pid2=$!
wait "$pid1"; wait "$pid2"
assert_pair distinct_a distinct_b "accepted,rejected" "concurrent cap contention never overshoots"

# Case 2: same canonical work under different request IDs debits once.
call_admit "20000000-0000-0000-0000-000000000003" c e 5 alias-principal 3 alias_a &
pid1=$!
call_admit "20000000-0000-0000-0000-000000000004" d e 5 alias-principal 4 alias_b &
pid2=$!
wait "$pid1"; wait "$pid2"
assert_pair alias_a alias_b "accepted,replay" "concurrent canonical-work aliases debit once"

# Case 3: one request ID cannot bind two different immutable payloads/work items.
call_admit "20000000-0000-0000-0000-000000000005" 1 3 5 conflict-principal 5 conflict_a &
pid1=$!
call_admit "20000000-0000-0000-0000-000000000005" 2 4 5 conflict-principal 6 conflict_b &
pid2=$!
wait "$pid1"; wait "$pid2"
assert_pair conflict_a conflict_b "accepted,rejected" "concurrent request-ID conflict binds one payload"

psql "$DATABASE_URL" -X -v ON_ERROR_STOP=1 <<'SQL'
DO $$
DECLARE
  v_obligations bigint;
  v_amount numeric;
  v_used numeric;
  v_allocated numeric;
  v_slice_used numeric;
  v_bindings bigint;
  v_accepted bigint;
  v_replays bigint;
BEGIN
  SELECT count(*), COALESCE(sum(amount_units),0)
    INTO v_obligations, v_amount
    FROM ncf.obligation WHERE instrument_id='NCF-RACE-UNIT';
  SELECT allocated_units, outstanding_units
    INTO v_allocated, v_used
    FROM ncf.community_instrument_cap
   WHERE community_id='community-a' AND instrument_id='NCF-RACE-UNIT';
  SELECT used_units INTO v_slice_used
    FROM ncf.reservation_slice
   WHERE reservation_id='bb000000-0000-0000-0000-000000000002';
  SELECT count(*), count(*) FILTER (WHERE decision='accepted'), count(*) FILTER (WHERE decision='replay')
    INTO v_bindings, v_accepted, v_replays
    FROM ncf.request_binding WHERE instrument_id='NCF-RACE-UNIT';

  IF v_obligations <> 3 OR v_amount <> 70 OR v_allocated <> 100
     OR v_used <> 70 OR v_slice_used <> 70
     OR v_bindings <> 4 OR v_accepted <> 3 OR v_replays <> 1 THEN
    RAISE EXCEPTION 'concurrency accounting mismatch: obligations %, amount %, allocated %, outstanding %, slice %, bindings %, accepted %, replays %',
      v_obligations, v_amount, v_allocated, v_used, v_slice_used, v_bindings, v_accepted, v_replays;
  END IF;
  IF (SELECT count(*) FROM ncf.holochain_outbox
       WHERE event_type='obligation_accepted'
         AND obligation_id IN (SELECT obligation_id FROM ncf.obligation WHERE instrument_id='NCF-RACE-UNIT')) <> 3 THEN
    RAISE EXCEPTION 'outbox count mismatch for accepted canonical obligations';
  END IF;
  IF (SELECT count(*) FROM (
        SELECT canonical_work_digest FROM ncf.obligation
         WHERE instrument_id='NCF-RACE-UNIT'
         GROUP BY canonical_work_digest HAVING count(*) > 1
      ) d) <> 0 THEN
    RAISE EXCEPTION 'duplicate canonical work digest';
  END IF;
END;
$$;
SELECT 'NCF_POSTGRES_CONCURRENCY_PASS' AS result;
SQL

echo "NCF PostgreSQL concurrency conformance PASS"
