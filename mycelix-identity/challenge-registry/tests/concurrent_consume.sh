#!/usr/bin/env sh
# SPDX-License-Identifier: AGPL-3.0-or-later
# Requires a disposable PostgreSQL test database with migration 0001 applied.
set -eu

if [ -z "${DATABASE_URL:-}" ]; then
  echo "ERROR: DATABASE_URL must identify a disposable test database" >&2
  exit 2
fi
if ! command -v psql >/dev/null 2>&1; then
  echo "ERROR: psql is required" >&2
  exit 2
fi
if ! command -v od >/dev/null 2>&1; then
  echo "ERROR: od is required to generate a unique test challenge" >&2
  exit 2
fi

tmp_dir="$(mktemp -d)"
trap 'rm -rf "$tmp_dir"' EXIT HUP INT TERM

challenge_hex="$(od -An -N32 -tx1 /dev/urandom | tr -d ' \n')"
request_hex="$(printf '%064d' 0 | tr '0' '1')"
holder_hex="$(printf '%064d' 0 | tr '0' '2')"
transaction_id="c2a1dfb6-2a15-4ac7-8e6a-6d21f6b8a002"

registration_sql="
SELECT identity_security.register_verifier_challenge(
  decode('$challenge_hex', 'hex'),
  'concurrent-test-verifier',
  'urn:mycelix:concurrent-test',
  '$transaction_id'::uuid,
  decode('$request_hex', 'hex'),
  decode('$holder_hex', 'hex'),
  clock_timestamp() + INTERVAL '5 minutes'
);
"
registered="$(psql "$DATABASE_URL" -X -qAt -v ON_ERROR_STOP=1 -c "$registration_sql")"
if [ "$registered" != "t" ]; then
  echo "ERROR: fixture registration failed: $registered" >&2
  exit 1
fi

consume_sql="
SELECT pg_sleep(0.25);
SELECT identity_security.consume_verifier_challenge(
  decode('$challenge_hex', 'hex'),
  'concurrent-test-verifier',
  'urn:mycelix:concurrent-test',
  '$transaction_id'::uuid,
  decode('$request_hex', 'hex'),
  decode('$holder_hex', 'hex')
);
"

psql "$DATABASE_URL" -X -qAt -v ON_ERROR_STOP=1 -c "$consume_sql" >"$tmp_dir/one.out" &
pid_one=$!
psql "$DATABASE_URL" -X -qAt -v ON_ERROR_STOP=1 -c "$consume_sql" >"$tmp_dir/two.out" &
pid_two=$!

wait "$pid_one"
wait "$pid_two"

true_count="$(cat "$tmp_dir/one.out" "$tmp_dir/two.out" | awk '$0 == "t" { count++ } END { print count + 0 }')"
false_count="$(cat "$tmp_dir/one.out" "$tmp_dir/two.out" | awk '$0 == "f" { count++ } END { print count + 0 }')"

if [ "$true_count" -ne 1 ] || [ "$false_count" -ne 1 ]; then
  echo "FAIL: expected exactly one successful consume and one rejection; got true=$true_count false=$false_count" >&2
  echo "process one: $(cat "$tmp_dir/one.out")" >&2
  echo "process two: $(cat "$tmp_dir/two.out")" >&2
  exit 1
fi

echo "PASS: two independent psql processes produced exactly one successful consume"
