#!/usr/bin/env bash
set -euo pipefail

: "${DATABASE_URL:?Set DATABASE_URL to a disposable PostgreSQL test database}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CLIENTS="${PG_NCF_CLIENTS:-8}"
THREADS="${PG_NCF_THREADS:-4}"
DURATION="${PG_NCF_DURATION_SECONDS:-30}"
OUT_DIR="${PG_NCF_OUTPUT_DIR:-$(mktemp -d "${TMPDIR:-/tmp}/ncf-pgbench.XXXXXX")}"
mkdir -p "$OUT_DIR"
# mktemp prevents same-second reruns from sharing raw logs or mixing transaction counts.
TMP="$(mktemp -d "$OUT_DIR/run-$(date -u +%Y%m%dT%H%M%SZ)-XXXXXX")"

if ! command -v pgbench >/dev/null 2>&1; then
  echo "pgbench executable is required (install the PostgreSQL client tools)" >&2
  exit 2
fi
if ! command -v psql >/dev/null 2>&1; then
  echo "psql executable is required (install the PostgreSQL client tools)" >&2
  exit 2
fi
if ! command -v python3 >/dev/null 2>&1; then
  echo "python3 is required to create the benchmark receipt" >&2
  exit 2
fi
if ! [[ "$CLIENTS" =~ ^[1-9][0-9]*$ && "$THREADS" =~ ^[1-9][0-9]*$ && "$DURATION" =~ ^[1-9][0-9]*$ ]]; then
  echo "PG_NCF_CLIENTS, PG_NCF_THREADS, and PG_NCF_DURATION_SECONDS must be positive integers" >&2
  exit 2
fi

SERVER_INFO="$(psql "$DATABASE_URL" -XAt -v ON_ERROR_STOP=1 -c "SELECT version() || ' | isolation=' || current_setting('transaction_isolation') || ' | synchronous_commit=' || current_setting('synchronous_commit') || ' | fsync=' || current_setting('fsync');")"
PGBENCH_INFO="$(pgbench --version)"
echo "$SERVER_INFO"
echo "$PGBENCH_INFO"
psql "$DATABASE_URL" -X -v ON_ERROR_STOP=1 -f "$ROOT/920_pgbench_setup.sql"

set +e
pgbench --no-vacuum --random-seed=20261028 --protocol=simple \
  --client="$CLIENTS" --jobs="$THREADS" --time="$DURATION" --log \
  --log-prefix="$TMP/pgbench" --failures-detailed --max-tries=1 --exit-on-abort \
  --progress=5 --report-per-command --file="$ROOT/pgbench_admit.sql" "$DATABASE_URL" \
  2>&1 | tee "$TMP/pgbench.stdout"
rc=${PIPESTATUS[0]}
set -e
if [[ "$rc" -ne 0 ]]; then
  echo "pgbench failed with exit code $rc" >&2
  exit "$rc"
fi

# Parse microsecond transaction logs and cross-check stdout before reconciling SQL state.
python3 "$ROOT/summarize_pgbench.py" "$TMP"

python3 - "$TMP" "$SERVER_INFO" "$PGBENCH_INFO" "$CLIENTS" "$THREADS" "$DURATION" "$ROOT" <<'PY'
import hashlib, json, pathlib, sys
root=pathlib.Path(sys.argv[1]); server=sys.argv[2]; client_version=sys.argv[3]
summary_path=root/"latency-summary.json"
o=json.loads(summary_path.read_text(encoding="utf-8"))
successful_count=int((root/"successful-count.txt").read_text(encoding="utf-8").strip())
if successful_count != o["logged_successful_transactions"]:
    raise SystemExit(f"successful-count receipt mismatch: file={successful_count} logs={o['logged_successful_transactions']}")
if o["pgbench_failed_transactions"] != 0 or o["logged_failed_or_unparseable_transactions"] != 0:
    raise SystemExit("refusing benchmark receipt with failed or unparseable transaction rows")
if o["pgbench_processed_transactions"] != successful_count:
    raise SystemExit("pgbench stdout count does not match successful per-transaction log count")
if not successful_count:
    raise SystemExit("refusing empty benchmark receipt")
o["runtime"]={"postgres_server":server,"pgbench_client":client_version}
o["parameters"]={"clients":int(sys.argv[4]),"threads":int(sys.argv[5]),"duration_seconds":int(sys.argv[6]),"query_mode":"simple","retry_limit":1}
source=pathlib.Path(sys.argv[7])
o["source_sha256"]={
  "schema":hashlib.sha256((source/"001_schema.sql").read_bytes()).hexdigest(),
  "transaction_functions":hashlib.sha256((source/"002_functions.sql").read_bytes()).hexdigest(),
  "pgbench_script":hashlib.sha256((source/"pgbench_admit.sql").read_bytes()).hexdigest(),
  "runner":hashlib.sha256((source/"run-pgbench-benchmark.sh").read_bytes()).hexdigest(),
  "log_parser":hashlib.sha256((source/"summarize_pgbench.py").read_bytes()).hexdigest(),
  "log_parser_tests":hashlib.sha256((source/"test_summarize_pgbench.py").read_bytes()).hexdigest()
}
o["pgbench_stdout"]=(root/"pgbench.stdout").read_text(encoding="utf-8")
summary_path.write_text(json.dumps(o,indent=2)+"\n",encoding="utf-8")
print("Benchmark receipt enriched with runtime, exact source hashes, and verified log/stdout transaction counts.")
PY

psql "$DATABASE_URL" -XAt -v ON_ERROR_STOP=1 <<'SQL' | tee "$TMP/reconciliation.txt"
SELECT jsonb_build_object(
  'obligations', count(*),
  'obligation_units', coalesce(sum(amount_units),0),
  'unique_work_digests', count(distinct canonical_work_digest),
  'request_bindings', (SELECT count(*) FROM ncf.request_binding WHERE instrument_id='NCF-PGBENCH-UNIT'),
  'accepted_outbox_events', (SELECT count(*) FROM ncf.holochain_outbox WHERE event_type='obligation_accepted' AND obligation_id IN (SELECT obligation_id FROM ncf.obligation WHERE instrument_id='NCF-PGBENCH-UNIT')),
  'community_allocated', (SELECT allocated_units FROM ncf.community_instrument_cap WHERE community_id='pgbench-community-a' AND instrument_id='NCF-PGBENCH-UNIT'),
  'community_outstanding', (SELECT outstanding_units FROM ncf.community_instrument_cap WHERE community_id='pgbench-community-a' AND instrument_id='NCF-PGBENCH-UNIT'),
  'edge_allocated', (SELECT allocated_units FROM ncf.agreement_direction_cap WHERE agreement_id='aa000000-0000-0000-0000-000000000010' AND direction='a_to_b'),
  'edge_outstanding', (SELECT outstanding_units FROM ncf.agreement_direction_cap WHERE agreement_id='aa000000-0000-0000-0000-000000000010' AND direction='a_to_b'),
  'slice_used', (SELECT used_units FROM ncf.reservation_slice WHERE reservation_id='bb000000-0000-0000-0000-000000000010')
)
FROM ncf.obligation WHERE instrument_id='NCF-PGBENCH-UNIT';
SQL

EXPECTED_TRANSACTIONS="$(cat "$TMP/successful-count.txt")"
python3 - "$TMP/reconciliation.txt" "$EXPECTED_TRANSACTIONS" <<'PY'
import json,sys
value=json.loads(open(sys.argv[1],encoding="utf-8").read().strip().splitlines()[-1])
n=value["obligations"]
expected=int(sys.argv[2])
assert n == expected, {"database_committed":n,"pgbench_successful_transactions":expected,"reconciliation":value}
assert value["obligation_units"] == n, value
assert value["unique_work_digests"] == n, value
assert value["request_bindings"] == n, value
assert value["accepted_outbox_events"] == n, value
assert value["community_outstanding"] == n, value
assert value["edge_outstanding"] == n, value
assert value["slice_used"] == n, value
assert value["community_allocated"] == value["edge_allocated"], value
assert value["slice_used"] == n and value["community_outstanding"] == n and value["edge_outstanding"] == n, value
print("NCF PostgreSQL pgbench ledger reconciliation PASS")
print(json.dumps(value,indent=2))
PY

cat "$TMP/pgbench.stdout"
echo "NCF PostgreSQL benchmark artifacts retained at: $OUT_DIR"
