#!/usr/bin/env bash
set -euo pipefail

: "${DATABASE_URL:?Set DATABASE_URL to a disposable PostgreSQL test database}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CLIENTS="${PG_NCF_CLIENTS:-8}"
THREADS="${PG_NCF_THREADS:-4}"
DURATION="${PG_NCF_DURATION_SECONDS:-30}"
OUT_DIR="${PG_NCF_OUTPUT_DIR:-$(mktemp -d "${TMPDIR:-/tmp}/ncf-pgbench.XXXXXX")}"
mkdir -p "$OUT_DIR"
TMP="$OUT_DIR"

if ! command -v pgbench >/dev/null 2>&1; then
  echo "pgbench executable is required (install the PostgreSQL client tools)" >&2
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

python3 - "$TMP" <<'PY'
import glob, json, math, pathlib, statistics, sys
root=pathlib.Path(sys.argv[1])
samples=[]; failed=[]
for path in glob.glob(str(root/"pgbench.*")):
    p=pathlib.Path(path)
    if p.name=="pgbench.stdout": continue
    try:
        rows=p.read_text(encoding="utf-8").splitlines()
    except (OSError, UnicodeError):
        continue
    for row in rows:
        parts=row.split()
        if len(parts)<6: continue
        latency=parts[2]
        if latency in {"failed","serialization","deadlock","skipped"}:
            failed.append({"file":p.name,"row":row})
            continue
        try:
            samples.append(float(latency)/1000.0)  # pgbench transaction time is microseconds
        except ValueError:
            failed.append({"file":p.name,"row":row})
def pct(a,p):
    if not a: return None
    a=sorted(a)
    return round(a[max(0, math.ceil(p*len(a))-1)],4)
summary={
 "pgbench_transaction_log_files":len(glob.glob(str(root/"pgbench.*"))),
 "logged_successful_transactions":len(samples),
 "logged_failed_or_unparseable_transactions":len(failed),
 "latency_ms":{"p50":pct(samples,.50),"p95":pct(samples,.95),"p99":pct(samples,.99),"max":round(max(samples),4) if samples else None},
 "claim_ceiling":"PostgreSQL shared-coordinator function benchmark only; this is not Holochain DHT throughput or distributed/offline settlement throughput."
}
(root/"latency-summary.json").write_text(json.dumps(summary,indent=2)+"\n",encoding="utf-8")
print(json.dumps(summary,indent=2))
if not samples or failed:
    sys.exit(3)
PY

python3 - "$TMP" "$SERVER_INFO" "$PGBENCH_INFO" "$CLIENTS" "$THREADS" "$DURATION" "$ROOT" <<'PY'
import hashlib, json, pathlib, sys
root=pathlib.Path(sys.argv[1]); server=sys.argv[2]; client_version=sys.argv[3]
summary_path=root/"latency-summary.json"
o=json.loads(summary_path.read_text(encoding="utf-8"))
o["runtime"]={"postgres_server":server,"pgbench_client":client_version}
o["parameters"]={"clients":int(sys.argv[4]),"threads":int(sys.argv[5]),"duration_seconds":int(sys.argv[6]),"query_mode":"simple","retry_limit":1}
o["source_sha256"]={
  "schema":hashlib.sha256((pathlib.Path(sys.argv[7])/"001_schema.sql").read_bytes()).hexdigest(),
  "transaction_functions":hashlib.sha256((pathlib.Path(sys.argv[7])/"002_functions.sql").read_bytes()).hexdigest(),
  "pgbench_script":hashlib.sha256((pathlib.Path(sys.argv[7])/"pgbench_admit.sql").read_bytes()).hexdigest(),
  "runner":hashlib.sha256((pathlib.Path(sys.argv[7])/"run-pgbench-benchmark.sh").read_bytes()).hexdigest()
}
o["pgbench_stdout"]=(root/"pgbench.stdout").read_text(encoding="utf-8")
summary_path.write_text(json.dumps(o,indent=2)+"\\n",encoding="utf-8")
print("Benchmark receipt enriched with runtime, parameters and exact source SHA-256 values.")
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
assert value["community_allocated"] == value["edge_allocated"] == value["slice_used"] and value["slice_used"] == n, value
print("NCF PostgreSQL pgbench ledger reconciliation PASS")
print(json.dumps(value,indent=2))
PY

cat "$TMP/pgbench.stdout"
echo "NCF PostgreSQL benchmark artifacts retained at: $OUT_DIR"
