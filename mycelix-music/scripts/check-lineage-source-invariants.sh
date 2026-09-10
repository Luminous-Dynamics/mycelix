#!/usr/bin/env bash
set -euo pipefail

fail() {
  echo "lineage source invariant failed: $1" >&2
  exit 1
}

sql=packages/database/prisma/accounting-lineage-source.sql
adapter=packages/database/src/accounting-lineage-source.ts
integration=packages/database/src/accounting-lineage-source.integration.ts
workflow=../.github/workflows/music-accounting-persistence.yml
package=packages/database/package.json
schema=packages/database/prisma/schema.prisma

[[ -f "$sql" ]] || fail "serialized lineage source SQL is missing"
[[ -f "$adapter" ]] || fail "lineage source adapter is missing"
[[ -f "$integration" ]] || fail "live lineage source regression is missing"

# Cursor order must reflect serialized committed-source order. A free-running
# sequence can reserve numbers in one order and commit them in another.
if grep -Eqi 'CREATE[[:space:]]+SEQUENCE|nextval[[:space:]]*\(' "$sql"; then
  fail "lineage ingestion cursor must not use a free-running PostgreSQL sequence"
fi
grep -Fq "pg_advisory_xact_lock" "$sql" \
  || fail "lineage source must serialize writers and snapshots with a transaction advisory lock"
grep -Fq "lock_music_accounting_allocation_lineage_source" "$sql" \
  || fail "shared lineage source lock function is missing"
grep -Fq 'SELECT COALESCE(MAX("ingestSeq"), 0) + 1' "$sql" \
  || fail "ingestion cursor must be assigned as MAX+1 while holding the shared lock"
grep -Fq 'settlement_allocation_lineage_writer_lock' "$sql" \
  || fail "allocation inserts must acquire the shared lineage source lock"
grep -Fq 'settlement_allocation_successor_link_writer_lock' "$sql" \
  || fail "successor-link inserts must acquire the shared lineage source lock"
grep -Fq 'settlement_allocation_ingest_register' "$sql" \
  || fail "allocation inserts must register in the serialized ingestion namespace"
grep -Fq 'settlement_allocation_successor_link_ingest_register' "$sql" \
  || fail "successor-link inserts must register in the serialized ingestion namespace"
grep -Fq 'settlement_allocation_lineage_ingest_cursor_is_database_assigned' "$sql" \
  || fail "callers must not be able to choose ingestion cursor values"
grep -Fq 'settlement_allocation_lineage_ingest_append_only_guard' "$sql" \
  || fail "ingestion registry must reject UPDATE and DELETE"
grep -Fq 'settlement_allocation_successor_link_append_only_guard' "$sql" \
  || fail "successor links must reject UPDATE and DELETE"

# The cursor is operational completeness evidence, not part of immutable
# economic allocation authority. Never mutate/backfill allocation roots to add it.
allocation_model=$(sed -n '/^model SettlementAllocationRecord {/,/^}/p' "$schema")
[[ -n "$allocation_model" ]] || fail "SettlementAllocationRecord model is missing"
if grep -Fq 'ingestSeq' <<<"$allocation_model"; then
  fail "ingestion cursor must not be embedded in SettlementAllocationRecord authority"
fi

# Snapshot capture must take the same lock, bind a source clock/high-water mark,
# and detect eligible rows that are absent from the ingestion registry. An inner
# join without this negative check could silently certify an omitted row.
grep -Fq 'LINEAGE_LOCK_SQL' "$adapter" \
  || fail "snapshot adapter must use the database lineage lock"
grep -Fq '$queryRawUnsafe<void>(LINEAGE_LOCK_SQL)' "$adapter" \
  || fail "snapshot adapter must acquire advisory lock through the query path"
grep -Fq 'clock_timestamp() AS "observedThrough"' "$adapter" \
  || fail "snapshot must bind the serialized database observation time"
grep -Fq 'MAX("ingestSeq")' "$adapter" \
  || fail "snapshot must bind the committed ingestion high-water mark"
grep -Fq 'unregistered eligible evidence' "$adapter" \
  || fail "snapshot must fail closed on eligible source rows missing from registry"
grep -Fq 'r."ingestSeq" <= $3::bigint' "$adapter" \
  || fail "snapshot rows must be bounded by the captured high-water mark"

# Qualification must install and exercise this source against real PostgreSQL,
# including a privileged trigger-bypass omission attack.
grep -Fq 'db:accounting-lineage-source' "$package" \
  || fail "database package must expose serialized source installation"
grep -Fq 'accounting-lineage-source.integration.ts' "$package" \
  || fail "live serialized-source test must be part of accounting guard regressions"
grep -Fq 'db:accounting-lineage-source --workspace=@mycelix/database' "$workflow" \
  || fail "exact-head persistence workflow must install serialized lineage source"
grep -Fq 'check-lineage-source-invariants.sh' "$workflow" \
  || fail "exact-head persistence workflow must enforce lineage source invariants"
grep -Fq 'session_replication_role = replica' "$integration" \
  || fail "live regression must simulate a privileged trigger-bypass omission"
grep -Fq 'unregistered eligible evidence' "$integration" \
  || fail "live regression must prove privileged omission is detected"
grep -Fq 'cursor_is_database_assigned' "$integration" \
  || fail "live regression must attack caller-selected ingestion cursor"
grep -Fq 'ingestion cursor must preserve serialized source order' "$integration" \
  || fail "live regression must prove monotonic source ordering"

echo "lineage source invariants passed"
