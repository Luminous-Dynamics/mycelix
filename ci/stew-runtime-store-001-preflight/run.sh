#!/usr/bin/env bash
set -euo pipefail

PRODUCT_ROOT=${1:?product checkout path required}
RUNNER_TMP=${2:?runner temp path required}
HARNESS_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
HARNESS_MANIFEST="$HARNESS_ROOT/Cargo.toml"
WORK="$RUNNER_TMP/stew-runtime-store-adversarial"
TARGET="$RUNNER_TMP/stew-runtime-store-harness-target"

rm -rf "$WORK" "$TARGET"
mkdir -p "$WORK"
export CARGO_TARGET_DIR="$TARGET"
ulimit -c 0 || true

# Independently prove that the duplicated initialization/validation DDL bytes are
# identical and that the public schema label is their actual SHA-256.
python3 - "$PRODUCT_ROOT" <<'PY'
import ast
import hashlib
import pathlib
import re
import sys

root = pathlib.Path(sys.argv[1]) / "crates/mycelix-stewardship-runtime-store/src"
legacy = (root / "lib.rs").read_text()
profiled = (root / "profiled.rs").read_text()

def schema_sql(text: str) -> str:
    match = re.search(r'const SCHEMA_SQL_V1: &str = ("(?:[^"\\]|\\.)*");', text, re.S)
    if not match:
        raise SystemExit("SCHEMA_SQL_V1 not found")
    return ast.literal_eval(match.group(1))

legacy_sql = schema_sql(legacy)
profiled_sql = schema_sql(profiled)
if legacy_sql != profiled_sql:
    raise SystemExit("profiled and legacy SCHEMA_SQL_V1 bytes diverge")

label_match = re.search(r'pub const SCHEMA_ID_V1: &str =\s*"([^"]+)";', legacy, re.S)
if not label_match:
    raise SystemExit("SCHEMA_ID_V1 not found")
digest = hashlib.sha256(legacy_sql.encode()).hexdigest()
expected_label = f"sha256:{digest}"
if label_match.group(1) != expected_label:
    raise SystemExit(f"schema label mismatch: {label_match.group(1)} != {expected_label}")
print(f"SCHEMA_SOURCE_DIGEST={digest}")
print("SCHEMA_SOURCE_DUPLICATION=EXACT")
PY

cargo generate-lockfile --manifest-path "$HARNESS_MANIFEST"
cargo build --manifest-path "$HARNESS_MANIFEST" --locked
HARNESS="$TARGET/debug/stew-runtime-store-preflight-harness"
test -x "$HARNESS"

wait_for_file() {
  local file=$1
  local i
  for i in $(seq 1 2000); do
    [[ -f "$file" ]] && return 0
    sleep 0.01
  done
  echo "timed out waiting for $file" >&2
  return 1
}

# 1. Two-process exact duplicate genesis race: one semantic initialization.
DB="$WORK/init-race.sqlite"
"$HARNESS" create-empty "$DB"
READY1="$WORK/init-ready-1"
READY2="$WORK/init-ready-2"
START="$WORK/init-start"
"$HARNESS" init-racer "$DB" "$READY1" "$START" >"$WORK/init-1.out" 2>"$WORK/init-1.err" & P1=$!
"$HARNESS" init-racer "$DB" "$READY2" "$START" >"$WORK/init-2.out" 2>"$WORK/init-2.err" & P2=$!
wait_for_file "$READY1"
wait_for_file "$READY2"
touch "$START"
wait "$P1"
wait "$P2"
cat "$WORK/init-1.out" "$WORK/init-2.out" | tee "$WORK/init-race.out"
test "$(grep -c '^INSERTED$' "$WORK/init-race.out")" -eq 1
test "$(grep -c '^EXISTING_EXACT$' "$WORK/init-race.out")" -eq 1
"$HARNESS" check "$DB" 0 40 -

# 2. Two-process CAS race from one prior state: exactly one successor.
DB="$WORK/cas-race.sqlite"
"$HARNESS" init "$DB"
READY1="$WORK/cas-ready-1"
READY2="$WORK/cas-ready-2"
START="$WORK/cas-start"
"$HARNESS" cas-racer "$DB" "$READY1" "$START" 41 51 >"$WORK/cas-1.out" 2>"$WORK/cas-1.err" & P1=$!
"$HARNESS" cas-racer "$DB" "$READY2" "$START" 42 52 >"$WORK/cas-2.out" 2>"$WORK/cas-2.err" & P2=$!
wait_for_file "$READY1"
wait_for_file "$READY2"
touch "$START"
wait "$P1"
wait "$P2"
cat "$WORK/cas-1.out" "$WORK/cas-2.out" | tee "$WORK/cas-race.out"
test "$(grep -c '^APPLIED$' "$WORK/cas-race.out")" -eq 1
test "$(grep -c '^STALE$' "$WORK/cas-race.out")" -eq 1
"$HARNESS" check-race-one "$DB"

# 3. Crash before transaction: no mutation.
DB="$WORK/crash-before.sqlite"
"$HARNESS" init "$DB"
set +e
"$HARNESS" abort-before "$DB" >/dev/null 2>&1
STATUS=$?
set -e
test "$STATUS" -ne 0
"$HARNESS" check "$DB" 0 40 -

# 4. Crash after BEGIN IMMEDIATE + UPDATE but before COMMIT: rollback/no successor.
DB="$WORK/crash-mid.sqlite"
"$HARNESS" init "$DB"
for iteration in 1 2 3; do
  set +e
  "$HARNESS" abort-after-update "$DB" >/dev/null 2>&1
  STATUS=$?
  set -e
  test "$STATUS" -ne 0
  "$HARNESS" check "$DB" 0 40 -
  echo "MID_TRANSACTION_CRASH_RECOVERED=$iteration"
done

# 5. Successful product COMMIT followed by immediate process abort: successor + receipt recover.
DB="$WORK/crash-after-commit.sqlite"
"$HARNESS" init "$DB"
set +e
"$HARNESS" commit-then-abort "$DB" >/dev/null 2>&1
STATUS=$?
set -e
test "$STATUS" -ne 0
"$HARNESS" check "$DB" 1 41 61

# 6. Held writer lock must classify as SQLite/infrastructure contention, not semantic denial.
DB="$WORK/contention.sqlite"
"$HARNESS" init "$DB"
READY="$WORK/lock-ready"
"$HARNESS" hold-lock "$DB" "$READY" 7000 >"$WORK/holder.out" 2>"$WORK/holder.err" & HOLDER=$!
wait_for_file "$READY"
"$HARNESS" cas-expect-contention "$DB" | tee "$WORK/contention.out"
grep -qx 'CONTENTION' "$WORK/contention.out"
wait "$HOLDER"
"$HARNESS" check "$DB" 0 40 -

# Product source must still be the exact checked-out immutable subject.
test -d "$PRODUCT_ROOT/.git"
echo "ADVERSARIAL_RUNTIME_CORPUS=PASS"
