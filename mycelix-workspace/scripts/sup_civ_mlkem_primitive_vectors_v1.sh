#!/usr/bin/env bash
set -euo pipefail

PARENT_SCRIPT='mycelix-workspace/scripts/sup_civ_openssl_364_mlkem_preflight_v1.sh'
EXPECTED_PUBLIC='c23e23dd3d485a9256cda09358a4a286e00b373db10761eadf99f710649ca31c'
EXPECTED_KEMCT='39826fe40dc54a3fef68b7c228ed8fb22931012b6fa3bd3e7f204d54db0ac1e1'
EXPECTED_SHARED='0118707cb4fee1ea9004263262448f62d25696336983f298091637c25f1a12dd'

LOG="${RUNNER_TEMP:-/tmp}/sup-civ-000d1c2a1.log"
rm -f "$LOG"

if ! bash "$PARENT_SCRIPT" >"$LOG" 2>&1; then
  cat "$LOG"
  exit 1
fi

cat "$LOG"

grep -Fx "C2A: public-key-sha256=$EXPECTED_PUBLIC" "$LOG"
grep -Fx "C2A: kemct-sha256=$EXPECTED_KEMCT" "$LOG"
grep -Fx "C2A: shared-secret-sha256=$EXPECTED_SHARED" "$LOG"
grep -Fx 'C2A PASS: pinned OpenSSL 3.6.4 ML-KEM/CMS tooling preflight completed' "$LOG"

printf 'SUP-CIV-000D1C2A1 PASS: exact deterministic ML-KEM-768 primitive hashes reproduced\n'
