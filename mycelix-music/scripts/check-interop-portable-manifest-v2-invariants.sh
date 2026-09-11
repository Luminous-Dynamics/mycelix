#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
manifest="$root/packages/accounting/src/interop-portable-manifest-v2.ts"
test_file="$root/packages/accounting/src/interop-portable-manifest-v2.test.ts"
index="$root/packages/accounting/src/index.ts"

require() {
  local file="$1"
  local pattern="$2"
  local message="$3"
  if ! grep -Fq -- "$pattern" "$file"; then
    echo "portable manifest v2 invariant failed: $message" >&2
    exit 1
  fi
}

forbid() {
  local file="$1"
  local pattern="$2"
  local message="$3"
  if grep -Fq -- "$pattern" "$file"; then
    echo "portable manifest v2 invariant failed: $message" >&2
    exit 1
  fi
}

require "$manifest" "PORTABLE_REPORTING_MANIFEST_V2_PROTOCOL" "manifest v2 must have an explicit protocol identifier"
require "$manifest" "accountingWireDigestFromText" "manifest must bind exact canonical wire bytes"
require "$manifest" "buildMerkleCommitmentV2([payload])" "manifest must bind the complete decoded payload with Merkle v2"
require "$manifest" "accounting_merkle_v1_legacy" "legacy bundle root must be labeled as historical Merkle v1"
require "$manifest" "accounting_merkle_v2_wire" "portable payload root must be labeled as Merkle v2 over wire semantics"
require "$manifest" "legacy_v1_domain_verification_required" "manifest must explicitly preserve nested legacy verification requirement"
require "$manifest" "portable_reporting_evidence_manifest_v2" "manifest root must be domain/record separated"
require "$index" "export * from './interop-portable-manifest-v2.js';" "portable manifest v2 API must be exported"

forbid "$manifest" "issuer_attested" "transport manifest must not pretend the new root is an issuer signature"
forbid "$manifest" "ledger_authority" "transport manifest must not promote reporting evidence to ledger authority"

require "$test_file" "a4623f5c498ad7b5e753f0cb086f162a25a26b80c8558b4c7572c8cea5bf9f98" "wire digest vector must remain fixed"
require "$test_file" "c6a04ad0b236d62e925170e2564a974a577aa33d2884a6c128504b25a0a6d9a4" "portable payload v2 root vector must remain fixed"
require "$test_file" "e0c209f261a098dc764fe7fb40b4e8bd3f4cd3d39e61b3d5bbb4dd6f2f3aefb1" "manifest v2 root vector must remain fixed"
require "$test_file" "without rewriting the v1 bundle root" "legacy root preservation regression must remain"
require "$test_file" "changes the portable v2 commitment when payload contents change" "payload sensitivity regression must remain"
require "$test_file" "rejects manifest tampering and extra fields" "manifest tamper regression must remain"

echo "portable dual-commitment manifest v2 source invariants: PASS"
