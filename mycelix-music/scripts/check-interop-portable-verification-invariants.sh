#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
portable="$root/packages/accounting/src/interop-portable-verification.ts"
test_file="$root/packages/accounting/src/interop-portable-verification.test.ts"
index="$root/packages/accounting/src/index.ts"

require() {
  local file="$1"
  local pattern="$2"
  local message="$3"
  if ! grep -Fq -- "$pattern" "$file"; then
    echo "portable reporting verification invariant failed: $message" >&2
    exit 1
  fi
}

forbid() {
  local file="$1"
  local pattern="$2"
  local message="$3"
  if grep -Fq -- "$pattern" "$file"; then
    echo "portable reporting verification invariant failed: $message" >&2
    exit 1
  fi
}

require "$portable" "PORTABLE_REPORTING_EVIDENCE_PROTOCOL = 'mycelix-reporting-evidence-portable-v1'" "portable bundle must be explicitly versioned"
require "$portable" "accountingWireDigestFromText" "verification must bind exact canonical wire bytes"
require "$portable" "deserializeAccountingWire" "verification must start from canonical wire decoding"
require "$portable" "verifyEconomicAuditCapsuleAttestationWithAnchoredIssuerTrust" "portable verification must replay root-attested audit issuer trust"
require "$portable" "verifyReportingEvidencePackageAttestationWithAnchoredIssuerTrust" "portable verification must replay root-attested reporting issuer trust"
require "$portable" "auditIssuerRoot" "audit root policy must be supplied separately from the wire artifact"
require "$portable" "reportingIssuerRoot" "reporting root policy must be supplied separately from the wire artifact"
require "$portable" "portable_external_root_policy_v1" "portable receipts must commit exact external root policies"
require "$portable" "auditRootPolicyRoot" "portable receipt must bind the exact audit root policy"
require "$portable" "reportingRootPolicyRoot" "portable receipt must bind the exact reporting root policy"
require "$portable" "assertAuditReference" "portable verification must cross-check report audit provenance against verified capsule authority"
require "$portable" "portable reporting disclosure sidecar does not match verified audit capsule" "sidecar must be cross-bound to the verified audit capsule"
require "$portable" "portable_reporting_evidence_verification_receipt_v1" "portable verification must emit a domain-separated receipt"
require "$portable" "issuer_attested_compiler_derived_merkle_not_zero_knowledge" "portable receipt must preserve the line-proof authority limit"
require "$portable" "VERIFIED_PORTABLE_REPORTING_EVIDENCE" "verified portable authorities must carry runtime provenance"
require "$index" "export * from './interop-portable-verification.js';" "portable verification API must be exported"

forbid "$portable" "privateKey" "portable verification must not custody signing keys"
forbid "$portable" "sign(" "portable verification must not create signatures"

require "$test_file" "replays both root-attested trust chains from canonical wire bytes" "full portable trust-chain replay regression must remain"
require "$test_file" "not.toContain(auditRootPublicKeyPem)" "audit root policy must remain external to the wire artifact"
require "$test_file" "not.toContain(reportingRootPublicKeyPem)" "reporting root policy must remain external to the wire artifact"
require "$test_file" "rejects transport-level top-level field injection" "wire-shape injection regression must remain"
require "$test_file" "rejects package content tampering" "package tampering regression must remain"
require "$test_file" "requires the separately pinned audit root key" "audit root pinning regression must remain"
require "$test_file" "enforces reporting anti-rollback floor outside the wire artifact" "reporting anti-rollback regression must remain"
require "$test_file" "does not let structural clones regain portable verification authority" "runtime provenance regression must remain"

echo "portable reporting verification source invariants: PASS"
