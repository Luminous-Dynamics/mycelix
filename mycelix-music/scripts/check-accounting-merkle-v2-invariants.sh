#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ts_impl="$root/packages/accounting/src/merkle-v2.ts"
ts_test="$root/packages/accounting/src/merkle-v2.test.ts"
python_impl="$root/scripts/accounting_merkle_v2_conformance.py"
index="$root/packages/accounting/src/index.ts"

require() {
  local file="$1"
  local pattern="$2"
  local message="$3"
  if ! grep -Fq -- "$pattern" "$file"; then
    echo "accounting Merkle v2 invariant failed: $message" >&2
    exit 1
  fi
}

forbid() {
  local file="$1"
  local pattern="$2"
  local message="$3"
  if grep -Fq -- "$pattern" "$file"; then
    echo "accounting Merkle v2 invariant failed: $message" >&2
    exit 1
  fi
}

require "$ts_impl" "ACCOUNTING_MERKLE_V2_PROTOCOL = 'mycelix-accounting-merkle-v2'" "Merkle v2 must have an explicit protocol identifier"
require "$ts_impl" "mycelix-accounting-merkle-leaf-v2\\0" "leaf hashing must be domain separated"
require "$ts_impl" "mycelix-accounting-merkle-node-v2\\0" "node hashing must be domain separated"
require "$ts_impl" "mycelix-accounting-merkle-empty-v2\\0" "empty-tree hashing must be domain separated"
require "$ts_impl" "serializeAccountingWire(value)" "Merkle v2 leaves must hash canonical accounting-wire bytes"
require "$ts_impl" "Buffer.from(value, 'hex')" "Merkle v2 nodes must consume raw child digest bytes"
require "$ts_impl" "protocolVersion: 2" "Merkle v2 commitments and proofs must be explicitly versioned"
require "$index" "export * from './merkle-v2.js';" "Merkle v2 API must be publicly exported"

forbid "$ts_impl" "canonicalAccountingValue" "Merkle v2 must not reuse JavaScript-specific Merkle v1 canonicalization"
forbid "$ts_impl" "JSON.stringify" "Merkle v2 must not bypass accounting-wire canonicalization"
forbid "$ts_impl" "Number.toString" "Merkle v2 must not depend on ECMAScript number formatting"

require "$ts_test" "c3f59a327d432c5ca62e0c83780a2f7b4979d2d67cec42949f69ca0312ba5d5e" "empty-tree golden root must remain fixed"
require "$ts_test" "1ae6b649185c96953b157f9226fb286a45056f03e7f682c2fa268aa958f63531" "odd-width UTF-8 golden root must remain fixed"
require "$ts_test" "keeps historical Merkle v1 roots distinct" "Merkle v2 must not silently rewrite v1 authority roots"
require "$ts_test" "rejects leaf, sibling, side and root tampering" "Merkle v2 proof attacks must remain covered"

require "$python_impl" "from accounting_wire_conformance import serialize_wire" "Python Merkle v2 must build on the independent Python wire implementation"
require "$python_impl" "mycelix-accounting-merkle-leaf-v2\\0" "Python leaf domain must match v2"
require "$python_impl" "mycelix-accounting-merkle-node-v2\\0" "Python node domain must match v2"
require "$python_impl" "mycelix-accounting-merkle-empty-v2\\0" "Python empty domain must match v2"
require "$python_impl" "c3f59a327d432c5ca62e0c83780a2f7b4979d2d67cec42949f69ca0312ba5d5e" "Python empty-tree root must match the TypeScript vector"
require "$python_impl" "1ae6b649185c96953b157f9226fb286a45056f03e7f682c2fa268aa958f63531" "Python odd-tree root must match the TypeScript vector"
require "$python_impl" "accounting Merkle v2 Python stdlib conformance: PASS" "Python Merkle v2 conformance must be executable"

forbid "$python_impl" "subprocess" "Python Merkle v2 must not delegate to Node or shell processes"
forbid "$python_impl" "accounting-wire.js" "Python Merkle v2 must not import the JavaScript wire implementation"

echo "accounting Merkle v2 source invariants: PASS"
