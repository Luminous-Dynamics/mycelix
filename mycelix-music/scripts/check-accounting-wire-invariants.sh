#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
wire="$root/packages/accounting/src/accounting-wire.ts"
test_file="$root/packages/accounting/src/accounting-wire.test.ts"
index="$root/packages/accounting/src/index.ts"

require() {
  local file="$1"
  local pattern="$2"
  local message="$3"
  if ! grep -Fq -- "$pattern" "$file"; then
    echo "accounting wire invariant failed: $message" >&2
    exit 1
  fi
}

forbid() {
  local file="$1"
  local pattern="$2"
  local message="$3"
  if grep -Fq -- "$pattern" "$file"; then
    echo "accounting wire invariant failed: $message" >&2
    exit 1
  fi
}

require "$wire" "ACCOUNTING_WIRE_FORMAT = 'mycelix-accounting-wire'" "wire protocol must have an explicit format tag"
require "$wire" "ACCOUNTING_WIRE_VERSION = 1" "wire protocol must be versioned"
require "$wire" "mycelix-accounting-wire-v1\\0" "wire digest must be domain separated"
require "$wire" "['integer', value.toString(10)]" "arbitrary-precision integers must use canonical decimal text"
require "$wire" "BINARY64_HEX = /^[0-9a-f]{16}$/" "finite numbers must use fixed-width lowercase IEEE-754 binary64 hex"
require "$wire" "writeDoubleBE" "binary64 encoding must use explicit big-endian bytes"
require "$wire" "readDoubleBE" "binary64 decoding must reconstruct exact big-endian bytes"
require "$wire" "compareUtf8Keys" "object-key ordering must be explicit lexicographic UTF-8 byte ordering"
require "$wire" "Buffer.compare(Buffer.from(left, 'utf8'), Buffer.from(right, 'utf8'))" "UTF-8 key ordering must be byte-defined"
require "$wire" "canonicalUnicodeString" "wire strings and keys must reject non-scalar surrogate sequences"
require "$wire" "must not contain lone UTF-16 surrogates" "lone surrogate rejection must remain explicit"
require "$wire" "accounting wire object keys must be strictly sorted and unique by UTF-8 bytes" "wire objects must have canonical key order"
require "$wire" "accounting wire text is not in canonical serialized form" "wire parser must reject alternate JSON spellings"
require "$wire" "accountingWireDigestFromText" "canonical serialized bytes must have a stable digest"
require "$index" "export * from './accounting-wire.js';" "wire API must be publicly exported"

forbid "$wire" "Number.toString" "wire number canonicality must not depend on ECMAScript decimal formatting"
forbid "$wire" "Object.keys(record).sort()" "wire key canonicality must not depend on JavaScript UTF-16 sort order"
forbid "$wire" "JSON.stringify(value)" "raw JSON serialization must not be used as the accounting wire representation"

require "$test_file" "preserves authority digests" "wire round-trip must preserve EconomicAuditCapsule authority digest"
require "$test_file" "matches cross-language golden bytes and wire digests" "cross-language conformance vectors must remain"
require "$test_file" "a0b055b121a7e855f6eb22fec26eec04b847653c9eed355f6f9bbb2c7b3a0c26" "null wire digest vector must remain fixed"
require "$test_file" "52cee7923ac978c5bdd8c0d1d1b25b6c324b4a2845ae96615381793d4d0c7909" "unicode/object-order wire digest vector must remain fixed"
require "$test_file" "dba4eb609b3466d6b42b49fa360390559fc755955012855453a31c7c365c556f" "UTF-8-vs-UTF-16 ordering vector must remain fixed"
require "$test_file" "deterministic across object insertion order" "object-order determinism regression must remain"
require "$test_file" "IEEE-754 numbers without precision loss" "binary64 round-trip regression must remain"
require "$test_file" "8000000000000000" "negative-zero binary64 representation must remain explicit"
require "$test_file" "rejects noncanonical or non-finite binary64 encodings" "binary64 canonicality attacks must remain"
require "$test_file" "rejects noncanonical integer text" "integer canonicality regression must remain"
require "$test_file" "rejects unsorted or duplicate object keys" "object-key canonicality regression must remain"
require "$test_file" "rejects JavaScript UTF-16 key order" "UTF-8 ordering regression must remain"
require "$test_file" "rejects lone UTF-16 surrogates in string values and object keys" "Unicode scalar-value regression must remain"
require "$test_file" "rejects noncanonical JSON bytes" "byte-level canonical JSON regression must remain"

echo "accounting wire source invariants: PASS"
