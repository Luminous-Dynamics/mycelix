#!/usr/bin/env bash
set -euo pipefail

fixture="${1:-tools/finance/sap-payment-saga-go-verifier/fixtures/sap-payment-journal-checkpoint-v1.kat}"
command -v awk >/dev/null
command -v xxd >/dev/null
command -v sha256sum >/dev/null
test -f "$fixture"

value_for() {
  local key="$1"
  awk -F= -v target="$key" '
    $1 == target { count++; value = $2 }
    END {
      if (count != 1 || value == "") exit 1
      print value
    }
  ' "$fixture"
}

verify_vector() {
  local byte_key="$1"
  local digest_key="$2"
  local bytes expected actual

  bytes="$(value_for "$byte_key")"
  expected="$(value_for "$digest_key")"
  if [[ ! "$bytes" =~ ^([[:xdigit:]]{2})+$ ]]; then
    printf 'invalid hex bytes for %s\n' "$byte_key" >&2
    return 1
  fi
  actual="$(printf '%s' "$bytes" | xxd -r -p | sha256sum | awk '{ print "sha256:" $1 }')"
  if [[ "$actual" != "$expected" ]]; then
    printf '%s mismatch: got %s, expected %s\n' "$byte_key" "$actual" "$expected" >&2
    return 1
  fi
  printf 'PASS %s -> %s\n' "$byte_key" "$actual"
}

verify_vector subject-bytes-hex subject-digest
verify_vector intent-bytes-hex intent-digest
