#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
python_impl="$root/scripts/accounting_wire_conformance.py"

require() {
  local pattern="$1"
  local message="$2"
  if ! grep -Fq -- "$pattern" "$python_impl"; then
    echo "accounting wire Python conformance invariant failed: $message" >&2
    exit 1
  fi
}

forbid() {
  local pattern="$1"
  local message="$2"
  if grep -Fq -- "$pattern" "$python_impl"; then
    echo "accounting wire Python conformance invariant failed: $message" >&2
    exit 1
  fi
}

require 'WIRE_FORMAT = "mycelix-accounting-wire"' "Python implementation must pin the wire format"
require 'WIRE_VERSION = 1' "Python implementation must pin wire version 1"
require 'WIRE_DIGEST_DOMAIN = b"mycelix-accounting-wire-v1\0"' "Python implementation must use the same digest domain"
require 'struct.pack(">d", value).hex()' "Python float encoding must use big-endian IEEE-754 binary64 bits"
require 'struct.unpack(">d", bytes.fromhex(value))[0]' "Python float decoding must use big-endian IEEE-754 binary64 bits"
require '.encode("utf-8", errors="strict")' "Python key ordering must use strict UTF-8 bytes"
require 'must not contain lone UTF-16 surrogates' "Python strings must reject non-scalar surrogate sequences"
require 'json.dumps(value, ensure_ascii=False, separators=(",", ":"), allow_nan=False)' "Python canonical JSON must be byte-stable"
require 'dba4eb609b3466d6b42b49fa360390559fc755955012855453a31c7c365c556f' "UTF-8-vs-UTF-16 golden digest must remain fixed"
require 'a0b055b121a7e855f6eb22fec26eec04b847653c9eed355f6f9bbb2c7b3a0c26' "null golden digest must remain fixed"
require 'run_conformance()' "Python implementation must have an executable self-test"
require 'accounting wire Python stdlib conformance: PASS' "Python conformance success marker must remain explicit"

forbid 'import subprocess' "Python conformance must not import subprocess"
forbid 'from subprocess' "Python conformance must not import subprocess helpers"
forbid 'import os' "Python conformance must not gain shell/process escape hatches"
forbid 'from os' "Python conformance must not gain shell/process escape hatches"
forbid 'import importlib' "Python conformance must not dynamically load the reference implementation"
forbid 'from importlib' "Python conformance must not dynamically load the reference implementation"
forbid 'import requests' "Python conformance must remain standard-library and offline"
forbid 'import urllib' "Python conformance must remain offline"
forbid 'accounting-wire.js' "Python conformance must not import the JavaScript reference implementation"
forbid 'npm ' "Python conformance must not invoke npm"

echo "accounting wire Python conformance source invariants: PASS"
