#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
[[ $# -eq 1 ]] || { echo "usage: $0 bundle.tar.gz" >&2; exit 2; }
python3 "$ROOT/scripts/verify-promotion-bundle.py" "$1"
