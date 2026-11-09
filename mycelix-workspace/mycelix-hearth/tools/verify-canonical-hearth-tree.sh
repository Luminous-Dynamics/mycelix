#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../" && pwd)"
cd "$ROOT"

# The canonical tree itself is excluded. This guard looks only for references
# from other repository content into the legacy root-level Hearth tree.
legacy=0

check() {
  local pattern="$1"
  if git grep -nE -- "$pattern" -- ':!mycelix-hearth/**' ':!mycelix-workspace/mycelix-hearth/**' >/tmp/hearth-legacy-refs.txt 2>/dev/null; then
    echo "FAIL: legacy Hearth reference detected: $pattern"
    cat /tmp/hearth-legacy-refs.txt
    legacy=1
  fi
}

check '(^|[^/[:alnum:]_-])mycelix-hearth/'
check '(^|[[:space:]="'"'"''])\./mycelix-hearth/'
check '\.\./mycelix-hearth/'

if [[ "$legacy" -ne 0 ]]; then
  echo
  echo "Canonical Hearth source guard: FAIL"
  exit 1
fi

echo "Canonical Hearth source guard: PASS"
