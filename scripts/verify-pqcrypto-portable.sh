#!/usr/bin/env sh
set -eu

ROOT="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
cd "$ROOT"

check_no_avx2() {
    manifest="$1"
    features="$2"
    echo "=== $manifest :: --features $features ==="
    tree="$(cargo tree --manifest-path "$manifest" --features "$features" --edges features --invert pqcrypto-dilithium)"
    printf '%s\n' "$tree"
    if printf '%s\n' "$tree" | grep -Fq 'feature "avx2"'; then
        echo "ERROR: pqcrypto-dilithium AVX2 feature is active" >&2
        return 1
    fi
    if ! printf '%s\n' "$tree" | grep -Fq 'pqcrypto-dilithium'; then
        echo "ERROR: expected pqcrypto-dilithium dependency was not resolved" >&2
        return 1
    fi
}

# Force each optional consumer path to be present in the resolved feature graph.
check_no_avx2 "crates/mycelix-zkp-core/Cargo.toml" "full"
check_no_avx2 "mycelix-identity/crates/mycelix-crypto/Cargo.toml" "native"
check_no_avx2 "mycelix-core/libs/rb-bft-consensus/Cargo.toml" "pq"
