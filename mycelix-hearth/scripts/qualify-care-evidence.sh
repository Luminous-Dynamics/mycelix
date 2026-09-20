#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

EXPECTED_RUSTC_PREFIX="rustc 1.96.0 "
MAX_WASM_BYTES="${MAX_WASM_BYTES:-8388608}"
EVIDENCE_DIR="${EVIDENCE_DIR:-qualification-evidence/care-evidence}"
rm -rf "$EVIDENCE_DIR"
mkdir -p "$EVIDENCE_DIR"

subject_sha="$(git rev-parse HEAD)"
expected_sha="${EXPECTED_SUBJECT_SHA:-$subject_sha}"
if [[ "$subject_sha" != "$expected_sha" ]]; then
  echo "subject SHA mismatch: expected $expected_sha, got $subject_sha" >&2
  exit 1
fi

rustc_version="$(rustc --version)"
cargo_version="$(cargo --version)"
if [[ "$rustc_version" != "$EXPECTED_RUSTC_PREFIX"* ]]; then
  echo "unexpected Rust toolchain: $rustc_version" >&2
  exit 1
fi
printf '%s\n' "$subject_sha" > "$EVIDENCE_DIR/subject-sha.txt"
printf '%s\n' "$rustc_version" > "$EVIDENCE_DIR/rustc-version.txt"
printf '%s\n' "$cargo_version" > "$EVIDENCE_DIR/cargo-version.txt"

cargo generate-lockfile
cp Cargo.lock "$EVIDENCE_DIR/Cargo.lock"
sha256sum Cargo.lock | tee "$EVIDENCE_DIR/cargo-lock.sha256"

packages=(
  hearth-care-occurrence-time-evidence
  hearth-care-occurrence-authority
  hearth-care-completion-authority
  hearth-care-digest-v4
  hearth-planner-evidence-v4
  hearth-care-evidence-wasm-probe
)

fmt_args=()
for package in "${packages[@]}"; do
  fmt_args+=( -p "$package" )
done
cargo fmt --check "${fmt_args[@]}"

for package in "${packages[@]}"; do
  echo "=== native test: $package ==="
  cargo test --locked -p "$package" -- --test-threads=1
  echo "=== wasm32 check: $package ==="
  cargo check --locked --target wasm32-unknown-unknown -p "$package"
done

: > "$EVIDENCE_DIR/dependency-trees.txt"
for package in "${packages[@]}"; do
  echo "=== $package ===" >> "$EVIDENCE_DIR/dependency-trees.txt"
  cargo tree --locked -p "$package" >> "$EVIDENCE_DIR/dependency-trees.txt"
  echo >> "$EVIDENCE_DIR/dependency-trees.txt"
done

if grep -E '(^|[^[:alnum:]_-])(hdk|hdi) v|holochain_integrity_types|holochain_serialized_bytes' \
    "$EVIDENCE_DIR/dependency-trees.txt" >/dev/null; then
  echo "pure Care evidence stack unexpectedly depends on Holochain runtime/integrity crates" >&2
  exit 1
fi

# A separate package-by-package check cannot prove that the full theorem chain
# final-links together. Build a non-production binary that executes the path:
# occurrence authority -> completion authority -> Digest v4 -> planner v4.
cargo build --locked --release --target wasm32-unknown-unknown \
  -p hearth-care-evidence-wasm-probe

wasm="target/wasm32-unknown-unknown/release/hearth_care_evidence_wasm_probe.wasm"
if [[ ! -f "$wasm" ]]; then
  echo "expected Care evidence WASM probe not found: $wasm" >&2
  exit 1
fi
wasm_bytes="$(wc -c < "$wasm" | tr -d '[:space:]')"
if (( wasm_bytes > MAX_WASM_BYTES )); then
  echo "Care evidence WASM probe exceeds qualification ceiling: ${wasm_bytes} > ${MAX_WASM_BYTES}" >&2
  exit 1
fi
printf '%s\n' "$wasm_bytes" | tee "$EVIDENCE_DIR/wasm-size-bytes.txt"
printf '%s\n' "$MAX_WASM_BYTES" > "$EVIDENCE_DIR/wasm-size-limit-bytes.txt"
sha256sum "$wasm" | tee "$EVIDENCE_DIR/wasm.sha256"

cat > "$EVIDENCE_DIR/manifest.txt" <<EOF
subject_sha=$subject_sha
rustc=$rustc_version
cargo=$cargo_version
pure_holochain_dependency_check=PASS
native_tests=PASS
wasm32_checks=PASS
end_to_end_wasm_link=PASS
wasm_bytes=$wasm_bytes
wasm_limit_bytes=$MAX_WASM_BYTES
qualification=PASS
EOF

echo "Hearth Care evidence qualification PASS for $subject_sha"
