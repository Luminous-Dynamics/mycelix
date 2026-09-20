#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

EXPECTED_RUSTC_PREFIX="rustc 1.96.0 "
MAX_WASM_BYTES="${MAX_WASM_BYTES:-8388608}"
EVIDENCE_DIR="${EVIDENCE_DIR:-qualification-evidence/care-recurrence}"
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

# Hearth intentionally does not currently commit a workspace Cargo.lock. Resolve
# exactly once for this qualification run, preserve the lock as evidence, and
# require every subsequent Cargo invocation to use that immutable resolution.
cargo generate-lockfile
cp Cargo.lock "$EVIDENCE_DIR/Cargo.lock"
sha256sum Cargo.lock | tee "$EVIDENCE_DIR/cargo-lock.sha256"

packages=(
  hearth-care-recurrence
  hearth-care-timezone-jiff
  hearth-care-recurrence-enumerator
  hearth-care-recurrence-composer
  hearth-care-recurrence-state
  hearth-care-occurrence-admission
  hearth-recurrence-wasm-probe
)

fmt_args=()
for package in "${packages[@]}"; do
  fmt_args+=( -p "$package" )
done
cargo fmt --check "${fmt_args[@]}"

# Run each package separately to avoid accidental cross-package feature
# unification hiding a package-local problem.
for package in "${packages[@]}"; do
  echo "=== native test: $package ==="
  cargo test --locked -p "$package" -- --test-threads=1
done

# Repeat the time-sensitive layers against the same process-independent inputs.
# This catches accidental dependence on wall clock, host timezone, or mutable
# process-global timezone state that a single run could miss.
for pass in 1 2; do
  echo "=== deterministic replay pass $pass: timezone ==="
  cargo test --locked -p hearth-care-timezone-jiff -- --test-threads=1
  echo "=== deterministic replay pass $pass: composer ==="
  cargo test --locked -p hearth-care-recurrence-composer -- --test-threads=1
  echo "=== deterministic replay pass $pass: linked probe ==="
  cargo test --locked -p hearth-recurrence-wasm-probe -- --test-threads=1
done

# Preserve the exact resolved feature/dependency surface. Exact Jiff versions
# are additionally enforced by the crate manifests and unit tests.
cargo tree --locked -e features -p hearth-care-timezone-jiff \
  | tee "$EVIDENCE_DIR/timezone-feature-tree.txt"
grep -F 'jiff v0.2.37' "$EVIDENCE_DIR/timezone-feature-tree.txt" >/dev/null
grep -F 'jiff-tzdb v0.1.8' "$EVIDENCE_DIR/timezone-feature-tree.txt" >/dev/null

# Compile the pure state/admission theorem for wasm32 and perform a real final
# WASM link that pulls in the bundled timezone DB plus authoritative composer.
cargo check --locked --target wasm32-unknown-unknown \
  -p hearth-care-recurrence-state \
  -p hearth-care-occurrence-admission
cargo build --locked --release --target wasm32-unknown-unknown \
  -p hearth-recurrence-wasm-probe

wasm="target/wasm32-unknown-unknown/release/hearth_recurrence_wasm_probe.wasm"
if [[ ! -f "$wasm" ]]; then
  echo "expected WASM probe not found: $wasm" >&2
  exit 1
fi

wasm_bytes="$(wc -c < "$wasm" | tr -d '[:space:]')"
if (( wasm_bytes > MAX_WASM_BYTES )); then
  echo "WASM probe exceeds qualification ceiling: ${wasm_bytes} > ${MAX_WASM_BYTES}" >&2
  exit 1
fi
printf '%s\n' "$wasm_bytes" | tee "$EVIDENCE_DIR/wasm-size-bytes.txt"
printf '%s\n' "$MAX_WASM_BYTES" > "$EVIDENCE_DIR/wasm-size-limit-bytes.txt"
sha256sum "$wasm" | tee "$EVIDENCE_DIR/wasm.sha256"

cat > "$EVIDENCE_DIR/manifest.txt" <<EOF
subject_sha=$subject_sha
rustc=$rustc_version
cargo=$cargo_version
wasm_bytes=$wasm_bytes
wasm_limit_bytes=$MAX_WASM_BYTES
qualification=PASS
EOF

echo "Hearth Care recurrence qualification PASS for $subject_sha"
