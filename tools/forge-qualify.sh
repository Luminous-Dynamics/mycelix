#!/usr/bin/env bash
set -Eeuo pipefail

readonly FORGE_RUST_VERSION="1.96.0"
cleanup_files=()
cleanup() {
  if ((${#cleanup_files[@]})); then
    rm -f -- "${cleanup_files[@]}"
  fi
}
trap cleanup EXIT

usage() {
  cat <<'EOF'
Usage: tools/forge-qualify.sh [--check-only | --list | --lane <path>]

Runs the same Forge qualification commands used by GitHub Actions:
  cargo fmt --all -- --check
  cargo clippy --all-targets --all-features -- -D warnings
  cargo test --all-features

Default mode qualifies every lane from tools/forge-qualification-paths.txt.
Qualification mode requires a clean exact Git HEAD and Rust 1.96.0.

Options:
  --check-only   Verify that the local lane manifest exactly matches forge.yml.
  --list         Print the canonical qualification lane paths.
  --lane PATH    Qualify exactly one canonical lane.
  -h, --help     Show this help.
EOF
}

fail() {
  printf 'forge-qualify: %s\n' "$*" >&2
  exit 1
}

command -v git >/dev/null 2>&1 || fail "git is required"
repo_root="$(git rev-parse --show-toplevel 2>/dev/null)" || fail "run inside the Mycelix Git repository"
cd "$repo_root"

readonly manifest="$repo_root/tools/forge-qualification-paths.txt"
readonly workflow="$repo_root/.github/workflows/forge.yml"
[[ -f "$manifest" ]] || fail "missing $manifest"
[[ -f "$workflow" ]] || fail "missing $workflow"

check_matrix() {
  local manifest_sorted workflow_sorted
  manifest_sorted="$(mktemp)"
  workflow_sorted="$(mktemp)"
  cleanup_files+=("$manifest_sorted" "$workflow_sorted")

  if grep -Ev '^[[:space:]]*$' "$manifest" | grep -qvE '^(crates|adapters)/[^[:space:]]+$'; then
    fail "qualification manifest contains a malformed lane path"
  fi

  local manifest_count unique_count
  manifest_count="$(grep -Ec '^(crates|adapters)/[^[:space:]]+$' "$manifest")"
  unique_count="$(grep -E '^(crates|adapters)/[^[:space:]]+$' "$manifest" | sort -u | wc -l | tr -d ' ')"
  [[ "$manifest_count" == "$unique_count" ]] || fail "qualification manifest contains duplicate lanes"

  grep -E '^(crates|adapters)/[^[:space:]]+$' "$manifest" | sort >"$manifest_sorted"
  sed -n '/^[[:space:]]*matrix:/,/^[[:space:]]*defaults:/p' "$workflow" \
    | sed -n 's/^[[:space:]]*path:[[:space:]]*//p' \
    | sed 's/[[:space:]]*$//' \
    | sort >"$workflow_sorted"

  [[ -s "$workflow_sorted" ]] || fail "could not derive Forge matrix paths from $workflow"
  if ! diff -u "$manifest_sorted" "$workflow_sorted"; then
    fail "qualification manifest differs from the GitHub Actions matrix"
  fi

  local lane
  while IFS= read -r lane; do
    [[ -n "$lane" ]] || continue
    [[ -f "$repo_root/$lane/Cargo.toml" ]] || fail "lane has no Cargo.toml: $lane"
  done <"$manifest"
}

mode="all"
selected_lane=""
case "${1:-}" in
  "") ;;
  --check-only)
    [[ $# -eq 1 ]] || fail "--check-only accepts no additional arguments"
    mode="check"
    ;;
  --list)
    [[ $# -eq 1 ]] || fail "--list accepts no additional arguments"
    mode="list"
    ;;
  --lane)
    [[ $# -eq 2 ]] || fail "--lane requires exactly one path"
    mode="lane"
    selected_lane="$2"
    ;;
  -h|--help)
    usage
    exit 0
    ;;
  *)
    usage >&2
    fail "unknown argument: ${1:-}"
    ;;
esac

check_matrix

if [[ "$mode" == "check" ]]; then
  printf 'Forge qualification matrix is synchronized (%s lanes).\n' "$(wc -l <"$manifest" | tr -d ' ')"
  exit 0
fi

if [[ "$mode" == "list" ]]; then
  cat "$manifest"
  exit 0
fi

if [[ "$mode" == "lane" ]] && ! grep -Fxq -- "$selected_lane" "$manifest"; then
  fail "lane is not in the canonical qualification manifest: $selected_lane"
fi

if [[ -n "$(git status --porcelain=v1 --untracked-files=all)" ]]; then
  fail "qualification requires a clean working tree at an exact Git HEAD"
fi
readonly head="$(git rev-parse --verify HEAD)"

cargo_cmd=(cargo)
rustc_cmd=(rustc)
if command -v rustup >/dev/null 2>&1 \
  && rustup toolchain list 2>/dev/null | grep -Eq '^1\.96\.0(-|[[:space:]])'; then
  cargo_cmd=(rustup run "$FORGE_RUST_VERSION" cargo)
  rustc_cmd=(rustup run "$FORGE_RUST_VERSION" rustc)
fi

command -v "${rustc_cmd[0]}" >/dev/null 2>&1 || fail "rustc is required"
command -v "${cargo_cmd[0]}" >/dev/null 2>&1 || fail "cargo is required"

rustc_version="$("${rustc_cmd[@]}" --version)"
cargo_version="$("${cargo_cmd[@]}" --version)"
[[ "$rustc_version" == "rustc $FORGE_RUST_VERSION "* ]] \
  || fail "expected rustc $FORGE_RUST_VERSION, got: $rustc_version"
[[ "$cargo_version" == "cargo $FORGE_RUST_VERSION "* ]] \
  || fail "expected cargo $FORGE_RUST_VERSION, got: $cargo_version"

qualify_lane() {
  local lane="$1"
  printf '\n== Forge qualification: %s ==\n' "$lane"
  (
    cd "$repo_root/$lane"
    "${cargo_cmd[@]}" fmt --all -- --check
    "${cargo_cmd[@]}" clippy --all-targets --all-features -- -D warnings
    "${cargo_cmd[@]}" test --all-features
  )
}

printf 'Forge exact-head qualification\n'
printf '  head:  %s\n' "$head"
printf '  rustc: %s\n' "$rustc_version"
printf '  cargo: %s\n' "$cargo_version"

lane_count=0
if [[ "$mode" == "lane" ]]; then
  qualify_lane "$selected_lane"
  lane_count=1
else
  while IFS= read -r lane; do
    [[ -n "$lane" ]] || continue
    qualify_lane "$lane"
    lane_count=$((lane_count + 1))
  done <"$manifest"
fi

printf '\nForge qualification PASS\n'
printf '  head:  %s\n' "$head"
printf '  lanes: %d\n' "$lane_count"
