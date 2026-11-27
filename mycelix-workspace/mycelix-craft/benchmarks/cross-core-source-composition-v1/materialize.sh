#!/usr/bin/env bash
set -euo pipefail

: "${1:?usage: materialize.sh OUTPUT_DIRECTORY}"
OUT_DIR="$1"
REMOTE="${MYCELIX_GIT_REMOTE:-origin}"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
MANIFEST="$SCRIPT_DIR/components.tsv"
REPO_ROOT="$(git rev-parse --show-toplevel)"

if [[ -e "$OUT_DIR" ]] && find "$OUT_DIR" -mindepth 1 -maxdepth 1 -print -quit | grep -q .; then
  printf 'Refusing to materialize over a non-empty directory: %s\n' "$OUT_DIR" >&2
  exit 2
fi
mkdir -p "$OUT_DIR/crates"
: > "$OUT_DIR/source-receipt.tsv"

count=0
declare -A seen_crates=()
while IFS=$'\t' read -r pr branch expected_commit crate expected_manifest_blob expected_lib_blob; do
  [[ -z "${pr:-}" || "$pr" == \#* ]] && continue
  [[ -n "${branch:-}" && -n "${expected_commit:-}" && -n "${crate:-}" ]] || {
    printf 'Malformed source manifest row for PR %s\n' "${pr:-unknown}" >&2
    exit 2
  }
  [[ -z "${seen_crates[$crate]+x}" ]] || {
    printf 'Duplicate crate entry: %s\n' "$crate" >&2
    exit 2
  }
  seen_crates["$crate"]=1

  printf 'Fetching PR #%s source branch %s\n' "$pr" "$branch"
  git -C "$REPO_ROOT" fetch --no-tags "$REMOTE" "refs/heads/$branch"
  actual_commit="$(git -C "$REPO_ROOT" rev-parse FETCH_HEAD)"
  if [[ "$actual_commit" != "$expected_commit" ]]; then
    printf 'SOURCE DRIFT for PR #%s: expected %s, fetched %s\n' \
      "$pr" "$expected_commit" "$actual_commit" >&2
    exit 1
  fi

  manifest_path="mycelix-workspace/mycelix-craft/crates/$crate/Cargo.toml"
  source_path="mycelix-workspace/mycelix-craft/crates/$crate/src/lib.rs"
  actual_manifest_blob="$(git -C "$REPO_ROOT" rev-parse "$expected_commit:$manifest_path")"
  actual_lib_blob="$(git -C "$REPO_ROOT" rev-parse "$expected_commit:$source_path")"
  if [[ "$actual_manifest_blob" != "$expected_manifest_blob" ]]; then
    printf 'MANIFEST BLOB DRIFT for PR #%s: expected %s, got %s\n' \
      "$pr" "$expected_manifest_blob" "$actual_manifest_blob" >&2
    exit 1
  fi
  if [[ "$actual_lib_blob" != "$expected_lib_blob" ]]; then
    printf 'SOURCE BLOB DRIFT for PR #%s: expected %s, got %s\n' \
      "$pr" "$expected_lib_blob" "$actual_lib_blob" >&2
    exit 1
  fi

  destination="$OUT_DIR/crates/$crate"
  mkdir -p "$destination/src"
  git -C "$REPO_ROOT" show "$expected_commit:$manifest_path" > "$destination/Cargo.toml"
  git -C "$REPO_ROOT" show "$expected_commit:$source_path" > "$destination/src/lib.rs"

  printf '%s\t%s\t%s\t%s\t%s\t%s\n' \
    "$pr" "$branch" "$expected_commit" "$crate" \
    "$expected_manifest_blob" "$expected_lib_blob" >> "$OUT_DIR/source-receipt.tsv"
  count=$((count + 1))
done < "$MANIFEST"

if [[ "$count" -ne 6 ]]; then
  printf 'Expected exactly six component cores, materialized %s\n' "$count" >&2
  exit 1
fi

# The harness source belongs to the exact harness commit already checked out
# by CI. Preserve it as a distinct workspace member, not as a seventh owner.
HARNESS_SRC="$SCRIPT_DIR/harness"
test -s "$HARNESS_SRC/Cargo.toml"
test -s "$HARNESS_SRC/src/lib.rs"
mkdir -p "$OUT_DIR/harness/src"
cp "$HARNESS_SRC/Cargo.toml" "$OUT_DIR/harness/Cargo.toml"
cp "$HARNESS_SRC/src/lib.rs" "$OUT_DIR/harness/src/lib.rs"
sha256sum "$HARNESS_SRC/Cargo.toml" "$HARNESS_SRC/src/lib.rs" \
  | awk '{print "harness_source_sha256\\t" $1}' >> "$OUT_DIR/source-receipt.tsv"

cat > "$OUT_DIR/Cargo.toml" <<'TOML'
[workspace]
resolver = "2"
members = [
  "crates/need-intake-v1",
  "crates/project-formation-v1",
  "crates/claim-support-v1",
  "crates/decision-closure-v1",
  "crates/evidence-lineage-v1",
  "crates/impact-qualification-v1",
  "harness",
]
TOML

printf 'MYCELIX-CRAFT-CROSS-CORE-SOURCE-COMPOSITION-V1\n' > "$OUT_DIR/profile.txt"
printf 'harness_head\t%s\n' "$(git -C "$REPO_ROOT" rev-parse HEAD)" >> "$OUT_DIR/source-receipt.tsv"
printf 'component_count\t%s\n' "$count" >> "$OUT_DIR/source-receipt.tsv"
sha256sum "$MANIFEST" | awk '{print "source_manifest_sha256\t" $1}' >> "$OUT_DIR/source-receipt.tsv"

printf '\nMaterialized %s exact component revisions into %s\n' "$count" "$OUT_DIR"
cat "$OUT_DIR/source-receipt.tsv"
