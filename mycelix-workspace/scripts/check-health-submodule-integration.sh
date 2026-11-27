#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
repo_root="$(cd "$workspace_dir/.." && pwd -P)"
fail() {
  printf 'HEALTH INTEGRATION CHECK: FAIL: %s\n' "$*" >&2
  exit 1
}

gitmodules="$repo_root/.gitmodules"
[[ -f "$gitmodules" ]] || fail "missing root .gitmodules"

submodule_path="$(git -C "$repo_root" config -f "$gitmodules" --get submodule.mycelix-health.path || true)"
submodule_url="$(git -C "$repo_root" config -f "$gitmodules" --get submodule.mycelix-health.url || true)"
[[ "$submodule_path" == "mycelix-health" ]] || fail "expected submodule path mycelix-health, got: ${submodule_path:-<missing>}"
[[ "$submodule_url" == "https://github.com/Luminous-Dynamics/mycelix-health.git" ]] || fail "unexpected Health submodule URL: ${submodule_url:-<missing>}"

health_link="$workspace_dir/happs/health"
[[ -L "$health_link" ]] || fail "happs/health must be a symlink to the canonical root submodule"
link_target="$(readlink "$health_link")"
[[ "$link_target" == "../../mycelix-health" ]] || fail "happs/health must use ../../mycelix-health, got: $link_target"

[[ -d "$health_link" ]] || fail "Health submodule is not initialized at $repo_root/mycelix-health"
linked_real="$(cd -P "$health_link" && pwd -P)"
submodule_real="$(cd -P "$repo_root/mycelix-health" && pwd -P)"
[[ "$linked_real" == "$submodule_real" ]] || fail "happs/health does not resolve to the root submodule"

for required in Cargo.toml dna/dna.yaml happ.yaml; do
  [[ -f "$submodule_real/$required" ]] || fail "missing mycelix-health/$required"
done

pinned_sha="$(git -C "$repo_root" ls-tree HEAD -- mycelix-health | awk '{print $3}')"
[[ "$pinned_sha" =~ ^[0-9a-f]{40}$ ]] || fail "root HEAD does not contain a gitlink for mycelix-health"
checkout_sha="$(git -C "$submodule_real" rev-parse HEAD)"
[[ "$checkout_sha" == "$pinned_sha" ]] || fail "submodule checkout $checkout_sha differs from parent pin $pinned_sha"

manifest="$workspace_dir/happs/mycelix-unified-happ.yaml"
[[ -f "$manifest" ]] || fail "missing unified hApp manifest"
grep -Fq -- '  - name: health' "$manifest" || fail "unified hApp manifest has no health role"
grep -Fq -- '      path: ./health/dna/health.dna' "$manifest" || fail "health role does not reference ./health/dna/health.dna"

printf 'HEALTH INTEGRATION CHECK: PASS\n'
printf '  submodule: %s\n' "$checkout_sha"
printf '  link:      %s -> %s\n' "$health_link" "$link_target"
printf '  manifest:  %s\n' "$manifest"
