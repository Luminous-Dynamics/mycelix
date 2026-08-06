# Prism Hardening Wave 6 Validation

**Base:** `fd6d2c80728a22c0d1270d8d94c939b107c98b13`
**Implementation head:** `031859c6054ca85da32c59b4f688d895a4cca2a4`
**Implementation tree:** `3e14b0eca2b4248a40f6ae2a8e3e0d784a9a3a2f`
**Implementation patches:** 14
**Implementation delta:** 26 files changed, 4,194 insertions, 101 deletions

## Result

Wave 6 passed every validation gate available in this environment. The ordered mail patch series was generated from the Wave 5 base, applied with `git am` to a fresh clone of the untouched Wave 5 bundle, and produced the exact same Git tree as the hardened working repository.

The implementation remains intentionally **not release-admitted**. Release-mode supply-chain verification fails on exactly one declared blocker: a reviewed `flake.lock` has not yet been generated on a Nix-enabled host.

## Replay validation

- Generated 14 ordered patches from `fd6d2c8..031859c`.
- Applied all patches to a fresh clone of `prism-hardening-wave5.bundle` at the exact Wave 5 base commit.
- Hardened implementation tree: `3e14b0eca2b4248a40f6ae2a8e3e0d784a9a3a2f`.
- Replayed implementation tree: `3e14b0eca2b4248a40f6ae2a8e3e0d784a9a3a2f`.
- Replay worktree was clean.
- `git diff --check fd6d2c8..HEAD` passed.
- `git fsck --full` passed.

## Static and structural validation

- All 76 Rust files produced error-free Tree-sitter syntax trees.
- All 27 TOML files parsed successfully.
- All 33 JSON files parsed successfully.
- The GitHub Actions YAML parsed successfully.
- The workspace contains 18 members.
- `Cargo.lock` contains 836 packages: 23 local and 813 registry packages.
- The tree contains 333 Rust test, Tokio test, and property-test annotations.
- No Python bytecode or `__pycache__` artifacts were present.
- The repository worktree was clean.

## Reproducible security evidence

The following repository-native gates passed against their committed receipts:

- `scripts/verify-wave6-static.py --check security/wave6-static-evidence.json`
- `scripts/verify-supply-chain.py --check security/supply-chain-evidence.json`

The static evidence gate checks the Wave 6 hybrid-review contract, secure trust-file admission, verifier-agent executable measurement, reviewed-index publication, executable admission for Compatibility mode, canonical build-evidence machinery, release attestation machinery, immutable workflow actions, pinned toolchain declarations, and the explicit shared-network degradation policy.

The dependency-admission gate found:

- no Git dependencies;
- no unknown registries;
- no registry package without a checksum;
- no wildcard dependency versions;
- no path dependency escaping the repository;
- no mutable GitHub Action reference.

The committed `actions/checkout` reference is pinned to `11bd71901bbe5b1630ceea73d27597364c9af683`.

## Expected release blocker

Running the supply-chain verifier in release mode returned failure with exactly this blocker:

> `flake.lock has not been generated and reviewed on a Nix-enabled host`

This is deliberate. Wave 6 does not synthesize or claim a Nix lock without executing and reviewing the Nix dependency resolution.

## Validation not performed

The environment did not contain `cargo`, `rustc`, `nix`, or `bwrap`. Consequently, this validation does not claim:

- Rust type checking or compilation;
- Cargo tests, Clippy, Rustfmt, or WASM builds;
- Nix flake evaluation, lock generation, or closure builds;
- live Bubblewrap or Chromium isolation;
- live Spore process execution;
- live Ed25519 or ML-DSA primitive verification by an admitted production agent;
- kernel-mediated Compatibility egress.

Compatibility networking remains fail-closed unless the operator explicitly acknowledges shared-network loopback proxying or direct networking. A kernel-enforced network namespace connected only to an authenticated Prism egress broker remains future work.

## Canonical validation to run on the target workspace

```bash
python3 scripts/verify-wave6-static.py --check security/wave6-static-evidence.json
python3 scripts/verify-supply-chain.py --check security/supply-chain-evidence.json

cargo metadata --frozen --format-version 1
cargo fmt --all -- --check
cargo check --workspace --all-targets --frozen
cargo test --workspace --all-targets --frozen
cargo clippy --workspace --all-targets --frozen -- -D warnings

nix flake lock
# Review and commit flake.lock before release admission.
nix flake metadata --no-update-lock-file
nix flake check --no-update-lock-file

python3 scripts/verify-supply-chain.py --release
```

Canonical Rust and Nix build receipts should then be captured with `scripts/capture-build-evidence.py`, followed by reviewed-index publication, release-attestation preparation, hybrid signing, and independent release verification.
