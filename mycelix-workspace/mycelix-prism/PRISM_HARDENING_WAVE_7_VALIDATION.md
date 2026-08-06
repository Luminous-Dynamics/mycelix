# Prism Hardening Wave 7 Validation

**Base:** `4e18e5ddb58d98b902107d12b0f19962a64b9bb8`
**Implementation head:** `c3ff7e78652852bcfe1c07adefbca79d43a77fe3`
**Implementation tree:** `08a030d5adec37de534270e34aceb9dff7d78392`
**Implementation patches:** 14
**Implementation delta:** 26 files changed, 4,008 insertions, 127 deletions

## Result

Wave 7 passed every validation gate available in this environment. The ordered
mail patch series was generated from the Wave 6 base, applied with `git am` to a
fresh clone of the untouched Wave 6 bundle, and reproduced the exact hardened
Git tree.

The implementation is intentionally **not release-admitted**. Release-mode
supply-chain verification fails on exactly one declared blocker: a canonical,
reviewed `flake.lock` has not yet been generated on a Nix-enabled host.

## Replay validation

- Generated 14 implementation and campaign patches from
  `4e18e5d..c3ff7e7` before adding this validation report.
- Applied the complete final Wave 7 series to a fresh clone of
  `prism-hardening-wave6.bundle` at the exact Wave 6 base commit.
- Compared the replayed and hardened Git trees byte-for-byte.
- Replay worktree was clean.
- `git diff --check` passed.
- `git fsck --full` passed.

The final tree and patch counts are recorded again after this report in the
packaged validation manifest.

## Static and structural validation

- All 79 Rust files produced error-free Tree-sitter syntax trees.
- All 28 TOML files parsed successfully.
- All 34 JSON files parsed successfully.
- The GitHub Actions YAML parsed successfully.
- All six Python scripts compiled through Python's parser without emitting
  bytecode.
- The workspace contains 19 members.
- `Cargo.lock` contains 837 packages: 24 local and 813 registry packages.
- The repository-native counter found 349 Rust test, Tokio-test, and
  property-test annotations.
- No Python bytecode or `__pycache__` artifacts were present.
- The implementation worktree was clean.

The namespace runner also received a separate portability correction: Linux
contains the full kernel-egress implementation, while non-Linux targets compile
an explicit fail-closed stub instead of losing the binary's `main` function at
compile time.

## Reproducible security evidence

The following repository-native gates passed against their committed receipts:

- `scripts/verify-wave7-static.py --check security/wave7-static-evidence.json`
- `scripts/verify-supply-chain.py --check security/supply-chain-evidence.json`

The Wave 7 gate checks:

- raw destination policy and all-answer-public DNS validation;
- admitted-port enforcement and pinned outbound addresses;
- keyed, domain-separated tunnel authentication;
- peer-UID validation, strict sequencing, replay rejection, and tunnel budgets;
- network-namespace and loopback-only browser wiring;
- secret-free egress evidence and worker-finalized counters;
- durable Compatibility start/completion receipts;
- reviewer policy-history anchoring and prior-quorum transitions;
- prohibition of historical key rewriting, removal, and retroactive changes;
- policy-history roots in verified ledgers, index schema v6, and receipt schema
  v2;
- policy-history roots in release attestation schema v2;
- the cross-crate history-substitution regression gate;
- immutable workflow actions and the exact pinned Rust toolchain;
- inherited Wave 1-6 static security invariants.

The dependency-admission gate found:

- no Git dependencies;
- no unknown registries;
- no registry package without a checksum;
- no wildcard dependency versions;
- no path dependency escaping the repository;
- no mutable GitHub Action reference.

The committed `actions/checkout` reference is pinned to
`11bd71901bbe5b1630ceea73d27597364c9af683`.

## Expected release blocker

Running the supply-chain verifier in release mode returned failure with exactly
this blocker:

> `flake.lock has not been generated and reviewed on a Nix-enabled host`

This remains deliberate. Wave 7 does not synthesize a Nix dependency lock or
claim a Nix closure without executing and independently reviewing the actual
resolution.

## Validation not performed

The environment did not contain `cargo`, `rustc`, `nix`, `bwrap`, or a Chromium
binary suitable for the Compatibility ceremony. Consequently, this report does
not claim:

- Rust type checking or compilation;
- Cargo tests, Clippy, Rustfmt, or WASM builds;
- Nix flake evaluation, lock generation, or closure builds;
- live Bubblewrap namespace creation;
- proof from inside the browser namespace that direct TCP, UDP, DNS, QUIC, and
  WebRTC egress are blocked;
- live Chromium use of the loopback CONNECT relay;
- DNS-rebinding tests against a controlled resolver;
- concurrent tunnel accounting under real sockets;
- live Spore process execution;
- live Ed25519 or ML-DSA primitive verification by an admitted production agent;
- a completed hybrid-signed release ceremony.

Tree-sitter establishes syntax structure, not Rust name resolution, ownership,
trait satisfaction, target-specific linking, or runtime behavior. Those remain
canonical-workspace gates.

## Canonical target validation

```bash
python3 scripts/verify-wave7-static.py --check security/wave7-static-evidence.json
python3 scripts/verify-supply-chain.py --check security/supply-chain-evidence.json

cargo metadata --frozen --format-version 1
cargo fmt --all -- --check
cargo check --workspace --all-targets --frozen
cargo test --workspace --all-targets --frozen
cargo clippy --workspace --all-targets --frozen -- -D warnings

nix flake lock
# Independently review and commit flake.lock before release admission.
nix flake metadata --no-update-lock-file
nix flake check --no-update-lock-file
python3 scripts/verify-supply-chain.py --release
```

The target security campaign should additionally:

1. launch a kernel-mediated Compatibility session with admitted Chromium,
   Bubblewrap, and runner artifacts;
2. assert that arbitrary direct sockets fail inside the namespace;
3. exercise accepted public HTTPS destinations and rejected private, mixed-DNS,
   disallowed-port, replayed, wrong-key, and wrong-UID requests;
4. confirm final audit counters after concurrent upload/download tunnels;
5. rotate reviewer keys through a signed history and verify old decisions under
   their historical key metadata;
6. build and reopen reviewed index snapshots under the exact history;
7. capture canonical Rust and Nix build receipts;
8. prepare, hybrid-sign, and independently verify release attestation schema v2.
