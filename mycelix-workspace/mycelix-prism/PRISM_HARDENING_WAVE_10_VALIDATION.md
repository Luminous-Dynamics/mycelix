# Prism hardening Wave 10 validation

This report records the validation that was actually possible in the patching
environment. It deliberately separates structural and replay evidence from
Cargo, Nix, namespace, browser, and production cryptographic execution that
could not be performed here.

## Validated source

Wave 10 begins at the untouched Wave 9 tip `a147e94` and contains twenty-one
implementation, evidence, and campaign patches before this validation report.
The validated pre-report tree is:

`69c3193b98064af5c73c1e2ea700f0f5e3151025`

The validation report itself is an additional final documentation patch and does
not alter the implementation or machine-readable evidence inputs.

## Available gates that passed

### Static security evidence

```sh
python3 scripts/verify-wave10-static.py \
  --check security/wave10-static-evidence.json
```

The receipt reproduced byte-for-byte and reported no static invariant failures.
It covers:

- exact typed index publication receipts;
- externally anchored release policy;
- signed channel, sequence, predecessor, and policy digest;
- append-only rollback journal;
- checkpoint-derived successor policy;
- typed measured release payload manifests;
- segment-exact version policy;
- stable pathname-to-inode reads;
- admitted Git use and hardened live publication;
- Wave 10 Rust, Nix, CI, release, and live evidence references; and
- the documented operator ceremony.

### Supply-chain admission

```sh
python3 scripts/verify-supply-chain.py \
  --check security/supply-chain-evidence.json
```

The committed supply-chain receipt reproduced exactly. It admitted:

- 20 portable workspace members;
- 838 locked packages;
- 25 local packages and 813 registry packages;
- no Git dependencies;
- no unknown registry sources;
- no registry package without a checksum;
- no wildcard dependency versions;
- no path dependency escaping the portable workspace; and
- immutable GitHub Action references.

Release-mode supply-chain admission was also executed:

```sh
python3 scripts/verify-supply-chain.py --release
```

It failed on exactly one declared release blocker:

> `flake.lock` has not been generated, committed, semantically reviewed, and
> hybrid-signed on a Nix-enabled host.

No other release-mode failure was reported.

### Structural parsing

The complete tree was parsed using the available structural parsers:

- 93 Rust source files: zero syntax-tree errors;
- 29 TOML files: all parsed;
- 38 JSON files: all parsed;
- 15 Python scripts: all compiled to Python bytecode successfully; and
- the GitHub workflow YAML: parsed by the available YAML loader.

The source contains 381 `#[test]`, `#[tokio::test]`, and property-test macro
annotations. This is a source count, not a claim that the tests executed.

### Repository integrity

The following checks passed:

```sh
git diff --check
git fsck --full
```

The Wave 10 implementation/campaign delta before this report changed 30 files
with 3,718 insertions and 414 deletions.

### Patch replay

The ordered mail-patch series was validated in two independent lanes:

1. all 22 Wave 10 patches were applied with `git am` to a fresh clone of the
   untouched Wave 9 bundle at `a147e94`; and
2. all 127 cumulative patches were applied with `git am` to a fresh checkout of
   the imported baseline root.

Both replays completed without conflict, left clean worktrees, and produced the
same Git tree as the hardened checkout. This validates patch order and packaging
independently of the working repository used to author the changes.

## Manual semantic review

Because no Rust compiler was available, the modified interfaces were also
reviewed manually across their call boundaries. The review checked:

- all exhaustive `ReleaseArtifactKind` mappings after adding the payload
  manifest kind;
- preparation, assembly, and verification agreement on release schema v6 and
  unsigned-request schema v6;
- exact argument ordering for payload manifest preparation and release
  verification;
- release policy use at preparation, assembly, and verification;
- journal advancement only after primitive hybrid verification;
- canonical payload ordering, exact directory file-set comparison, and dual
  digest measurement;
- overlap rejection among source locks, evidence, policy, state, payloads, and
  outputs;
- create-once publication for ceremony files and no-replacement live evidence;
- stable opened-file and pathname identity checks; and
- matching Wave 10 command labels in Rust, Python, Nix, and workflow evidence.

No additional inconsistency was found during that review. This does not replace
a compiler or executable tests.

## Gates not executed

The environment did not contain `cargo`, `rustc`, `nix`, or `bwrap`. Therefore
the following remain unexecuted and must not be inferred from the passing static
gates:

- `cargo fmt --all -- --check`;
- `cargo check --workspace --all-targets --frozen`;
- `cargo test --workspace --frozen`;
- `cargo clippy --workspace --all-targets --frozen -- -D warnings`;
- Nix flake metadata and `flake check --no-update-lock-file`;
- live Bubblewrap namespace isolation;
- live Compatibility broker and adversarial CONNECT tests;
- Chromium launch and payload execution checks;
- production Ed25519 and ML-DSA-65 verifier-agent calls;
- hybrid-signed `flake.lock` review;
- complete release preparation, offline signing, independent verification, and
  rollback-journal advancement; and
- the fixed `run-wave10-live-gates.py` ceremony.

## Canonical host completion commands

After generating, committing, semantically reviewing, and hybrid-signing
`flake.lock`, execute the full procedure in `security/RELEASE_CEREMONY.md`. At a
minimum, the canonical host must run:

```sh
cargo fmt --all -- --check
cargo check --workspace --all-targets --frozen
cargo test --workspace --frozen
cargo clippy --workspace --all-targets --frozen -- -D warnings

python3 scripts/verify-wave10-static.py \
  --check security/wave10-static-evidence.json
python3 scripts/verify-supply-chain.py --release

nix --extra-experimental-features 'nix-command flakes' \
  flake metadata --json --no-update-lock-file
nix --extra-experimental-features 'nix-command flakes' \
  flake check --no-update-lock-file --print-build-logs
```

Then complete payload measurement, the hybrid signing ceremony, independent
release verification, and the Wave 10 live gate exactly as documented.

## Verdict

Wave 10 is structurally coherent and replay-ready, and every available static,
schema, dependency, and repository-integrity gate passed. It is **not yet a
release**. The single admitted blocker is still the absent generated and
hybrid-reviewed `flake.lock`, and all live Rust/Nix/namespace/cryptographic gates
remain mandatory on the canonical host.
