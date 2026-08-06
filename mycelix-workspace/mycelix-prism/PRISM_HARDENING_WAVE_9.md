# Prism Hardening Wave 9

Wave 9 hardens the parsers, filesystem reads, source-identity oracle, and publication operations that feed Prism's release ceremony. The objective is to remove ambiguity and time-of-check/time-of-use gaps from evidence that may otherwise be correctly signed but interpreted or loaded unsafely.

This wave follows Wave 8 and assumes its hybrid reviewer policy, semantic Nix lock review, complete build-evidence bundles, kernel-mediated Compatibility egress, and release-attestation machinery.

## Threat model

Wave 9 addresses an attacker who can influence security-critical JSON, inherited process configuration, mutable executable paths, ceremony filesystem layout, or evidence files while they are being read or published.

The primary failure classes are:

- duplicate JSON keys interpreted differently by separate implementations;
- unknown fields carrying shadow semantics;
- same-size file replacement during an admitted read;
- symlink, hard-link, ancestor-overlap, or output-replacement attacks;
- a malicious or mutable `git` selected through `PATH` or configuration;
- unbounded reviewer policy, ledger, signature, rationale, or history data;
- evidence receipts whose accepted object shapes differ between Rust and Python;
- active release lanes still attesting the prior Wave 8 command set.

## Ordered patch sets

1. **Duplicate-key-rejecting JSON core**
   - Adds `prism-strict-json` as a portable workspace crate.
   - Recursively rejects duplicate keys before typed deserialization.
   - Rejects trailing JSON and non-finite numbers.

2. **Strict release and lock artifacts**
   - Routes release attestations, lock reviews, index receipts, and assembly bundles through the strict parser.

3. **Strict reviewer trust artifacts**
   - Routes reviewer anchors, policies, policy histories, ledgers, and verifier-agent responses through the same parser.

4. **Exact trust schemas**
   - Adds `deny_unknown_fields` to signed and policy-bearing structures.
   - Prevents shadow fields from acquiring meaning in another implementation.

5. **Stable ceremony reads**
   - Fingerprints the already-open file before and after each read.
   - Includes inode/device, mode, owner-relevant metadata, link count, size, and timestamps where available.

6. **Stable build-evidence bundles**
   - Fingerprints the evidence directory and each admitted file throughout measurement.
   - Rejects same-size or metadata-changing substitutions.

7. **Stable reviewer trust reads**
   - Applies the same open-file fingerprint discipline to anchors, policies, histories, and ledgers.

8. **Normalized, non-overlapping ceremony paths**
   - Requires absolute lexically normalized paths.
   - Rejects duplicate and ancestor/descendant input-output layouts.

9. **Admitted Git source oracle**
   - Centralizes source identity in `SourceIdentity`.
   - Requires an absolute Git executable admitted by immutable Nix-store path or exact BLAKE3 pin.
   - Remeasures Git before and after every command.
   - Clears the environment and disables external/system configuration, hooks, fsmonitor, replace refs, credential helpers, optional locks, and submodule recursion.
   - Verifies clean status, full commit/tree identities, and Git object types.

10. **Canonical release identities**
    - Fixes the project identity to `mycelix-prism`.
    - Bounds and canonicalizes release versions and signer key identifiers.

11. **Bounded reviewer documents**
    - Caps reviewer keys, ledger records, signatures, signature sizes, evidence sources, rationale lengths, and human-readable identifiers.

12. **Bounded policy rotation histories**
    - Caps transition count, transition signatures, signature sizes, and transition reasons.

13. **Create-once ceremony publication**
    - Requires a caller-owned, non-writable, non-symlinked parent directory.
    - Creates private temporary files exclusively.
    - Publishes through an atomic hard link that fails if the destination exists.
    - Verifies final mode, owner, size, and link count before directory `fsync`.

14. **Strict Python JSON admission**
    - Adds one duplicate-key-rejecting Python loader.
    - Aligns static, supply-chain, and build-evidence readers with Rust semantics.

15. **Exact build receipt schemas**
    - Rust and Python now require the same exact top-level, source, platform, environment, tool, command, and stream fields.
    - Tool sets and evidence file sets must be exact.

16. **Wave 9 canonical release lane**
    - Advances Rust, Nix, CI, release verification, and live ceremony references from `wave8-static` to `wave9-static`.
    - Retains Wave 8 scripts and receipts as historical evidence.

17. **Reproducible Wave 9 static evidence**
    - Adds `scripts/verify-wave9-static.py`.
    - Records explicit Wave 8 supersessions instead of modifying the historical verifier.
    - Regenerates the supply-chain receipt for the new workspace crate and lockfile identity.

## Security invariants

After Wave 9:

- one sequence of JSON bytes has one admitted object interpretation;
- signed trust objects cannot carry unrecognized fields;
- security files must remain the same open-file object throughout admission;
- release inputs and outputs cannot alias or contain one another;
- release source identity cannot depend on `PATH`, user Git configuration, repository hooks, or replace refs;
- mutable Git executables require an exact BLAKE3 pin;
- reviewer documents are bounded before expensive verification or indexing;
- signed ceremony outputs cannot silently replace an existing artifact;
- Rust and Python agree on build-evidence object shapes and exact file sets;
- canonical build evidence names the Wave 9 static gate.

## Canonical validation commands

Static and supply-chain admission:

```bash
python3 scripts/verify-wave9-static.py \
  --check security/wave9-static-evidence.json
python3 scripts/verify-supply-chain.py \
  --check security/supply-chain-evidence.json
```

Release-mode supply-chain admission is expected to fail until the reviewed lock exists:

```bash
python3 scripts/verify-supply-chain.py --release
```

On a complete Rust and Nix host, run the frozen build-evidence lanes and then the live ceremony:

```bash
python3 scripts/capture-build-evidence.py --help
python3 scripts/run-wave9-live-gates.py --help
```

The live gate requires the exact Rust evidence bundle, Nix evidence bundle, reviewed index receipt, hybrid-signed `flake.lock` review, and final hybrid release attestation.

## Remaining release blocker

Wave 9 does not fabricate `flake.lock`. Release admission remains blocked until a Nix-enabled host:

1. generates the lock without mutable inputs;
2. commits it;
3. passes semantic lock admission;
4. completes the offline Ed25519 + ML-DSA-65 review ceremony;
5. captures canonical Nix evidence without updating the lock;
6. executes the complete Wave 9 live gate.

## Validation limits in this environment

The available environment did not contain Cargo, `rustc`, Nix, or Bubblewrap. Therefore this wave does not claim that:

- the Rust workspace compiled;
- Clippy or Rust tests passed;
- the Nix flake evaluated;
- the namespace broker ran under a live kernel;
- production Ed25519 or ML-DSA verification ran;
- a release ceremony completed.

The committed static evidence covers source structure, parsers, manifests, workflow pins, supply-chain policy, and exact security-critical file hashes. Executable claims remain gated by the live ceremony.
