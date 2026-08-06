# Prism Hardening Wave 16 Validation

## Verdict

Wave 16 passes every validation gate available in the patching environment.
The implementation is structurally coherent, the repository-native evidence
receipts reproduce exactly, dependency admission is clean, and the ordered
patch series replays without conflict onto the untouched Wave 15 bundle.

This is not a claim that executable release admission has passed. The
environment did not contain Cargo, rustc, Nix, Bubblewrap, Chromium, or the
production verifier agent and hybrid signing infrastructure.

## Scope validated

The validation covered:

- bounded health epochs and explicit promotion evaluation time;
- independent source and target incident-authority histories;
- dual-authorized, non-retroactive incident-policy rotation;
- measured verifier-agent history verification;
- separate source/target incident-history roots in promotion receipts;
- threshold, nonce-bound, expiring recovery authorization;
- re-verification of every guardian at final recovery;
- exact, closed-set promotion-evidence reconstruction;
- independent reconstruction remeasurement;
- Wave 16 continuity and live-gate definitions; and
- canonical Wave 16 build-evidence admission.

## Structural results

- Rust files parsed through the Rust tree-sitter grammar: **151**
- Rust syntax errors: **0**
- TOML files parsed: **29**
- JSON files parsed: **54**
- Python scripts parsed: **30**
- Workspace crates: **20**
- Admitted Cargo lock packages: **838**
- Rust test, Tokio-test, and property-test annotations: **427**

The repository's static and supply-chain evidence receipts regenerated
byte-for-byte after the final semantic corrections.

## Security-specific findings corrected during validation

### Exact reconstruction bytes

The initial reconstruction implementation normalized embedded JSON by appending
a trailing newline when absent. That produced valid JSON but was not guaranteed
to reproduce the exact original evidence bytes. The final implementation:

- preserves the embedded bytes exactly;
- compares each object's BLAKE3 digest to the digest already committed by the
  promotion receipt; and
- independently re-verifies the complete restored directory.

### Cross-channel history substitution

Source and target incident-policy roots were represented by separate fields but
root equality was not explicitly rejected. Since the history root commits to the
channel, equal roots indicate substitution or a cryptographic collision. The
final promotion creation and receipt validation paths reject equality.

### Process-boundary recovery verification

A structurally valid recovery-authorization receipt is not treated as proof that
a prior process actually performed cryptographic verification. Finalization
reloads the independently anchored policy, reparses the embedded bundle,
re-runs every Ed25519 and ML-DSA-65 check through the measured verifier agent,
and compares the resulting private-field verification tokens to the receipt.

## Evidence and repository gates

The following passed:

- `scripts/verify-wave16-static.py --check`
- `scripts/verify-supply-chain.py --check`
- Rust tree-sitter parsing for every `.rs` file
- TOML parsing for every manifest and policy file
- strict JSON parsing for every committed JSON file
- Python AST parsing for every committed Python script
- `git diff --check`
- `git fsck --full`
- exact Wave 16 patch replay onto the Wave 15 bundle
- clean replay worktree
- exact replay/source Git-tree equality

## Patch replay

Before this validation report was added, all **32 implementation,
security-test, evidence, and documentation patches** were emitted with
`git format-patch`, applied with `git am` to the untouched Wave 15 bundle, and
produced the exact source Git tree:

`ec88e3b180e0094dbab6fcfdc2c735b581c36c11`

The validation report is the final Wave 16 patch. Packaging performs a second
sealed replay including this report and a separate complete-campaign replay
from the imported baseline.

## Release admission

Release-mode supply-chain verification fails on exactly one deliberate blocker:

> `flake.lock` has not been generated, committed, semantically reviewed, and
> hybrid-signed on a Nix-enabled host.

No other dependency-admission failure was reported.

## Gates not executed

The following require the canonical Nix/Rust host and production ceremony
infrastructure and were not executed here:

- `cargo fmt`, `cargo check`, `cargo test`, and Clippy;
- WASM target compilation;
- Nix flake evaluation and closure builds;
- generation and semantic review of `flake.lock`;
- Bubblewrap/Chromium namespace execution;
- live verifier-agent peer, executable, purpose, challenge, and key checks;
- real Ed25519 and ML-DSA-65 incident-policy signatures;
- real release-health and recovery-guardian ceremonies;
- cross-channel promotion finalization and recovery activation; and
- live evidence reconstruction on production release artifacts.

Those remain mandatory release gates. Static admission does not substitute for
them.
