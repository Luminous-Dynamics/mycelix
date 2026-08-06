# Prism Hardening Wave 13 Validation

## Scope

This report covers the Wave 13 delta applied after the Wave 12 hardened tree.
The implementation introduces witness-policy rotation, immutable transparency
checkpoints, externally anchored recovery, and signed auxiliary-evidence archive
handoffs.

## Patch replay

Before this report was committed, all 16 implementation, security, evidence,
and ceremony patches were formatted as mail patches and applied with `git am`
to a fresh clone of the untouched Wave 12 Git bundle. The replay produced the
same Git tree as the Wave 13 working repository and left no tracked or untracked
changes.

The sealed final replay includes this validation report and is repeated during
deliverable packaging.

## Available validation performed

The following gates passed in this environment:

- Wave 13 static evidence generation and exact receipt comparison.
- Supply-chain admission and exact receipt comparison.
- Rust syntax-tree parsing for all 114 `.rs` files.
- TOML parsing for all 29 manifests and configuration files.
- JSON parsing for all 45 JSON files.
- Python AST parsing for all 21 Python scripts.
- Workflow YAML parsing where the Python YAML package was available.
- `git diff --check`.
- `git fsck --full` with unreachable-object diagnostics ignored as non-tree
  repository history.
- Exact Wave 13 replay onto the untouched Wave 12 bundle.

The workspace contains:

- 20 workspace crates.
- 838 locked packages.
- 402 Rust test, Tokio-test, and property-test annotations.

Before this report, Wave 13 changed 28 files with 5,632 insertions and 8
deletions across 16 patches.

## Security properties inspected

### Transparency policy history

- Policy changes require the previous policy's threshold.
- Both Ed25519 and ML-DSA-65 components are required per transition witness.
- Transition sequence, previous policy, next policy, reason, and exact policy
  bytes are committed.
- Identical current policy bytes reached through different transition histories
  produce different audit roots.
- Same-sequence conflicting receipts are rejected as equivocation.
- Sequence gaps and broken checkpoint predecessors are rejected.
- Unix journal locks use kernel-released advisory locks rather than stale
  create/delete lock state.

### Recovery

- Recovery requires externally supplied expected sequence, release statement,
  checkpoint, and full audit root.
- Recovery re-verifies the complete witness and policy-transition history.
- Head, audit, and recovery receipts publish idempotently only for identical
  bytes.

### Archive handoff

- Archive objects are limited to a closed auxiliary-evidence taxonomy.
- Only retention-plan entries classified as eligible for offline archival may
  enter a manifest.
- Release and transparency journal checkpoints cannot be described as archive
  object kinds.
- Staging paths must remain beneath a private canonical root and must name
  direct regular files rather than symlinks.
- Extra, missing, executable, modified, linked, or path-replaced objects fail
  verification.
- Destinations use a closed scheme set and reject credentials, queries,
  fragments, control characters, and non-canonical paths.
- The exact manifest, retention root, transparency-history root, release
  statement, release policy, and destination count are hybrid-signed.
- Final verification rechecks the release attestation, archive handoff, and
  staged file set through purpose-bound verifier-agent requests.

## Release blocker accounting

Non-release supply-chain admission passed.

Release-mode supply-chain admission failed on exactly one condition:

> `flake.lock` has not been generated, committed, semantically reviewed, and
> hybrid-signed on a Nix-enabled host.

No other release blockers were reported.

## Validation not performed

This environment did not provide `cargo`, `rustc`, `rustfmt`, Nix, Bubblewrap,
Chromium, the production verifier agent, witness signers, release signers, or a
cold-storage backend. Therefore this report does not claim that:

- the Rust workspace compiled;
- Clippy or rustfmt passed;
- unit, integration, property, or live namespace tests executed;
- the Nix flake evaluated or built;
- production Ed25519 or ML-DSA-65 verification executed;
- an actual witness rotation, recovery drill, or cold-storage transfer ran.

Those executable requirements remain mandatory before release admission.
