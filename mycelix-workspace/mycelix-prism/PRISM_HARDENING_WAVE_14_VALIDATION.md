# Prism Hardening Wave 14 Validation

## Scope

This report covers the Wave 14 delta applied after the Wave 13 hardened tree.
The implementation adds exact archive-restoration receipts, deterministic
three-history disaster-recovery drills, byte-identical cross-channel release
promotion, and an immutable promotion journal with complete re-audit.

## Patch replay

Before this report was committed, all 24 implementation, security, evidence,
and ceremony patches were formatted as mail patches and applied with `git am`
to a fresh clone of the untouched Wave 13 Git bundle. The replay produced the
same Git tree as the Wave 14 working repository:

`69102452bb729b8a0a09b86dace60c7d4624902c`

The replay worktree was clean. The sealed final replay, including this report,
is repeated during deliverable packaging.

## Available validation performed

The following gates passed in this environment:

- Wave 14 static evidence generation and exact receipt comparison.
- Supply-chain admission and exact receipt comparison.
- Rust syntax-tree parsing for all 124 `.rs` files.
- TOML parsing for all 29 manifests and configuration files.
- JSON parsing for all 48 JSON files.
- Python AST parsing for all 24 Python scripts.
- Workflow YAML parsing for the one GitHub Actions workflow.
- `git diff --check`.
- `git fsck --full`, ignoring only ordinary dangling/unreachable history.
- Exact 24-patch replay onto the untouched Wave 13 bundle.

The workspace contains:

- 20 workspace crates.
- 838 locked packages.
- 410 Rust test, Tokio-test, and property-test annotations.

Before this report, Wave 14 changed 31 files with 4,931 insertions and 15
deletions across 24 patches.

## Security properties inspected

### Archive restoration

- The restore root must be canonical, caller-owned, mode `0700`, and free of
  symlink traversal.
- The restored file set must exactly equal the signed archive manifest.
- Each object is remeasured for size, SHA-256, BLAKE3, and executable intent.
- The restore receipt binds the release statement, archive handoff statement,
  exact manifest and handoff bytes, object set, and stable object-kind code.
- Extra, missing, renamed, replaced, or permission-changed objects are rejected.

### Disaster-recovery convergence

- The release-journal audit, transparency-journal audit, and archive-restore
  receipt must identify the same project, channel, release sequence, and release
  statement.
- Plans and receipts bind all three exact evidence roots.
- A configurable minimum restored-object count prevents a metadata-only drill.
- Verification and publication are idempotent only for identical bytes.

### Cross-channel promotion

- Source and target releases are independently hybrid-verified through the
  measured verifier agent.
- Exact source/target policies and attestations are embedded in the promotion
  receipt.
- Release version, source commit, source tree, payload-manifest SHA-256, and
  payload-manifest byte count must match.
- Promotion admission requires the private-field `VerifiedPromotion`
  capability minted by successful verification.
- Promotion checkpoints are contiguous and create-once.
- Same-target-sequence conflicts are rejected as equivocation and older target
  sequences as rollback.
- Complete history audit re-verifies every embedded release signature.
- The journal directory inode, ownership, and permissions are pinned across
  admission, and enumeration errors fail closed.

### Live continuity

- The Wave 14 continuity configuration has an exact schema and private,
  non-aliased artifact paths.
- Recovery verification, promotion admission, and promotion-history audit are
  rerun through frozen Cargo commands.
- The exact promotion-state tree is measured before and after; any mutation
  during the idempotent continuity gate is a failure.
- The outer Wave 14 live evidence bundle records the continuity configuration
  and retains command stdout/stderr.

## Defect found during semantic review

Syntax parsing alone did not detect that `verify_embedded()` had been attached
to `ReleasePromotionPolicy` while accessing fields owned by
`ReleasePromotionReceipt`. The method was moved to the receipt implementation
before evidence was sealed. The workspace lock was also synchronized with the
new `serde_json` test dependency.

## Unexecuted validation

This environment does not contain Cargo, Rust, Nix, Bubblewrap, Chromium, or the
production verifier-agent/signing infrastructure. Therefore this report does
not claim success for:

- Rust formatting, compilation, Clippy, or test execution.
- Nix flake evaluation or closure builds.
- Live archive restoration or cold-storage retrieval.
- Live disaster-recovery drills.
- Live source-to-target release promotion.
- Production Ed25519 or ML-DSA-65 verification.
- Namespace and egress adversarial execution.

## Release blocker

Release-mode supply-chain admission fails on exactly one deliberate condition:

> `flake.lock` has not been generated, committed, semantically reviewed, and
> hybrid-signed on a Nix-enabled host.

No other release blocker was reported by the available static or supply-chain
gates.
