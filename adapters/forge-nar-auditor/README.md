# mycelix-forge-nar-auditor

FORGE-004D2B3B implements the first concrete producer for the runtime-closure evidence contract.

It deliberately does **not** ask the Nix daemon or store database what a path's `narHash` is. Instead it walks the actual filesystem object and computes the canonical NAR SHA-256 directly using `cachix/nix-archive` **0.6.0**, whose upstream release commit is pinned in the auditor subject as `b4ecefa4c0c47e7ae7446cd59e901aa2d95f8414`.

## Why this implementation

The pinned NAR implementation:

- preserves raw-byte filenames and symlink targets;
- streams regular-file contents rather than materializing complete archives;
- performs descriptor-relative traversal designed to resist symlink-swap races;
- has no Nix daemon/runtime dependency;
- exposes explicit `CaseHack` semantics because case-hack behavior changes NAR identity.

M0 Linux fixes `CaseHack::Disabled`; the policy never relies on a platform-dependent default.

## Two-pass stability

The entire committed closure is scanned twice, in opposite path order. Any digest or NAR-size change between passes fails closed before an observation is emitted.

This closes ordinary in-audit TOCTOU races. The eventual verifier launcher must still perform runtime-closure audits before **and** after the real verifier execution so host mutation between audit and verification cannot disappear from the evidence chain.

## Auditor subject

The producer emits an explicit `auditor_subject` commitment over:

- implementation identifier;
- exact crate version `0.6.0`;
- exact upstream release commit;
- explicit NAR SHA-256 suite;
- explicit disabled case-hack behavior.

That subject identifies semantics, not executable provenance. The final execution composition must additionally bind and verify the actual auditor binary as a FORGE-004D1 tool artifact.

## Claim boundary

This crate produces D2B3A observations and can structurally qualify them. It does not prove its own binary provenance, kernel isolation, source reproducibility, verifier trust, or repository `OfflineEvidence`.
