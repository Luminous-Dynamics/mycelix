# AC-080: Exact-Head Hearth Qualification

## Purpose

Generic Mycelix CI uses path-filtered jobs and can legitimately report skipped jobs. That makes it unsuitable as the sole qualification source for a hardening loop when an exact Hearth commit must be demonstrably executed.

AC-080 adds an independent qualification workflow whose checkout is explicitly bound to the pull request head SHA.

## Exact-head invariant

The workflow records the requested PR head SHA, checks out that SHA explicitly, and fails when `git rev-parse HEAD` differs.

No merge-commit SHA is substituted for the source commit under qualification.

## Executed checks

- Hearth workspace formatting;
- Hearth workspace unit/integration tests included by its Cargo workspace;
- release WASM build for the Hearth zomes;
- Hearth DNA packing;
- the Decision/Kinship Sweettest target with ignored tests enabled.

## Concurrency

Qualification runs are keyed by the exact commit and are not cancelled when a newer commit appears. This preserves one immutable hosted result per qualified source tree.

## Trigger boundary

The workflow runs on non-draft Hearth pull requests and explicitly admits `ready_for_review`. It also supports manual dispatch.

## Qualification boundary

A green AC-080 run qualifies only the exact commit and test commands executed by this workflow.

It does not establish DHT-wide completeness, exactly-once distributed semantics, real-world identity, legal authority, substantive legitimacy, or that a skipped generic CI job was actually executed.