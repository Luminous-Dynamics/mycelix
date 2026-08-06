# Mycelix-DeSci Refoundation — Tranche 4

**Date:** 2026-08-04
**Status:** implementation patchset; Rust build verification still required

## Objective

Make scientific-event acceptance independently auditable at receipt time without retroactively re-evaluating legitimate history against today's actor roles or key registry.

## Implemented

### Signed authority receipts

Every governed append now prepares a separately signed authority receipt that binds the event hash, server receipt time, actor key authorization, organization membership, roles, normalized action, policy identity, and decision reason.

### Two-phase audit journal

A new memory and file audit-store capability supports prepare, commit, abort, lookup, per-event status, stream-head, ordered stream export, reconciliation, and health summaries. The file adapter persists one receipt per event, rejects symbolic links and malformed filenames, enforces receipt size limits, and verifies trusted service signatures.

### Crash reconciliation

Startup finalizes pending receipts whose scientific events committed, removes pending receipts whose event append never completed, rejects committed orphan receipts, verifies receipt-chain continuity, and classifies missing receipts as a legacy prefix or an unsafe post-cutover gap.

### Historical cutover

A contiguous pre-receipt canonical prefix remains explicitly legacy-unattested. It may be acknowledged for a bounded cutover with an environment flag, but no synthetic receipt or reconstructed authorization claim is created. Missing receipts after journaling begins are unsafe and never waivable.

### Receipt-key rotation

The current private receipt key is separated from the trust set. Historical receipt public keys can remain trusted through a JSON trust file, allowing private-key rotation without invalidating prior receipts.

### API and migration integration

Canonical append responses now include the signed authority receipt. Event reads expose typed authority status, a dedicated event receipt endpoint returns the receipt and hash, and a stream endpoint exports the ordered receipt chain plus unattested classifications. Offline legacy migration requires a separate receipt signing key and authority-audit path.

### Readiness hardening

Readiness now depends on durable events, configured actor authority, configured receipt signing, zero pending receipts, no unsafe receipt gaps, and an acknowledged or absent legacy prefix.

### Additional receipt invariants

The receipt service key must differ from the scientific event key. Receipt policy and decision fields are bounded, all organization identifiers are revalidated, event clock skew is rechecked during audit replay, and parallel receipt forks at one stream sequence are rejected before event commit.

### Baseline repairs

The tranche also removes a duplicated `RetractClaim` variant and duplicated `#[async_trait]` attribute that would otherwise block compilation.

## Deliberate limitations

- The reference journal is single-process filesystem storage, not distributed consensus.
- The event and receipt directories are separate; atomicity is achieved through detectable two-phase recovery rather than a cross-directory transaction.
- Dynamic credential refresh and governed key-compromise annotations are not yet implemented.
- A complete `cargo fmt`, `cargo clippy`, and test pass requires the pinned Rust toolchain and sibling `mycelix-zkp-core` dependency.
