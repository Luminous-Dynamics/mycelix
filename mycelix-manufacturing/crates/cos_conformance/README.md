# COS conformance harness

This crate is the executable companion to Mycelix issues #3332/#3333/#3334 and
`docs/integral/cos-reference-node-v1.md`.

It tests semantic evidence boundaries only. It deliberately does not execute
manufacturing, Holochain, Integral governance, ITC policy, or FRS operations.

## Run

`cargo test -p cos_conformance`

## Report

The library exposes `conformance_report_json()` for machine-readable export.
The report contains the corpus identity, formal-obligation mappings, and claim
ceiling. It is intentionally not a scalar verification score.

## Claim ceiling

A passing suite establishes only that this reference model rejects the specified
semantic collapses and accepts their explicitly bound counterparts. It does not
establish physical productivity, safety, qualification, economic/ecological
outcomes, or Integral validation.

## Heterogeneous federation

`federation.rs` provides the deterministic reference oracle for Integral/Mycelix heterogeneous federation. It preserves local-vs-foreign authority, logical delivery identity, schema/authorization generations, causal dependencies, reconnect idempotence, conflicting observations, and privacy-minimized projections. See `docs/integral/heterogeneous-federation-reference-v1.md`.

## Branch reconciliation

`federation_reconciliation.rs` extends the federation oracle with explicit branch identity, frontier closure, compatibility classification, conflict-preserving reconciliation, capacity double-spend detection, authority-validity fencing, and branch-aware cockpit projections. See `docs/integral/heterogeneous-federation-reconciliation-v1.md`.

## Identity and alias integrity

`federation_identity.rs` keeps identifiers, credentials, principals, accounts, devices, resources, locators, and entities distinct. It provides scoped equivalence classes, typed substitution profiles, append-only lifecycle events, resource-capacity alias checks, privacy projections, and deterministic identity-resolution witnesses. See `docs/integral/semantic-identity-alias-integrity-v1.md`.

## Causal time

`federation_causal_time.rs` separates causal ancestry from wall-clock observations, distinguishes concurrent from incomparable histories, models bounded clock uncertainty, evaluates profile-bound freshness, and requires explicit revalidation for long-offline branches. See `docs/integral/causal-time-long-lived-federation-v1.md`.

## No-resurrection integrity

`no_resurrection.rs` models first-class tombstones, semantic generations, explicit successor/reactivation, resource/authority conservation across generations, cache invalidation, compaction preservation, and branch lifecycle reconciliation. See `docs/integral/tombstone-generation-no-resurrection-v1.md`.

## Stable frontier and safe reclamation

`stable_frontier.rs` models closed authority-bearing membership, explicit frontier coverage, stable-frontier certificates, retention boundaries, pruning receipts, cold-start reconstruction, rejoin fencing, and conservation of authority/capacity/consent claims across reclamation. See `docs/integral/stable-frontier-safe-history-reclamation-v1.md`.

## Semantic archive continuity

`archive_continuity.rs` models historical-evidence profiles, archive manifests, explicit archive/frontier continuity, profile and membership transitions, historical-claim ceilings, contested archive sets, and reconstruction gates. Archives can support historical analysis or cold-start reconstruction but cannot become current authority, actuation, or policy authority by themselves. See `docs/integral/semantic-archive-continuity-v1.md`.

## Semantic recovery and provider substitution

`substitution_continuity.rs` binds provider routes to one stable Mycelix semantic effect, exact request/resource/tenant/amount/unit/authority/consent semantics, provider-profile allow-lists, lifecycle generation, and explicit route succession. Unknown or pending outcomes block independent failover unless an exact outcome-resolution witness or contract-wide idempotency witness qualifies continuation. Provider operation IDs remain provider-scoped and cannot replace the semantic effect ID. Archive recovery is exposed as reconstruction input only; it cannot authorize current provider execution. See `docs/integral/semantic-recovery-provider-substitution-v1.md`.

