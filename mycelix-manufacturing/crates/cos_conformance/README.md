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
