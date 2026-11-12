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


## Textile recovery conformance

The `textile_recovery` module checks whether a lifecycle event is properly
bound to source-lot/custody evidence and whether a separately claimed
fiber-to-fiber output grade has composition, contamination, process-route,
secondary-grade, and functional-acceptance evidence.

It intentionally separates:
- an admitted event record from eligibility of the material for a claimed
  secondary-fiber use;
- collection/sorting/reuse/downcycling from fiber-to-fiber recycling;
- lot or batch attribution from physical atom identity;
- declared quantities from the conservative bound
  `attributable output <= attributable input + separately evidenced additions`.

Run the same conformance target with `cargo test -p cos_conformance`. The tests
are synthetic contract tests only: they do not authenticate external evidence,
qualify a real recycling process, establish recycled content, or certify a
textile product.


## Integration provenance

This change is stacked on the restored D6S-CANON-1 / D6S-CANON-2 reference source. The combined exact-head candidate must pass both named D6S integration tests and both independent Python verifiers. A prior D6S PASS on the base commit does not qualify a later textile-recovery commit.
