# MOBILITY-COMMONS-018: Executable Reconciliation Witness Revision Corpus

This corpus is the executable qualification layer for the witness revision contract.

## Scope

The corpus tests semantic/provenance behavior only. Passing cases do not establish:
- physical correctness
- physical equivalence
- safety
- certification
- regulatory approval
- measurement truth
- authorship beyond protocol-level metadata

## Differential contract

Two independent evaluators consume the same structured corpus:
1. Rust implementation in the mobility qualification crate.
2. Python reference implementation.

Both emit `mobility-reconciliation-witness-revision-normalized-v1`. The comparator fails closed on envelope, cardinality, identifier, operation, expected, or actual-result differences.

## Cases

WRV-001..016 cover:
- stable same-identity payload
- changed claim pair rejection
- changed applicability rejection
- changed reconciliation-input/result rejection
- missing supersession rejection
- exact successor-to-predecessor supersession acceptance
- historical projection binding
- successor cannot retarget predecessor projection
- predecessor remains valid after supersession
- Holochain-shaped identity rejection
- invalid temporal interval rejection
- tampered stored result rejection
- disputed conflict preservation
- disputed sequential result remaining non-disputed
- explicit supersession classification
- projection rejection when the supplied witness identity does not match the projection target

## Why explicit dependencies matter

The corpus keeps the supersession edge as an explicit input rather than synthesizing it from witness identities. This mirrors the broader requirement that validation dependencies be explicitly addressable and deterministically reconstructible rather than inferred from mutable context.

## Failure model

A corrupted normalized output must be rejected by the comparator.

A corpus mutation that reverses or collapses a supersession edge must be rejected independently by both evaluators.

This is a qualification mechanism for implementation agreement, not a claim that either implementation has established the truth of an engineering assertion.