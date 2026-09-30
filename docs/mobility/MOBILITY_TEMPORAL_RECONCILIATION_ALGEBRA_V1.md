# Mobility Temporal Conflict and Reconciliation Algebra v1

**Status:** qualification contract  
**Scope:** semantic/provenance only

## Purpose

This layer classifies the relationship between two explicit claims using caller-supplied comparability, compatibility, temporal applicability, and supersession evidence. It does not decide whether a physical claim is true.

## Inputs

Each claim must already have explicit identity and configuration scope under the identity/lineage and relationship contracts. The reconciliation function receives:
- a temporal applicability interval for each claim;
- a comparison disposition: comparable, incomparable, or unknown;
- a compatibility disposition: compatible, incompatible, or unknown;
- an explicit supersession flag, only when supported by a lineage edge;
- an explicit dispute flag, only when dispute evidence exists.

This algebra does not infer comparability from matching labels, timestamps, hashes, geometry, or consensus.

## Classification

| Classification | Rule |
|---|---|
| Superseded | An explicit supersession relation is supplied. Historical claim remains preserved. |
| Incomparable | Claims are explicitly not comparable. |
| Indeterminate | Comparability/compatibility is unknown, or required interval bounds are unknown for an incompatible pair. |
| Coexistent | Claims are comparable and compatible. |
| Conflicting | Claims are comparable, incompatible, and have known overlapping applicability intervals. |
| Sequential | Claims are comparable, incompatible, and have known non-overlapping applicability intervals. |

A conflict may additionally carry a **disputed** marker. Dispute is not an adjudication and does not prove either claim physically false. Classification precedence is: explicit supersession, explicit incomparability, insufficient comparison/compatibility data, then compatibility and temporal relation.

## Temporal semantics

Intervals use normalized UTC Unix seconds. The temporal applicability contract owns interval validity and overlap semantics. Unknown bounds are not replaced with guessed values. For incompatible claims, an unknown overlap result yields Indeterminate, never Conflicting or Sequential.

## Non-implications

- overlap does not imply incompatibility;
- incompatibility does not imply comparability;
- conflict does not imply physical falsehood or unsafe condition;
- sequential does not imply causal change;
- supersession does not imply physical replacement or erase history;
- dispute does not resolve a conflict;
- protocol validity, consensus, reputation, or publication time does not settle engineering truth;
- no result implies safety, certification, regulatory approval, physical equivalence, or engineering correctness.

## Qualification vectors

- TC-RECON-001: comparable + compatible + overlapping => Coexistent
- TC-RECON-002: comparable + incompatible + overlapping => Conflicting
- TC-RECON-003: comparable + incompatible + non-overlapping => Sequential
- TC-RECON-004: incompatible + unknown interval bound => Indeterminate
- TC-RECON-005: explicitly incomparable claims => Incomparable
- TC-RECON-006: unknown comparability => Indeterminate
- TC-RECON-007: unknown compatibility => Indeterminate
- TC-RECON-008: explicit supersession => Superseded, predecessor preserved
- TC-RECON-009: conflicting + explicit dispute => Conflicting with disputed marker
- TC-RECON-010: compatible claims with unknown time bounds => Coexistent (temporal overlap itself remains unknown)
- TC-RECON-011: different physical artifact without explicit comparison profile => Incomparable
- TC-RECON-012: later publication timestamp does not alter applicability classification

All fixtures are synthetic. Qualification is structural and semantic only.
