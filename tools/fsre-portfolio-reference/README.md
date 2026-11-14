# FSRE portfolio reference evaluator v0.1

**Status:** Deterministic, read-only reference over one frozen proposed plan, scenario, interval and evidence snapshot. Synthetic fixtures only; not a dispatch optimiser or operational authority.

## Purpose

Verify whether a declared recovery portfolio can meet service floors for distinct service cohorts while respecting owner-qualified option capacity, shared-resource capacity and scenario-specific common-cause states.

The evaluator consumes already-qualified receipts. It does not calculate physical capacity, infer dependency independence, optimise allocations, schedule resources, authenticate owner evidence, grant authority or actuate infrastructure.

## Input contract

One PSV file contains a frozen snapshot. Every record includes an exact scenario, interval, owner and evidence reference. Quantities use non-negative integers and exact unit-string equality; implicit unit conversion is rejected.

Required record families:
- `OPTION_QUALIFICATION` and `OPTION_CAPACITY` — option/service eligibility and upper capacity.
- `ALLOCATION` and `SERVICE_FLOOR` — proposed delivered quantities and floors by service cohort.
- `RESOURCE_PROFILE`, `RESOURCE_USE`, `RESOURCE_CAPACITY` — owner assertion that resource use is complete plus each proposed shared-resource load and its available capacity.
- `DOMAIN_PROFILE`, `OPTION_DOMAIN`, `DOMAIN_STATE` — owner assertion that common-cause coverage is complete, option membership in shared failure domains, and domain state under the frozen scenario.

The cohort-floor regression intentionally creates a plan whose aggregate delivered volume equals the aggregate of all declared floors, while households receive only 60 against a floor of 70 and hospitals receive 60 against a floor of 50. The evaluator must still report `NOT_FEASIBLE`; surplus in one cohort never silently compensates for a protected cohort's shortfall.

An `UNKNOWN` option capacity, resource use or resource capacity must carry no numeric quantity or unit; a numeric bound on an unknown record is rejected. Missing or UNKNOWN required profile/capacity/state is not zero and yields `NOT_ESTIMABLE`. FAILED domains block affected options. Resource demand is summed once per option/service across the submitted proposal. Service floors are enforced separately for each cohort. Each option ID is restricted to one service lane in v0.1 to prevent capacity double-counting across services. Shared failure domains are reported even when currently CLEAR; the evaluator does not certify independence.

## Output

The report echoes all input evidence IDs, source owners and opaque evidence references in stable order. This preserves a replay trail; it does not authenticate the referenced records or prove the source owners' assertions.

The report contains each option disposition, cohort floor result, shared-resource capacity result, common-cause concentration, every input record's source owner/evidence reference and a non-authorisation statement. Dispositions:
- `PORTFOLIO_FEASIBLE_FOR_DECLARED_SCENARIO` — known inputs support the declared proposed allocation in this one scenario/interval.
- `NOT_FEASIBLE` — known option, resource, failure-domain or service-floor constraints are violated.
- `NOT_ESTIMABLE` — required evidence is missing, unknown or incomplete.

If known infeasibility and unknown evidence coexist, the claim is conservatively `NOT_ESTIMABLE`; known violations remain visible.

## Independent reference

The separately authored [portfolio oracle](../fsre-portfolio-oracle/README.md) uses its own parser and evaluator. The differential workflow compares the two implementations on the frozen fixture; shared specification errors still require a disjoint external challenge set.

## Verification

The deterministic test suite also contains five explicit unknown-bound controls (numeric bounds are rejected; genuinely unknown bounds abstain). It includes a generated 283-configuration boundary sweep across resource capacity, option capacity and per-cohort service floors, in addition to targeted common-cause, unknown-evidence, unit-mismatch, lineage and overflow controls.

`cargo test --locked --offline --manifest-path tools/fsre-portfolio-reference/Cargo.toml`

`cargo run --locked --offline --manifest-path tools/fsre-portfolio-reference/Cargo.toml -- tools/fsre-portfolio-reference/fixtures/synthetic-feasible.psv`

This is a deterministic accounting/constraint-checking reference for the submitted allocation, not an optimiser, scheduler, probability model, physical twin or field validation. Owner references are carried but not authenticated. The service floor, resource inventory and dependency profiles are assumed to be supplied by authorised owners; their completeness and truth are not established by this evaluator.
