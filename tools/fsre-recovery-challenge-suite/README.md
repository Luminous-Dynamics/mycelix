# FSRE recovery challenge suite v0.1

**Status:** Public deterministic synthetic challenge pack; not an independently sealed test set and not a field validation result.

## Purpose

Exercise the production recovery-option evaluator and the separately implemented Rust reference against a broader, frozen option-level challenge set. The suite covers valid positive cases, missing predicates, non-qualified owner dispositions, temporal/clock boundaries, mobilisation limits and compound failures.

The two implementations must agree exactly on each option disposition and reason-code set. The public expected output is also checked independently of either executable. The workflow separately mutates owner, schema, service, scenario, digest and time-basis bindings and requires both evaluators to reject each malformed input.

## Frozen suite

- 50 option cases under one declared synthetic disruption scenario.
- 10 positive cases.
- 10 missing-predicate cases.
- 10 validly scoped but non-qualified disposition cases.
- 10 stale/clock-uncertainty cases.
- 5 mobilisation-horizon cases.
- 5 compound negative controls.
- 340 predicate evidence rows (the 10 missing-predicate cases omit one row each).
- Seven predicate families with their existing owner/schema namespaces.

Files:
- `fixtures/scenario-options.psv` — 50 service-bound recovery candidates.
- `fixtures/predicate-evidence.psv` — owner-scoped synthetic evidence records.
- `fixtures/case-register.psv` — case family and expected result inventory.
- `fixtures/expected-options.txt` — frozen expected independent-reference output.

## Interpretation

All values and source references are synthetic. This pack tests deterministic software conformance, not owner authentication, physical feasibility, real dependency independence, municipal identity resolution, operational authority, field reliability or causal service improvement. It does not replace the canonical Mycelix dependency/authority systems, Symtropy physical evaluators, Sol-Atlas spatial evidence, or operator controls.

The expected answers are public, so this is an open regression suite, not a blinded or sealed generalisation benchmark. The next independent step is to have a second reviewer author a disjoint challenge corpus from the contract and an independently adjudicated answer key, then keep that corpus out of the evaluator developers' tuning loop.
