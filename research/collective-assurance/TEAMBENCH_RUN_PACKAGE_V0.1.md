# TeamBench Run Package Adapter v0.1

**Status:** read-only Rust ingestion and qualification contract with synthetic mock fixture. It does not execute the upstream benchmark, reimplement its task harness, replace the native deterministic grader, or claim a Mycelix performance result.

## Upstream authority

Use the official upstream source at https://github.com/ybkim95/TeamBench and its MIT license. Pin the exact upstream commit, data/task manifest and native grader. The public protocol exposes five ablation conditions: Solo, Restricted, Full, No Plan and No Eval. Preserve upstream identifiers and semantics; do not create an unrecorded condition alias.

The official upstream project documents a deterministic mock-provider smoke path. Run it before using model/API credentials. A mock run can prove only that the harness and ingestion path work; it is never eligible for a model-performance score.

## Package contract

The suite manifest binds:
- official repository URL and 40-hex upstream commit;
- benchmark version, dataset and task manifests, native grader digest;
- exact eligible task IDs and a separate excluded-task manifest with per-item reason/evidence;
- shared model, tokenizer, prompt, tool, decoding and seed configuration;
- explicit per-task time, token and compute budgets;
- exactly one run for each of the five official conditions.

Each condition run records its system and role-profile digests, run identity, agent count, verifier presence, completeness of role-boundary observation inventory and raw-result manifest digest. Each task row binds raw output bytes and native-grader output bytes by digest, task state, native disposition, optional verifier disposition and role-boundary observations.

Task inventories must match exactly across conditions. Missing rows are INDETERMINATE, not silently dropped; extra, duplicate or excluded task IDs are FAIL. Re-curated/excluded items remain explicit and cannot disappear from the denominator unnoticed.

## Verifier audit

False acceptance is counted when the verifier accepts a submission that the native deterministic grader marks Fail. Its denominator is all known grader-failing tasks in verifier-enabled conditions. False rejection is the reverse and uses all known grader-passing tasks in verifier-enabled conditions. Explicit NoVerdict, missing verdicts, unknown grader results and infrastructure failures are retained as separate counts/states.

Role-boundary events bind the role, exact boundary ID, policy's expected allow/deny disposition, independently observed outcome and evidence digest. An observed allow on a deny boundary is a hard failure. An incomplete boundary inventory or unknown observation is INDETERMINATE. A blocked action that policy allowed is reported as a false denial rather than hidden inside the violation count.

## Comparison rules

The adapter does not compute a universal TeamBench score. Preserve the native task score and Team Necessity Index where provided, then report per-condition and paired outcomes, solo-performance strata, verifier false accept/reject, role-boundary violations, missing verdicts, tokens, wall-clock latency, inter-agent messages, compute and total monetary cost separately.

Match exact model snapshot, tokenizer, tool versions, dataset, task IDs, native grader and per-task budgets across conditions. Differences that define the ablation (agent topology/roles and corresponding system/role profile) remain explicit in each condition manifest. The full five-condition set is required before a suite is eligible for a model score.

## Initial execution sequence

1. Pin official upstream commit, task release and native grader; record exact exclusions and source rationale.
2. Run the official mock-provider smoke path twice without provider credentials. Compare canonical task/outcome manifests after excluding explicitly nondeterministic runtime metadata.
3. Ingest both raw bundles and prove repeatable canonical qualification.
4. Execute all five official conditions on the exact eligible task set with a named model, matched budgets and native deterministic grader.
5. Independently replay the native grader and compute verifier/role/cost metrics with denominators and intervals.

## Current benchmark comparison

TeamBench's published evaluation reports conditional team value rather than a universal multi-agent advantage: the weakest Solo-performance quintile improved by 15.7 percentage points, the strongest quintile lost to coordination overhead, and mean uplift in one reference pool was 0.5 points (p=0.20). The role-mixing pool reported 49.4% verifier false acceptance on grader-failing submissions; a separate TeamBench-Verified evaluation reported 38.7%. These are distinct subsets/protocols and are not interchangeable baseline values. Reproduce the intended official subset and native metrics before comparing.

## Tests and claim ceiling

The Rust unit suite tests exact five-condition coverage, pinning, mock-score exclusion, task-set parity, duplicate/extra tasks, explicit exclusions, false-accept and false-reject denominators, NoVerdict accounting, incomplete grader evidence, role-boundary violations, unknown boundary inventory, infrastructure errors and verifier absence/missing verdict.

A PASS qualifies only the completeness and internal consistency of the supplied manifests and observations. It does not authenticate the upstream repository or result bytes, establish real role isolation, validate the provider's reported model identity, measure human outcomes, or prove model/verifier quality outside this finite run. The Rust adapter is not a substitute for running the Python/Docker upstream harness; no production Python dependency is added to Mycelix.
