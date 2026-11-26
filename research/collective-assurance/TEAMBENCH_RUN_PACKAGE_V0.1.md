# TeamBench Run Package Adapter v0.1

**Status:** read-only Rust ingestion and qualification contract with synthetic mock fixture. It does not execute the upstream benchmark, reimplement its task harness, replace the native deterministic grader, or claim a Mycelix performance result.

## Upstream authority

Use the official upstream source at https://github.com/ybkim95/TeamBench and its MIT license. Pin the exact upstream commit, data/task manifest and native grader. The exact upstream CLI condition IDs are `oracle`, `restricted`, `team_no_verify`, `team_no_plan` and `full`. Their role topology is not interchangeable: Oracle is one agent with full spec/workspace/exec/write access; Restricted is one executor-only agent with brief/workspace/exec/write access; Team-NoVerify is Planner+Executor with no Verifier (the harness writes a stub attestation); Team-NoPlan is Executor+Verifier without a Planner; Full is Planner+Executor+Verifier with remediation. Preserve these exact identifiers and semantics.

The official upstream project documents a deterministic mock-provider smoke path. Run it before using model/API credentials. A mock run can prove only that the harness and ingestion path work; it is never eligible for a model-performance score.

## Canonical task-manifest commitment

The suite carries both the upstream Git commit and exact Git blob identities for the v2.1 task manifest and the README re-curation notice. The adapter recomputes a domain-separated SHA-256 over those source identities, the sorted eligible task IDs, and a separately recomputed exclusion-manifest digest. The fixture uses the actual pinned upstream commit `d185aef1916fd86a9ba554d581fd256319a973af`, manifest blob `59125b3fa9a82eede1f5a832c51c2ba32f5af591`, and README blob `6c39b66fed83c52f397c2d3b4f84e7bafe854061`.

The checked-in `benchmark-manifests/teambench-leaderboard-90-v2.1.json` preserves all 90 official entries, with 89 currently evaluable task IDs and one explicit re-curation exclusion (`GH120_redis-py_3863`). A suite run must bind that exact manifest and explicit exclusion rationale; the synthetic example fixture uses synthetic task IDs only to test the adapter and is not the 90-task benchmark dataset.

## Package contract

The suite manifest binds:
- official repository URL and 40-hex upstream commit;
- benchmark version, dataset and task manifests, native grader digest;
- exact eligible task IDs and a separate excluded-task manifest with per-item reason/evidence;
- shared model, tokenizer, prompt, tool, decoding and seed configuration;
- explicit per-task time, token and compute budgets;
- exactly one run for each of the five official conditions.

Each condition run records its system and role-profile digests, run identity, agent count, exact role inventory, execution backend, optional runtime-isolation manifest, completeness of role-boundary observation inventory and raw-result manifest digest. The execution backend distinguishes in-process tool/path boundary enforcement from separately evidenced Docker/container isolation. The validator checks the role inventory against the upstream condition definition; verifier presence is derived from that inventory rather than trusted as a separate boolean. Each task row binds raw output bytes and native-grader output bytes by digest, native pass/fail disposition, the native partial score in fixed-point millionths, optional verifier disposition and role-boundary observations. The native partial score is retained for score/TNI analysis rather than discarded after thresholding.

Task inventories must match exactly across conditions. The official TeamBench-90 task manifest contains 90 IDs; the upstream README currently marks `GH120_redis-py_3863` as under re-curation, so the other 89 are currently fully evaluable. Record the 90-task official manifest plus a per-task exclusion record; do not silently replace the official denominator with an unversioned 89-task local split. Missing rows are INDETERMINATE, not silently dropped; extra, duplicate or excluded task IDs are FAIL.

## Verifier audit

False acceptance is counted when the separate Verifier accepts a submission that the native deterministic grader marks Fail. Its denominator is all known grader-failing tasks in conditions that include the Verifier (`team_no_plan` and `full`). False rejection is the reverse and uses all known grader-passing tasks in those conditions. `team_no_verify` writes a stub attestation automatically; that is not a verifier judgement and must never enter false-accept statistics. Explicit NoVerdict, missing verdicts, unknown grader results and infrastructure failures are retained as separate counts/states.

Role-boundary events bind the role, exact boundary ID, policy's expected allow/deny disposition, independently observed outcome and evidence digest. An observed allow on a deny boundary is a role-enforcement violation; a blocked action that policy allowed is a false denial. An incomplete boundary inventory or unknown observation leaves the role-enforcement axis unqualified. These observations do not rewrite or invalidate the native task score. Benchmark-score eligibility and OS-isolation eligibility are separate: an in-process tool-boundary run may reproduce the official ablation score, but cannot establish OS/container isolation. OS-isolation eligibility requires a separately committed runtime manifest for every condition.

## Resource and cost evidence

Native benchmark scores remain eligible when task results are complete, even when a benchmark release does not expose full usage telemetry. Cost-adjusted comparisons are a separate qualification: every task in every condition must have an independently committed usage observation before aggregate totals are published.

Per-task resource observations record input/output tokens, wall-clock time, provider cost in micro-USD, tool calls and inter-agent messages, plus an evidence digest. Missing or invalid observations are counted as unknown; they are not treated as zero. The aggregate resource table stays empty unless all eligible task-condition cells are complete. This makes cost-adjusted TeamBench uplift auditable without penalizing a score reproduction for telemetry that the upstream artifact does not provide.

## Exact upstream score composition

The adapter preserves each task's native partial score on a fixed-point millionths scale and reproduces the formulas in upstream `harness/compute_tni.py`:

- Necessity gap: `Oracle − Restricted`.
- Per-task TNI: `clamp((Full − Restricted) / (Oracle − Restricted), −2, 2)`; omit when the absolute necessity gap is below 0.05.
- Aggregate TNI: average only the per-task TNI values whose necessity gap is greater than 0.05, matching the upstream `to_json` aggregation.
- Team uplift: `Full − Restricted`; team-helps count uses uplift greater than 0.01.
- Collaboration efficiency: `(Full − Restricted) / Oracle`, or zero when Oracle is below 0.01.
- Planning value: `Full − TeamNoPlan`; verification value: `Full − TeamNoVerify`; preserve the upstream missing-value convention when both component scores are zero.

Mock-provider smoke and incomplete suites expose **no aggregate score or TNI fields**. Fixed-point rounding can differ from the upstream Python float representation by at most a small final-digit amount; the raw native partial scores remain preserved for independent recomputation.

## Comparison rules

The adapter does not compute a universal TeamBench score. Preserve the native task score and Team Necessity Index where provided, then report per-condition and paired outcomes, solo-performance strata, verifier false accept/reject, role-boundary violations, missing verdicts, tokens, wall-clock latency, inter-agent messages, compute and total monetary cost separately.

Match exact model snapshot, tokenizer, tool versions, dataset, task IDs, native grader and per-task budgets across conditions. Differences that define the ablation (agent topology/roles and corresponding system/role profile) remain explicit in each condition manifest. The full five-condition set is required before a suite is eligible for a model score.

## Independent qualification axes

The report intentionally does not collapse benchmark correctness, verifier quality, role enforcement, OS isolation and cost evidence into one status:

- **Native benchmark score:** eligible only when the pinned official task manifest, all five native conditions, raw outputs and deterministic grader dispositions/scores are complete.
- **Verifier audit:** reports false accepts, false rejects, NoVerdict, and missing verifier evidence separately. Missing verifier evidence does not erase a valid native score; it makes the verifier-audit axis incomplete.
- **Role-boundary audit:** reports violations, false denials, unknown observations and incomplete inventories independently. A boundary violation does not rewrite the native task score, but it makes the enforcement qualification fail.
- **OS isolation:** requires Docker/container runtime evidence for every condition. An unknown or in-process backend blocks this claim without invalidating a faithful native score reproduction.
- **Cost/resource comparison:** requires independently committed telemetry for every task-condition cell. Missing cost data remains unknown and never becomes zero.

This is a multi-axis assurance vector, not a single trust score. A result can be benchmark-score eligible while verifier, enforcement, OS-isolation or cost evidence remains incomplete. Conversely, complete security telemetry cannot repair missing native task scores or invalid benchmark provenance.

## Execution-boundary evidence correction

The adapter must not infer OS isolation from the benchmark's role names, role-profile digest, or a successful tool-boundary test. The official `harness.ablation` path uses `AgentLoop` with `ReadFileTool`, `WriteFileTool` and path maps; the upstream `agent_interface.py` labels its path mapping as being for **non-Docker agent runs**. Its tools execute in-process and enforce file access through application-level path checks. The repository also contains a Docker Compose role topology and interactive `run_task` path, but that does not by itself prove that a given ablation result used that backend.

Therefore each condition explicitly records one of:
- `in_process_tool_boundary`: logical tool/path restrictions only; no OS-isolation claim.
- `docker_container_isolation`: independently captured container runtime manifest is mandatory.
- `unknown`: qualification is INDETERMINATE.

The report exposes `benchmark_score_eligible` and `os_isolation_eligible` independently. This is deliberate: it preserves comparability of a faithful upstream score reproduction without silently upgrading application-level restrictions into an OS-enforced security claim. The host-side tool wrappers also merit a separate path-normalization and symlink-escape adversarial audit before any strong containment claim.

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
