# Authorized Action Denominator Benchmark Protocol V1

Status: proposed synthetic qualification protocol. The reference core and test suite are under development; no field benchmark or comparative performance result is claimed.

## 1. Benchmark question

Given one ratified, versioned service-action plan and a set of independently sourced observations, can a verifier establish coverage of the exact authorized action set without allowing denominator shrinkage, duplicate inflation, wrong-context evidence, common-cause duplication, stale evidence or post-hoc relabelling?

The evaluated object is not a mutable integer. It is the canonical set of action records committed by one immutable plan digest.

## 2. Correctness baselines

Compare distinct designs, using the same frozen service episode, action list, evidence records, timestamps and amendment schedule.

| Baseline | Strength | Limitation to record |
|---|---|---|
| Mutable count / spreadsheet baseline | Simple action-count arithmetic and low processing cost | A count alone does not bind action identity, scope, revision or evidence lineage |
| Ordinary CMMS/work-order checklist | Assignment, task status, completion notes and work history | Capture the vendor's native task-set revision and evidence semantics; do not presume it has or lacks immutable manifest binding |
| ITSM task/checklist workflow | Incident-to-task relationship, assignment and escalation | Record whether task membership is frozen at authorization or can change in-place |
| Event-sourced task history | Append-only updates and deterministic state replay | Replay does not automatically establish that the denominator was authority-ratified or complete |
| Mycelix immutable denominator | Canonical action-set digest, authority-bound amendments, plan-scoped observations, independent-group accounting and explicit blockers | Current reference core does not authenticate authority/source truth or live work completion |
| Independent set-based oracle | Frozen mathematical expected result independent of production code | Oracle is test evidence, not an operational authorization service |

A baseline may have native immutability or stronger source authentication. Test actual configured behavior and version, not marketing labels. If a comparator cannot represent an obligation, mark it “not represented”; do not invent a failed result.

## 3. Frozen correctness matrix

For each test, preserve the plan JSON-equivalent typed packet, exact canonical action set, observations, authority decision references, time basis, expected action dispositions and expected report digest.

Minimum matrix:
- 6 authorized / 5 qualified observations -> Incomplete, exact uncovered action ID.
- 6 authorized / 6 qualified observations -> CompleteForAuthorizedPlan only.
- same count but one action substituted -> different plan digest.
- input action ordering and prerequisite ordering permuted -> identical canonical plan digest.
- denominator shrinks or expands without a new sequential authority decision -> amendment validation fails.
- valid amendment P2 supersedes P1 -> P1 historical result remains unchanged; P2 evaluated independently.
- identical repeated observation ID/content -> idempotent.
- repeated observation ID with changed content -> Conflict.
- repeated observations from one common-cause group -> count once.
- evidence-group revocation or explicit expiry marker -> that group no longer contributes, even when it had an earlier positive observation.
- old observation whose own validity lease elapsed -> ignored, but does not revoke a newer fresh observation from the same source group.
- disputed evidence group -> Conflict.
- wrong plan / episode / zone / jurisdiction / context / unknown action -> rejected and not counted.
- stale or future observation -> not counted.
- revoked permit/crew qualification or unsatisfied prerequisite -> action remains blocked pending a separately qualified authorization owner.
- prerequisite DAG cycle, missing prerequisite and duplicate action ID -> reject at plan sealing.
- plan queried before issue time or after expiry -> Indeterminate.
- exact action coverage -> still no service closure or external effect authorization.

The independent oracle operates on the canonical set of action IDs, a separate revision relation and frozen observation records. It must not call the Rust evaluator or use its dispositions as expected values.

## 4. Proposed qualification targets (not results)

- zero denominator shrink/expansion/substitution without explicit authority-bound new revision;
- zero numerator inflation by duplicate, split, wrong-plan or wrong-context observations;
- 100% exact plan digest parity across input permutations that preserve set semantics;
- 100% exact covered/uncovered ID parity with the independent set oracle;
- zero stale/revoked/disputed evidence counted as current;
- zero common-cause independence inflation;
- 100% deterministic historical replay by plan digest;
- zero false CompleteForAuthorizedPlan outcomes;
- explicit abstention/Conflict rate and false-hold rate reported separately.

No mutation score is reported until every registered transformation is executed and each result compared to the independent oracle.

## 5. Workload sizes and performance

Run canonical action plans at 1, 10, 100, 1,000 and 10,000 actions. For each size vary:
- 0%, 1%, 10%, 50%, 99% and 100% observed actions;
- 1, 2, 3 and 5 independent evidence groups per action;
- 0%, 1%, 10% and 100% observations duplicated;
- 0%, 1%, 10% and 100% actions amended in a new plan revision;
- 0%, 1%, 10% and 100% expired/revoked/disputed source groups;
- prerequisite graph depth 0, 1, 5 and 20, with fan-out 1, 10 and 100.

Report p50/p95/p99 plan-seal, observation validation and evaluation latency; CPU; peak memory; canonical plan bytes; observation bytes; report bytes; and full recompute versus incremental amendment cost. Freeze CPU/RAM/OS, toolchain, lockfile, source SHA, input/output digests, seeds and concurrency. Publish raw machine-readable results.

Correctness gates precede all performance comparisons. A faster mutable counter is not equivalent to a verifier that recomputes action identity, authority revision and evidence scope.

## 6. Claim ceiling

The reference core proves deterministic accounting over typed, caller-supplied plans and observations. It does not authenticate the issuing authority, signatures, source observations, permits, professional approvals, field completion, service restoration, legal effect or permission to perform an external action. A production composition must bind its inputs to the actual ratification, identity, permit, evidence-lineage and decision owners.
