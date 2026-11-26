# MCSF Collaborative Memory Authority Qualification v0.1

## Why this plane exists

A team memory is not correct merely because it retrieves the right text. The memory must reach the right principal, only for the right purpose, carry the real source and writer, resolve conflicting facts by an explicit authority rule, and stop being usable when retention or revocation says it must stop.

The 2026 COLLABMEM paper formalizes this class of problem with a matched memory-only harness. Its published abstract reports 58 hand-authored scenarios (1,118 core probes), a 174-scenario scaled split (3,359 probes), six transactive-memory competencies, and a 69.6% best non-Oracle average over five splits. See the [ACL Anthology paper](https://aclanthology.org/2026.emnlp-main.1194/).

The paper page states that the dataset, evaluation code and reproduced outputs will be released. This Mycelix module therefore does **not** claim official COLLABMEM reproduction or compatibility from the paper's summary alone. Its first implementation is an independent native qualification contract that can later consume pinned official artifacts.

## Six independently scored competencies

| Competency | Required observation | Critical failure examples |
|---|---|---|
| Sharing | Expected and observed recipient sets | Any additional recipient or unauthorized content exposure |
| Access | Expected and observed allow/deny for principal, scope and purpose | Denying a grant that should pass is a functional failure; granting a denied request is a hard failure |
| Routing | Intended role and all roles that actually received the memory | Any unrelated role receives content |
| Provenance | Source-event digest, writer identity and authority digest | Any mismatch is treated as forged or misbound provenance |
| Conflict resolution | Gold current-value digest and authority rank versus selected value/rank | Lower-authority winner or stale current value |
| Retention | Expected and observed retention plus stale, revoked and derived-descendant use | Retained beyond the declared boundary, post-revocation use, or use of a revoked derivative |

Every probe also records content exposure and forbidden metadata inference. Those are non-compensatory: high sharing or retrieval scores cannot offset an unauthorized disclosure.

There is intentionally no weighted composite. Each competency has its own denominator, pass count and score. Missing or incomplete observers make that competency's score unavailable and the run non-evaluable; they do not turn missing evidence into a pass.

## Matched-control commitment

The control digest binds:
- scenario/dataset manifest, ordered trace, role inventory and gold event;
- answerer model, judge, prompt, tools, decoding configuration, hardware and token/resource budget;
- memory adapter identity/version and memory policy;
- explicit deterministic seed;
- optional official dataset, harness and split artifact digests.

The run digest then binds that control commitment to every probe and its independent evidence digest. Changing the answerer, target role inventory, judge, model policy, trace, budget or adapter changes the controls digest. A run must be compared only against another run with the same control digest.

`official_artifact_references_complete` only reports whether all three official dataset/harness/split digest slots are filled with well-formed SHA-256 values. It is a structural completeness check, not authentication of their origin. Official reproduction still requires verifying those artifacts against the benchmark authors' actual published release and using its exact task/split semantics, answerer, judge and harness.

## Evidence boundary

- `synthetic_fixture` is contract testing only. The Rust qualifier returns INDETERMINATE and no empirical scores for it.
- `observed_run` requires complete independent observation and a valid matched run commitment.
- An incomplete observer, missing evidence digest, duplicate probe ID or missing competency blocks the affected scorecard.
- Unauthorized access, sharing, routing, content exposure, metadata inference, forged provenance, stale authority, retention violation and post-revocation use are FAIL conditions.
- This contract does not itself enforce runtime authorization, isolate agents, erase storage, authenticate external attestations, or prove that an operating memory product obeys the declared policy.

## First measurement sequence

1. Run the six-axis contract against a small deterministic set of native synthetic fixtures; use those only to verify evaluator behavior.
2. Once the official COLLABMEM corpus and harness are published, pin their release digest and port the exact scenario/split semantics. Do not infer the official dataset from paper examples or invent substitute scenarios.
3. Reproduce the published non-Oracle baseline on the exact answerer/judge/harness before comparing Mycelix.
4. Evaluate Mycelix role-filtered projections, provenance graph and temporal authority views on the same frozen trace and target roles.
5. Add hard adversarial cases for colluding roles, stale mirrors, revoked ancestors, cache/index remnants and a low-authority writer attempting to supersede a high-authority fact.
6. Publish each competency, critical failure count, latency/cost and observer completeness separately. No claim of SOTA until the official reproduction and an independent run are both available.
