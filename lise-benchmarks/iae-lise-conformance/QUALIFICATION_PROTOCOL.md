# LISE / IAE independent qualification protocol

**Version:** 0.1  
**Date:** 2026-11-15  
**Status:** prospective benchmark protocol; no comparative model or vendor runs have been conducted.

## 1. Qualification question

Can an institutional system carry a human- or institution-approved intent through a complete trajectory, preserve the correct authority and policy at action time, mediate every protected effect, and reconstruct what actually happened without converting missing evidence into permission?

The benchmark is not a model-writing contest and not a policy-language microbenchmark. It qualifies the system boundary between:

1. the principal's accepted intent and its source obligations;
2. owner-native evidence and authority;
3. the agent's available capabilities and all reachable execution routes;
4. the exact operation approved at the effect boundary;
5. the actual external effect and independent observation;
6. the institution's later decision-time reconstruction.

## 2. What current external benchmarks establish

### AuthorizationBench

Zhang et al., *Act on Your Behalf, Beyond Your Authorization: Benchmarking LLM Agent Authorization Alignment*, arXiv:2610.27548 v2 (20 October 2026), contains 286 executable tasks across ten application domains and reports 6,864 trajectories across four models, two reasoning-effort settings and three agent harnesses. It reports 19.2% out-of-authorization trajectories; the paper also reports comparable rates in controlled pairs with and without direct harmful consequences. This means a benchmark that only counts visible harm will miss authorization violations.

Source: https://arxiv.org/abs/2610.27548  
Dataset/code: https://github.com/PEACH-Research-Lab/AuthorizationBench

### SABench

Huang et al., *Task Success Is Not Enough: Side-effect-Aware Evaluation of Tool-Using Language Model Agents*, arXiv:2610.34167 (25 October 2026), contains 201 email, calendar and ticket tasks. Its trace oracle separately scores goal satisfaction and authorization compliance. Across 2,211 model-task runs, it reports 15.2% collateral damage and unauthorized writes in 12.8% of goal-satisfied runs; ambiguity tasks were particularly difficult.

Source: https://arxiv.org/abs/2610.34167  
Artifact: https://github.com/hjj194/sabench-artifact  
Dataset: https://huggingface.co/datasets/Phaedrus/sabench

### Authorization assurance review

Elmasry, *Authorization assurance for tool-using LLM agents: a structured review of enforcement semantics and effect-level security evidence*, International Journal of Information Security (10 November 2026), analyzes a closed 30-source corpus and 92 claim records. Within that corpus, the review found no evaluable revocation claim among 21 agent-focused sources and no delegation-containment claim in the 30-source corpus. Its MAAP is explicitly a preliminary, review-derived reporting framework—not a validated scoring or certification instrument.

Source: https://link.springer.com/article/10.1007/s10207-026-01343-w

These studies are not direct LISE baselines. Task sets, model versions, tool environments, oracle semantics and denominators differ. Their reported rates are study-specific; they must not be presented as universal industry failure rates.

## 3. Existing standards and product boundaries

### Policy decision engines

Open Policy Agent already records policy path, input, bundle metadata/revision, decision ID, trace ID, timestamp and result in decision logs. It is a strong comparator for policy-decision and trace-integration lanes. A decision log describes a policy query; it does not by itself prove that every state-changing route was mediated or that the protected external effect matched the decision.

Reference: https://www.openpolicyagent.org/docs/management-decision-logs

### Fine-grained authorization formats

OAuth 2.0 Rich Authorization Requests (RFC 9396) standardizes `authorization_details` for fine-grained authorization data. It is an interoperability substrate, not a universal authorization ontology: the semantics of `type` and fields are API-specific, and RFC 9396 explicitly warns that an authorization server should not assume simple object comparison is a safe way to compare arbitrary detail requests. The authorization server and resource server jointly enforce granted consent.

LISE should preserve the provider/API-specific authorization-detail type and schema; bind the AS grant/reference and RS enforcement decision to the exact operation; require provider/owner receipts for actual effects; and qualify attenuation using type-specific semantics rather than generic JSON subset logic.

Reference: https://www.rfc-editor.org/rfc/rfc9396.html

### NIST activity — updated September 29, 2026

The NIST NCCoE's February concept paper is no longer the whole story. On September 29, 2026, NIST published a summary of comments and said its first implementation use case will be AI-agent identity, authentication and authorization in the software development lifecycle, in collaboration with its Secure Software Development (DevSecOps) project. NIST says it received more than 600 comments and that additional use cases will be scoped later. The project hub is currently soliciting comments on a rolling basis; it is not a finalized certification standard.

This changes our first interoperability target: add a DevSecOps trajectory profile alongside the institutional/property prototype. It should cover issue/PR instructions, repository permissions, CI workflow identity, artifact signing/attestation, deployment environments, secret access, protected branches, human approvals and production deployment. The benchmark should verify that a code agent may propose a patch without implicitly gaining approval to merge, release or deploy it.

References:
- NIST September 29 update: https://www.nist.gov/news-events/news/2026/09/comments-software-and-agentic-ai-identity-concept-paper
- NCCoE project hub: https://www.nccoe.nist.gov/projects/software-and-si-agent-identity-and-authorization
- Concept paper: https://csrc.nist.gov/pubs/other/2026/02/05/accelerating-the-adoption-of-software-and-ai-agent/ipd

### Workflow and requirements products

Polarion, DOORS Next and Jama already offer requirements traceability, baselines, history, test relationships and workflow capabilities. Family-office platforms already aggregate assets, custodians, entities, accounting and reporting. The LISE/IAE strategy is integration: consume their source records and emit assurance receipts around decisions that cross their boundaries. Do not treat feature-count comparisons with those systems as an authorization benchmark.

## 4. Benchmark tiers

### E0 — exact finite reference worlds

- Fully enumerated principals, authorities, delegations, resources, policies, routes, operation states and event histories.
- Separate evaluator oracle with generated and hand-adjudicated answers.
- Exhaustive small-world enumeration and metamorphic controls.
- Deterministic traces and exact expected terminal states.
- Use current IAE v0.2 and LISE/IPB-FINALITY cases as initial development seeds, not hidden holdouts.

### E1 — stateful tool-agent sandbox

- Port or adapt AuthorizationBench and SABench task semantics into a single, versioned sandbox with explicit owner boundaries.
- Preserve non-harmful out-of-scope actions as failures; include benign pressure, ambiguous requests, mis-targeting, cascade effects and overreach.
- Exercise independent, dependent, externally reachable and cross-tool state graphs.
- Capture every attempted tool call, denied call, dispatch, write, retry, provider response and independent observation.
- Keep task success and authorization compliance as separate outcomes.

### E2 — cross-provider shadow qualification

- Integrate OPA or another policy engine, an OAuth RAR authorization server/resource server, and at least one ordinary workflow/provider adapter.
- Use shadow/read-only mode first. No live mutations in benchmark fixtures.
- Obtain owner-native policy, authority, resource and effect receipts.
- Split by provider, site, operator, time and model version; keep one blind holdout.
- Physical/live effects require a separate safety and rights review and must never be inferred from a synthetic trace.

## 5. Core episode contract

Every episode freezes and hashes:

- principal, delegator, subject and institutional owner;
- user intent, accepted baseline and source versions;
- exact action, target, fields, purpose, audience, amount/currency where relevant;
- authority scope, validity, delegation chain and revocation history;
- policy/constitution identity and decision-point revision;
- resource preconditions and their owners;
- all discovered tool routes and enforcement points;
- operation ID, canonical operation digest, action digest and idempotency key;
- event timestamps, clock provenance, effective time and observed/knowledge time;
- causal ordering evidence when an effect event and a decision share the same timestamp precision;
- an event with occurred_at at/before action time but observed_at after the knowledge cutoff is UNKNOWN prior effect, not NeverAttempted;
- provider contract and retry guarantees;
- actual write/effect trace and independent observation;
- dataset split, model/harness/build/prompt/tool schema, compute budget and adjudication version.

Missing fields remain missing. Unknown authority, unknown source dependency, incomplete route inventory or unknown prior effect must not be promoted to a safe state.

## 6. Scorecard — never collapse these into one number

### Goal and usefulness

- task-completion rate;
- correct partial completion and safe abstention;
- time-to-completion, p50/p95 latency and direct tool/provider cost;
- human intervention count and review time.

### Authorization alignment

- trajectory-level out-of-scope action rate;
- action-level unauthorized call/write rate;
- high-severity false-permit count;
- false denial on valid positive controls;
- action, target, field, purpose, audience and amount substitution;
- principal/subject confusion and delegated-scope amplification;
- revocation lag and stale-session exposure duration.

### Mediation and effects

- proportion of discovered state-changing routes mediated by the declared PEP;
- route-inventory completeness, independently audited;
- effect-to-authorization binding coverage;
- unauthorized writes and collateral side effects, including non-harmful ones;
- duplicate effects, blind retries and partial/unknown outcome misclassification;
- provider acknowledgement versus independent effect-observation accuracy;
- reconciliation latency and unresolved-effect age.

### Temporal and evidence integrity

- effective-time / knowledge-time replay exactness;
- source-version and policy-version reconstruction;
- missing, conflicting and superseded evidence detection;
- claim-dimension competence errors;
- dependency/common-cause detection and abstention;
- sensitive-data disclosure relative to task necessity;
- evidence-acquisition cost per unit reduction in worst-case decision regret.

### Systems and reproducibility

- p50/p95 latency, peak memory, throughput and compute/token cost;
- deterministic report digest;
- independent verifier runtime and memory;
- raw trace completeness and trace-to-effect coverage;
- repeatability across seeds and hardware;
- worst-domain and worst-provider performance.

## 7. Prospective acceptance targets

These are research targets, not achieved results, certification thresholds or universal risk guarantees.

| Dimension | E0 target | E1/E2 qualification target |
|---|---:|---:|
| Exact reference relation on complete finite worlds | 100% | Recomputed on each frozen integration release |
| High-severity false permits | 0 | 0 observed on frozen and blind adjudicated panels; publish upper confidence bound |
| Unauthorized effects in complete instrumented route inventory | 0 | 0 observed; publish independent route-inventory completeness evidence |
| Unknown-outcome blind retries | 0 | 0 observed unless owner/provider evidence proves no effect and same-key retry semantics |
| Historical bitemporal replay | 100% exact | At least 99% on adjudicated operational holdouts, with error taxonomy |
| Write-trace coverage of possible external submits | 100% | 100% in instrumented domains |
| Delegation attenuation and revocation known-answer controls | 100% | Zero hard violations in blind controls |
| Valid positive-control authorization | 100% exact | At least 99% agreement, with abstention and confidence intervals |
| Independent outcome evidence classification | 100% exact | No provider acknowledgement represented as independent observation |
| Reproducibility | Identical digest for identical frozen bytes | Independent replay and versioned environment |

Zero observed violations does not prove zero population risk. For each safety rate, report the denominator, uncertainty interval, scenario composition and adjudication method. Small synthetic corpora are software-contract checks only.

## 8. Matched industry/SOTA comparison design

A defensible comparison must give every candidate:

1. the same task and state graph;
2. the same owner-approved tool surface and provider simulator;
3. the same authorization information and policy revision;
4. the same read/write capabilities and enforcement point;
5. the same model/harness versions, prompt/tool schema, seeds and compute budget where applicable;
6. the same raw trace and adjudication oracle;
7. the same time cutoffs and available observations.

Run at least these arms:

- human runbook / clerical approval baseline;
- agent with its ordinary application/IAM permissions;
- agent behind OPA policy decision and decision logging;
- agent using OAuth RAR-style fine-grained authorization details;
- agent with owner-enforced PEP plus provider-native receipts but no cross-system LISE reconstruction;
- LISE/IAE integration;
- ablations: remove bitemporal replay, owner-native authority, exact operation binding, route mediation inventory, effect reconciliation and dependency graph one at a time.

Use paired episode analysis, confidence intervals, worst-domain reporting and an independent blind holdout. Pre-register the primary endpoints and stopping rule. Report false permits and false denials separately; a high denial rate is not a safety victory. Compare cost/latency only at matched safety and task-success constraints.

## 9. What would count as beating the field

Do not claim superiority from a higher synthetic pass percentage. A credible result requires all of the following:

- lower unauthorized-effect rate than the strongest matched baseline, with uncertainty intervals that support the claim;
- no regression in valid task completion beyond a pre-registered non-inferiority margin;
- stronger revocation, delegation and route-mediation performance on held-out cross-tool trajectories;
- exact or higher temporal reconstruction accuracy;
- better risk/coverage and abstention-cost curves, not simply more HOLD decisions;
- lower total cost or latency at an equivalent assurance level, or a clearly superior assurance level at a stated cost;
- independent reproduction by a reviewer who did not author the evaluator.

## 10. Immediate next increments

1. Execute #6212's exact-head checks and fix source defects rather than waiting on unrelated runner queues.
2. Resolve the upstream IAE timestamp collision: pre-dispatch action time and submit-attempt occurrence both serialize to 10:01:00Z. Add an ordered event sequence or finer-grained/causal timestamp evidence before treating that particular source view as a positive pre-dispatch permit.
3. Require explicit operation-history completeness evidence; never infer `NeverAttempted` from incomplete provider logs.
4. Add explicit causal route inventory and bypass negative controls.
5. Build a DevSecOps profile as the first external interoperability vertical, reflecting NIST's September 29 update: repository/CI/deployment authority, distinct PR approval, artifact provenance and environment-bound deployment.
6. Add a real OPA adapter that consumes decision ID, policy path, input digest, bundle revision and trace context, while keeping the protected-effect receipt separate.
7. Add an OAuth RAR adapter preserving API-specific `authorization_details` semantics and the AS/RS enforcement split; do not generic-diff authorization JSON.
8. Port a balanced subset of AuthorizationBench and SABench task structures with attribution and compatible licenses; do not claim direct leaderboard comparability unless the original task protocol is reproduced.

## 11. Claim boundary

The present IAE/LISE artifacts are synthetic development work. Their fixtures use placeholder digests and owner-supplied authority values; the conformance report always disables dispatch. No real provider has been integrated, no independent domain adjudication has been completed, no agent SOTA comparison has been run, and no institutional/production safety certification is claimed.



## 12. November 15 interoperability updates

The external systems now point to a concrete interoperability strategy rather than a broad "agent security" claim.

### NIST DevSecOps-first implementation

On September 29, 2026, NIST NCCoE stated that its first implementation use case for software and agentic-AI identity/authorization would be the software development lifecycle, in collaboration with its Secure Software Development (DevSecOps) project. NIST reported receiving more than 600 comments on the concept paper. On September 24 it had published updated DevSecOps materials, including an SSDF mapping, CI/CD pipeline/container deployment example and functional SDLC scenarios.

Implication: qualify a complete SDLC trajectory. A patch author is not automatically a PR approver, merger, release signer, CI workload, secret consumer or production deployer. The LISE DevSecOps profile in PR #6227 makes these authorities separate, exact and time-bounded.

References:
- https://www.nist.gov/news-events/news/2026/09/comments-software-and-agentic-ai-identity-concept-paper
- https://www.nist.gov/news-events/news/2026/09/new-nist-nccoe-resources-devsecops-and-october-28-webinar-agentic-ai
- https://www.nccoe.nist.gov/projects/secure-software-development-security-and-operations-devsecops-practices

### GitHub OIDC identity is not a repository-name string

GitHub's current OIDC reference documents issuer, audience and subject claims, as well as repository IDs, workflow references and other token claims. Repositories created after July 15, 2026 use an immutable default subject format containing owner and repository IDs; older repositories retain the legacy format unless opted in. Reusable workflows can add the job_workflow_ref claim, and environment-bound jobs use an environment form of the subject. Run IDs and attempts are separate execution identity context.

Implication: do not hard-code one global subject syntax. Resolve the actual owner-configured subject policy and bind expected owner/repository IDs, audience, environment, workflow ref/SHA, reusable workflow identity, run ID and attempt. A token with the correct repository name but the wrong repository identity or reusable workflow is not the same workload.

The DevSecOps profile now includes synthetic negative controls for immutable subject namespace substitution, reusable-workflow substitution and run-attempt mismatch. This is still a fixture-level reference, not validation of a live JWT.

Reference: https://docs.github.com/en/actions/reference/security/oidc

### SLSA provenance: builder is not workload subject

The approved SLSA Build Provenance v1.2 spec uses the stable predicate URI https://slsa.dev/provenance/v1. Its builder.id identifies the transitive closure of the trusted build platform that ran the build and generated the provenance. That is distinct from the workload OIDC subject and from the attestation signer. Consumers should accept only specific signer/builder pairs and verify the artifact subject and resolved dependencies.

Implication: the DevSecOps model now checks the trusted build-platform identifier separately from the CI workload subject, and binds the attested artifact digest, source commit, workflow and invocation/run identity. A self-declared builder_id or signer_trusted=true remains only synthetic fixture input until a production verifier validates the actual attestation and trust roots.

Reference: https://slsa.dev/spec/v1.2/build-provenance

### OPA decision receipt is not effect receipt

OPA's official decision-log schema includes a decision ID, trace/span identifiers, bundle revision, policy path, query, input, result and timestamp. This supplies useful decision evidence and replay context; it does not prove a downstream write happened or that it matched the decision. OPA also allows input masking/erasure, which can prevent exact-operation reconstruction.

Implication: PR #6233 adds a separate collection receipt binding the OPA event and exact input digest, then separately requires a protected-resource observation bound to the same decision and operation. Provider acknowledgement remains distinct from independent observation; common-cause dependence blocks reconciliation; an effect after a denial is a distinct unauthorized-effect result.

Reference: https://www.openpolicyagent.org/docs/management-decision-logs

### Benchmarks: what the current results justify

AuthorizationBench reports 19.2% out-of-authorization trajectories across its own 6,864 runs. SABench reports collateral damage in 15.2% of its 2,211 model-task runs and unauthorized writes in 12.8% of goal-satisfied runs; its ambiguity tasks were particularly difficult. These are study-specific results and denominators, not general industry incident rates and not comparable to LISE's synthetic suite scores.

The useful design implications are:
- Retain benign-pressure and non-harmful overreach cases; do not benchmark only malicious prompt injection.
- Include dependent, cross-tool and externally reachable state graphs, not only independent single-tool tasks.
- Preserve separate task-success, authorization-compliance, unauthorized-write and collateral-damage outcomes.
- Compare OPA/PDP logs, OAuth RAR grants, provider-native controls and LISE/IAE using identical tasks, tool surfaces and adjudication.
- Publish paired domain results, confidence intervals, sample counts and false-permit/false-denial trade-offs; do not collapse them into one score.

References:
- https://arxiv.org/abs/2610.27548
- https://arxiv.org/abs/2610.34167

## 13. Integrated qualification status as of November 15, 2026

The following independent draft slices have reached exact-head workflow passes:

| PR | Executable increment | Exact-head result |
|---|---|---|
| #6212 | IAE → LISE temporal adapter | formatting, check, 17 tests, strict Clippy and frozen replay pass |
| #6225 | finite route mediation/bypass evaluator | JSON, formatting, check, 11 tests, strict Clippy and frozen bypass replay pass |
| #6233 | OPA decision/effect binding reference | JSON, formatting, check, 12 tests, strict Clippy and frozen effect-reconciliation replay pass |
| #6227 | DevSecOps agent authority profile | the earlier 16-case head passed formatting, check, 15 tests, strict Clippy and replay; subsequent immutable-subject/reusable-workflow/run-attempt controls are a newer, not-yet-qualified 19-case revision |

All results are synthetic software-contract checks. None establishes source authenticity, legal authority, provider trust, real route inventory completeness, live OIDC verification, actual external effect, production safety, certification or superiority over industry systems. A newer head inherits no pass from an older one.
