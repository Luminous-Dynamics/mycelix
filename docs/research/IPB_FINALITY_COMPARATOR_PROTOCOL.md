# IPB-FINALITY — matched comparator and SOTA protocol

**Status:** preregistration proposal for a synthetic development benchmark; not an achieved performance comparison.

## 1. The comparison problem

Authorization engines, AI-agent benchmarks and execution-finality protocols answer different questions. A policy decision can be correct while an application ignores it. An agent can finish the task while taking an unauthorized side effect. A provider acknowledgement can be authentic while failing to establish what the protected resource actually did.

The comparator protocol therefore separates five outcomes:

1. **Task utility:** did the system complete the legitimate goal?
2. **Authorization correctness:** did the exact operation remain within the principal's current authority and policy?
3. **Effect safety:** did the system avoid unauthorized or duplicate protected effects?
4. **Outcome accounting:** did it preserve unknown and partial outcomes and reconcile them?
5. **Assurance evidence:** can an independent reviewer reconstruct the decision and verify the relevant trace?

No weighted average may offset a high-severity unauthorized effect.

## 2. Current external benchmark reference points

### AuthorizationBench (20 October 2026)

AuthorizationBench evaluates whether tool-using agents remain within the scope actually granted by the user. It contains 286 executable tasks across ten application domains. The authors ran four models at two reasoning-effort levels through three agent harnesses, producing 6,864 trajectories. The reported out-of-authorization behavior rate was 19.2%, with a 10.5–26.6% range across 24 configurations. Their controlled principal-consultation comparison reports 14.0% versus 21.6% out-of-authorization behavior with versus without consultation. Those numbers are specific to the authors' tasks, sandbox and model/harness configurations; they are not direct scores for LISE.

Paper and artifact: https://arxiv.org/abs/2610.27548  
Dataset/code: https://github.com/PEACH-Research-Lab/AuthorizationBench

### SABench (25 October 2026)

SABench evaluates collateral side effects in tool-using agents with 201 email, calendar and ticket tasks. The 2,211 model-task runs cover four state structures (independent, dependent, externally reachable, cross-tool) and four trap families (ambiguity, mis-targeting, cascade harm, overreach), with deterministic trace oracles for task goals and authorization compliance. The paper reports 15.2% collateral damage across runs, 38.8% violation on ambiguity tasks versus 8.4% on other traps, and 12.8% unauthorized writes among goal-satisfied runs. Cross-tool and dependent states had about twice the collateral-damage rate of independent states.

Paper and artifacts: https://arxiv.org/abs/2610.34167  
Dataset: https://huggingface.co/datasets/Phaedrus/sabench  
Code and traces: https://github.com/hjj194/sabench-artifact

### Zanzibar (USENIX ATC 2019)

Zanzibar is a mature production authorization system, not an agent benchmark. The published design reports causal ordering/external consistency for authorization decisions amid ACL and object changes, p95 latency below 10 ms, and availability above 99.999% over three years of production use. It is a demanding comparator for relationship-based permission consistency and scale. It does not claim to provide external-effect finality, provider reconciliation or constitutional adjudication.

Paper: https://www.usenix.org/conference/atc19/presentation/pang

### Cedar / Amazon Verified Permissions

Cedar's authorization request is principal–action–resource–context. The authorizer evaluates applicable policies and returns Allow or Deny with diagnostics. Cedar explicitly uses skip-on-error: a policy that returns an evaluation error is skipped and its error appears in diagnostics. Applications can choose to treat a mandatory guardrail error as a separate hold; the benchmark tests that integration behavior rather than labeling Cedar's policy semantics defective. AWS documentation also states that the application is responsible for enforcing the decision it receives.

Cedar algorithm: https://docs.cedarpolicy.com/auth/authorization.html  
Verified Permissions: https://docs.aws.amazon.com/verifiedpermissions/latest/userguide/

### Open Policy Agent (OPA)

OPA decision logging supports filtering and rate limits; configured drop rules and rate limiting can drop log events. The benchmark must therefore distinguish decision execution, log capture, effect observation and absence of evidence. A missing decision log cannot establish that no decision or operation occurred.

Documentation: https://www.openpolicyagent.org/docs/management-decision-logs

### NIST AI RMF critical-infrastructure profile

NIST describes the Trustworthy AI in Critical Infrastructure profile as an ongoing profile-development project, with a community of interest. Treat it as a risk-management alignment reference, not a completed standard, conformance test or certification.

Project: https://www.nist.gov/programs-projects/concept-note-ai-rmf-profile-trustworthy-ai-critical-infrastructure

## 3. Matched comparator arms

Run every arm on the same frozen input, same owner-supplied current-state snapshots, same provider capability declarations, same time cutoffs and same compute/reviewer budgets.

| Arm | Implementation under test | What it should demonstrate | What it does not establish by itself |
|---|---|---|---|
| A — human expert | Two independent reviewers plus adjudication | Correct scope and evidence reconstruction; review time | Runtime enforcement or provider safety |
| B — fixed checklist | Structured manual approval and review workflow | Human baseline quality and burden | Automated path coverage or race handling |
| C — OPA | Version-pinned policy bundle and PEP adapter | Policy decision, input/revision binding, decision logging coverage | Effect completion or exactly-once provider behavior |
| D — Cedar / Verified Permissions | Version-pinned Cedar schema/policies and application adapter | PARC evaluation, diagnostics, skip-on-error integration handling | Application enforcement correctness unless the adapter is tested |
| E — relationship authorization | Zanzibar-style or equivalent consistency-aware tuple/relationship service | Current relationship checks, causal ordering, stale ACL behavior | Exact external operation and provider-effect finality |
| F — workflow/approval | Signed approval and conventional workflow system | Approval identity, separation of duties, audit and change handling | Current operation binding unless tested at execution |
| G — agent + LISE composition | Candidate agent/tool layer plus owner-native authority/finality interfaces | End-to-end claim/evidence/authority/effect reconstruction | General superiority outside tested scope |

Do not compare an OPA policy engine alone against a complete agent/workflow composition. The system boundary must include the application-side enforcement code for OPA and Cedar. Publish exact versions, policy bundles, adapters, entity data, configuration digests and unsupported features.

## 4. Frozen test protocol

### Development panel

The current 34-case panel is a public development seed:
- 8 positive controls;
- 15 high-severity authorization attacks;
- 11 recovery, partial, unsupported or unknown cases.

(The exact class denominators are derived from the input manifest at evaluation time; these categories are not a universal distribution.)

The input manifest and known-answer oracle are separate. The candidate receives only the input manifest. The evaluator requires a one-to-one result for every scenario ID and pins the exact input and oracle Git blob identities. This is a development set, not a hidden holdout.

### Confirmatory panel

Before an external superiority claim, preregister a separately held-out panel of at least 200 scenarios across:
- at least four asset/resource domains;
- at least three source-system or provider families;
- at least two jurisdictional or institutional-authority profiles;
- at least five distinct authorization/side-effect classes;
- one-field and compound mutation controls;
- concurrency, crash, timeout, failover, partial-effect and delayed-revocation cases.

Use two independent adjudicators with a recorded disagreement protocol. Freeze the scenario generator, seed set, oracle version and evaluator before exposing confirmatory cases.

### Repeated runs

For stochastic systems, run at least 30 seeds per scenario in the development/confirmatory comparison. Report per-scenario failure counts, not just pooled mean rates. Use scenario-clustered uncertainty intervals or bootstrap procedures so repeated runs on one scenario do not masquerade as independent task variety. Preserve worst-case results for all high-severity scenarios.

## 5. Metrics — separate dimensions

### Non-compensatory safety and authority

- High-severity false-PASS / false-ALLOW count and denominator.
- Unauthorized dispatch and protected-effect counts.
- Operation substitution accepted by field: principal, action, target, recipient, amount, purpose, destination, audience, resource revision, canonical digest.
- Stale constitution, policy, authority, resource and fencing acceptance.
- Delegation amplification by edge and scope dimension.
- Excess trajectory-wide effective privilege.
- Reachable unmediated routes to protected effects.
- Post-revocation active/cached/delegated session acceptance.
- Blind retries after unknown outcomes.
- Duplicate effect count under replay, concurrency, crash and failover.

### Completeness and utility

- Positive-control false-denial rate.
- Task-success rate under the same task definition.
- Clarification precision, recall and unnecessary-interruption burden.
- Correct handling of unsupported provider semantics.
- Correct distinction among not attempted, confirmed no effect, fully applied, partially applied and unknown.

### Evidence and operations

- Required-assertion precision/recall.
- Claim-to-source and claim-to-authority coverage.
- Independent observation coverage and known shared-dependency rate.
- Bitemporal reconstruction accuracy.
- Reconciliation time p50/p95/p99.
- Revocation propagation lag p50/p95/p99.
- Authorization overhead p50/p95/p99; throughput; memory; provider calls; operator/reviewer time; total cost per qualified decision.
- Reproduction rate from frozen inputs, binaries, configuration, raw traces and SHA-256 receipts.

Do not collapse these into a single benchmark score. Publish the vector and the raw per-case matrix.

## 6. Prospective targets — not achieved results

For the finite known-answer panel:
- zero high-severity false permits;
- zero post-revocation allows;
- zero accepted exact-operation substitutions;
- zero accepted delegation amplification;
- zero reachable bypass accepted as complete mediation;
- zero blind retries after unknown outcomes;
- 100% preservation of partial/unknown outcome distinctions;
- at least 95% agreement on benign controls after independent adjudication.

For the held-out panel:
- target zero high-severity false permits and report the one-sided confidence bound, not simply "0% observed";
- target 100% correct classification of unsupported and unknown cases;
- reduce expert reconstruction time by at least 30% without increasing critical errors;
- no task-success improvement may count as progress if unauthorized side effects increase.

These targets are preregistration goals, not evidence of achieved performance or a claim to beat current SOTA.



## 8. Statistical power, rare failures and scenario independence

A zero observed failure count is not a zero failure probability. Under an idealized independent Bernoulli model, after zero failures in (n) independent trials, the exact one-sided 95% upper confidence bound is:

`1 - 0.05^(1/n)`.

For orientation:

| Independent scenario trials, zero failures | One-sided 95% upper bound |
|---:|---:|
| 34 | 8.43% |
| 200 | 1.49% |
| 300 | 0.994% |
| 1,000 | 0.299% |
| 2,995 | 0.100% |
| 29,956 | 0.010% |

These are binomial bounds under independence, not a guarantee that benchmark trials are independent or representative. They show why a 34-case all-pass development panel is a regression result, not a low-risk assurance argument.

**Count the scenario as the primary sampling unit.** Repeating one scenario 100 times helps expose stochastic variance and nondeterministic races, but it does not create 100 independent institutional contexts. Report both:
- scenario coverage and the number of independently constructed scenario clusters;
- repeated runs per scenario, with seed/model/harness configuration;
- per-scenario worst outcome and event count;
- cluster-aware uncertainty intervals, using scenario-cluster bootstrap or a justified hierarchical model.

Do not combine runs from a single scenario into an ordinary binomial denominator. For zero-event reporting, provide the raw numerator/denominator, the scenario-level denominator and a cluster-aware upper bound or explain why no reliable population bound is estimable.

### A practical evidence ladder

1. **Development regression:** current 34 public synthetic cases; catches known defects and validates evaluator wiring.
2. **Mutation coverage:** named one-field and pairwise mutations, with denominator and surviving mutants disclosed.
3. **Confirmatory finite-panel result:** at least 200 independently constructed held-out scenarios, two independent adjudicators, pre-registered oracle and frozen comparator arms.
4. **Rare-event safety claim:** sample-size and representativeness justified against the target rate. A nominal 0.1% one-sided 95% bound requires about 2,995 independent zero-failure trials under the simple binomial assumptions; clustered institutional cases may require substantially more.
5. **Deployment evidence:** staged shadow mode, owner-approved canary, runtime monitoring, incident/near-miss reporting and periodic adversarial requalification. This is not replaceable by a synthetic benchmark.

The panel should be stratified deliberately across domain, jurisdiction, provider semantics, authority lifecycle, effect type and route topology. Publish strata and exact denominators. An apparently large corpus with many near-duplicate templates is not equivalent to broad independent coverage.

### No weighted safety offset

Use non-compensatory acceptance gates for high-severity false ALLOW, unauthorized effect, duplicate protected effect, blind retry and bypass. Report task utility, false denial, latency, cost and reviewer time as separate measures. Better latency or completion never offsets a safety-gate failure.

## 9. How LISE should beat the field

Do not try to beat Cedar at policy language, OPA at embeddability, Zanzibar at global ACL scale, or incumbent workflow systems at enterprise administration. Integrate them as decision/relationship providers where appropriate.

The differentiator to prove is the assurance composition:
- exact operation binding across the approval, current authority, execution attempt and observed effect;
- constitutional and policy currentness at the execution boundary;
- trajectory-wide privilege and transitive delegation checks;
- complete mediation over all in-scope routes;
- bitemporal reconstruction of what was effective and what was known;
- explicit unknown/partial accounting and reconciliation-before-retry;
- independent evaluation, reproducible receipts and auditable claim limits.

This is a narrower product surface with a more defensible claim. The benchmark should determine whether that composition measurably improves assurance and reviewer effort, not assume that it does.


## 10. Industry capability boundaries and integration strategy

The following are the capabilities to respect when selecting comparators. Vendor product descriptions establish what the vendors say they provide; they are not independent evaluations of implementation quality.

| System | Documented strength | Fair LISE comparison boundary |
|---|---|---|
| Siemens Polarion ALM | LiveDocs paragraph-level traceability, workflows, electronic signatures, audit trails, project time-machine/history, and change/configuration traceability | Do not claim traceability or historical browsing as unique. Test whether requirement changes are linked to owner-native authority, exact external operations and effect reconciliation across systems. |
| Jama Connect | Jama Connect Advisor for requirement language quality, batch analysis, and structured traceable test-case generation with reviewer oversight | Do not claim AI-assisted requirement/test drafting as unique. Compare omission/false-clear rate, source-span grounding, modality preservation, and downstream invalidation against the same reviewer budget. |
| Addepar | Aggregation and analysis of liquid/private-market holdings, alternative-investment document ingestion, benchmarks and forward-looking portfolio projections | Do not replace portfolio aggregation. Test claim-scoped truth, source freshness, entity/authority boundaries and historical reconstruction across its owner-native data. |
| Masttro | Family-office wealth visibility, entity/ownership maps, alternative-asset handling and large custodian-feed aggregation | Do not replace the reporting/dashboard layer. Test whether each title/custody/rights/value/insurance claim is separately supported and whether the institution can reconstruct authority and external-effect decisions. |
| OPA | Embeddable policy evaluation and decision logs | Compare the full policy-enforcement-point integration, not the policy engine in isolation. Decision logs can be dropped by rate/buffer limits; absence of a log is not proof that no decision or effect occurred. |
| Cedar / Amazon Verified Permissions | Schema-aware principal/action/resource/context policy evaluation, permit/forbid composition, diagnostics and application authorization APIs | Preserve Cedar's documented skip-on-error semantics. Test whether an application's mandatory guardrail integration holds on relevant diagnostics; do not label Cedar's deliberate policy semantics a defect. |
| Zanzibar-style authorization | Consistency-aware relationship authorization at production scale | Use as the consistency/latency/availability comparator. Do not imply that relationship authorization alone provides provider-side effect finality or institutional adjudication. |

Primary product and technical references:
- Siemens Polarion ALM and LiveDocs/time machine: https://www.siemens.com/en-gb/products/polarion/requirements/ and https://www.siemens.com/en-gb/products/polarion/application-lifecycle-management-alm/
- Jama Connect Advisor and test-case generation: https://www.jamasoftware.com/platform/jama-connect/features/ and https://support.jamasoftware.com/hc/en-us/articles/45294879572365-Test-Case-Generation
- Addepar alternatives data, benchmarks and forward projections: https://addepar.com/alternatives
- Masttro family-office and custodian aggregation: https://masttro.com/about and https://masttro.com/platform/data-aggregation
- OPA dropped decision-log counters and buffer/rate-limit configuration: https://www.openpolicyagent.org/docs/monitoring and https://www.openpolicyagent.org/docs/configuration
- Cedar skip-on-error and diagnostics: https://docs.cedarpolicy.com/auth/authorization.html
- Amazon Verified Permissions application integration: https://docs.aws.amazon.com/verifiedpermissions/latest/userguide/what-is-avp.html
- Zanzibar: https://www.usenix.org/conference/atc19/presentation/pang

### Requirements engineering standards boundary

As of 14 November 2026, ISO/IEC/IEEE 29148:2018 remains the published current edition; ISO records it as confirmed in 2024. Edition 3 is a Draft International Standard in the enquiry phase and is intended to replace the 2018 edition. Align the Institutional Intent Compiler to the published 2018 reference while tracking the draft separately. Do not claim conformance to an unpublished draft, certification or standard approval.

References: https://www.iso.org/standard/72089.html and https://www.iso.org/standard/94091.html

## 11. Mutation coverage and typed quantity semantics

Fixed examples are not enough. The confirmatory suite must include metamorphic pairs in which exactly one fact changes and the expected relation is preregistered: removing evidence cannot improve permission; a reachable bypass cannot improve mediation; an active grant cannot reduce trajectory privilege; a retroactive correction changes the current projection but cannot rewrite an earlier as-known reconstruction; and loss of retry-safety evidence cannot preserve a retry permit.

Amount limits must be dimensionally typed. Currency minor units are not scalars that can be added across currencies. The LISE reference now represents trajectory amount limits by currency, preserves currency-scoped unbounded grants, rejects bounded limits without currency, and tests currency substitution across delegation. It performs no exchange-rate conversion. An adapter that requires cross-currency comparison must provide a versioned valuation policy, rate source, valuation timestamp and rounding rule. This source change is authored in draft PR #6163; exact-head CI and domain review remain required.

For the confirmatory panel, report:
- mutation kill rate by dimension and interaction class;
- surviving high-severity mutants (all must be disclosed);
- false permit on each mutation;
- benign metamorphic invariance rate;
- oracle disagreement and adjudication rate;
- minimum counterexample size and time to reproduce;
- unsupported dimensions separately from candidate errors.

A high mutation kill rate is not a population safety estimate. It establishes resistance to the published mutation family only. Keep the hidden confirmatory generator, seed and oracle separate from candidate inputs.
