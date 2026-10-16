# WISH Statistical Qualification Protocol v0.1

**Status:** research proposal; no confirmatory outcomes exist.  
**Issue:** [WISH-008 #5103](https://github.com/Luminous-Dynamics/mycelix/issues/5103)  
**Parent:** [WISH-001 #5069](https://github.com/Luminous-Dynamics/mycelix/issues/5069) and draft [contract/census PR #5070](https://github.com/Luminous-Dynamics/mycelix/pull/5070).  
**Compose with:** [CIV-IMPACT-001 #5046](https://github.com/Luminous-Dynamics/mycelix/issues/5046) and [CIV-IMPACT-002 #5053](https://github.com/Luminous-Dynamics/mycelix/issues/5053).

## 1. The main statistical correction

The draft's 120 synthetic episodes × 12 arms yield 1,440 planned arm/episode rows. They do **not** yield 1,440 independent experimental units. The same synthetic episode is intentionally paired across arms, so the unit of comparison is the episode (or a higher independent source/family cluster if multiple episodes share a generated template).

The current 120-episode corpus is for contract, workflow and harness development. It is not a powered real-world impact study, not a sealed holdout, and not evidence of a performance win. Repeating an episode across arms or seeds improves measurement/reliability analysis but does not multiply the number of independent scenarios.

## 2. Claims are earned in stages

Do not jump from a parseable schema to “safe and useful.” Publish a claim level with every result:

| Level | Evidence available | Allowed claim |
|---|---|---|
| L0 — structural | JSON/schema/ID/digest checks | The declared artifacts pass the named structural checks |
| L1 — deterministic replay | Same pinned inputs/builds yield the same schedule/report | The harness is reproducible under those pins |
| L2 — synthetic exploratory | Public development scenarios and controls | Diagnostic behavior on the named development corpus only |
| L3 — confirmatory synthetic | Frozen analysis plan, independently generated holdout, all hard gates and powered scenario count | The preregistered result on that holdout, within its defined scope |
| L4 — shadow/observe-only | Separately authorized, privacy-reviewed real workflow with no external side effects | Operational observations in the scoped environment |
| L5 — opted-in pilot | Consent, recipient-owned outcome criteria, steward/escalation, safety and comparison design | Pilot results for the enrolled population and period |
| L6 — causal/impact claim | Appropriate design, counterfactual, sampling and independent outcome adjudication | Only the causal claim supported by that design |

A stronger-sounding narrative cannot raise the level. Public fixtures never become a sealed holdout because more runs have been executed on them.

## 3. Pre-register one primary estimand and contrast

Before confirmatory runs, freeze:

- target population and eligible scenario distribution;
- independent unit and pairing/cluster structure;
- one primary comparison (for example, the composed arm against one named baseline);
- one primary outcome definition and measurement cutoff;
- minimum practically meaningful effect, not only a null-hypothesis threshold;
- analysis, interval, multiplicity, missingness and stopping rules;
- arm/build/configuration/runner/evaluator/schema/monitor/corpus digests;
- exact holdout generation and access policy.

**Candidate primary endpoint:** proportion of initiated episodes reaching the requester-defined *complete, verified outcome* by the declared cutoff. Only evidence that passes the relevant integrity, authenticity, authorization, recipient-criterion and sufficiency gates can establish the endpoint. A plan, resource offer, dispatch record or agent assertion alone does not count.

Partial outcomes, declines, abstentions, blocked actions, cancellation, timeout, unresolved evidence and withdrawals remain explicit dispositions. Do not silently turn them into success or remove them from the denominator. Report these dispositions separately even if the primary binary endpoint classifies them as not completely fulfilled.

All other outcomes—including partial fulfillment, cost, elapsed time, human effort, resource waste, access disparity and requester agency—remain separately reported secondary dimensions or receive their own properly adjusted confirmatory tests. Do not blend safety, privacy or consent breaches into a weighted score that high task success can offset.

## 4. Power and sample-size plan

Do not invent a sample size from the number of planned arm rows. The analysis plan must estimate or conservatively bound the variation at the **independent episode/source-family level**.

For a paired continuous outcome with within-episode difference standard deviation \(\sigma_D\), two-sided type-I error \(\alpha\), target power \(1-\beta\), and minimum worthwhile mean difference \(\delta\), a rough planning approximation is:

\[
n \approx \left(\frac{(z_{1-\alpha/2}+z_{1-\beta})\sigma_D}{\delta}\right)^2
\]

This is a planning approximation, not a substitute for a design-specific analysis. The primary endpoint above is binary: use paired-binary power analysis (such as exact or simulated McNemar-type analysis) driven by the two discordant-pair probabilities, or simulate the preregistered analysis directly. If scenarios share a template/source family, simulate or model that clustering; do not pretend templated variants are independent.

**Proposed planning defaults:** family-wise two-sided \(\alpha=0.05\), target power \(90\%\), and 95% intervals for the one primary contrast. Preselect a minimum practically meaningful difference; +10 absolute percentage points in verified fulfillment is a candidate threshold to review, not a scientific constant or measured improvement. Cost-per-verified-outcome reduction of 10% is a candidate secondary decision threshold, conditional on safety and the primary endpoint, not a result.

The actual sample size remains **undetermined** until there is a defensible estimate of baseline completion and paired discordance/variance from a development run, along with the target population and minimum worthwhile effect. The public 120-episode set can inform instrumentation or exploratory assumptions, but using it to choose an effect and then claim a confirmatory result on that same set would be leakage. Better: use it for harness development, then use an independent pilot/calibration set and a fresh sealed holdout.

Publish the power/sensitivity curve over plausible assumptions. If the available sample cannot detect the desired effect with the target power, either collect more independent scenarios or state that the result is exploratory/underpowered. Do not simply lower the target after seeing data.


### Illustrative sensitivity only — not a sample-size recommendation

For a paired binary endpoint, let \(D=Y_{\text{new}}-Y_{\text{baseline}}\in\{-1,0,1\}\), \(\delta=E[D]\), and \(q=P(D\ne0)\) be the discordant-pair rate. Then \(\operatorname{Var}(D)=q-\delta^2\). A rough normal-approximation calculation for a two-sided \(\alpha=0.05\), 90%-power target and \(\delta=0.10\) gives:

| Assumed discordant-pair rate \(q\) | Approximate independent episode count |
|---:|---:|
| 0.10 | 95 |
| 0.15 | 148 |
| 0.20 | 200 |
| 0.25 | 253 |
| 0.30 | 305 |
| 0.35 | 358 |
| 0.40 | 410 |
| 0.50 | 515 |
| 0.60 | 620 |

These values use \(n\approx(z_{1-\alpha/2}+z_{1-\beta})^2(q-\delta^2)/\delta^2\), with independent pairs, no multiplicity correction, no family/source clustering, no monitor-unqualified episodes, and a normal approximation to the paired mean difference. They are **not** an exact McNemar calculation, they are not based on observed WISH data, and they do not prove that 120 episodes are adequate. If scenarios share source/template families, or if the actual primary test is different, sample size may be higher. The confirmatory number must come from simulation of the exact preregistered test using justified nuisance-parameter ranges.

## 5. Paired estimation, uncertainty and multiplicity

- Compute episode-level paired outcomes/differences for the preregistered primary contrast.
- Use a stratified paired bootstrap or preregistered randomization/permutation method with the **episode** as the resampling unit; if source-family clustering is present, resample clusters. Freeze the resampling method, number of replicates, seed derivation and interval type.
- For paired binary outcomes, report the full paired table: success/success, success/failure, failure/success and failure/failure. The discordant pairs carry information about the difference.
- Report effect sizes and confidence intervals, not only p-values. Report all strata and heterogeneity, but label underpowered subgroup comparisons exploratory.
- Select one primary comparison before results. If multiple comparisons are necessary, predeclare familywise correction (for example Holm) or a hierarchical testing order. A “best baseline” chosen after observing results requires a separate holdout or appropriate selection adjustment.
- For costs and time, retain the full distribution and heavy tails. Preregister treatment of zero-success arms and censored/winsorized values; never report a ratio without its denominator and uncertainty.
- Keep safety hard gates conjunctive: one qualifying critical violation blocks the safety qualification regardless of average task success. Do not use statistical averaging to make a demonstrated unauthorized effect disappear.

## 6. Missingness, stopping and deviations

Every initiated episode/arm row has one final census state, including not-started, attempted, completed, blocked, cancelled, timeout, monitor-unqualified, unresolved and protocol-deviation states as applicable. Each must include a reason code and a receipt or an explicit missing-evidence record.

The preregistration must choose the primary handling of incomplete pairs before execution. If an outcome cannot be verified, it is not a verified success. Publish both the prespecified analysis and sensitivity/bounds for missing outcomes; do not silently use complete cases if monitor failure or difficult scenarios are systematically missing. Any hard-gate counter or missing monitor evidence that breaks qualification cannot be treated as an ordinary random missing outcome.

Freeze stopping rules and whether any interim look is permitted. Repeatedly checking significance and stopping when a favorable result appears invalidates the ordinary fixed-sample claim unless a valid sequential method was preregistered. Every analysis change creates a new benchmark version and is labelled exploratory unless independently re-confirmed.

## 7. Holdout independence and anti-leakage

- Keep public fixture corpora clearly marked `public_development_not_holdout`.
- Generate holdout scenario templates independently from development fixtures, ideally through a separately controlled generator or curator and a source/family partition. Use novel compositions and adversarial variations, not only changed IDs or surface wording.
- Avoid putting evaluator truth, expected disposition, scoring weights, hidden credentials, private success criteria or monitor oracles in the policy-visible observations.
- Pin a digest of the hidden corpus *commitment* without exposing the contents to candidate policies. Have an independent evaluator hold the decryption/access boundary, with audit receipts for every access.
- Test leakage by near-duplicate and template-family searches, and preserve a versioned leakage report. A test set repeatedly used to tune the policy becomes development data.
- Rotate holdouts with declared version boundaries and report performance across versions. Never combine incomparable benchmark versions into a single headline score without a justified bridging study.

## 8. Multidimensional value, access and agency

Report a profile, not a universal “human flourishing” scalar:

1. complete and partial verified outcomes;
2. unauthorized effects, harm and safety outcomes (hard gates);
3. consent state, revocation latency and agency violations;
4. privacy exposure, data minimization and access-scope violations;
5. requester-defined utility/satisfaction where voluntarily measured, with nonresponse and disagreement visible;
6. confirmed resources, total resource/cash cost, human labor and opportunity cost;
7. elapsed time, latency, retries, deferral and abstention quality;
8. availability and outcome differences across preregistered need/resource/difficulty strata, with uncertainty and privacy protections.

Use a Pareto/frontier view to make trade-offs visible. A system that fulfills more wishes by extracting more sensitive data, increasing coercion, or shifting hidden labor to people has not unambiguously improved. User-defined criteria and non-goals remain the source of outcome semantics.

## 9. Suggested benchmark progression

| Stage | Suggested scale/design | Exit condition |
|---|---|---|
| Harness/dev | Current 120 episode × 12 arm design | Stable generation/replay; negative controls detected; no result-claim inflation |
| Calibration | Separate development/calibration set with episode-family metadata | Baseline/discordance/variance estimates and power simulations are documented |
| Confirmatory synthetic | Sample size from frozen plan; one primary contrast; sealed independent holdout | Hard gates all pass, analysis matches preregistration, uncertainty meets decision criterion |
| Repeatability | Re-run selected cases with pinned and deliberate perturbations | Determinism and expected nondeterminism budgets are characterized |
| Shadow mode | Narrow, explicitly approved workflow, no action side effects | Evidence completeness, privacy, operator burden and escalation are acceptable |
| Opt-in pilot | Consent-based, low-risk, stewarded use with comparison strategy | Independent outcome review and adverse-event criteria satisfied |
| Impact evaluation | Population and counterfactual appropriate to the claim | Outcome, cost and distributional effects supported by design |

These are stages, not promises that each larger stage is automatically safe or justified.

## 10. Research alignment

This protocol responds to current agent-benchmark gaps without copying another benchmark wholesale:

- [AgencyBench (ACL 2026)](https://aclanthology.org/2026.acl-long.337/) emphasizes long-horizon deliverables, functional rubrics, self-correction and resource efficiency across scenarios. WISH adds requester-defined success and evidence/authority checks.
- [DeepPlanning (ACL 2026)](https://aclanthology.org/2026.acl-long.335/) highlights proactive information gathering and global time/budget constraints. WISH should report these as constraints and costs, not let local task success erase global failures.
- [NIST AI Agent Standards Initiative (17 February 2026)](https://www.nist.gov/news-events/news/2026/02/announcing-ai-agent-standards-initiative-interoperable-and-secure) motivates secure, interoperable, verifiable agent operations; it is context, not a WISH validation result.
- [OWASP GenAI Security guidance on excessive agency](https://genai.owasp.org/llmrisk/llm062025-excessive-agency/) emphasizes bounded functionality, permissions and autonomy, with enforcement at downstream boundaries. WISH's safety gates should measure actual boundary decisions, not just policy prose.

## 11. Acceptance gates (proposed, not measured)

- All initiated runs appear in the census; no silent deletion of failed/unresolved outcomes.
- The primary estimand, contrast, effect threshold, sample size, analysis method, multiplicity and stopping rules are frozen before confirmatory outcomes.
- No treatment of 1,440 planned run rows as 1,440 independent episodes.
- No confirmatory claim from the public development set or a post-hoc-selected metric.
- The power/sensitivity analysis shows what effect sizes can be detected under declared assumptions; otherwise label the result exploratory.
- Paired effect sizes, intervals, strata summaries and missingness sensitivity are generated reproducibly.
- Every hard safety/consent/privacy gate is evaluated separately and cannot be compensated by better task score or lower cost.
- Confirmatory holdout data are independently generated/controlled, with no candidate access to evaluator truth.
- Synthetic success is not presented as real-world causal impact.

## Status and limitations

The document is a preregistration design template, not a completed statistical analysis. There is no frozen primary estimand, no derived confirmatory sample size, no sealed holdout, no executed 1,440-run evaluation and no observed win. The existing WISH benchmark remains `preregistration_draft` and its execution lock remains deliberately unfrozen.