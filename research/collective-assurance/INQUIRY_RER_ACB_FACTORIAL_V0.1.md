# Inquiry × RER × ACB Factorial Qualification v0.1

**Status:** structural reference validator and synthetic eight-arm fixture. No GroupAssistBench result, causal effect estimate, human-subject outcome, or SOTA claim is asserted.

## Purpose

The benchmark programme already has distinct owners:

- #6089: authorized, targeted inquiry and acquisition of information held across participants.
- #6688: bounded, rights-aware Relational Evidence Refinement (RER) over available evidence.
- #6619: evidence ledger, response-constraint binding (ACB-shaped) and independent response audit.
- #6700: signed generation-input exposure receipt contract.
- #6701: relation-edge paired counterfactual qualification.
- #6677: leave-one-source-origin-out replay.
- #6678: benchmark/evaluator qualification.
- #6685: non-compensatory quality/safety/agency/cost comparison.

The open research question in #6698 is whether these pipeline layers have independent benefits, complementary effects, redundancy, or harmful interactions. This module qualifies one matched eight-arm block; it does not reimplement any of the owners above.

## 2×2×2 treatment design

Each block contains exactly these eight arms:

| Code | Authorized inquiry | RER | ACB |
|---:|---|---|---|
| 000 | Off | Off | Off |
| 001 | Off | Off | On |
| 010 | Off | On | Off |
| 011 | Off | On | On |
| 100 | On | Off | Off |
| 101 | On | Off | On |
| 110 | On | On | Off |
| 111 | On | On | On |

The arm bits are an assignment contract. Every run carries an explicit on/off execution receipt for each treatment, including OFF controls. A treatment cannot be inferred merely from a missing receipt.

### Treatment semantics

**Inquiry ON** executes the preregistered #6089 information-acquisition policy. Request creation, delivery, response, disclosure, verification and source-origin identity remain separate events. A request is not proof of delivery; a response is not proof of truth; repeated holders of one source do not create independent corroboration. An ON treatment that discovers no additional source is a valid observation.

**RER ON** runs #6688's bounded, verified relation traversal over the evidence pool available after the inquiry phase. RER cannot grant disclosure permission, convert unknown permission to authorized, or promote unverified relations. RER OFF must record zero relationally added items.

**ACB ON** runs #6619's existing response-use constraints over every item in the actual generator-visible evidence inventory. The processed inventory must equal the visible inventory; unaccounted items remain measurable failures.

**ACB OFF** disables only the response-use constraint treatment. It does **not** disable rights, source provenance, source verification, recipient/purpose scope, generation-input exposure observation or independent response auditing. Those protections remain mandatory in every arm. ACB OFF has an explicit NotApplied disposition and no ACB constraint-processing count.

## Matching and treatment isolation

Within a block, every arm binds exactly the same:

- benchmark/version/certificate and split;
- source group, task and frozen history cutoff;
- initial candidate pool and initial retrieval seeds;
- model, tokenizer, system/user prompt templates, tools, decoding and toolchain;
- scorer, evaluator, preregistered inference plan;
- randomization policy and independent seed;
- context/output, compute-cost, wall-clock and memory budgets;
- agent count and generation-exposure witness profile.

The initial candidate pool and seeds are identical across all eight runs. Inquiry ON may legitimately change the post-inquiry candidate pool. Inquiry OFF must not issue requests, acquire new source origins or change the initial pool. This distinguishes a true inquiry treatment from accidental differences in task setup.

All runs require independently retained receipts for:
- rights and provenance;
- exact generation-input exposure;
- execution;
- inquiry / RER / ACB treatment enforcement (including OFF states);
- RER bundle and relation-index path;
- constraint processing;
- independent response audit;
- evaluator output.

Exposure instrumentation must be identical across arms. Do not compare signed exposure in one arm with unobserved exposure in another; that would confound better observability with answer quality. The signed receipt contract in #6700 is a structural prerequisite; an actual independent runtime witness and trust adapter remain separate deployment work.

## Outcomes — retain the full vector

### Primary benchmark outcomes
- GroupAssistBench L2 service-request quality; L1 recall remains separate.
- M1 single, M2 distributed and M3 conflicting-memory results remain separate.
- Verified-evidence coverage.
- Recalled-but-unused pair rate.
- Unsupported material claims.
- Conflict-preservation failures.

### Mechanism outcomes
- Inquiry requests and newly verified source origins.
- RER-added evidence.
- Generator-visible evidence count.
- Number of visible items processed by ACB.
- Unaccounted visible evidence.
- Source-origin and relation-edge contributions, investigated with #6677 and #6701.
- Independent response-audit disposition.

### Safety and efficiency outcomes
- Rights/provenance gate disposition.
- Unauthorized disclosure and unauthorized action counts.
- Over-refusal rate.
- p50/p95 latency in the experiment result card.
- Total deployment cost, with separate input/output tokens, query, indexing, construction, retries, witness, evaluator and judge costs.
- Human agency and burden when the relevant human experiment is separately approved and run.

Rights-based withholding is not a reasoning failure. A permission-unknown or unavailable holder must remain unknown/unavailable; it cannot be converted to a successful disclosure or to proof that a fact does not exist.

A correct answer produced by unauthorized disclosure or invalid provenance is not a passing system outcome. Gate failures remain observations in the arm; they do not disappear from analysis merely because the quality score is high.

## Statistical design

The validator checks one eight-arm block; it does not calculate statistical effects or declare a sample powered.

For a confirmatory tournament:

1. Randomize treatment assignment within matched task/source-group blocks using a preregistered schedule.
2. Treat source group as the independent cluster; multiple questions from the same group are not independent.
3. Estimate the inquiry, RER and ACB main effects plus all preregistered pairwise and three-way interactions.
4. Use paired group-clustered inference and family-wise correction over the confirmatory contrasts.
5. Report effect sizes, confidence intervals, missingness, indeterminate rates and arm-level distributions.
6. Keep M1/M2/M3 and L1/L2 separate; report difficult/worst-condition strata, not only means.
7. Seal source groups, task templates and adversarial relation patterns for held-out evaluation.
8. Reproduce the official scorer and matched same-model/full-context comparator before a superiority claim.

A one-block structural PASS is not a causal estimate. Sample-size/power adequacy, cluster ICC, missing-data handling, evaluator validity and independent replication require separate preregistration and qualification.

## Proposed research targets (not achieved results)

- Reproduce all official GroupAssistBench baselines and eight arms before any superiority claim.
- Reduce recalled-but-unused evidence to 10% or less using the benchmark's exact pair definition, while separately diagnosing retrieval, exposure and application losses.
- Achieve GroupAssistBench L2 non-inferiority within a preregistered 0.02 margin to a same-model, same-resource full-context baseline at 50% or less of total deployment cost.
- Show the combined inquiry+RER+ACB arm improves a preregistered outcome over the strongest matched single-layer arm, with cluster-aware uncertainty and no mandatory assurance-gate regression.
- Identify at least one regime where a layer is redundant or harmful; do not reward indiscriminate inquiry, graph expansion or constraint overhead.
- Preserve 100% of rights-withheld and unknown-permission states without coercion.
- Report separate quality, rights/privacy, evidence, agency, over-refusal, latency and cost outcomes; no scalar aggregate may compensate for a failed mandatory gate.

The 0.02 / 50% / 10% values are targets for a future tournament, not results of this reference validator.

## External benchmark positioning

- GroupAssistBench is the primary target for group-history recall and open-ended assistance; preserve M1/M2/M3 and L1/L2.
- COLLABMEM diagnoses sharing, access, routing, provenance, conflict resolution and retention with a matched memory-only harness.
- HiddenBench stresses latent information asymmetry and the failure to elicit facts that no agent initially sees.
- GroupMemBench tests multi-party updates, speaker grounding, term ambiguity, temporal reasoning and abstention.
- Common Ground remains the joint-action / clarification / repair track.
- AgentDojo/AgentDyn and related adversarial suites remain separate security/effect-sink tracks.

Do not merge benchmark scores into one artificial leaderboard. Each result card should state what capability was measured and what it cannot establish.

## Claim ceiling

A structural PASS establishes that the eight treatment arms exist, that the expected treatment bits are enforced by their declared receipts, that the starting identity and initial pool are matched, and that rights/provenance/exposure instrumentation is present in every arm.

It does not prove:
- that receipts are authentic without independently qualified trust adapters;
- that relations or sources are factually true;
- that a treatment improves performance;
- that factorial effects are statistically significant;
- that results generalize to unseen tasks, models or human collectives;
- or that any human study is ethically approved.

Human recruitment requires independent ethics review, accessible consent, compensation, preregistration, pilot-informed power and ICC analysis, privacy minimization, withdrawal and data-retention rules.
