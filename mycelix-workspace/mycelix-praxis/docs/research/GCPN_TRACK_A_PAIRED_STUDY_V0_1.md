# GCPN — Track A paired independent-block study v0.1

**Parent:** [GCPN #5587](https://github.com/Luminous-Dynamics/mycelix/issues/5587)  
**Child:** [GCPN-TRACK-A-PAIRED-STUDY-001 #5951](https://github.com/Luminous-Dynamics/mycelix/issues/5951)  
**Single-bundle evaluator:** [#5947](https://github.com/Luminous-Dynamics/mycelix/issues/5947)  
**Comparison protocol:** [#5940](https://github.com/Luminous-Dynamics/mycelix/issues/5940)  
**Status:** synthetic preview implementation; no controlled study or SOTA result  
**Claim ceiling:** paired Track A statistics over declared independent blocks only

## 1. Why a second evaluator is needed

The single-bundle Track A evaluator (#5947) verifies event-envelope accounting and calculates deterministic per-pipeline metrics. A single bundle is not enough to support a claim that a candidate workflow is superior to a baseline.

Repeated event rows from one site-day, incident, equipment outage, sensor deployment or shared failure epoch are correlated. Treating every row as an independent sample creates artificially narrow confidence intervals and exaggerated significance.

This crate therefore:
1. re-runs the existing Track A evaluator on each sealed block bundle;
2. pairs the baseline and candidate on that exact block;
3. calculates per-block paired endpoint values;
4. resamples independent blocks (not event rows);
5. reports point estimates and deterministic bootstrap intervals;
6. refuses inferential conclusions for synthetic fixtures or fewer than the preregistered minimum independent units.

It composes the existing Track A evaluator and does not introduce a second event schema, evidence receipt, signature format or trust root.

## 2. Evaluation unit and pairing

The independent unit is declared as either:
- `SITE_DAY`: one predeclared site-day block with a defensible event inclusion/observation window; or
- `FAILURE_EPOCH`: one independently adjudicated failure epoch, with dependence between episodes assessed before treating them as independent.

Each block contains the exact raw Track A run bundle and its SHA-256 digest. The existing evaluator is invoked for every block. The selected baseline and candidate must both be present and scored in each block.

Within each block, the two pipelines share:
- exact event-set digest and eligible cases;
- oracle labels and adjudication revision;
- monitoring duration and observation window;
- operator policy and review budget;
- privacy and data-rights envelope.

Across blocks, event-set digests must be distinct. The operator policy digest must remain constant. Any missing/mismatched case coverage, pipeline, event set, ground-truth qualification or policy yields `INCOMPARABLE` (or `BLOCKED`/ `REJECTED` for the corresponding hard boundary or malformed evidence).

The same case IDs appearing on different days does not make the observations independent. Independent-unit definition, site/day separation, failure-epoch clustering, and any residual cross-block dependence require independent domain review. The software can validate uniqueness and declared digests; it cannot prove statistical independence.

## 3. Preregistered endpoints and hypothesis

The fixture demonstrates three paired endpoints:

1. **False-review requests per monitored hour.** Includes repeated requests on confirmed-benign cases; the single-bundle evaluator supplies the adjudicated false-review request count and monitoring duration.
2. **Human-review seconds per monitored hour.** Sum of declared human-review seconds across all cases divided by monitoring hours. A case with no review contributes zero. This avoids rewarding a pipeline simply because its mean *per reviewed case* is calculated on a smaller denominator.
3. **Event recall.** Per-block recall on confirmed-relevant oracle cases. Unresolved cases do not enter binary recall and remain separately represented by the single-bundle evaluator.

The example preregistration fields are:
- confidence level: 95%;
- paired bootstrap resamples: 2,000;
- deterministic bootstrap seed: fixed and included in the study bundle;
- relative reduction target: 20% for both review burden endpoints;
- recall non-inferiority margin: 2 percentage points;
- minimum independent units: 30.

These are **example research parameters**, not universal safety thresholds or established industry standards. In particular, 30 independent blocks is only a starter floor. Before a real study, a statistician and domain professionals must select the estimand, expected effect, variance assumptions, multiplicity policy, acceptable recall margin, missing-data handling, and a sample-size/power plan. Thirty blocks may be inadequate.

## 4. Paired bootstrap method

For each independent block (i), calculate:
- (F_{b,i}), (F_{c,i}): baseline and candidate false-review requests/hour;
- (H_{b,i}), (H_{c,i}): baseline and candidate human-review seconds/hour;
- (R_{b,i}), (R_{c,i}): baseline and candidate recall.

Point estimates are means across independent blocks (each block receives equal weight). The relative reduction for an endpoint is:

`1 - mean(candidate) / mean(baseline)`

If the baseline mean is zero, the relative reduction is null—not infinity or an arbitrary large value.

Each bootstrap replicate samples (n) paired blocks with replacement, preserving baseline/candidate pairing within a selected block. It recomputes the two relative reductions and the recall difference:

`mean(candidate recall - baseline recall)`

The implementation uses a deterministic xorshift64* generator and a declared seed so the exact synthetic result is reproducible. Confidence interval endpoints use the nearest-rank percentile method. The report exposes the number of valid resamples for each interval; zero-baseline bootstrap samples are omitted from relative-reduction distributions.

This bootstrap is not a substitute for design-based randomization inference or a justified hierarchical model where site, day, operator, equipment or incident clustering requires one. The resampling unit must match the actual study design.

## 5. Decision rule

A controlled study may report the preregistered hypothesis as supported only when:
- the lower confidence bound for false-review relative reduction is at least the preregistered target;
- the lower confidence bound for human-review-time relative reduction is at least the preregistered target;
- the lower confidence bound for candidate-minus-baseline recall is no worse than the negative non-inferiority margin;
- all privacy/authority hard gates remain zero;
- required independent-unit and review evidence is present.

If any condition fails, `hypothesis_supported = false`. A null interval/relative reduction cannot pass the rule.

The implementation does not combine the endpoints into one weighted score. A large workload reduction cannot compensate for recall degradation; nor can any performance result compensate for a privacy or authority violation.

## 6. Evidence tiers and dispositions

### Synthetic preview

The embedded 8-block fixture is synthetic and has no field/independent-unit validity. It always returns `SYNTHETIC_PREVIEW`, even if someone duplicates it to reach 30 rows. Point estimates and bootstrap intervals are debugging illustrations only. `hypothesis_supported` is null and `inference_permitted` is false.

### Controlled replay

A controlled study must:
- declare `CONTROLLED_REPLAY`;
- provide an independent-unit assessment receipt reference;
- provide at least the preregistered minimum (the starter floor is 30) unique independent units;
- provide unique per-block raw bundle digests and event-set digests;
- make every nested Track A bundle pass the single-bundle evaluator with its own ground-truth, privacy and authorization review references;
- maintain the same baseline/candidate pipeline IDs and operator policy across blocks.

Receipt references are format-checked here; their signatures, issuer identity, professional competence, independence and substantive scope must be verified through the shared evidence plane and designated human reviewers.

### Terminal statuses

- `SYNTHETIC_PREVIEW`: valid synthetic calculation; non-inferential.
- `INSUFFICIENT_INDEPENDENT_UNITS`: valid controlled blocks but below the registered minimum; no hypothesis decision.
- `CONTROLLED_PAIRED_RESULT`: valid controlled study with sufficient declared independent units; report interval-based hypothesis outcome, subject to the claim ceiling.
- `INCOMPARABLE`: missing/mismatched blocks, labels, pipelines, event sets, policy or required endpoints.
- `BLOCKED`: any child Track A bundle crosses privacy/authority boundaries, or required controlled-study independence evidence is missing.
- `REJECTED`: invalid hash, schema, duplicate unit, malformed nested bundle or impossible preregistration.

## 7. Paradigm shifts and research contribution

This evaluator encodes five principles that should distinguish Luminous' evidence work:

1. **Resample at the level where independence is justified.** More event rows do not mean more independent evidence.
2. **Compare paired workflows under the same conditions.** Never give a candidate a different denominator, review budget or observation window.
3. **Treat non-inferiority as a hard condition.** Workload and false-review improvements cannot hide lost relevant events.
4. **Treat zero denominators honestly.** No invented infinite cost reduction, false precision or dropped cases.
5. **Refuse synthetic-to-field promotion.** More synthetic rows do not create independent field observations.

The evaluator is a reproducible arithmetic component. The difficult scientific work remains study design, ground-truth independence, block independence, site/time holdout selection, sample-size/power analysis, and external review.

## 8. Professional and independent review required

Before a controlled replay is used to claim superiority:
- an independent statistician reviews the estimand, block independence, bootstrap method, intervals, sample-size/power analysis and non-inferiority margin;
- an independent ground-truth adjudicator reviews oracle construction and unresolved-label policy;
- the participating site/operator reviews the unit definition, event windows, baseline and human-review budget;
- privacy/data-protection professionals review data minimization, access, purpose, retention and controlled-replay permissions;
- a domain safety reviewer confirms that the event workflow and endpoints cannot create unauthorized escalation or action authority;
- the Mycelix shared evidence-plane owner verifies reviewer identity/authority and receipt semantics.

Each review must bind exact study and block digests, protocol revision, role/competence, scope, exclusions, findings, disposition and date. No outside professional approval is claimed by this synthetic implementation.

## 9. Non-claims

The synthetic fixture does not represent a real site-day, independent failure epoch, vendor, detector, human workflow or Luminous system performance. The reported point estimates/intervals are only examples for unit testing.

This tool does not establish:
- physical sensor detection or tracking performance;
- vendor or SOTA superiority;
- causal reduction in harm;
- CWA 18150 conformity;
- SACAA/ICASA or POPIA compliance;
- independence of ground truth or study blocks;
- professional approval or operational protection.

Track B physical sensor testing remains separately blocked.