# Randomized soil-intervention causal-claim auditor v0.1

**Status:** synthetic-only qualification and estimator for a two-arm randomized complete-block design. It does not verify that a real randomization was performed, authorize a field trial, infer causal effects from observational data or assess ecological safety.

## Why this matters

A single before/after change is not an intervention effect. Soil is changing under weather, crop, fertilizer, residue, irrigation, tillage, depth redistribution and measurement error. A high peer percentile is not a treatment effect; a positive SOC concentration change is not necessarily a stock increase; a recovered respiration signal does not rule out a damaging transient pulse.

The 2026 review of decision-grade soil/groundwater digital twins stresses that monitoring must test causal process hypotheses, competing explanations and secondary impacts—not merely calibrate trends. For soil, this means distinguishing transformation from retention, redistribution, crop export, leaching, dilution/concentration effects and rebound.

## v0.1 design

The first estimator accepts only:
- a two-arm randomized complete-block trial;
- plot-level randomization (or another true independent treatment unit only in a future version);
- one treatment and one control unit in each block;
- exactly 0.5 treatment assignment probability per unit;
- one predeclared primary outcome, unit, method and depth support;
- one baseline measurement before randomization and one follow-up after randomization for every randomized unit;
- a registered analysis plan timestamped before randomization;
- an immutable randomization allocation-record digest and a preregistration digest;
- declared interference / spillover status;
- treatment assignment kept separate from treatment received / adherence.

Cores, subsamples, sensor readings and repeated timepoints do not increase the randomized-unit count. V0.1 uses the baseline-to-follow-up change for each plot and calculates the paired block difference:
`(treatment follow-up - treatment baseline) - (control follow-up - control baseline)`.

The overall intention-to-treat (ITT) estimate is the mean of the block contrasts. The displayed standard error is the standard error across block contrasts. The two-sided exact randomization p-value enumerates all within-block assignment sign flips under the sharp null. Exact enumeration is limited to 20 blocks. It is not silently replaced by an asymptotic test.

## Causal claim eligibility

An ITT estimate is emitted only when:
- all blocks are complete and internally randomized;
- every randomized unit has baseline and follow-up;
- analytical method, unit and depth support match across units;
- assignment and preregistration record digests are present;
- registration precedes randomization;
- the trial declares no relevant interference and no unit reports contamination/spillover.

Where interference is possible or unassessed, a numeric assignment contrast can still be shown as descriptive, but the causal ITT claim and randomization p-value abstain. Missing outcomes, incomplete blocks, invalid allocation structure, method mismatch or invalid timing reject the input rather than silently dropping units.

## Intention-to-treat, not per-protocol

Assignment determines the primary analysis. The report separately counts non-adherence (assigned treatment but not received; or control units receiving treatment). V0.1 does not compute a per-protocol effect or reinterpret adherence as randomization.

## Claim ontology

- **Descriptive state:** measured value and support at a time.
- **Within-unit change:** paired temporal difference, not an intervention effect.
- **Randomized ITT effect:** assignment-based contrast under this declared blocked randomization design.
- **Observational association:** not accepted by this causal estimator.
- **Modelled counterfactual:** a prediction from an explicit model, not an observed randomized effect.
- **Mechanistic claim:** requires additional process-specific measurements that distinguish competing pathways.

## Negative controls

The workflow tests:
- future-dated preregistration;
- missing/invalid assignment digest;
- missing endpoint;
- duplicate randomized-unit identity;
- two treatment or two control units in a block;
- non-0.5 assignment probability;
- outcome method/depth mismatch;
- possible interference;
- treatment received versus assigned does not change ITT allocation;
- exact randomization p-value on known-truth paired block contrasts;
- no output overwrite.

## Proposed scientific benchmarks (not achieved)

- Type-I error at nominal 5% under many seeded synthetic sharp-null trials.
- Power and minimum detectable effect across between-block heterogeneity, within-plot variance, missingness and measurement error.
- Randomization p-values match independent brute-force enumeration on all known-truth small designs.
- No pseudo-replication: inferential n equals randomized plots and blocks, never cores or sensor observations.
- Report ITT, adherence, missingness, contamination and block heterogeneity separately.
- External transfer across whole sites, soils, crops and seasons.
- No universal treatment-effect claim from one site, crop, soil or implementation.

## Scientific basis

Cycoń (2026), *Decision-grade digital twins for in situ bioremediation of contaminated soil and groundwater: a critical review*, Frontiers in Bioengineering and Biotechnology 14:1866957, DOI [10.3389/fbioe.2026.1866957](https://doi.org/10.3389/fbioe.2026.1866957). The review emphasizes evidence-to-causal-process mapping, tests of competing hypotheses, uncertainty and auditable decision logic. This is an assurance principle applied here to regenerative-soil claims, not empirical evidence for any particular amendment.

## Boundaries

All fixture values are manufactured. A green test proves only that the declared synthetic trial record is internally coherent and that the estimator reproduces its blocked randomization design. It does not verify real randomization, consent, access, agronomic appropriateness, ecological safety, intervention efficacy or MRV eligibility. Real claims require preregistration, qualified local review, appropriate permissions, field records and independent replication.


## Exact p-value resolution warning

The report exposes the minimum possible two-sided p-value for B paired blocks: 2 / 2^B. The known-truth four-block fixture has a lower bound of 0.125 and therefore cannot resolve nominal alpha = 0.05. Six blocks are the first count whose theoretical minimum is at most 0.05; eight blocks are the first whose theoretical minimum is at most 0.01. These are resolution facts, not power guarantees or a sample-size recommendation. A separate operating-characteristics benchmark is required for power, Type-I error and precision.
