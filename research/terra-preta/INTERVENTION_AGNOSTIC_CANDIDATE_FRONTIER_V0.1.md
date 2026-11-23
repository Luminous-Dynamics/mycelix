# Treatment-agnostic regenerative soil candidate frontier v0.1

**Status:** synthetic, evidence-gated candidate comparison. This is not an agronomic recommender or a treatment authorization system.

## Research premise

Biochar, compost, residue management, gypsum, water management and no intervention are candidate actions—not assumed benefits. A 2026 meta-analysis of 113 publications and 714 paired comparisons in saline–alkali soils reports average improvements in SOC and available nutrients, while also reporting reduced pH, EC, exchangeable sodium and soluble chloride in that studied domain. Those findings do not establish that the same material, rate or response transfers to a South African soil. Other long-term field trials show divergent persistence and chloride effects across soils, crops and management regimes.

The first decision must be whether there is a diagnosed limiting process and sufficient evidence to assess candidate interventions. Only then should candidates be compared against the same endpoint vector and baseline.

## Hard constraints before Pareto optimization

Each candidate has independent evidence fields for:
- applicability domain;
- critical evidence completeness / adverse findings;
- product-quality profile (where a material input is involved);
- sourcing / ecological-retention status (where relevant);
- a full set of predicted outcome intervals.

A candidate with failed quality, prohibited sourcing, adverse evidence or an out-of-domain context is blocked. Unknown critical evidence, missing outcomes or a prediction interval crossing a hard guardrail is unresolved. Neither can enter the Pareto frontier.

## Outcome vector, not weighted scalar

Each outcome has an explicit direction:
- maximize (e.g. yield or SOC stock under an approved context);
- minimize (e.g. nutrient export, EC or chloride where specified);
- target_range (e.g. a property with a context-appropriate interval, not a universal monotonic direction).

Hard minima/maxima are stored per outcome and are marked synthetic in v0.1. A production system needs external, locally approved sources and methods. No weighted sum or global 'soil improvement score' is emitted.

## Robust Pareto rule

For a maximize objective, candidate A can robustly dominate B only when A's lower bound is at least B's upper bound. For minimize, A's upper bound must be at most B's lower bound. For a target range, A's worst possible distance to the target must be no greater than B's best possible distance. A must be no worse on every objective and strictly better on at least one.

This deliberately conservative interval rule can leave several candidates non-dominated. That is preferable to manufacturing certainty from overlapping intervals. A human or separately authorized decision process may then choose further experiments or declare priorities; this auditor does neither.

## Synthetic fixture

The manufactured scenario includes no action, residue-cover management, gypsum, a biochar batch with chloride uncertainty crossing a hard limit, a compost candidate with unknown product quality, and a biochar batch that fails its quality profile. The expected output keeps management and gypsum on a Pareto frontier, shows no action dominated by management, and excludes unresolved / blocked materials.

All thresholds and values are fictional test data. They are not South African agronomic limits or application advice.

## Comparators and future SOTA evaluation

Future empirical evaluation should compare the same candidate set against:
- current local expert decision / no-change baseline;
- fixed-rule soil diagnosis and extension recommendations;
- single-objective yield optimization;
- multi-objective Pareto analysis without robust intervals;
- the interval-aware hard-gate/Pareto method here;
- Symthaea decision-aware experiment selection that requests extra measurements when the candidate frontier is unresolved.

Evaluate false eligibility, hard-constraint violations, Pareto rank stability, decision regret under known-truth simulation, abstention quality, value of information, cost per resolved decision, subgroup transfer and long-term outcome coverage. Any comparison must use the same observations, candidates, thresholds, treatment effects and resource budgets.

## Qualification targets (proposed, not achieved)

- Zero blocked or unresolved candidate on the eligible frontier.
- Zero unknown quality / ecological evidence interpreted as pass.
- Zero hard-limit interval overlap interpreted as pass.
- Zero compensation of chloride, nutrient-loss, contamination or ecological harm by higher yield / SOC.
- Zero synthetic threshold presented as a real soil standard.
- No-action remains a first-class baseline.
- Report all non-dominated candidates; do not force a single winner.

## Sources

- Soothar et al. (2026), *Biochar Application Improves Chemical Properties and Nutrient Availability in Saline–Alkali Soils: A Meta-Analysis*, Land Degradation & Development, DOI https://doi.org/10.1002/ldr.70742. 113 publications / 714 paired comparisons (2014–2025).
- Zhang et al. (2026), eight-year two-soil field study, DOI https://doi.org/10.1007/s42773-026-00623-x.
- Machado et al. (2026), ten-year dryland wheat–pea trial, DOI https://doi.org/10.3389/fsufs.2026.1638625.
- Li et al. (2026), eight-year frequency study, DOI https://doi.org/10.1016/j.eti.2026.104932.

## Boundaries

Only synthetic candidate data are processed. The output is a comparison artifact, not a treatment recommendation, amendment rate calculation, carbon credit claim, regulatory determination, or authorization to apply materials. Real thresholds and suitability decisions require locally qualified authorities and prospective replicated evidence.