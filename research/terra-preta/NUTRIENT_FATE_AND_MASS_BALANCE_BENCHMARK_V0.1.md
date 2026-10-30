# Nutrient Fate and Boundary Mass-Balance Benchmark v0.1

**Status:** Measurement / accounting contract and research benchmark only. It is not a fertilizer recommendation, biochar formulation, application rate, trial authorization, or claim of crop efficacy.

## 1. Paradigm shift: nutrient availability is not nutrient fate

A soil can show higher extractable nutrient concentrations and still lose more nutrient below the root zone or through runoff. Likewise, a lower leaching concentration can reflect lower input, lower mobilization, increased plant uptake, or a change in soil storage. None of these alone identifies improved nutrient-use efficiency.

The benchmark therefore separates:

1. **Boundary mass balance:** external inputs, external exports, and the change in total nutrient stock inside a declared soil–crop control volume.
2. **Internal transfers:** mineralization, immobilization, sorption/desorption, root uptake and transfer into standing biomass. These move nutrient between pools inside the control volume and must not be double-counted as new input/output.
3. **Availability indicators:** operationally defined extraction measurements (for example, Bray-1 P, Olsen P, exchangeable K, or mineral N). These are method-dependent indicators, not additional disjoint nutrient stocks. They must never be added to total stock pools.

The boundary equation for each element and period is:

`R = I - O - ΔS`

where `I` is the sum of measured external inputs, `O` is the sum of measured external exports, `ΔS = S_end - S_start` is the change in the sum of mutually exclusive stock pools, and `R` is the unclosed residual.

A near-zero residual is an accounting consistency check—not proof that every flux was measured, that a treatment caused an effect, or that the soil is agronomically improved.

## 2. Why this is a high-value research gap

Dissanayake et al. (2026) synthesized 1,798 data points from 48 articles on biochar applied with or without fertilizer, compared with fertilized controls. The authors reported average changes of −13.2% nitrate leaching, −1.5% ammonium leaching and −16.5% phosphorus leaching, alongside +21.9% available phosphorus for biochar-plus-fertilizer contrasts. Responses varied by application method, feedstock, pyrolysis temperature, soil pH, application rate, plant presence and time since application.

The contrast population matters: more than 85% of the observations involved biochar applied with fertilizer. This evidence cannot be generalized into a universal biochar-only effect, fertilizer-substitution claim or site-specific recommendation.

The central implication is that nutrient availability, plant uptake, nutrient export and nutrient stocks must be measured together. A single extractable-P or yield endpoint cannot establish where the nutrient went.

A 2025 review of biochar and phosphorus availability also identifies pH, adsorption/desorption, physical properties, organic matter and microbial/enzymatic activity as interacting explanatory mechanisms. These are candidate moderators, not universally beneficial pathways.

## 3. Control-volume and replicate contract

Each budget record must identify:

- element (N, P or K) on an **elemental-mass basis**; do not mix P with P₂O₅, K with K₂O, or N with fertilizer-product mass;
- period start/end dates, area, top/bottom depth and a plain-language boundary description;
- site, study, experimental unit, treatment arm, comparator arm, and source provenance;
- external fluxes with category, mass per hectare per period, method, evidence reference and standard uncertainty;
- non-overlapping start/end stock pools with compartment, method, evidence and uncertainty;
- availability indicators with extraction method, unit, sampling depth and separate provenance;
- missingness, deviations, censoring/detection limits and any unmeasured fluxes in the study report.

Recommended external inputs: fertilizer, organic amendment, irrigation water, atmospheric deposition, biological fixation, weathering and seed/material inputs where applicable.

Recommended external exports: harvest and other biomass removed from the declared boundary, leaching below the profile boundary, runoff, erosion, gaseous losses where applicable, and other declared exports.

The control volume must be stated so that no internal transfer is counted as a boundary flux. For example, if standing crop biomass and retained residues are inside the system boundary, root uptake and residue return are internal transfers; crop harvest removed from the boundary is an export. If the experiment uses a soil-only boundary, plant uptake is an export from that soil-only compartment and must not also be counted as a whole-system harvest export.

Storage pools must be mutually exclusive. An extractable-P result is an availability indicator and must not be added to a total-P stock pool. The input carries an explicit stock-partition review reference and declarations that pools are mutually exclusive and that boundary stocks are covered. It also carries a boundary-flux review reference and a list of known unmeasured pathways. Overlapping stock pools are rejected; incomplete stock or flux coverage produces an unqualified result even when the numeric residual is zero. These review fields are declarations, not independent proof of physical completeness.

The experimental unit is the independently assigned plot/pot/mesocosm. Soil subsamples, plant subsamples, leachate aliquots and repeated instrument readings are nested measurements, not additional independent treatment replicates. Preserve their hierarchy outside the aggregate budget.

## 4. Uncertainty and closure interpretation

Every flux and start/end stock measurement should provide a standard uncertainty in the same elemental mass-per-area basis. The auditor computes a conservative linear uncertainty bound by summing the declared standard uncertainties of all terms in the residual. This is an upper bound on standard uncertainty under unspecified covariance, not a confidence interval or a substitute for a covariance model.

Three outcomes are permitted:

- `within_conservative_uncertainty_bound`: the absolute residual is no greater than the conservative linear bound. This is a consistency screen only.
- `outside_conservative_uncertainty_bound`: the residual exceeds that bound. Investigate unmeasured fluxes, stock-pool overlap, boundary errors, units, timing, sampling, and analytical bias.
- `unqualified_missing_uncertainty`: at least one required term lacks an uncertainty estimate. The residual is reported, but no closure qualification is made.

Do not repair a budget by forcing the residual to zero, imputing an unmeasured flux without a labelled model, dropping a difficult term, or changing the system boundary after outcomes are inspected. Any model-estimated flux must be labelled as estimated and independently evaluated; it is not a measured flow.

## 5. New executable contract

The standalone Rust tool in `tools/nutrient-budget-audit/` will:

- validate elemental basis, dates, control-volume dimensions, IDs, and non-negative finite terms;
- require matching, non-overlapping pool identities at the start and end of the period;
- compute inputs, exports, stock change, residual and plot-scaled values;
- report pathway-specific input/output category totals and uncertainty sums so a pooled residual cannot hide a change in leaching, runoff, harvest export or another pathway;
- compute the conservative linear uncertainty bound only when all terms have standard uncertainties;
- keep availability indicators outside the mass-balance arithmetic;
- bind the report to the exact input bytes with SHA-256;
- refuse to overwrite the source or an existing report;
- fail closed on unknown fields and invalid records.

It does not establish that all real-world fluxes were measured, that methods are valid, or that a residual inside the uncertainty bound proves scientific completeness.

## 6. Proposed research benchmarks

All are proposed targets, not measured achievements:

| Dimension | Target |
|---|---|
| Unit/provenance integrity | 100% of confirmatory terms specify element basis, unit, method, period, boundary and source |
| Stock partition | Zero overlapping stock pools; start/end pool identities reconcile exactly |
| Uncertainty | 100% of confirmatory boundary terms have declared uncertainty; missing uncertainty blocks closure qualification |
| Accounting | Residual reported in kg element/ha/period and kg element for the declared plot area |
| Environmental loss | Leaching, runoff, erosion and gaseous pathways separately reported where applicable; no pooled loss hides a pathway increase |
| Agronomic delivery | Plant uptake/harvest export distinguished from extractable availability and total stock |
| Prediction | Any forecast compared against zero-change, persistence and appropriate mixed-effects baselines on identical study/site/batch-held-out partitions |
| Reproducibility | Exact raw input, method, configuration and report digests retained |
| Safety | Zero unapproved feedstock, material, trial or application decisions by the model |

A model-improvement target (for example, ≥10% lower error than the strongest valid baseline) must be assigned to a specific predeclared flux or trajectory endpoint, not to an ambiguous aggregate “nutrient efficiency” score.

## 7. Research hypotheses

- **N1 — availability/loss decoupling:** extractable nutrient changes do not reliably predict leaching/export changes across soil and material classes.
- **N2 — context moderation:** soil pH, mineralogy, organic carbon, texture, feedstock and co-applied nutrient programme interact to determine nutrient retention and plant availability.
- **N3 — compartment-specific transformation:** soil, root and soil-fauna compartments can respond differently to the same treatment; bulk-soil taxonomic summaries may conceal functionally important shifts.
- **N4 — plant-mediated effects:** plant presence changes nutrient retention/export through uptake and rhizosphere processes; unplanted column results may not transfer directly to cropped systems.
- **N5 — time dependence:** nutrient release, immobilization and export trajectories vary after application; first-week measurements may not predict season-scale behavior.
- **N6 — model value:** a constrained stock-flow model coupled to measured availability and plant/soil observations can better identify missing fluxes and decision-relevant uncertainty than endpoint-only nutrient scoring.

A 2026 TPI-inspired factorial pot study reported compartment-specific soil, root and earthworm-gut microbial responses and non-additive outcomes under a combined system. It motivates N3 but does not establish universal microbial mechanisms; the article's data are available on request, so its results must not be treated as an open training dataset.

## 8. Comparators and evaluation design

Every study must define the treatment and comparator arms and identify whether it estimates:

- incremental biochar effect under matched fertilizer/organic inputs;
- a combined amendment package versus fertilized control;
- fertilizer substitution under a nutrient-matched comparator;
- an organic-input or management effect;
- an unidentifiable contrast.

For literature synthesis, use study-aware hierarchical or cluster-robust estimation and reconcile shared controls/repeated observations. Random-row evaluation is diagnostic only. For prospective tests, freeze study/site/plot/batch group assignments before fitting.

Report N/P/K separately, and by relevant flux pathway. Report total stock and availability separately. If the study boundary does not include plant biomass, state where plant uptake crosses the boundary. Missing pathway measurements are not zero.

Baseline comparisons should include:
- zero-change / no-effect;
- input-minus-output stock accounting;
- suitable empirical or mixed-effects baseline;
- mechanistic nutrient transport/sorption models only when required parameters and boundary conditions are measured;
- local current practice for any agronomic decision.

Do not treat lower leaching as successful if it results from lower nutrient input and plant uptake is also impaired. Do not treat higher available nutrient concentration as success if runoff/leaching or contaminant mobility increases.

## 9. Relationship to Mycelix, Symtropy and Symthaea

- **Mycelix:** source, batch, sample, treatment/comparator, experimental-unit, measurement and review provenance.
- **Symtropy:** mass-conserving soil water/nutrient state models, transport and sorption simulations with explicit units and measured boundary conditions.
- **Symthaea:** hypothesis management and recommendation-only selection of the next approved measurement; abstain where the budget or causal contrast is incomplete.
- **Independent verifier / qualified reviewers:** verify the measurement basis, boundary definition, stock partition, uncertainty model, trial design, safety and local legal requirements.

A hash proves which bytes were audited; it does not prove sampling representativeness or laboratory correctness. A closed arithmetic budget is not evidence of efficacy, safety, permanence or fertilizer replacement.

## References

- Dissanayake et al. (2026). “Applying Biochar With Fertilisers Decreases Nitrogen and Phosphorus Leaching and Improves Phosphorus Availability: A Meta-Analysis.” *GCB Bioenergy*. https://doi.org/10.1111/gcbb.70165
- Zhang et al. (2025). “Biochar application to soils can regulate soil phosphorus availability: a review.” *Biochar* 7, 13. https://doi.org/10.1007/s42773-024-00415-1
- Leitão et al. (2026). “Compartment-specific responses of soil, root, and earthworm gut microbiomes to Terra Preta de Índio–inspired amendments in a tropical latosol.” *Applied Soil Ecology* 225, 107278. https://doi.org/10.1016/j.apsoil.2026.107278
