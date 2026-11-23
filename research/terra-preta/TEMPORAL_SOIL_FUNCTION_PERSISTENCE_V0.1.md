# Temporal Soil-Function Persistence Benchmark v0.1

**Status:** Research and evaluation design only. This document is not a biochar recipe, application recommendation, field-trial authorization, carbon-credit methodology, or claim of historical Terra Preta reconstruction.

## 1. The core paradigm shift: measure three different kinds of persistence

The field often collapses “biochar persistence” and “soil benefit persistence” into one idea. They are not the same outcome.

This benchmark separates three coupled but non-equivalent clocks:

1. **Material persistence:** what remains of a characterized material and its carbon over time? Track mass, molecular markers (where validated), oxidation, fragmentation, mineral/organic coatings, migration, and recovered fractions using methods appropriate to the material and sampling design.
2. **Soil-function persistence:** do specified functions remain improved? Track water retention and conductivity, aggregation/structure, nutrient retention and losses, and other predeclared properties.
3. **Ecosystem persistence and resilience:** do plant, nutrient-cycling and landscape-level outcomes persist across seasons and recover after stress? Track crop performance, roots/cover, plant-water status, nutrient export, GHG fluxes, and management/input dependence.

A stable carbon pool can coexist with a declining water-retention benefit. A transient physical effect can coexist with longer-lived carbon storage. A soil function can be sustained by repeated management even if one amendment's direct effect fades. Reports must preserve these distinctions.

## 2. Why this is a timely research gap

Two recent field syntheses provide a direct motivation:

- Jiang, Huang and Meng (2025) synthesized field experiments spanning 1–10 years. They report that biochar-associated increases in total porosity, field capacity and available-water capacity peaked around year two and then declined, while a positive effect remained after three years in the included evidence. Feedstock and soil texture modified outcomes. The paper emphasizes the limited number of long-duration field observations and the need for more field experiments.
- Yang et al. (2025) synthesized 438 studies / 3,229 paired observations and examined 29 long-term field experiments spanning 4–12 years. Their analysis found that benefits under single applications tended to weaken over time, while repeated annual application sustained or enhanced several reported outcomes. The authors discuss multiyear reapplication intervals as a possible cost-effective strategy, tailored to local soil, climate and cropping conditions. These pooled values are not a prescription for any particular site; they do not establish that annual reapplication is optimal, affordable, safe, or environmentally preferable everywhere.

A separate analysis of Amazonian Dark Earth biochar particles reports oxidized surfaces and mineral–organic coatings and suggests that both organic coatings and inorganic/mineral interactions may contribute to persistence. It also cautions that the observed particle-level patterns vary and cannot yet be generalized.

The opportunity is to test how the three clocks relate, and to compare management policies by their full trajectory rather than by one endpoint.

## 3. Research questions and falsifiable hypotheses

### Questions

- Does the trajectory of soil function depend on material identity, initial soil state, soil texture/mineralogy, climate, crop/root system and management?
- When a measured functional benefit declines, is the cause loss/aging of the material, pore blockage, aggregate change, nutrient depletion, plant/management change, or a different process?
- Does a repeated or condition-triggered management strategy sustain function at lower total cost or lower environmental burden than a fixed schedule?
- Can the system forecast a decline early enough to select a safe, useful measurement or approved management response?
- Which functions improve, remain neutral, or deteriorate on different time horizons?

### Hypothesis set

- **H1 — clock decoupling:** material persistence, soil-function persistence and ecosystem persistence have measurably different trajectories.
- **H2 — context interaction:** the trajectory varies materially with soil mineralogy/texture, climate, crop and management; a global mean time trend is not transportable to every site.
- **H3 — non-monotone response:** some physical functions may improve after initial establishment, peak, and subsequently attenuate; monotonic decay is not a default assumption.
- **H4 — management-policy effect:** a prespecified adaptive policy can maintain a target function with less cumulative input/cost than a fixed high-frequency schedule in at least some predeclared contexts.
- **H5 — stress recovery:** systems that show similar average function under ordinary conditions may differ in resistance and recovery after drought, intense rainfall, heat or harvest disturbance.
- **H6 — model usefulness:** a state-space or hierarchical dynamic model can improve out-of-site/time trajectory forecasts over static endpoint regression and persistence-only baselines, while maintaining calibrated uncertainty.

Each hypothesis is permitted to fail. Null effects, harmful effects, and evidence of non-identifiability are valid results.

## 4. Define the treatment-effect trajectory before selecting a model

For a predeclared endpoint (j), define its treatment-effect trajectory:

`τ_j(t) = E[Y_j(t) | policy/treatment] - E[Y_j(t) | comparator]`

The estimator must respect randomized unit, blocks, sites, treatment assignment, repeated measures, and clustering. Define effect direction before scoring so that positive benefit has a consistent interpretation. For endpoints where higher is worse (for example, a harmful contaminant concentration or nutrient loss), reverse neither the raw data nor the scientific meaning; define a separate utility/constraint interpretation.

Report at least:

- **Pointwise effect:** `τ_j(t)` and uncertainty at each prespecified time.
- **Cumulative functional benefit:** area under the effect trajectory through a declared horizon, with a stated sign convention and missing-timepoint rules.
- **Minimum-important-effect duration:** time for which the effect remains above a locally justified `δ_j` threshold. If no defensible threshold exists, report the continuous trajectory and do not invent one.
- **Attenuation / recovery:** change after predefined stress events and time required to return to the pre-event or comparator-relative range.
- **Trade-off trajectory:** adverse and beneficial endpoints plotted on the same time axis, without netting non-compensable safety constraints into one score.

A “functional half-life” may be reported only when the endpoint has a positive, interpretable response trajectory and an explicit threshold definition; otherwise it creates false precision. Do not extrapolate a short observation record into decades or centuries.

## 5. Experimental design: compare strategies, not just doses

The target comparison is among management *policies* under the same defined soil/crop context, not a universally recommended material or rate. A qualified agronomist and applicable authorities must approve any physical work.

Possible policy classes for design consideration include:

- local standard practice / business-as-usual comparator;
- one-time intervention plus monitoring;
- fixed periodic maintenance/reapplication strategy;
- condition-triggered strategy using a preregistered measurement threshold;
- no-intervention/reference arm where ethical and agronomically appropriate.

These are experimental design categories, not instructions to apply a material. Treatment components, nutrient programmes, water management and comparator arms must be sufficiently described to identify the intended causal contrast.

Where staged decisions are central, consider a sequential randomized design (for example, a SMART-style design) or randomized policy assignment, provided the sample size and design identify the policy effects. Do not let a model change treatment assignment informally after seeing outcomes. Every trigger, threshold, allowed action, human-review requirement and stop rule must be preregistered.

### Temporal sampling hierarchy

The schedule must match the expected mechanism and local growing cycle. A defensible starting framework is:

- baseline: before any intervention;
- establishment: after the prespecified equilibration/establishment period;
- within-season: at scientifically justified crop/water stages;
- season boundary: harvest or end of crop cycle;
- annual: same season/phenological stage to reduce seasonal aliasing;
- event-triggered: after defined stress or disturbance, with event and sampling delay recorded.

A study may choose other cadences, but must justify them from mechanism, measurement burden and local conditions. Use consistent depth, sample support, season and methods where possible. Record weather, rainfall, irrigation, soil moisture state, crop, cover, management and deviations. A monthly sensor record does not make monthly observations independent treatment replicates.

### Replicate structure

Preserve assignment unit → block/site → plot/pot → core/specimen → laboratory subsample → instrument repeat → temporal observation. Model this hierarchy; never inflate sample size by counting subsamples or sensor readings as independent treatments. Preserve sample replacement, destructive sampling, missing measurements, plot loss and protocol deviations as explicit events.

## 6. Model class: a measured state-transition system

Use a hierarchy of models, not one opaque model:

1. **Baselines:** no-effect / local-practice baseline, persistence-only forecast, linear time trend, and endpoint-specific static model.
2. **Repeated-measure inference:** mixed-effects or cluster-robust trajectory analysis with site, block, experimental unit and time dependence represented.
3. **Nonlinear dynamics:** only where data support it, fit predeclared segmented, saturating, peak-and-decay, or other response shapes. Compare them against simpler models out-of-site and out-of-time.
4. **State-space / mechanistic model:** represent latent states and observations separately, and make every state observable through named measurements or clearly marked as unidentifiable.

A candidate state representation is:

`x_(t+1) = f(x_t, u_t, w_t; θ) + η_t`

`y_t = h(x_t; φ) + ε_t`

where `x_t` may include material state, soil physical structure, nutrient pools and plant/system state; `u_t` is the approved management input; `w_t` captures measured weather and exogenous forcing; `y_t` is the measured observation vector; and `η_t` / `ε_t` represent process and observation uncertainty. This is an architecture, not evidence that these latent states are currently identifiable. Do not infer a hidden mechanism from a good forecast score alone.

For every model, state the calibration data, temporal resolution, boundary conditions, uncertainty sources, missing-data treatment, parameter identifiability, applicability domain and forecast horizon. Training, feature selection, calibration and model selection must not access held-out sites, future observations or future treatment changes.

## 7. Persistence and adaptive-policy benchmarks

### A. Trajectory forecast benchmark

Evaluate frozen predictions on held-out sites, seasons and material batches. Compare identical targets and input availability against:

- persistence-only forecast;
- static endpoint model;
- mixed-effects time model;
- non-linear trajectory baseline where justified;
- mechanistic model only where parameter and boundary-condition support exists.

Report per-endpoint MAE/RMSE/bias, trajectory-integrated error, timing error for peak/threshold crossing, uncertainty coverage, interval width, proper interval score, and per-site/soil/crop/method strata. A pooled score cannot hide a failed critical domain.

**Provisional model target:** at least 10% improvement over the strongest valid baseline on a predeclared trajectory metric, with a group-cluster bootstrap interval for the improvement excluding zero, while meeting uncertainty-calibration and domain guardrails. This is a research ambition, not an achieved result or an agronomic meaningful-effect threshold.

### B. Persistence benchmark

- Report observed follow-up duration and the percentage of planned observations actually measured.
- Report functional effect trajectory separately from material-carbon persistence.
- Report cumulative benefit and time above a predeclared minimum-important effect where one is justified.
- No “persistent for X years” claim beyond actual observation or a separately validated inference method.
- Claims of durable carbon storage must use the applicable carbon accounting/methodology and its uncertainty, baseline, leakage and reversal requirements; a soil-function result is not a carbon removal certificate.

### C. Adaptive-policy benchmark

Compare a prespecified adaptive measurement/management policy with:
- random selection of the next measurement/action from the permitted set;
- fixed cadence / fixed schedule;
- qualified expert-designed schedule;
- no-change / current-practice policy where appropriate.

Under the same budget, report cumulative verified functional benefit, safety/adverse outcomes, cost, labour, sampling burden, missed-event rate, and regret relative to the best policy that could be selected with hindsight (clearly labelled as an oracle comparator). The adaptive policy must never be allowed to bypass material or regulatory gates.

**Provisional efficiency target:** reduce total monitoring and management cost by at least 20% at equivalent predeclared functional performance and safety, with uncertainty reported. If no safe adaptive policy improves on a fixed schedule, report that result and prefer the simpler policy.

### D. Stress and resilience benchmark

Use predefined, locally relevant stress classes and record actual exposure. Evaluate:
- baseline function before stress;
- minimum function during/after stress;
- recovery slope and time;
- cumulative shortfall relative to control;
- whether the intervention shifts risk from one function to another.

Do not manufacture extreme conditions in field settings without approvals and a defensible design. Controlled laboratory/mesocosm stress tests and natural field events are distinct evidence classes.

## 8. Carbon accounting and industry comparison

Do not use soil carbon concentration alone as proof of sequestration. Measure stocks at defined depth intervals with bulk density and coarse-fragment corrections as relevant; use equivalent-soil-mass methods where compaction or bulk density differs. Separate added pyrogenic carbon from native SOC and new plant-derived carbon where the measurement methods allow. Record uncertainty and sampling design.

Industry schemes already require substantial quality and MRV discipline. For example, Puro.earth's approved Biochar Methodology Edition 2025 v2 (approved 27 November 2025) contains requirements for biomass sourcing, production, eligible uses, reversal risk, net removal quantification, lifecycle emissions, leakage, monitoring, uncertainty assessment, data QC and record keeping. It defines a site-specific monitoring plan and periodic monitoring. EBC's update log identifies guideline version 10.5E dated 14 August 2025. These are important comparators, not substitutes for local soil-function validation.

The research opportunity is to add an explicit **soil-function trajectory and adaptive-policy layer** alongside material quality and carbon MRV. It is not to claim that existing certifications do not monitor, or that our additional benchmark replaces them.

## 9. Data requirements and analysis safeguards

A trajectory record needs, at minimum:

- immutable study/site/plot/block/experimental-unit identifiers and treatment assignment;
- timestamp, time relative to treatment, season/phenological stage and event markers;
- endpoint ID, value, unit, method, sample basis, depth, lab, instrument and uncertainty;
- soil and crop baseline state, rainfall/weather/irrigation and management history;
- material batch and application/input events where applicable;
- missingness reason, destructive sampling, sample replacement, deviations and quality flags;
- source record, raw-file hash, transformation code/config digest and reviewer status.

Preserve actual measurement timestamps. Never coerce a seasonal sample into evenly spaced time or hide sampling delay. Treat imputed variance and digitized figures distinctly from directly measured replicate data. Split and bootstrap at the true independent unit (usually site/experiment/study depending on the claim), not observation rows.

## 10. Proposed work packages

1. **Trajectory contract:** schema for timepoint, temporal observation, event, treatment policy and repeated-measure hierarchy.
2. **Trajectory auditor:** fail closed on inconsistent time origins, invalid units, repeated IDs with ambiguous identity, future leakage, impossible time ordering, and unlogged missingness.
3. **Persistence metric library:** pointwise treatment effects, cumulative benefit, threshold-duration, attenuation/recovery, and trajectory-integrated scoring with synthetic fixtures.
4. **Dynamic model bake-off:** persistence-only, static, hierarchical and suitable nonlinear/state-space methods on identical frozen time/site groups.
5. **Adaptive policy simulator:** compare fixed expert schedules, random sampling and preregistered value-of-information policies using only approved candidate measurements/actions and blinded held-out scenarios.
6. **Prospective protocol:** field/lab partnership, methods, randomization, timeline, sampling, power/precision and approvals before any physical work.
7. **Independent report:** raw and processed digests, method review, model card, failures, per-domain uncertainty, code/data/config versions and independent sign-off.

## 11. Non-goals and stop conditions

This benchmark does not:
- prescribe feedstock, pyrolysis conditions, amendment formulation, application rate or reapplication cadence;
- infer a universal strategy from the global meta-analyses;
- certify product safety, crop efficacy, carbon permanence, historical identity or commercial readiness;
- allow the adaptive model to choose or apply an unapproved material;
- equate a persistent carbon marker with persistent soil function;
- extrapolate a short incubation or limited field study into centuries or millennia;
- treat a signed record, hash, model output or simulation as proof of the physical truth.

Stop confirmatory claims when the treatment contrast is not identifiable, the experimental unit is unclear, key covariates are unsupported, safety or permissions are absent, required method metadata are missing, the observation horizon is shorter than the claim, or the model is outside its validated domain.

## References

- Jiang, Z., Huang, S. & Meng, Z. (2025). “Long-term effects of biochar on the hydraulic properties of soil: A meta-analysis based on 1–10 years field experiments.” *Geoderma* 458, 117318. https://doi.org/10.1016/j.geoderma.2025.117318
- Yang, J. et al. (2025). “Sustained benefits of long-term biochar application for food security and climate change mitigation.” *PNAS* 122(33), e2509237122. https://doi.org/10.1073/pnas.2509237122
- “Towards understanding the long-term fate of biochar in Terra Preta” (2025). https://doi.org/10.1080/17583004.2025.2560126
- Puro.earth (2025). “Biochar Methodology, Edition 2025 v2 (Approved 27 November 2025).” https://puro.earth/cdr-infrastructure/methodologies/document-library/
- European Biochar Certificate. “Changes from version 10.4 to version 10.5E (14 August 2025).” https://www.european-biochar.org/en/ct/7-Updates
- FAO Global Soil Partnership. “Equivalent soil mass” carbon stock calculation sub-protocol. https://www.fao.org/fileadmin/user_upload/GSP/MRV/GSP_MRV_Protocol_A4_EN_002.pdf
