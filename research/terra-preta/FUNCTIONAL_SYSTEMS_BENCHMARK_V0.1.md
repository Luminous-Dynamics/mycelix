# Functional Soil-System Benchmark v0.1

**Status:** Research benchmark proposal only. It is not a formulation, manufacturing protocol, application-rate recommendation, trial authorization, product certification, or claim that a modern system recreates archaeological Terra Preta.

## 1. Objective and claim boundary

The objective is to test whether a locally designed soil-management system produces durable, independently measured improvements in specified soil functions, without unacceptable trade-offs.

Use three distinct terms:

- **Historical reconstruction:** a site-specific archaeological and soil-science explanation of how a documented Amazonian Dark Earth (ADE) formed. This requires historical, archaeological, landscape, and community-governance evidence.
- **Functional analogue:** a modern, locally appropriate system tested for specified functions and a declared observation period. It is not historically identical by default.
- **Regenerative soil-management system:** a longer-horizon system that includes material inputs, soil mineralogy, organisms, plants, water, management, human labour, and material/energy flows.

Do not collapse Terra Preta, Terra Mulata / Amazonian Brown Earth, and all anthropogenic dark soils into one universal target. Do not equate black colour, total carbon, biochar presence, or high pH with soil health.

A system is “better” only relative to a declared purpose, soil, crop, climate, comparator, time horizon, and set of non-compensable constraints. There is no scientifically meaningful universal scalar for “better than Terra Preta.”

## 2. Benchmark unit and evidence classes

The experimental unit is the smallest unit independently assigned to a treatment (for example, a plot, pot, or mesocosm). Repeated samples, pressure steps, instrument readings, subsamples, and repeated seasons are nested observations, not independent treatment replicates.

Every observation must preserve, where applicable:

- study, site, block, experimental unit, plot, specimen/core, sample, depth interval, and replicate identifiers;
- material batch, feedstock class, process record, material sample, and custody;
- treatment arm, comparator arm, nutrient programme, crop, water regime, management, and timepoint;
- method, instrument, calibration, operator/laboratory, units, reporting basis, detection limit, and uncertainty;
- original-source locator, raw-file digest, transformation history, exclusions, and reason for each exclusion.

Keep these evidence classes distinct:

1. synthetic fixtures (software invariants only);
2. literature-extracted observations (correlated and method-heterogeneous);
3. field-survey context (site, soil, land-use and management descriptors);
4. new laboratory measurements;
5. randomized controlled pot/mesocosm observations;
6. prospective field trials and longitudinal monitoring;
7. archaeological / historical evidence.

Never merge classes just because columns have matching names. A survey of soil properties is not a treatment-effect dataset; literature-extracted summaries are not raw replicate data; synthetic arithmetic is not experimental evidence.

## 3. Multi-objective function vector

Publish the complete outcome vector and uncertainty before any optional composite score. A candidate can lie on a Pareto frontier; it must not be allowed to compensate for a safety failure by scoring well on yield or carbon.

| Function | Primary measurements | Essential interpretation controls |
|---|---|---|
| Water regulation | Full water-retention curve; declared field-capacity / wilting-point endpoints; available-water fraction and layer storage; infiltration, saturated hydraulic conductivity, drainage and runoff | Measurement basis, bulk density, method, pressure range, soil depth, wetting/desorption branch, plant-available endpoint assumptions |
| Soil structure | Bulk density, aggregate stability, porosity, penetration / rooting resistance, erosion or surface-crust indicators | Depth, sampling support, texture, moisture state, method and equivalent-soil-mass context |
| Nutrient cycling | Total and method-specific available N/P/K, exchangeable bases, plant uptake, nutrient-use efficiency, runoff and leaching losses | Extraction method, input nutrient balance, pH, co-applied fertilizer/manure, crop demand and time |
| Carbon stocks and persistence | Soil organic carbon and pyrogenic-carbon indicators, stocks by depth and equivalent soil mass, particulate/mineral-associated fractions where measured | Baseline stock, bulk density, carbon input, erosion/deposition, sampling depth, analytical method and repeated timepoints |
| Biological function | Predeclared functions such as respiration, mineralization, enzyme activity, microbial biomass, mycorrhizal/root measures, and soil-fauna indicators | Do not substitute diversity indices for function; specify assay and distinguish relative abundance from absolute activity |
| Plant performance | Marketable yield, biomass, quality, root distribution, crop failure probability, plant-water status and water productivity | Crop/variety, planting density, weather, nutrient programme, irrigation, season and local-practice comparator |
| Environmental performance | N₂O, CH₄ and CO₂ fluxes where relevant; nutrient export; contaminant mobility; lifecycle energy and emissions | Include feedstock counterfactual, conversion energy, transport, application, soil fluxes and displaced residue fate |
| Circularity and access | Traceable residual inputs, competing uses, mass/energy balance, labour, maintenance, cost and benefit distribution | A “waste” input is not impact-free; account for collection, treatment, transport and alternative uses |
| Safety and governance | Material identity, required chemical/pathogen/physical panels, ecological safeguards, permissions, consent and benefit-sharing | Qualified review and legal approvals are non-compensable gates, not model features |

Report endpoints separately. Increased available water does not establish yield gain; increased total phosphorus does not establish plant-available phosphorus; increased soil carbon concentration does not establish increased stock; a microbial shift does not establish useful function.

## 4. Causal contrast design

Before building a treatment matrix, state the estimand in plain language and encode its treatment and comparator components. A “biochar + nutrient” treatment versus untreated soil estimates a package contrast, not the incremental effect of biochar and not fertilizer substitution.

A mechanism programme should consider only contrasts needed to identify the predeclared questions, such as:

- local baseline practice versus no-amendment / reference condition;
- characterized carbon material versus matched control;
- organic-input component versus matched control;
- nutrient programme with and without the candidate component, with nutrients and management matched where required;
- combined modules versus their component contrasts, to estimate interactions;
- plant-covered versus unplanted system when root and rhizosphere feedback is the question.

These are design classes, not a recipe or authorization to use any particular material. Do not add factors merely to increase the number of combinations. Fractional-factorial, mixture, response-surface, or Bayesian-design methods are acceptable only when the target contrasts remain identifiable and the design has a defensible precision/power justification.

Randomize the actual experimental unit. Block on important site/soil gradients. Preserve allocation, seed, protocol version, deviations, missingness, and stopping decisions. Use independent material batches when batch transfer is part of the claim. Repeated measurements must be analysed as repeated measures.

## 5. Staged evidence gates

| Gate | Evidence required | Release interpretation |
|---|---|---|
| F0 — executable definitions | Endpoint definitions, units, schemas, synthetic invariants, negative controls and reproducible fixtures | Software contract only |
| F1 — measurement qualification | Method review, instrument calibration, specimen/replicate hierarchy, uncertainty budget and inter-lab transfer evidence | Qualified measurement process for named methods |
| F2 — material characterization | Traceable feedstock/process/batch and qualified safety/contaminant characterization | Batch-specific research eligibility only |
| F3 — mechanism identification | Preregistered, controlled contrasts with replicate-level measurements and mass-balance accounting | Mechanism evidence in the tested domain |
| F4 — plant-system validation | Controlled plant trials with predeclared agronomic endpoints, comparator and analysis | Plant response in tested soil/crop conditions |
| F5 — prospective field transfer | Frozen protocol / model, independent sites and seasons, longitudinal monitoring and adverse-outcome reporting | Local efficacy claim only within the validated domain |
| F6 — persistence and lifecycle | Repeated multi-year soil stocks, lifecycle accounting, maintenance/economic performance and independent review | Time-bounded persistence / net-benefit claim; never infer millennia from short incubations |

Passing a software gate does not pass a laboratory gate. Passing a laboratory gate does not authorize a field trial. A model output, ledger attestation, or digital-twin simulation cannot replace regulatory permission or qualified human review.

## 6. Predictive and mechanistic baselines

Use identical frozen partitions, permissible information, and outcome definitions for every comparator.

For soil-water retention, include as appropriate:

- direct measured-curve interpolation (the non-model reference);
- van Genuchten–Mualem and Brooks–Corey parameterizations, with declared fit domain and boundary assumptions;
- suitable pedotransfer functions, with their documented input domain;
- matched published machine-learning methods;
- a simple regularized or hierarchical baseline.

For field water and crop questions, consider HYDRUS-1D / SWAP and FAO AquaCrop only when the measured inputs, crop, boundary conditions and target problem genuinely match. These are domain comparators, not universal biochar-effect models.

For treatment-effect synthesis, include a zero-effect baseline and a study-aware hierarchical / meta-regression baseline. Where repeated controls or multiple outcomes are shared, preserve their correlation and use study/site-clustered inference.

A highly flexible model must demonstrate value beyond these baselines. If it does not, prefer the simpler model.

## 7. Split protocol and external validity

Freeze and digest-bind split manifests before fitting. Never use random rows as the confirmatory split for a literature dataset with repeated study, site, experiment, arm, batch, or curve dependencies.

Report separately:

1. within-source / in-domain holdout (diagnostic only);
2. leave-study-out;
3. leave-site-out where IDs support it;
4. leave-material-batch-out;
5. leave-soil-class or method-family-out;
6. prospective site/season evaluation.

Use a reviewed dependency graph across source publication, study, experiment, site, material batch, plot, specimen and repeated curve. A connected component must not cross confirmatory split labels. Report key coverage and unresolved dependencies; software cannot prove that a publisher identifier captures every real-world dependence.

Report per-domain results and failure strata. A pooled metric cannot hide a failure on a soil class, laboratory, pressure range, crop, or season that matters to the stated use case.

## 8. Proposed quantitative targets

All values below are **provisional research targets**, not observed performance and not universal agronomic thresholds. They must be finalized before confirmatory evaluation with laboratory and agronomic partners.

### Measurement and computation

- 100% of published physical values have an explicit unit, basis, method, provenance, and raw-source digest.
- 100% of reported field-capacity / wilting-point endpoints have measured support or documented interpolation within the measured pressure range; zero extrapolations.
- Zero silent smoothing, unit guessing, missing-row deletion, branch mixing, or unlogged repairs.
- Deterministic synthetic arithmetic error no greater than 1e-12 in well-conditioned normalized water-content fixtures.
- Replicate, instrument, calibration, specimen, and between-laboratory uncertainty are reported separately where the design supports their identification.

### Uncertainty and predictive performance

- Nominal 90% predictive intervals should achieve 85–95% empirical coverage on independent held-out groups, with mean width and proper interval score reported at the same time.
- Beat the strongest valid baseline by at least 10% on the predeclared primary error metric as a *target*, with the 95% group-cluster bootstrap interval for improvement excluding zero.
- No material degradation on any predeclared critical domain; if the overall target is achieved by sacrificing a relevant soil class or measurement method, the benchmark fails for that domain.
- For near-zero values, use absolute or scientifically normalized errors, not unstable percentage errors.
- Report calibration, sharpness, bias, error distribution, applicability domain, abstention rate, and the fraction of observations rejected for insufficient support.

A 10% relative error improvement is not a scientific minimum important difference. It is an engineering ambition for model comparison. Practical improvement in water supply, nutrient-use efficiency, crop stability or cost must be set with local partners and measured against the local comparator.

### Physical and environmental accounting

- Require water, carbon, nutrient and energy balance residuals to be commensurate with the documented combined measurement uncertainty. If measurement uncertainty is not available, mark the balance as unqualified rather than inventing a tolerance.
- Report water storage per declared depth and equivalent-soil-mass carbon stocks, not only concentrations.
- Report net lifecycle emissions per hectare and per unit of marketable output with sensitivity bounds.
- Require zero bypasses of safety, legal, environmental, and community-governance gates.

### Experiment efficiency

- Compare active experiment selection against random selection and a fixed expert schedule under the same budget.
- Score verified reduction in uncertainty for the prespecified estimands per unit of cost, sample, and elapsed experimental time.
- A provisional target is at least 20% lower total cost to reach the same predeclared evidence threshold, without worse coverage, safety, or domain performance.
- Report negative and null findings; do not reward the selector for merely finding positive effects.

## 9. Frontier hypotheses worth testing

1. **Mineral–carbon interface engineering:** test whether mineralogy and surface coatings explain persistence and nutrient retention beyond feedstock / pyrolysis descriptors.
2. **Water-pore architecture:** investigate whether pore-size distributions and soil aggregation can be jointly measured to distinguish useful plant-available storage from water retained too strongly or drained too rapidly.
3. **Time-dependent nutrient reservoirs:** distinguish fast, plant-available pools from slow release and immobilization; quantify plant uptake and losses rather than optimizing total concentration.
4. **Biological succession as a process:** test changes in specific nutrient-cycling, decomposition, root-association and aggregate-forming functions, not simply “more microbial diversity.”
5. **Living soil feedback:** determine whether cover, roots, crop rotations and residue return sustain or reverse the measured amendment effects.
6. **Adaptive management:** treat soil as a partially observed dynamic system; select the next measurement or approved management action by expected reduction in decision uncertainty, with human agronomic control.
7. **Circularity under constraints:** optimize use of traceable local residues only after competing uses, contaminants, treatment energy, transport and community priorities are included.

These are independent hypotheses. Do not assume all mechanisms will be beneficial or synergistic in every soil.

## 10. Industry comparison and what would count as advancement

Material-quality schemes and standards can improve identity, characterization, production traceability and measurement repeatability. They do not independently prove that a material improves a particular soil, crop, water regime or community outcome.

The proposed research programme goes further by linking:

- batch-specific material identity and qualified characterization;
- curve-level metrology and uncertainty;
- causal treatment/comparator records;
- study/site/batch-held-out model evaluation;
- multi-year physical, biological, agronomic and lifecycle outcomes;
- independently replayable evidence and fail-closed release gates.

The meaningful competitive claim is not “higher R².” It is a reproducible, independently evaluated ability to predict where a system will help, where it will not, what trade-offs it creates, how much uncertainty remains, and which next measurement is worth paying for.

Use current local regulatory classification, registration and permissions for the exact material, process, intended use and claims. Certification or an AI recommendation cannot substitute for them.

## 11. Data strategy and known limits

- **BiocharDS V1.0:** a global literature-extracted response dataset. Its publication describes the full workbook as containing a `Reference` sheet and an `Explanation and unit` sheet, and the CSV as a simplified target-information file. Reconcile the published 2,438 / 2,439 row descriptions and the reported experiment subtotals from exact artifacts. Do not infer the correct count, keys, or independent sample size from metadata.
- **Soils4Africa field observations:** a continental survey-context dataset with plot/location, land-use, crop, erosion, residue/manure/fertilizer and irrigation fields. The 2026 Zenodo record says it includes data from over 15,000 locations across 33 African countries and warns that quality control is ongoing. It is not a biochar treatment-effect dataset and must not be used as one.
- **Western Highveld soil spectroscopy/chemistry:** regional South African context only, subject to exact-file and license audit. It is not an ADE reference population or biochar-treated field trial.
- **2025 long-term biochar synthesis:** useful for time-horizon and hypothesis design, but source-study/site overlap with BiocharDS must be reconciled before it can be called independent validation.

Keep source versions, licenses, checksums, schema, and acquisition dates in the source register. Raw publisher files remain immutable and outside Git. Publish structural audit reports, discrepancy logs and review decisions before model fitting.

## 12. Symthaea / Symtropy / Mycelix division of responsibility

- **Mycelix:** evidence graph, raw-source custody, sample/batch identity, treatment/comparator semantics, consent/governance records, split manifests and reproducible reports. A signed record proves an attestation was made, not that the measurement is true.
- **Symtropy:** physical and biogeochemical simulation with explicit units, conservative balances, boundary conditions, measured parameter provenance and independent solver checks.
- **Symthaea:** competing hypotheses, uncertainty-aware experiment selection, causal question decomposition and calibrated abstention.
- **Independent verifier / qualified reviewers:** recompute balances, review methods, validate causal estimands, inspect safety/legal evidence and reject unsupported claims.

The system should optimize **verified functional improvement per unit cost, time and risk**, subject to non-compensable safety, ecological, legal and governance constraints.

## References

- Kern et al. (2017), “Terras pretas: Approaches to formation processes in a new paradigm.” https://doi.org/10.1002/gea.21647
- Neina & Glaser (2024), “Terra Preta production from Ghanaian and Zambian soils using domestic wastes.” https://doi.org/10.1038/s41598-024-75521-y
- Li et al. (2024), “A global dataset of biochar application effects on crop yield, soil properties, and greenhouse gas emissions.” https://doi.org/10.1038/s41597-023-02867-9
- Jiang et al. (2025), “Long-term effects of biochar on the hydraulic properties of soil: A meta-analysis based on 1–10 years field experiments.” https://doi.org/10.1016/j.geoderma.2025.117318
- “Towards understanding the long-term fate of biochar in Terra Preta” (2025). https://doi.org/10.1080/17583004.2025.2560126
- “Sustained benefits of long-term biochar application for food security and climate change mitigation” (2025). https://doi.org/10.1073/PNAS.2509237122
