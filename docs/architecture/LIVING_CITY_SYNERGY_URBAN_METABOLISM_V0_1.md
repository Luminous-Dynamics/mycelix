# Living-City: Urban Metabolism, System Synergy, and Thermodynamic Accounting (v0.1)

**Status:** proposed data contract and evaluation protocol; not a calibrated city model, engineering design, or municipal endorsement.  
**Initial context:** Johannesburg / Highveld civic corridors.  
**Purpose:** make cross-system benefits testable, make physical balances auditable, and keep simulated results separate from measurements and public decisions.

## Executive design decision

Treat Living-City as a network of linked services and flows—not a set of isolated green, mobility, energy, housing, water, waste, and public-space projects. The same intervention can provide multiple benefits, impose new burdens, and depend on another system to work.

The boundary must not stop at the municipal border when the relevant flows do not. Electricity, fuels, food, construction materials, spare parts, water, waste, and embodied emissions may originate in or be exported to other regions. Record those dependencies as explicit imports, exports, upstream processes, and downstream consequences; do not treat an imported resource as impact-free merely because its production occurred elsewhere. A 2025 review of 74 studies on interconnected urban metabolic networks argues that inter-city links are often overlooked, while research on material/energy flow analysis emphasizes the need for comparable, quantitative inventories. See [Metabolism of interconnected cities (2025)](https://doi.org/10.1016/j.resconrec.2025.108194) and [Material and energy flow analysis for circular-economy assessment (2024)](https://doi.org/10.1111/jiec.13504).

The v0.1 contract introduces:

- an explicit spatial, temporal, and process boundary plus a functional unit;
- separate flow records for final energy, useful energy, exergy, water, materials, greenhouse-gas impacts, thermal exposure, accessibility, and service uptime;
- conservation checks aligned with Symtropy's exact integer resource-balance kernel, including resolved evidence refs for real assessments;
- a four-scenario interaction method for checking non-additive effects;
- source/evidence links and pinned model-run provenance;
- explicit role boundaries between Mycelix, Symthaea, Symtropy, and Sol Atlas;
- no single overall city score, and no automatic authorization to build.

This is an interface contract first. It does not claim that the listed flows or the synthetic values in the validator fixture have been measured in Johannesburg. It is intentionally aligned with Symtropy’s isolated resource-balance kernel rather than creating a competing physical accounting engine.

## 1. What should “thermodynamically better” mean?

Use physical thermodynamics where the system boundary, units, and processes support it. Keep social, ecological, and governance outcomes as separate but connected measures.

### 1.1 Four linked ledgers

**Physical metabolism**
- Energy inputs by carrier and stage: electricity, fuel, solar, useful heat and cooling.
- Exergy inputs and outputs only where a defensible thermodynamic reference environment is defined.
- Water withdrawal, potable/non-potable use, reuse, retention, runoff export, and change in stored water.
- Material inflows, reclaimed/reused material, construction and maintenance inputs, and waste destinations.
- Operational and embodied greenhouse-gas emissions with explicit life-cycle stages.
- Thermal exposure, shade, accessible person-hours, and essential-service reliability where valid measurement protocols exist.

**Service outcomes**
- Hours of safe, step-free access; reliable water and power; useful transit access; habitable indoor conditions; and public-space use.
- Denominators must be explicit: per corridor-year, person-year, passenger trip, cubic metre delivered, or another justified functional unit.
- A more efficient system is not better if it supplies less service, excludes people, or transfers costs to another neighbourhood.

**Distribution and rights**
- Who receives each benefit, who pays, whose time or land is consumed, who bears exposure or risk, and who has decision power.
- Keep group- and neighbourhood-level distributions visible where data can be safely and lawfully shared; never reveal sensitive small-group records.
- Displacement, accessibility, cultural rights, consent, affordability, and privacy are non-compensatory constraints—not costs that can be offset by extra solar output or aesthetic appeal.

**Stewardship and resilience**
- Named operator/steward, recurring budget, replacement cycle, spare parts, repair skill, water/energy dependency, and a funded end-of-life plan.
- Performance under heat, drought, intense rain, power interruptions, maintenance delays, and demand spikes; include graceful degradation and stop/reversal conditions.
- A temporary activation is not a long-term service plan.

## 2. Energy, exergy, and conservation: keep the physics honest

### 2.1 Exact resource balances

The v0.1 balance fields align with the exact-integer convention in [Symtropy’s bounded garden resource-balance kernel, PR #1637](https://github.com/Luminous-Dynamics/symtropy/pull/1637):

`expected_storage_delta = inflow + generated_or_added - outflow - declared_loss`

`residual = expected_storage_delta - observed_storage_delta`

Energy, water, and conserved material mass use canonical base units of watt-hours (Wh), millilitres (mL), and grams (g). Quantities are decimal integer strings so clients cannot silently round large values through IEEE-754 floating-point numbers. The validator recomputes the residual from the fields and compares it to an explicit integer tolerance; a `balanced` label cannot override arithmetic. Each conservation check binds to flow-record IDs within the same scenario and verifies that converted base-unit totals match inflow, outflow, losses, and storage change. Unresolved or double-counted flow references fail closed. In a real `assessment` record, each balance check also requires Mycelix evidence-claim references that resolve; synthetic-only fixtures are forbidden from citing real-world evidence. Unit-conversion factors are evaluated as exact rational values (for example, 9 MJ equals 2,500 Wh), not rounded decimal approximations. The converted totals are compared against the integer ledger terms and explicit tolerance; no conversion factor is silently rounded before the balance check. A converted flow-role total that is fractional in its canonical integer unit is rejected even when its difference would fit inside the balance tolerance; callers must provide a precision/unit representation that closes exactly instead of hiding conversion loss as measurement tolerance.

This conservation schema deliberately excludes greenhouse-gas equivalents and exergy. CO2-equivalents are impact-accounting outputs rather than conserved mass, while exergy can be destroyed and is not a plain conserved-energy quantity. Those metrics remain separate flow records and need their own appropriate methods.

Internal transfers must cancel when both sides are inside the same boundary; do not count them as both new inputs and new outputs. A residual is an audit finding, not a score. A large residual means missing flows, poor measurements, wrong boundaries, or an incompatible model until resolved.

### 2.2 Energy is not exergy

Energy is conserved; exergy characterizes the potential to do useful work relative to a specified reference environment. The same quantity of thermal energy can have different exergy depending on its temperature relative to ambient. Therefore:

- never add energy in kWh to exergy in kWh-exergy as if they were the same quantity;
- record reference temperature/pressure and whether the environment is fixed or time-varying for a real exergy study;
- show useful service and comfort alongside exergy efficiency;
- do not maximize efficiency at the expense of minimum service, affordability, health, or resilience.

For buildings and corridors, the most valuable intervention may be to reduce demand through shade, envelope design, passive cooling, or better control—not to route more high-grade energy through a less efficient system.

### 2.3 Life-cycle impact matters

The boundary needs to include construction, operation, maintenance, replacement, and end of life, or explicitly document why any stage is omitted. ISO 14040 and ISO 14044 provide the established life-cycle-assessment framework for goal/scope, inventory, impact assessment, interpretation, limitations, and review. The schema is not a claim of ISO conformity; it borrows the discipline of explicit boundaries, inventories, and interpretation.

Do not count avoided emissions without an explicit counterfactual. Do not count recycled material as free: report quality, transport, processing energy, losses, and the next destination.

### 2.4 Do not turn information theory into a city fuel gauge

Symtropy's own Phi-coupled engine has a model-specific energy ledger and experimental J/Phi metric. Those quantities should not be silently mixed with measured city joules or treated as established physical laws. Phi is not an SI energy unit, and any proposed connection to physical energy needs a derivation, unit analysis, calibration, and independent tests.

Landauer's bound concerns heat dissipation associated with logically irreversible information erasure under specified thermodynamic conditions; it is not a generic measured minimum for every computation, organism, agent, or urban activity. The validator therefore records Symtropy model outputs with a human-readable engine version plus an exact 40-character source revision SHA, seed, time step, input-manifest hash, calibration/validation status, and evidence links. It never infers city-scale thermodynamic truth from a Phi value alone.

## 3. Measuring synergy rather than merely claiming co-benefits

A list that says “trees cool the street, bioswales reduce runoff, and solar canopies create energy” is a useful hypothesis—not proof that the bundle performs better than the sum of its parts.

For one outcome metric (Y) under comparable boundaries, units, periods, and functional units, test two interventions A and B with four counterfactuals:

`interaction = Y(A+B) - Y(A) - Y(B) + Y(0)`

where `Y(0)` is the baseline, `Y(A)` and `Y(B)` are single-intervention cases, and `Y(A+B)` is the combined case. The validator checks the arithmetic and unit consistency; it does not establish causal identification by itself. Real estimates still need defensible study design, evidence, uncertainty analysis, and attention to weather, demand, and other confounders. Outcome values, reported deltas, tolerances, and interval endpoints are parsed from JSON decimal literals without first converting them to binary floating point, then compared with exact rational arithmetic. This matters even for ordinary-looking decimal values: at zero tolerance, a difference-in-differences delta of `0.4` must match exactly, while `0.4000000000000001` must be rejected. Tolerance remains an explicit domain allowance; it is not an excuse for binary-rounding error. The parser also rejects `NaN`, `Infinity`, and `-Infinity`; RFC 8259 §6 explicitly excludes such values from the JSON number grammar ([RFC Editor](https://www.rfc-editor.org/info/rfc8259/)). This prevents non-standard constants from slipping into uncertainty or balance evaluation through permissive parser behavior. It also rejects duplicate object keys instead of silently choosing the first or last value. RFC 8259 §4 recommends unique names for interoperable objects; in an evidence-bearing ledger, rejecting ambiguous keys is the safer deterministic policy.

For metrics where lower is better, a negative interaction can indicate beneficial synergy; for metrics where higher is better, the sign reverses. If the uncertainty interval includes zero, report the result as inconclusive rather than announcing synergy. Keep the outcome's direction and uncertainty attached to the result.

### Propagate uncertainty instead of accepting a hand-written interval

For each of the four scenario estimates, record an interval on the same metric and unit. A conservative interval-arithmetic enclosure for the interaction is:

`delta_lower = combined_lower - A_upper - B_upper + baseline_lower`

`delta_upper = combined_upper - A_lower - B_lower + baseline_upper`

The declared interaction interval must be at least as wide as this propagated enclosure. If one of the four intervals is missing, a quantified interaction is blocked. This is conservative interval arithmetic, not a confidence interval or probability statement; it does not model correlation between scenario errors. For common weather, demand, or model errors, prefer a joint scenario ensemble or paired simulation runs with shared conditions, and document dependence assumptions. Never relabel a deterministic model range as a statistical confidence interval.

Also record interactions that are:
- **synergistic:** one intervention improves another's measured outcome;
- **antagonistic:** one undermines another;
- **conditional:** synergy exists only in stated conditions or seasons;
- **unknown:** the interaction remains a testable hypothesis.

Do not sum overlapping benefits across metrics. For example, the same cooling effect must not be counted once as a temperature benefit, again as an energy saving, and a third time as a monetary saving without showing those relationships and their counterfactuals.

## 4. Johannesburg pilot portfolio: design bundles with both benefits and trade-offs

These are candidate research bundles, not finished designs or community-approved proposals.

### Blue-green cooling and flood management

Combine shade trees, soil restoration, safe infiltration, bioswales or constructed wetlands where site conditions support them, permeable surfaces, accessible paths, and monitored stormwater overflow. Johannesburg's Paterson Park stormwater-channel naturalisation is a local precedent reporting ecological, recreation, infiltration, and runoff-retention benefits. It is a precedent to investigate, not a guarantee that a different site will perform similarly.

Measure surface/air thermal conditions, runoff volume and peak timing, infiltration/retention, plant survival, potable-water use, accessible person-hours, safety perception, and maintenance effort. Test the drought trade-off: tree shade and cooling are valuable, but species choice and establishment water matter as conditions become hotter/drier. Never recommend infiltration before checking contamination, soil, groundwater, utilities, and downstream risks.

### Solar shade plus transit and local livelihoods

A solar canopy at a transit/market edge could combine weather protection, lighting, phone charging, refrigeration, safe waiting, and electricity generation. The system assessment must include embodied structure/panels, inverter/battery replacement, cleaning, secure operation, electrical safety, accessible flows, market/trader needs, and who maintains or owns the assets. Local power generation is not a benefit if it displaces traders, blocks walking routes, adds unmaintainable equipment, or pushes risk onto people least able to bear it.

### Water and nutrient loops

Consider rainwater capture, fit-for-purpose non-potable reuse, leak repair, sanitation reliability, food growing, and source-separated organic waste only as a linked service plan. State the intended water quality and permitted use, health controls, seasonal availability, energy for pumping/treatment, nutrient contamination risk, and accountable operator. A circular loop that spreads pathogens or requires more energy/maintenance than the service merits is not better.

Food production is now an explicit scenario outcome through `edible_food_delivered` (g/kg/t), so four-way baseline / A-only / B-only / combined comparisons can test whether an intervention bundle actually changes edible food delivered rather than inferring output from planted area or gross biomass. Model or observe the entire pathway: seasonal harvest, edible fraction, post-harvest loss, storage/refrigeration energy, safe water, transport, labour, and actual delivery to the named beneficiary scope. The synthetic fixture is arithmetic-only and is not a Johannesburg crop/yield estimate. Each food-delivery flow must name a stable beneficiary-scope reference, a reporting-period reference, and the fixed basis `edible_portion_received_by_beneficiary`. Because this metric measures a service outcome received by people, its lifecycle stage is constrained to `operational`; construction, maintenance, and end-of-life burdens belong in separate lifecycle flows. All four counterfactuals must use the same scope, period and basis before the interaction is calculated; outputs from different communities or periods are not comparable just because their units are kilograms. These references must be aggregate or pseudonymous IDs, never a person's name or direct identifier.

Keep nutrition composition separate from conserved material mass. Resolve a versioned composition row for the exact food/preparation state and nutrient definition, then consume a source-scoped exact contribution receipt from [Symtropy garden resource/nutrient kernel PR #1637](https://github.com/Luminous-Dynamics/symtropy/pull/1637) rather than inventing a second nutrition calculator inside this schema. Bind every contribution to delivery scope and time period. For a South African implementation, investigate [SAFOODS](https://safoods.mrc.ac.za/composition.html) as a versioned composition source: its published food values are mean values per 100 g edible food, and the source explicitly notes biological and handling variation across cultivar, soil, climate, agricultural practice, processing, storage, and transport. Match the precise food and preparation state, not merely a common name; retain the composition source/version, original denominator, units, and whether a value is measured, calculated, imputed, or mean/reference data. Follow [FAO/INFOODS guidance on food matching, data quality, and units/denominators](https://www.fao.org/infoods/infoods/standards-guidelines/en/). The [SAFOODS FAQ](https://safoods.mrc.ac.za/faq.html) states that written permission from the South African Medical Research Council is required before any part of the database is reproduced. Until explicit rights/terms are reviewed and permission is recorded in the source catalogue where required, reference SAFOODS by URL and provenance only—do not vendor-copy or redistribute its underlying food rows. These composition data support nutrient-accounting inputs only—they do not estimate site-specific crop yield or household access. A supply total is not proof of dietary adequacy: meaningful portfolio assessment still needs sourced targets, seasonal continuity, diversity, affordability, allocation, safety, and appropriate population context. Nutrition quantities are service/accounting outputs, not another conserved mass ledger or a single wellbeing score.

### Repair, reuse, and local production

Map construction inputs, recoverable materials, repair capacity, transport distances, and end-of-life routes. Prefer durable, locally repairable designs where lifecycle evidence supports them; do not assume local production always has lower impact without counting material/energy intensity and logistics.

## 5. Create system synergy through explicit interfaces

Each system has a bounded role. Integration occurs through versioned artifacts and evidence, not through vague claims that every system is connected to everything.

| System | Role in the Living-City loop | Must not do |
|---|---|---|
| **Symthaea** | Generate diverse alternatives; state mechanisms, assumptions, predicted trade-offs, missing evidence, and falsifiable tests; explore uncertainty. | Declare a hypothesis true, culturally legitimate, safe, or approved. |
| **Symtropy** | Run version-pinned physical/agent/system scenarios; preserve energy/mass accounting inside the modeled boundary; expose solver settings and failure modes. | Convert an unvalidated model into a measurement, or treat Phi/J-Phi as physical city measurements without a validated bridge. |
| **Mycelix** | Version concepts, evidence claims, rights/consent state, model-run receipts, dissent, scenario comparisons, and governance decisions. | Treat schema validity or a signature as proof that a claim is true or a community has consented. |
| **Sol Atlas** | Read-only spatial/temporal views of the same boundary and scenario IDs; show inputs, outputs, uncertainty, data gaps, equity distributions, and interaction effects. | Hide missingness, interpolate beyond source support without labelling it, or collapse results to one “symtropy score.” |

### Nutrition assessment receipt boundary

The source-scoped nutrient calculator already exists in [Symtropy garden kernel PR #1637](https://github.com/Luminous-Dynamics/symtropy/pull/1637) as `NutrientPortfolioAssessmentV1`. Its raw Rust output carries scenario, target, nutrient definition, scope/period, source references, exact rational numerator/denominator fields, per-line receipts, blockers, and a four-state result, but the raw output type does not itself carry a receipt ID, envelope schema version, or producer revision.

Living-City therefore defines an optional wrapper envelope, `living-city-nutrient-assessment-envelope/0.1`, referenced from an `edible_food_delivered` flow by `nutrient_assessment_refs`. A real Symtropy-origin envelope pins the exact producer revision and carries a payload locator plus the SHA-256 declared for the exact serialized native receipt. The normalized fields retain the target/nutrient identity, scope and period, exact decimal-string arithmetic, source refs, status, and limitations. Synthetic test receipts are labelled separately and cannot cite real evidence. No receipt is mandatory merely to record edible food delivery: absence means **no nutrition assessment is attached**, not zero nutrient supply.

The contract validator checks that receipt IDs resolve and link to exactly one food-delivery flow, that scenario/scope/period match, that assessed edible mass reconciles exactly to the flow in grams, that the denominator is `100 × value_scale`, and that shortfall/status recompute from exact numerators. `NotAssessable` and `Blocked` receipts must withhold total/supply/target arithmetic rather than fill it with zero. A `meets_declared_minimum` result means only that the receipt's declared calculation meets its declared target; it does **not** establish overall dietary adequacy, food safety, equitable access, or an individual's nutritional suitability.

Important verification ceiling: the CI validator checks the declared hash's shape but does not retrieve the payload, recompute its SHA-256, or independently execute Symtropy's Rust calculator. Until the payload bytes can be fetched and hashed against the commitment, `payload_sha256` remains an unverified assertion. Mycelix's exact numerator checks are consistency checks, not cryptographic provenance or proof of source truth.

## Payload integrity gate (implemented, offline and fail-closed)

The companion verifier at \`scripts/living-city/verify_nutrient_payloads.py\` closes a specific gap in the schema-only validator. For a real \`symtropy_output\` receipt, it accepts only an explicit \`repo-file:<relative-posix-path>\` locator for offline checking. It rejects absolute paths, parent traversal, and symlinks that resolve outside the repository. It hashes and parses the same exact raw payload bytes (not re-serialized JSON; capped at 16 MiB per payload), rejects duplicate JSON keys and non-standard numeric constants, then checks that 17 native Symtropy fields—including scenario/target/nutrient identity, unit, beneficiary scope, period, exact numerator/denominator values, source refs and status—match the normalized envelope exactly.

Run it against a review JSON export from the repository root:

\`\`\`sh
python scripts/living-city/verify_nutrient_payloads.py /path/to/review.json --repo-root .
\`\`\`

The command exits non-zero when a payload is missing, a hash mismatches, the payload and envelope disagree, the locator escapes the repository, or the reference is external/opaque. It never accesses the network; external payloads remain \`external_unverified\` until an explicitly trusted resolver is supplied. Unit tests cover matching bytes, changed bytes, re-hashed-but-conflicting envelope values, missing files, traversal, absolute paths, symlink escape, remote references, malformed hashes, oversized payloads, duplicate keys, synthetic-only no-op input, and duplicate receipt IDs.

This verifies byte integrity and the binding between the byte-identified payload and the envelope, **not** the authenticity of the producer, truth of the source records, correctness of the nutritional standard, or adequacy of a diet. The declared producer revision remains a provenance claim until a trusted attestation or separate source/build verification binds it to a known Symtropy build. The design follows the separation between artifact identity, provenance statements, and verifier policy in [in-toto Attestation](https://github.com/in-toto/attestation/blob/main/spec/v1/README.md) and [SLSA provenance verification](https://slsa.dev/spec/v1.2/verifying-artifacts), and the raw-byte digest principle of [IETF RFC 9530](https://www.rfc-editor.org/rfc/rfc9530.html). Food matching, component identifiers, source quality, units and denominators should follow [FAO/INFOODS guidance](https://www.fao.org/infoods/infoods/standards-guidelines/en/).

### Minimum shared hand-off

1. **Concept/hypothesis ID** — what question is being evaluated.
2. **Boundary and functional unit** — what, where, when, and which service is counted.
3. **Scenario manifest** — baseline, single interventions, combined bundle; stable IDs and assumptions.
4. **Evidence/model receipt** — source claim IDs or model engine/version plus exact source revision SHA, input hash, seed, time step, calibration status, and uncertainty.
5. **Result ledger** — same units and metric definitions, conservation residuals, interaction estimates, distributional outcomes, limitations.
6. **Decision record** — who reviewed it, dissent, consent/rights status, remaining hard gates, pilot owner/funding, stop criteria.

Sol Atlas should consume results read-only; Mycelix keeps provenance/governance; Symtropy provides reproducible model outputs; Symthaea explores and explains. Each interface must still work when one system is missing: mark the field unavailable rather than fabricate or silently substitute a default.

## 6. Do not optimize one opaque “symtropy score”

For now, treat **symtropy** as an experimental design objective: improve the ability of linked systems to deliver durable, equitable service while respecting physical balances and reducing avoidable resource loss. This is a proposed working definition, not a recognized state function or validated scalar metric.

Publish a **vector** instead:
- physical performance (energy, exergy where valid, water, materials, emissions, thermal exposure);
- service performance (availability, travel/access time, comfort, reliability);
- lifecycle and stewardship (capital/operating costs, replacement, repair time, end of life);
- distribution (who benefits, who pays, exposure, affordability, displacement risk);
- resilience (performance under defined shock scenarios, recovery time, critical dependencies);
- uncertainty and evidence strength.

Use non-compensatory gates first, then show Pareto trade-offs among feasible scenarios. A candidate is Pareto-dominated only if another candidate is at least as good on every declared objective and strictly better on at least one, within the same boundary and uncertainty treatment. Keep preferences and weights visible if a human decision-maker later chooses a trade-off; don't hide them in an AI-produced scalar.

## 7. Acceptance and research plan

Before using a real Living-City record for a decision:

1. Locally review the site boundary, cultural and land rights, affected groups, functional unit, service goals, and exclusions.
2. Identify source data, reuse rights, spatial/temporal support, units, uncertainty, and missing measurements. A high-level climate projection is not street-level thermal evidence.
3. Build baseline and single-intervention scenarios before comparing a bundle; model each major physical pathway explicitly.
4. Validate conservation residuals and model behavior against measurements or an independently reviewed reference. Report failed validation as a failure, not a tuning target to be silently weakened.
5. Test sensitivity and uncertainty; retain negative and null outcomes; separate observation, source estimate, model result, assumption, and target.
6. Pay local participants for meaningful review; include disability/accessibility, maintenance/operators, residents, informal traders, and groups exposed to risk.
7. Assign operating ownership and recurring funding, and define reversal/stop criteria before any pilot.
8. Compare predicted benefits with measured post-pilot outcomes and lifecycle costs; update the model only through a versioned change process.

## Research references

- Galán-Cano et al. (2025), *Urban metabolism, sustainability and energy transition in cities: A comprehensive review*, DOI: https://doi.org/10.1016/j.rineng.2025.104278 — circular flows and the challenge of comparable urban data.
- Liu & Wu (2025 issue; published online 2024), *Thermodynamic-based ecological scaling theory in urban metabolic framework: a review*, DOI: https://doi.org/10.1007/s11783-025-1924-8 — theoretical urban-metabolism framing; not a universal calibration recipe.
- ISO 14040, life-cycle assessment principles and framework: https://www.iso.org/standard/37456.html
- ISO 14044, LCA requirements and guidelines: https://www.iso.org/standard/38498.html
- *Landauer principle and thermodynamics of computation* (2025 review), PMID 40345217: https://pubmed.ncbi.nlm.nih.gov/40345217/ — the bound concerns logically irreversible operations such as erasure, not an all-purpose energy floor.
- Breed & du Plessis, *How can Green Infrastructure Uptake Improve Urban Environmental Conditions Triggered by Climate Change?*, African Journal of Landscape Architecture: https://www.ajlajournal.org/articles/how-can-green-infrastructure-uptake-improve-urban-environmental-conditions-triggered-by-climate-change — includes the Paterson Park, Johannesburg stormwater-channel naturalisation example and stresses cross-sector planning and site-appropriate planting.
- Liu et al. (2015), *Urban Metabolism: A Review of Current Knowledge and Directions for Future Study*, DOI: https://doi.org/10.1021/acs.est.5b03060 — argues for standardized, multi-level and spatially explicit accounting.
- *Metabolism of interconnected cities: A review of the literature and analytical framework* (2025), DOI: https://doi.org/10.1016/j.resconrec.2025.108194 — synthesizes 74 studies and underscores the relevance of inter-city linkages.
- Papageorgiou et al. (2024), *Applying material and energy flow analysis to assess urban metabolism in the context of the circular economy*, DOI: https://doi.org/10.1111/jiec.13504 — quantitative material/energy inventories for assessing urban circular-economy strategies.

- *Nature-Based Solutions for Environmental Management: Effectiveness, Co-Benefits, and Monitoring* (2026 narrative review), DOI: https://doi.org/10.3390/su18104815 — emphasizes context-dependent performance, the distinction between claimed and measured co-benefits, distributional trade-offs, and monitoring logic from intervention to pathway to endpoint. It is a narrative synthesis, not a meta-analysis.
- Chelli, Brander & Geneletti (2025), *Cost-Benefit analysis of urban nature-based solutions: A systematic review of approaches and scales*, DOI: https://doi.org/10.1016/j.ecoser.2024.101684 — finds variation in which benefits are included across appraisals and highlights temporal dynamics.
- Ricciardi, Callegari & Leone (2026), *Guidelines for creating an urban digital twin module for urban regeneration scenarios assessment that include climate change mitigation and adaptation*, DOI: https://doi.org/10.1016/j.cities.2026.107020 — discusses data ownership, integration, privacy, and technical barriers in urban digital-twin modules.

The research motivates careful accounting and boundary discipline; it does not validate this schema, any proposed synergy, a specific city outcome, or a single symtropy measure.


## Paradigm shifts: from city dashboards to evidence-carrying civic experiments

The ambition should not be a prettier dashboard or a single number called city health. It should be a **reproducible public-interest decision system** in which each claim can be traced to observations, transformations, model versions, uncertainty, and an explicit decision boundary—while governance authority remains with accountable people and institutions.

### 1. Make every claim proof-carrying

Treat each reported number as a typed claim plus a provenance path: source identity, scope, period, method, units, transformations, model revision, raw-payload digest, and limitations. “Well-formed JSON,” “hash matched,” “producer authenticated,” and “scientifically supported” are different verification states. The new offline verifier closes the raw-byte and envelope-binding gap for repository-local payloads; it deliberately does not pretend that a digest authenticates a producer. This separation is consistent with the artifact/statement/predicate layers in [in-toto Attestation](https://github.com/in-toto/attestation/blob/main/spec/v1/README.md), [SLSA's artifact-verification guidance](https://slsa.dev/spec/v1.2/verifying-artifacts), and the raw-message digest model in [RFC 9530](https://www.rfc-editor.org/rfc/rfc9530.html).

### 2. Replace the monolithic digital twin with a network of bounded experiments

Keep each scenario tied to a declared functional unit and boundary, but model imports, exports, storage changes, upstream supply chains, and displaced impacts as connected flows rather than assuming a city is a closed box. Maintain the four-counterfactual interaction calculation and scenario uncertainty, but never equate arithmetic interaction with causal proof. A future causal-identification record should specify the assignment/design type, unit of analysis, intervention timing, counterfactual construction, pre-trend or balance checks where applicable, confounder strategy, interference/spillover assumptions, predeclared outcomes, and falsification controls. If that record is absent or fails review, the result ceiling stays “descriptive interaction,” not “causal synergy.”

### 3. Optimize decisions, not an opaque composite score

Keep a Pareto set across emissions, resilience, water, energy, food delivery, access, affordability, displacement, and maintenance. Apply non-compensatory safety, rights, and service floors before ranking trade-offs; no carbon or efficiency improvement should mathematically cancel a breach of a declared rights or service floor. When evidence is weak, the next action should often be to reduce uncertainty rather than choose a winner. A future value-of-information step can prioritize the measurement or field test most likely to change a decision, given its cost, urgency, uncertainty, and distribution of benefits.

### 4. Make the digital twin contestable and privacy-preserving

A simulation should inform a decision, not authorize procurement, benefits, zoning, or displacement by itself. Preserve the distinction among measured, source-estimated, model-derived, synthetic, blocked, and not-assessable values; make limitations and unresolved dependencies visible. Give affected communities a way to challenge a source, scope, target, or assumption, and use purpose-bound access plus aggregation/minimization for sensitive data. This approach complements the [FAIR principles](https://www.nature.com/articles/sdata201618) for reusable data and the European Commission's [Local Digital Twin work](https://regions-and-cities.ec.europa.eu/cities-portal/eu-initiatives-cities/eu-local-digital-twin-ldt-toolbox_en), which emphasizes scope, governance, data quality, and interoperability.

### Execution sequence

1. **Implemented in this PR:** exact-byte SHA-256 verification, envelope-to-payload binding across 17 native fields, safe repository-local paths, explicit external-unverified outcomes, and adversarial unit controls.
2. **Next integrity milestone:** introduce a signed attestation and explicit trust policy binding the payload digest, producer revision, build/workflow identity, and verifier version. A digest alone is not a signature.
3. **Next scientific milestone:** add a causal-identification contract to any claim that goes beyond descriptive interaction, with negative controls for absent or contradicted design evidence.
4. **Next decision milestone:** build a value-of-information and Pareto-based review process that points to the best next measurement without collapsing rights or uncertainty into one score.
5. **Pilot gate:** require local source licensing, community co-design, calibrated field evidence, privacy review, maintenance ownership, and independent exact-head verification. Protocol-only and synthetic fixtures remain prohibited as evidence of Johannesburg outcomes.

These are proposed engineering gates, not a claim that the latter stages already exist or that a city deployment has been validated.

### 5. Make distributional incidence a first-class outcome

Aggregate city improvement can hide a harmed neighborhood, a displaced tenant group, unpaid maintenance work, or uneven access. The schema now adds an optional \`distributional_impacts\` contract comparing a specified intervention scenario with an explicit baseline for a stable aggregate/pseudonymous affected-group reference. Quantified entries carry baseline value, scenario value, exact change, unit, uncertainty interval, evidence references, limitations, and a separate non-compensatory constraint status.

The validator recomputes \`scenario_value - baseline_value\` using exact decimal arithmetic, requires the uncertainty interval to contain that change, resolves both scenario references, rejects duplicate cohort/metric comparison identities, and requires a sourceable constraint reference when a hard floor is evaluated. For real assessments, quantified and qualitative incidence claims need evidence. Missing/not-assessed values remain null rather than being coerced to zero. A declared \`breach\` is surfaced as a decision boundary; it is not silently offset by another cohort's improvement.

This contract reduces the risk of an aggregate Pareto analysis hiding unequal incidence, but it does not prove all affected groups have been identified. A pilot still needs a documented coverage statement, community-defined cohort categories, privacy safeguards, and independent review of omissions. A single record per cohort is not an adequate substitute for intersectional analysis where impacts differ by sex, age, disability, tenure, income, location or other locally legitimate categories. Cohort references should be pseudonymous and must not include personal identifiers.

### 6. Interoperate with city standards without confusing indicators for values

ISO 37120 defines methodologies for comparable and verifiable indicators of city services and quality of life. As checked for this research pass, the published edition is ISO 37120:2018, while ISO lists Edition 3 as a working draft under development that is intended to replace it. The schema should therefore maintain a versioned crosswalk to indicator definitions rather than hard-code a presumed current edition or claim conformity. Indicators provide a consistent way to measure; they do not by themselves decide the values, distributional floors or trade-offs a city should adopt. See [ISO 37120:2018](https://www.iso.org/standard/68498.html) and the [ISO/WD 37120 working draft](https://www.iso.org/standard/90316.html).

The European Commission's Local Digital Twin Toolbox is now presented as a modular, standards-oriented set of open-source tools for simulation and evidence-based planning; its published framing emphasizes simulation rather than real-time operations. Living-City should interoperate with existing city data, models and governance rather than attempt to replace every specialized digital twin. Reuse the [EU LDT Toolbox](https://interoperable-europe.ec.europa.eu/collection/ldttoolbox) where appropriate, and retain clear boundaries for which component owns source data, simulation, provenance and civic decision authority.

### 7. Make distributional coverage explicit—and keep the unobserved visible

The distributional impact rows identify changes for named cohort references, but those rows alone cannot establish that the review found all materially affected groups. This release therefore adds a versioned `distributional_coverage` declaration required for a real `assessment`. It records the declared population frame, group-identification method, which pseudonymous cohort references appear in the impact ledger, known omissions, how omissions were reviewed, intersectional-review status and dimensions considered, evidence references, and limitations. The validator checks that coverage evidence references resolve and that every cohort appearing in a real impact row is included in the coverage declaration.

The coverage declaration is deliberately **not a completeness certificate**. An empty `known_omissions` list means the stated method identified none; it does not prove that no one was missed. A `not_assessed` or `blocked` status is a first-class result, not a pass. For privacy and safety, cohort refs remain pseudonymous and the coverage statement should prefer group-level categories to lists of identifiable people. Any pilot should invite affected communities and frontline operators to challenge the population frame, group definitions and omissions before the result is used to allocate resources or authorize a project.

This is informed by OECD guidance to identify who gains and loses, consider costs shifted to other groups, and preserve uncertainty in distributional analysis ([OECD infrastructure project-selection guidance](https://www.oecd.org/en/publications/efficient-and-sustainable-infrastructure-in-egypt_d6e57f17-en/full-report/guidelines-project-selection_439828d6.html)). It also responds to the structural caution that city-scale digital twins cannot fully consolidate all situated, tacit and changing urban knowledge ([Moroni, 2025](https://doi.org/10.1007/s43762-025-00174-0)). So the architectural goal is not a complete all-knowing city replica; it is a revisable, contestable model that declares its blind spots and makes uncertainty actionable.


### 8. Separate arithmetic interaction from causal inference

A four-scenario difference-in-differences calculation is an arithmetic interaction contrast. By itself it does not establish that an intervention caused the observed difference: time trends, selection, implementation changes, spillovers, concurrent interventions, and measurement changes can produce similar patterns. The UK government's current Magenta Book and Test and Learn guidance makes the counterfactual and theory of change central, recommends checking alternative explanations, and treats scale-up as a later stage that requires stronger evidence ([Magenta Book, updated 15 May 2026](https://www.gov.uk/government/publications/the-magenta-book), [Test and Learn](https://www.gov.uk/government/publications/the-magenta-book/test-and-learn-html), [analytical methods annex](https://www.gov.uk/government/publications/the-magenta-book/magenta-book-annex-a-analytical-methods-for-use-within-an-evaluation-html)).

The versioned `causal_evaluations` contract ties each inference statement to a known interaction and records claim level, design, unit of analysis, treatment assignment, counterfactual strategy, confounder strategy, interference assumptions, predeclared outcomes, evidence, falsification checks, and a review receipt. It distinguishes:
- `descriptive_interaction`: arithmetic/model contrast only; no causal language implied.
- `contribution_claim`: a documented contribution-analysis or process-tracing claim, requiring evidence and review.
- `causal_effect_claim`: a candidate claim requiring an experimental/quasi-experimental design, resolved evidence and independent review, at least one negative control plus a distinct robustness/falsification check, and all declared checks marked consistent with the design's assumptions.

The validator fails closed on unresolved interaction/evidence/review references, an ineligible design, a missing or unreviewed evidence chain, or falsification checks that are contradicted, inconclusive, not run, or unsupported. These gates improve traceability and discipline; they **do not independently prove causal identification**. For example, a difference-in-differences study still needs a defensible comparison group, treatment timing, sufficient pre-intervention data where available, a pre-trend assessment, spillover analysis, and sensitivity checks. A randomized design still needs allocation integrity, attrition and noncompliance assessment, pre-specified outcomes, and respect for clustered assignment. The appropriateness of robustness checks depends on the intervention and estimand.

## Benchmark targets (proposed; not yet measured)

These are release gates and engineering targets, not reported accomplishments. Method-specific thresholds should be preregistered and reviewed; no universal sample size or p-value makes every city intervention causal.

| Dimension | Acceptance benchmark | Evidence required |
| --- | --- | --- |
| Contract correctness | 100% of declared mutation controls reject for the intended reason; 100% of positive controls validate. | Exact-head deterministic test output; no muted controls. |
| Evidence integrity | 100% of referenced interaction, evidence, review, cohort, and model-run IDs resolve; zero silent fallback to zero for missing values. | Semantic validator report and adversarial fixtures. |
| Inference discipline | 100% of causal-effect claims blocked when design, review, negative control, or independent robustness/falsification evidence is absent or fails. | Mutation tests for each missing/contradicted gate; reviewer receipt. |
| Reproducibility | Two clean executions with the same commit, inputs, and pinned dependency set produce byte-identical normalized validation reports. | Environment/dependency manifest and report digest. |
| Validator scale (target) | On a pinned 4-vCPU/8-GiB reference runner: p95 under 2 s for 1,000 flows, under 10 s for 10,000 flows, and under 60 s for 100,000 flows; peak RSS under 1 GiB. | Seeded fixtures; five repeated runs per size; record median, p95, RSS, runner and exact commit. Timing excludes queue wait and must not weaken correctness checks. |
| Forecast calibration (future pilot) | For at least 100 held-out site/time cases per metric where feasible, nominal 80% and 95% predictive intervals should achieve empirical coverage within ±5 percentage points of nominal, with binomial uncertainty; compare forecast errors with a seasonal/persistence baseline. | Frozen out-of-sample protocol, baseline, calibration plots and metric-level results; report inconclusive when evidence is sparse. |
| Decision safety | 100% of critical rights/safety/service floors are named before selection; no scenario with a confirmed floor breach advances as eligible merely because another outcome improves. | Decision record, group-level incidence and explicit unresolved/blocked states. |
| Field impact | Before scaling, test the preregistered primary outcome against a credible counterfactual where feasible; report costs and unintended effects by affected group, and replicate or explain context-specific heterogeneity. | Independent evaluation, source provenance and operating-owner sign-off. |

The scale targets must first be benchmarked with a versioned generator and measured on the named runner. Forecast calibration and field-impact targets remain future pilot gates, not features already implemented. If a performance target is missed, report the baseline and bottleneck; do not weaken validation or shrink the adversarial corpus.


### Independent review must be more than a second label

For contribution or causal-effect claims, the review receipt must resolve separately from the outcome-source evidence references. Causal-effect claims must declare an `independent_reviewer` role; contribution claims require an independent or community reviewer role. This detects a simple but important self-attestation failure mode: reusing the same evidence identifier as both outcome support and review. The reviewer-role field is still a declaration, not cryptographic identity proof; the review record itself must be scrutinized under the host governance process.


### 9. Calibrate the digital twin throughout its lifecycle

A model's uncertainty should not be one static confidence badge. Recent built-environment digital-twin research proposes linking uncertainty to data quality, model form, calibration, benchmark residuals, lifecycle stage and level of detail, then updating against physical observations rather than treating uncertainty as post-hoc prose ([Malihi, 2026, *Towards Uncertainty-Aware Digital Twins in the Built Environment*](https://doi.org/10.1016/j.dte.2026.100140)). This motivates a future calibration receipt per metric/model/boundary: versioned calibration dataset, held-out split, level-of-detail, residual distribution, nominal-vs-empirical interval coverage, measurement uncertainty, decision tolerance and recalibration triggers.

The standardisation landscape is also active: ISO lists IEC/DTS 63526, a draft technical specification on standards related to City Information Modelling and Urban Digital Twins, as under development and in the approval phase (https://www.iso.org/standard/87250.html). Living-City should maintain versioned crosswalks and explicit conformance status instead of hard-coding an assumed stable standard. Neither the draft nor the research validates this schema; they are reasons to keep interoperability and calibration evidence explicit.


### 10. Make calibration an auditable, metric-specific claim

The new `calibration_receipts` contract makes prediction-interval coverage a per-model-run, per-metric record bound to the training and evaluation data digests. It records the evaluation window/design, source-evidence references, independently scored unit definition, observed hit count, nominal coverage, predeclared tolerance and minimum sample count, a recomputed Wilson interval, result, limitations and a recalibration trigger. Training and evaluation digests must differ; for this v0.1 Wilson method, sample count must equal independent-unit count so repeated correlated observations are not incorrectly treated as independent Bernoulli trials. Time-series, spatially clustered or repeated-observation data require a reviewed block/cluster method in a later version, not an unexamined use of Wilson intervals.

A receipt is `pass` only when the full coverage confidence interval falls inside the predeclared nominal-coverage tolerance band and the minimum sample thresholds are met. It is `fail` if adequately sampled interval coverage is entirely outside that band, and `inconclusive` when uncertainty overlaps the band boundary or sample thresholds are not met. The validator recomputes the empirical coverage from integer counts and the Wilson interval from counts/confidence level; it does not accept a user-entered pass label on its own. Calibrated/validated model runs must link to passing receipts for the model-output metrics they declare.

This follows the established distinction between calibration and sharpness in probabilistic prediction assessment (Gneiting, Balabdaoui & Raftery, 2007, [*Probabilistic Forecasts, Calibration and Sharpness*](https://doi.org/10.1111/j.1467-9868.2007.00587.x)) and is consistent with recent uncertainty-aware digital-twin work that treats calibration as lifecycle evidence rather than post-hoc prose ([Malihi, 2026](https://doi.org/10.1016/j.dte.2026.100140)). The receipt tests interval coverage only; future calibration work should also add proper scoring rules, sharpness diagnostics, metric-specific residual checks, measurement/model-form uncertainty separation, and baseline forecast comparisons. These controls do not validate the scientific model or prove transfer to unobserved locations, seasons or intervention types.


Calibration is not sufficient by itself: arbitrarily wide prediction intervals can attain high coverage while conveying little decision value. Each v0.1 calibration receipt therefore also reports mean interval width relative to a named baseline forecast evaluated on the same cases at the same nominal coverage, metric, and unit. The ratio is recomputed exactly from the declared widths, with lower values representing narrower intervals. This is a sharpness descriptor, **not** a stand-alone quality score and not a substitute for proper scoring rules. The calibration result continues to depend on interval coverage and its predeclared tolerance; decision-makers see width alongside coverage instead of silently blending the two.
