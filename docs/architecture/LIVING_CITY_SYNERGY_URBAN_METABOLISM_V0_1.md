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

Energy, water, and conserved material mass use canonical base units of watt-hours (Wh), millilitres (mL), and grams (g). Quantities are decimal integer strings so clients cannot silently round large values through IEEE-754 floating-point numbers. The validator recomputes the residual from the fields and compares it to an explicit integer tolerance; a `balanced` label cannot override arithmetic. In a real `assessment` record, each balance check also requires Mycelix evidence-claim references that resolve; synthetic-only fixtures are forbidden from citing real-world evidence.

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

where `Y(0)` is the baseline, `Y(A)` and `Y(B)` are single-intervention cases, and `Y(A+B)` is the combined case. The validator checks the arithmetic and unit consistency; it does not establish causal identification by itself. Real estimates still need defensible study design, evidence, uncertainty analysis, and attention to weather, demand, and other confounders.

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

The research motivates careful accounting and boundary discipline; it does not validate this schema, any proposed synergy, a specific city outcome, or a single symtropy measure.
