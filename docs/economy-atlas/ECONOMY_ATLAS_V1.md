# Economy Atlas v1 — model registry and executable visualization plan

**Status:** design proposal; no production UI or simulation qualification implied  
**Owner area:** Mycelix / Symthaea economics integration  
**Branch:** `feat/economy-atlas-model-registry-v1`

## Why this exists

The Economy Atlas should make economic models inspectable, explorable, and comparable without pretending that every model describes the same thing. It needs to show both the model and the evidence that warrants trusting a result.

The first implementation should reuse existing model code wherever possible. A browser-side reimplementation of an equation is a useful mock, but it is not a verified execution of the Rust model. The UI must label that difference.

## Findings from the source tree

This is an initial, evidence-linked inventory—not a claim that every economy-related model in every repository has been found. In particular, the current search pass did not resolve the canonical location/version of Creditism / FIN-ECO-006; that remains an explicit discovery item.

| Model ID | What it represents | Source of record | Evidence/status for Atlas |
|---|---|---|---|
| `mycelix.finance.tricurrency.v1` | MYCEL reputation, SAP circulation/demurrage, TEND mutual credit, treasury/reserves, fees and limits | [Mycelix Finance README](https://github.com/Luminous-Dynamics/mycelix/blob/main/mycelix-finance/README.md) | Implemented design/cluster code exists; root maturity matrix says Finance has partial multi-agent testing and no dedicated UI. Do not label it production-ready. |
| `symthaea.economics.kernel.v1` | Linear supply/demand equilibrium, administered prices, taxes, surplus/deadweight loss, inequality metrics, finance calculations and 2×2 games | [Crate README](https://github.com/Luminous-Dynamics/symthaea/blob/main/crates/domains/symthaea-economics/README.md), [public API](https://github.com/Luminous-Dynamics/symthaea/blob/main/crates/domains/symthaea-economics/src/lib.rs) | Executable pure-std Rust math kernel with input/error contracts and unit tests in source. Best first engine to wire into a live explorer. |
| `symthaea.economy.thermodynamic-ledger.v1` | A TEND-like supply initialized at 1,000, production-linked credit minting and surprise/entropy-linked burning | [economy.rs](https://github.com/Luminous-Dynamics/symthaea/blob/main/crates/core/symthaea-infrastructure/src/economy.rs) | Concrete implementation, but it is not semantically equivalent to the zero-sum TEND mutual-credit ledger. Inputs need a fail-closed validation adapter before exposure in a UI. |
| `symthaea.energy.lcoe.v1` | Energy project cost model: capital recovery, O&M, fuel, capacity factor, lifetime/payback and related viability metrics | [economics.rs](https://github.com/Luminous-Dynamics/symthaea/blob/main/crates/core/symthaea-core/src/physics/economics.rs) | Executable source model; input assumptions and engineering claims require their own provenance. Present currencies, horizon, capacity factor and units explicitly. |
| `symthaea.civilization.multiworld.v1` | Multi-sector production/resource simulation; paper describes eight-sector modified Cobb–Douglas production, resource consumption, demurrage, innovation and inter-world dynamics | [Planetary civilization paper](https://github.com/Luminous-Dynamics/symthaea/blob/main/papers/governance/planetary-civilization/planetary_civilization.tex) | Paper describes a reproducible simulator and JSONL telemetry, but the paper also says this economic layer does not yet model price dynamics, market formation or monetary policy. Atlas must trace the current executable/configuration before claiming reproduction. |
| `symthaea.governance.tricurrency.paper-results.v1` | Reported sensitivity experiments for demurrage and Jubilee compression under a triple-currency governance simulation | [Embodied governance paper](https://github.com/Luminous-Dynamics/symthaea/blob/main/papers/governance/embodied-governance/embodied_governance.tex) | Paper-reported results only in this inventory. The paper reports no significant velocity effect across 1–5% demurrage under its exempt-floor constraint and MYCEL Gini around 0.016 across tested Jubilee frequencies. Re-run and inspect the experiment before treating the numbers as qualified evidence. |
| `mycelix.finance.macro-evidence-profile.v1` | Provenance and semantic contract for external macro-financial observations (IMF, BIS, World Bank) | [FIN-SYS source profile](https://github.com/Luminous-Dynamics/mycelix/blob/main/docs/finance/FIN_SYS_SOURCE_PROFILE_V1.md) | Design contract only; explicitly grants no ingestion authority. It is an evidence/source profile, not an executable macroeconomic simulator. |
| `symthaea.concepts.tricurrency.essay16.v1` | A conceptual three-currency commons proposal with a different role assignment | [Essay 16](https://github.com/Luminous-Dynamics/symthaea/blob/main/book/src/sovereignty/essay-16.md) | Conceptual prose, not interchangeable with the implemented Finance README. Keep as a separately named variant until an owner-approved canonical relationship is documented. |
| `regenerative.economics.energy-network-science.v1` | Regenerative economics through circulation, internal reinvestment, flow-network resilience, diversity, mutualism and collective learning | [Fath et al., *Measuring regenerative economics* (2019)](https://doi.org/10.1016/j.glt.2019.02.002) | External framework; research-linked but not implemented or executed by this repository. The authors call for further testing and refinement of socio-economic applications. |
| `regenerative.doughnut.place-portrait.v1` | Place-based social foundation and ecological ceiling, analyzed across local/global and social/ecological lenses | [Doughnut Economics Action Lab](https://doughnuteconomics.org/doughnut/), [Data Portraits tool v1.1](https://doughnuteconomics.org/tools/doughnut-data-portraits-visualiser-data-hub) | External framework and tool. The published guide describes 44 dimensions across four lenses; Atlas indicators still need explicit targets, units, sources, transformations and missingness. |
| `regenerative.circular-economy.v1` | Product/material circulation, waste and pollution prevention, and nature regeneration | [Ellen MacArthur Foundation](https://www.ellenmacarthurfoundation.org/topics/circular-economy-introduction/overview) | External framework. The Atlas's candidate indicators are proposed integration fields, not a claim that the Foundation prescribes one universal metric set. |
| `regenerative.land.agroecology-outcomes.v1` | Outcome-oriented regenerative land and agriculture lens for soil, water, biodiversity, ecosystem services and rural livelihoods | [Newton et al. review (2020)](https://doi.org/10.3389/fsufs.2020.577723), [FAO overview](https://www.fao.org/americas/priorities/agricultura-sostenible/practicas-innovacion/3.-regenerative-agriculture/en) | External framework only. Definitions vary; practices are not proof of regenerative outcomes. Require a local baseline and measured outcomes. |
| `creditism.fineco.discovery.v1` | Creditism / FIN-ECO-006 model family | Canonical source not resolved in this pass | Discovery required. Do not invent parameters or present a UI sketch as a verified implementation. |

## Semantic conflicts the Atlas must show, not hide

### The currency names currently refer to different model contracts

**Mycelix Finance README contract**
- MYCEL: non-transferable reputation substrate, 0–1.
- SAP: transferable circulation medium, subject to demurrage.
- TEND: zero-sum mutual credit, with 1 TEND = 1 hour of service; balances sum to zero.

**Symthaea Essay 16 conceptual contract**
- MYCEL: medium of exchange.
- SAP: long-term commons-investment/nutrient currency with demurrage if idle.
- TEND: care currency with restricted/non-market conversion.

**Symthaea thermodynamic ledger contract**
- A ledger initialized with supply 1,000 adds production-linked credit and subtracts surprise-linked entropy tax. This is not a zero-sum bilateral mutual-credit ledger.

These are distinct variants, not aliases. The UI must display the source, variant ID, status, and disagreement. It must not silently choose a definition or merge the balance equations.

### Important adapter-safety gap in the thermodynamic ledger

The current `mint_production_credit` method gates on low efficiency and low collective Phi, but does not itself reject negative or non-finite production values or constrain efficiency to its natural range. The entropy method also does not reject negative or non-finite surprise. For example, a negative surprise can make the computed “tax” negative and increase the ledger supply instead of burning it.

Until the model contract and implementation are reconciled, any Atlas adapter should fail closed on non-finite values, negative production, efficiency outside [0, 1], negative surprise, and negative Phi. The maximum/domain of collective Phi is not specified by this method; do not invent a scientific bound. Display adapter validation separately from the model's own validation and preserve this finding as a visible qualification issue.

## Product experience

The Atlas has five tightly connected views, sharing one selected model/variant:

1. **Model map** — groups models by monetary systems, markets/welfare, finance/debt, commons/governance, resource/energy, and civilization-scale dynamics. Each card carries an evidence state: `Implemented`, `Paper-reported`, `Design-only`, `Conceptual`, `External framework`, or `Discovery required`.
2. **Explore** — a model-specific form with units, documented ranges, definitions, constraints, source location, and live results. Results must be calculated by the registered runner, not by presentation-only formulas.
3. **Compare** — run two or more versions against the same valid scenario. Only compare metrics with matching semantic meaning, units, temporal horizon, population/sector scope, and denominators. Do not create an aggregate “best economy” score by default.
4. **Scenario lab** — freeze inputs, model versions, seed (only for stochastic models), and scenario assumptions. Support paired baseline/intervention runs and sensitivity curves. Missing inputs remain `Unknown`, never silently imputed.
5. **Evidence & differences** — expose source links, exact revision, assumptions, equation/version, validation receipt, known limitations, provenance of empirical inputs, and disputed/contradictory definitions. Paper-reported findings are not presented as reproduced runs.

Suggested interaction pattern: select a model family → select a named variant → inspect its causal/flow diagram → edit validated parameters → run → inspect result series and metric definitions → compare against an explicitly compatible baseline → export an evidence packet.

## Technical boundary

- **UI:** Leptos, consistent with the existing Mycelix frontend stack. Use accessible tables and direct manipulation for parameters; use SVG/canvas only for the model-specific visual (flow, supply/demand curves, time series, network), not as the sole explanation.
- **Execution:** typed Rust adapters over existing Rust models. Prefer calling the same kernel used by the repository—e.g. `symthaea-economics`—rather than implementing an untracked JavaScript duplicate. If a temporary browser-only equation is used in a mock, label it as a prototype and do not badge it as source-engine output.
- **Registry:** versioned, data-driven manifest; avoid hard-coded source logic inside components. Minimum fields:
  - `model_id`, `variant_id`, `model_version`, `title`, `domain`;
  - `epistemic_status` and `execution_status` as separate fields;
  - `source_refs[]` with repository, path, ref/commit, and exact symbol/section;
  - `research_refs[]` for external papers, framework documentation, and tools, kept distinct from in-repository implementation evidence;
  - typed inputs with units, domains, default provenance and validation;
  - typed outputs with units, dimensions, aggregation/horizon and comparison keys;
  - runner/adapter ID, stochastic flag, required seed policy;
  - assumptions, exclusions, failure modes and known conflicts.
- **Run receipt:** each completed run records the exact model/adapter version, source revision, canonical input payload + digest, seed if applicable, engine version, units/schema version, output digest, warnings and validation outcome. “Queued” or “rendered” is not equivalent to “executed” or “passed.”
- **Evidence receipts:** empirical series need provider family, exact dataset/series/version, geography, sector, period, retrieval time, units/denominator, transformation, artifact digest, coverage/missingness and revision links, following the existing FIN-SYS source profile.
- **Authority:** visualization is inspect/compare only. It must not be able to change deployed finance parameters or treat Symthaea advisory output as policy authority.

## First vertical slice

Start with the already separated linear-market kernel because its mathematical contract and error behavior are explicit. Ship a narrow end-to-end slice before attempting one universal economy simulator.

1. Register demand/supply equilibrium, administered-price outcome, tax incidence/deadweight loss, and distribution metrics.
2. Use the Rust implementation as execution authority. Add a thin Leptos-facing adapter; do not fork the equations in UI code.
3. Render the demand/supply curves, equilibrium, tax wedge, surplus breakdown, and the exact inputs/units.
4. Add baseline-versus-policy run comparison, deterministic input snapshot, and exportable JSON receipt.
5. Test normal cases, zero boundaries, invalid parameters, no-solution cases, numerical overflow, and result identities with fixture-based UI/adapter tests.
6. Add the currency variant-comparison view and thermodynamic-ledger validation issue as the next slice.
7. Do not wire the planetary or paper-reported simulations into “Run” until the exact executable, configuration, seed behavior, and telemetry schema have been independently traced.

## Qualification criteria

A slice is not complete just because the page renders. At minimum:

- Registry validation rejects missing source/status/version fields and duplicate IDs.
- Model input validation rejects NaN/infinity and out-of-domain values before execution.
- Runner failures remain failures; no fallback to plausible dummy numbers.
- Identical deterministic model version + canonical inputs yields identical results/receipt outputs.
- Stochastic reruns with the same seed reproduce the same outputs where the engine claims determinism; unseeded results are not silently compared as paired runs.
- A semantic mismatch (different units, currency meaning, scope, or horizon) blocks or explicitly warns on comparison.
- Every chart has a text/table equivalent, units, interval/horizon, and linked assumptions.
- Paper numbers and external data are labeled with the actual evidence level and are never promoted to validated model output by rendering them.
- The export packet is replayable and includes failures/warnings—not only successful outputs.
- No model run can mutate Mycelix Finance production state.

## Open work / next discoveries

- Resolve the canonical Creditism / FIN-ECO-006 specification, implementation, test corpus and current evidence receipts.
- Trace actual multiworld simulation entrypoints, config files, seeds, output schema and latest executable behavior.
- Find/reproduce the source experiment behind the reported triple-currency governance findings.
- Decide with explicit provenance whether Essay 16 is a legacy conceptual variant, an intended alternative design, or a proposal for a future canonical version.
- Reconcile the thermodynamic ledger's issuance/burn semantics and input validation with its intended economy model.
- Extract actual Finance policy parameters and integrity/coordinator validations rather than trusting README defaults alone.
- Audit the Energy/LCOE model's assumptions and financial units before using it for consequential comparisons.

## Explicit non-claims

This proposal does not claim the full model inventory is complete, that all listed simulators have been executed in this pass, that any paper result was independently reproduced, that model outputs are empirically validated, that the currency conflict has been resolved, or that a working production UI already exists.

## Machine-checkable registry guardrails

The registry now pins its source references to the Mycelix snapshot `445a84abdaa64f3d05e2d5c51115e4d060b78ec7` and the Symthaea snapshot `b9af191f380d635f0459b24284c1d1fd6a5516aa`. Registry permalinks must match both the declared repository/path and the source snapshot map. A source update should create a reviewed registry revision, not silently move a branch pointer.

Artifacts added for this contract:

- `docs/economy-atlas/model-registry.schema.json` — JSON Schema shape for the v1 discovery registry.
- `scripts/validate_economy_atlas_registry.py` — dependency-free validation of unique model IDs, pinned commit references, source permalink consistency, status fields, and explicit currency-variant separation.
- `tests/test_economy_atlas_registry.py` — regression tests for duplicate IDs, mutable refs, source/url mismatch, missing source links, and attempts to mark the discovery registry as execution-qualified.

Run the checks from the repository root:

```bash
python3 scripts/validate_economy_atlas_registry.py
python3 -m unittest discover -s tests -p 'test_economy_atlas_registry.py'
```

These checks validate registry contract and provenance metadata only. They do not execute Rust models, prove that the pinned source implements the documented equations, reproduce a paper, or qualify an economic claim.

## Compost accounting and delivery findings surfaced by the Atlas

The registry now separates three SDK-utility findings in [decay_garden.rs](https://github.com/Luminous-Dynamics/mycelix/blob/445a84abdaa64f3d05e2d5c51115e4d060b78ec7/mycelix-workspace/sdk/src/economics/decay_garden.rs) from a fourth finding in the active Finance coordinator, [payments/coordinator/src/lib.rs](https://github.com/Luminous-Dynamics/mycelix/blob/445a84abdaa64f3d05e2d5c51115e4d060b78ec7/mycelix-finance/zomes/payments/coordinator/src/lib.rs):

1. `CompostDistribution::is_valid` checks only whether the sum of the ratios is approximately one. It does not separately enforce that each ratio is finite and within `[0, 1]`. A ratio set such as `(1.1, -0.1, 0.0)` satisfies the sum check but can allocate more compost to one pool than was received after float-to-integer conversion.
2. `CompostDistribution::distribute` truncates each ratio allocation independently. For total 11 at 70/20/10, the allocations are 7, 2 and 1, leaving one unit undistributed. The intended remainder policy is not documented in that primitive.
3. The low-level `calculate_demurrage` primitive returns zero when the rate is non-positive, but does not itself enforce the `DemurrageConfig` comment's constitutional 1–5% interval.
4. In `apply_hearth_demurrage`, pool balance deductions are persisted before treasury calls. Failed calls are logged but not queued for retry, and the function returns `redistributed: true` unconditionally. Missing optional destination IDs also skip that tier. The personal `apply_demurrage` path does have a pending queue for failed calls, but similarly skips absent optional destination IDs without setting `fully_redistributed = false`. Importantly, both active-zome paths assign the rounding residual to the global pool; that particular total-loss behavior belongs to the separate SDK utility and should not be generalized to the coordinator path.

These are source-level arithmetic/control-flow findings, not proof of an observed production loss or a reachable exploit. The active-zome finding is stronger than a README discrepancy because the control flow is present in the pinned coordinator source, but its runtime impact and callers still require integration evidence. The Atlas exposes the source and required follow-up without changing finance behavior.

Before any runtime change, tests should establish the intended settlement contract: missing destinations cannot be reported as fully redistributed; failed treasury delivery is represented in durable pending state or an explicit failure; recovery does not double-credit; and `deducted = delivered + pending/unresolved` holds exactly under the chosen integer remainder rule. Only then should the shared utility's ratio-domain and residual policy be fixed at the narrowest trust boundary.

## Thermodynamic ledger input findings

The structured registry also records three findings for Symthaea's pinned [thermodynamic ledger](https://github.com/Luminous-Dynamics/symthaea/blob/b9af191f380d635f0459b24284c1d1fd6a5516aa/crates/core/symthaea-infrastructure/src/economy.rs):

1. `mint_production_credit` does not itself reject all negative or non-finite production/efficiency/Phi inputs. The Atlas must validate these domains before calling the method; the caller path remains to be traced.
2. `apply_entropy_tax` does not reject negative surprise. A negative input produces a negative tax, and subtracting that tax increases `total_tend_supply`. Add direct boundary/regression tests and confirm all upstream call sites share the same validation contract.
3. The method gates low `collective_phi` values but does not define a maximum or canonical calibration scale. The UI must preserve the unspecified upper bound rather than inventing one.

All seven findings are structured records with globally unique IDs and an exact source repository/path/commit binding. The Leptos page expects this typed representation; unstructured strings under `validation_findings` are rejected by the registry validator and covered by regression tests. These are static source findings, not proof that an invalid value is reachable from a deployed caller. The registry still claims no execution or runtime qualification.

## Leptos integration status

The PR now also adds a registry-driven page at `/atlas` in the existing Finance Leptos app. It loads the checked-in JSON at compile time, exposes source snapshots, searches model titles/domains/statuses/limitations, and renders limitations and the structured findings above. A malformed registry produces an explicit error state; it does not substitute mock entries.

This page is a **registry explorer**, not yet a simulation runner. It deliberately does not import the entire SDK into the browser WASM bundle or call live finance mutations. The standalone HTML workbench remains a separate illustrative prototype. The next runtime milestone is a narrow, versioned adapter that invokes a source-backed economic primitive and returns a replayable run receipt.

## Regenerative economics as a first-class Atlas lens

The registry now includes three externally documented framework families, each marked `external_framework` with `execution_status` explicitly stating that a local runner has not been implemented. Their links live in `research_refs[]`, not `source_refs[]`: an external paper or website is research provenance, not evidence that Mycelix contains executable code. Each link records `retrieved_on` so future edits to a mutable webpage do not silently change the evidence snapshot; a retrieval date does not make that webpage immutable.

### 1. Energy Network Science / regenerative economics

Fath, Fiscus, Goerner, Berea, and Ulanowicz (2019) propose ten principles and associated measures for systemic socio-economic health. Their approach covers cross-scale circulation; reinvestment in human, social, natural, and physical capacity; reliable inputs and healthy outputs; a balance between efficiency and resilience; diversity; mutualistic relationships; constructive versus extractive/speculative activity; and adaptive collective learning. Some candidate measures include network aggradation, the Finn Cycling Index, and the efficiency/resilience “window of vitality”.

**Integration rule:** flow-network metrics need a declared boundary, typed flow units, time window, input/output flows, and enough network data to calculate them. Money alone cannot stand in for energy, materials, biodiversity, or care. The research itself says socio-economic application, interpretation, and refinement remain important. The Atlas should store metric definitions and coverage rather than emit an unqualified regenerative score.

### 2. Doughnut Economics / place portraits

DEAL's Data Portraits tool describes 44 dimensions grouped into four lenses: local-social, local-ecological, global-social, and global-ecological. Its display normalization differs by lens: social values are represented from deprivation (0) to target (1), while ecological values use 1 for the target boundary and values above 1 for overshoot. These are visualization conventions, not common native units.

**Integration rule:** every indicator needs a name, value, unit, geography, time period, target, directionality, transformation, data source, provenance, and missingness state. An unfilled field must not silently become zero or an apparent safe value. The Atlas should expose deprivation and ecological overshoot separately, with missing/unknown dimensions clearly marked. We have not imported local Johannesburg/South African indicator data or reproduced DEAL's visualiser in this change.

### 4. Regenerative land and agriculture — outcomes, not practice badges

The FAO-hosted literature review by Newton et al. (2020) reports that scholarly and practitioner definitions vary and have not converged on one universally accepted definition. The Atlas therefore treats regenerative agriculture as an outcome-oriented framework, not a binary certification label.

**Candidate outcomes** include soil organic matter/carbon and soil structure, water infiltration/retention/runoff and water quality, biodiversity measured through a declared method, nutrient/input loss, yield stability, and farm livelihoods. Each needs a site, baseline, sampling method, season/time horizon, uncertainty, and provenance record. Cover crops, no-till, livestock integration or other practices may be inputs/interventions; none by itself proves that outcomes improved. Avoid claims of verified sequestration or biodiversity gain without suitable observations.

### 3. Circular economy / material circulation

The Ellen MacArthur Foundation frames circular economy around three principles: eliminate waste and pollution, circulate products and materials at their highest value, and regenerate nature. The Atlas will treat repair, reuse, refurbishment, remanufacturing, recycling, and biological return as different pathways rather than collapse them into one recycling percentage.

**Candidate Atlas indicators** (proposed integration fields, not a claimed official universal metric set) include virgin-material input, material throughput by type, product life extension, repair/reuse/remanufacturing flows, waste and pollution, material leakage, and measured ecosystem restoration. Circularity rates must be read alongside absolute throughput and absolute harm; a higher proportion recycled does not by itself show that extraction fell or ecosystems recovered.

### Shared regenerative comparison contract

Regenerative economics should cross-link the finance, energy, commons, governance, and macro-evidence models, but it must not replace their distinct model contracts. For now:
- Keep social well-being, ecological pressure, monetary circulation, energy flows, and material flows as separate typed metrics.
- Compare only matching indicator definitions, units, geographic and population scope, time horizon, and provenance quality.
- Make trade-offs visible; do not invent default weights for a single “regeneration score”.
- Distinguish targets (normative choices), observations (empirical evidence), model outputs (computed values), and interpretation (inference).
- Keep missing data unknown; expose coverage and sensitivity to alternative targets.
- Preserve each source's own limitations and do not represent these external frameworks as Mycelix-validated methods.

The four regenerative framework records are mandatory registry entries and are covered by validation checks for HTTPS research metadata. The overall registry remains `qualified_for_execution=false`.




## Regenerative indicator catalog v1

The regenerative-economics lens now has a separate, versioned measurement catalog:

- `docs/economy-atlas/regenerative-indicators.v1.json` — 18 candidate indicator definitions across social needs, ecosystem extent/condition/services, climate pressure, freshwater quality, soil, biodiversity, materials, product/material circulation, waste, agricultural yield stability, and energy-flow networks.
- `docs/economy-atlas/regenerative-indicators.schema.json` — machine-readable shape for the candidate definitions, comparison contract, reference metadata and fail-closed qualification flags.
- `scripts/validate_regenerative_indicators.py` — standard-library structural/provenance validator; checks required indicator IDs, duplicate IDs, field contracts, source-reference IDs, HTTPS links, real retrieval dates, framework links back to the model registry, and all qualification flags.
- `tests/test_regenerative_indicators.py` — regression cases covering duplicates, missing required indicators, unknown citations/frameworks, non-HTTPS references, impossible dates, missing comparability keys and attempts to claim computed/validated results.
- The Leptos `/atlas` page renders the catalog and its comparison/missingness rules. Each card exposes its native unit, denominator, boundaries, time window, baseline/target requirements, uncertainty, required inputs, comparability key, missing-data rule, caveats and cited research.

### Catalog evidence contract

Every entry is `candidate_definition`, with `observations_loaded=false` and `computed=false`. The catalog's qualification flags for loaded observations, computed indicators, independent validation and composite scoring must remain false until each is deliberately implemented and supported by evidence. A linked paper or methodology describes the source of a proposed definition; it does not establish that Mycelix measured it or that a real-world outcome improved.

The initial source base combines the UN's [SEEA Ecosystem Accounting](https://seea.un.org/en/methodology/ecosystem-accounting), the [SDG indicator metadata repository](https://unstats.un.org/sdgs/metadata/), FAO's [Global Soil Health Indicators and Assessment](https://www.fao.org/soils-portal/soil-degradation-restoration/global-soil-health-indicators-and-assessment/en/), the Ellen MacArthur Foundation's [circular-economy measurement guidance](https://www.ellenmacarthurfoundation.org/topics/measurement/overview) and [Material Circularity Indicator](https://www.ellenmacarthurfoundation.org/material-circularity-indicator), DEAL's [Doughnut Data Portrait methodology](https://doughnuteconomics.org/tools/doughnut-data-portraits-a-methodological-guide), Fath et al.'s [energy-network/regenerative economics article](https://doi.org/10.1016/j.glt.2019.02.002), and the [review of regenerative-agriculture definitions](https://doi.org/10.3389/fsufs.2020.577723). Reference retrieval dates aid change tracking but are not content digests or proof that external pages are immutable.

### Rules that protect interpretation

- Preserve each native unit and component series. Normalization is a separately versioned transform; it must not overwrite the native value.
- Compare only when definition, denominator, geography/population, time window, boundary and relevant method versions match.
- Report absolute emissions/material flows alongside per-capita or intensity ratios. Intensity can improve while total pressure worsens.
- A missing observation, not-collected observation, suppressed value, below-detection-limit result, not-applicable case, and not-computable result are different states. None silently becomes zero.
- Ecosystem extent, condition, service flows, wellbeing, financial transactions, material throughput and energy flows remain separate dimensions. Money alone is not a proxy for ecological or care flows.
- Any future composite score requires an explicit target/weight provenance, sensitivity analysis, and separate presentation of components. There is no default regeneration score in v1.

Run the catalog check and tests from the repository root:

```bash
python3 scripts/validate_regenerative_indicators.py
python3 -m unittest discover -s tests -p 'test_regenerative_indicators.py'
```

These are intended checks; they are not reported as executed by this authoring pass. They validate catalog contracts only and do not load datasets, calculate indicators, establish causal effects, or qualify model execution.

### Next evidence milestone

Only after the catalog contract is reviewed should an observation dataset be attached. Each imported series should carry a source institution, dataset/series ID, source revision or retrieval date, geography/boundary, time range, native unit, denominator, transformation, coverage and missingness, uncertainty/quality flags, and a content digest where the source artifact can be retained. Preserve the original series and transformed series side by side. Start with one narrowly scoped, high-quality, repeatable series rather than bulk-importing poorly harmonized sustainability data.


## Regenerative observation receipt contract

The next layer is a **receipt format**, not a data import. It defines what evidence must accompany a future numeric observation and prevents a missing value from being silently represented as zero.

Artifacts:

- `docs/economy-atlas/regenerative-observation-record.schema.json` — JSON Schema contract for a single observation receipt.
- `scripts/validate_regenerative_observation.py` — standard-library validator bound to indicator IDs in the checked-in candidate catalog.
- `tests/test_regenerative_observation.py` — synthetic-only unit fixtures for source-digest requirements, date ordering, non-finite values, status/measurement consistency, below-detection-limit censoring, and anti-overclaiming.

Each receipt must carry the selected indicator ID and catalog SHA-256, source publisher/dataset/series/revision, HTTPS source URL, source retrieval date, access/licensing note, retained source-artifact digest, geographic and boundary identifiers, period and temporal resolution, native unit and denominator definition, explicit observation state, transformation method/version/parameters/input digest, data coverage, uncertainty status, and quality flags. The source-artifact and transformation-input hashes provide identity checks when the corresponding source bytes are available; the validator does not independently fetch or authenticate those bytes.

### Measurement/missingness rules

- `observed` and `estimated` require a finite numeric point value. Estimates remain marked as estimates.
- `below_detection_limit` requires an explicit censoring operator, threshold and unit; it is not converted to zero.
- `not_collected`, `not_applicable`, `suppressed`, `not_computable` and `unknown` require a null value with an explicit missing state.
- A complete coverage claim requires coverage fraction 1.0. Partial coverage needs a defined denominator. Unknown/not-applicable coverage carries a null fraction.
- Quantified uncertainty requires finite ordered bounds and a unit. Qualitative or not-assessed uncertainty must explain its basis rather than leave a blank.
- Every receipt remains `unreviewed_source_observation`. Structural acceptance does not establish source truth, causality, intervention impact, or indicator qualification.

Run the receipt validator with a source-bound JSON record:

```bash
python3 scripts/validate_regenerative_observation.py path/to/observation-record.json
# For a historical, retained catalog snapshot, pass --catalog /path/to/that-exact-catalog.json
python3 -m unittest discover -s tests -p 'test_regenerative_observation.py'
```

The test fixture is explicitly synthetic and is never registered as a real environmental or socioeconomic observation. This PR adds no real data values; source discovery, dataset licensing/retention, unit harmonization and quality review are a separate evidence-ingestion step.
