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

1. **Model map** — groups models by monetary systems, markets/welfare, finance/debt, commons/governance, resource/energy, and civilization-scale dynamics. Each card carries an evidence state: `Implemented`, `Paper-reported`, `Design-only`, `Conceptual`, or `Discovery required`.
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
