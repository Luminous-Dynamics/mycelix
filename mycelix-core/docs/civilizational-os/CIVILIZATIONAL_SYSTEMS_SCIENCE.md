# Civilizational Systems Science (CSS)

**Status:** Research programme proposal. This document is a specification, not a claim that the described system is implemented.  
**Version:** 0.1.0  
**Date:** 2026-11-02

## 1. Purpose

Civilizational Systems Science studies the emergence, evolution, coordination, transformation, resilience, and possible futures of intelligent societies as coupled, open systems.

Its central question is:

> Under what physical, ecological, biological, informational, economic, and institutional conditions can intelligent societies persist, adapt, cooperate, and flourish without exhausting the systems on which they depend?

CSS is not a ranking of cultures, a universal stage theory, or a claim that technological power is equivalent to wisdom. It does not presume that centralized states, cities, writing, markets, industrialization, or spaceflight are necessary features of every civilization.

The programme combines empirical historical research, current observations, mechanistic models, controlled simulation, and carefully bounded astrobiology hypotheses. These evidence classes must remain distinguishable in storage, analysis, visualization, and public communication.

## 2. Research questions

1. **Emergence:** Which combinations of environmental opportunity, cooperation, communication, cumulative learning, and institutions enable complex societies to emerge?
2. **Development:** How do knowledge systems, technology, infrastructure, ecological relations, and institutions co-evolve?
3. **Resilience:** Which mechanisms reduce cascading failures, and how are costs and harms distributed?
4. **Transformation:** Under what conditions do societies reorganize, decentralize, centralize, fragment, merge, recover, stagnate, or disappear?
5. **Flourishing:** Which arrangements protect autonomy, wellbeing, dignity, pluralism, and ecological integrity?
6. **Planetary limits:** What energy, material, land, water, climate, and biosphere constraints bound particular development paths?
7. **Extraterrestrial possibilities:** Which pathways from chemistry to life, complex cognition, technology, and detectable activity are physically plausible, and what observations could discriminate among them?
8. **Inter-civilizational relations:** How might distinct societies communicate, trade, negotiate, compete, coexist, or fail to understand one another?

These questions need not have one universal answer. Models may be valid for a bounded context and invalid elsewhere.

## 3. The core paradigm: an adaptive, open, multi-scale system

Represent a society as a time-varying, multiplex system rather than a single entity with one development score.

The main interacting domains are:

- **Physical substrate:** energy, materials, water, land, atmosphere, climate, geology, infrastructure, and transport.
- **Living systems:** organisms, populations, health, food webs, biodiversity, reproduction, and ecological dependencies.
- **Cognition and culture:** perception, communication, learning, social learning, norms, cultural transmission, language, memory, education, and innovation.
- **Institutions and coordination:** governance, law, property, dispute resolution, collective choice, public goods, trust, rights, and accountability.
- **Economy and technology:** production, exchange, finance, labour, knowledge, capital, industrial capability, and technological diffusion.
- **External relations:** migration, trade, conflict, diplomacy, treaties, interdependence, and off-world settlement where physically possible.

These are analytical layers, not a universal developmental ladder. Each model must declare its unit of analysis, temporal and spatial resolution, operating context, and excluded mechanisms.

### 3.1 Separate descriptive, explanatory, and predictive work

Every study must state whether it is:
- **Descriptive:** what does the evidence say happened or is happening?
- **Explanatory:** which mechanisms plausibly generated the observed pattern?
- **Predictive or scenario-based:** what may happen under specified conditions?

Predictive skill does not establish causal explanation. A plausible causal story does not establish forecasting skill. A scenario is not a forecast. Policy counterfactuals require explicit assumptions and a defensible identification strategy.

## 4. Epistemic contract and shared data model

Mycelix may provide a distributed record and provenance layer, but network consensus, contributor reputation, number of attestations, and model confidence must never be treated as substitutes for empirical truth.

### 4.1 Required record types

Distinguish at minimum:
- **Entity:** person, community, polity, institution, species, ecosystem, technology, planet, habitat, or other identified object.
- **Observation:** measurement or documented observation with method, source, units, spatial scope, and observation time.
- **Historical source:** archaeological object, inscription, archive, oral history, ethnography, text, or other source with date, provenance, authorship, translation, and interpretive context where known.
- **Claim:** proposition about an entity or event, linked to supporting, contradicting, and qualifying evidence.
- **Indicator:** transformation of observations or claims with documented formula, denominator, units, and limitations.
- **Model:** formal or executable representation with version, parameters, assumptions, boundary conditions, and intended use.
- **Model run:** result tied to model commit, dataset snapshots, configuration, random seed, environment, and output digest.
- **Scenario:** explicitly hypothetical assumptions and boundary conditions.
- **Interpretation:** documented synthesis or contested interpretation that does not silently become a primary observation.

### 4.2 Evidence classes

Use explicit classifications:
1. direct observation;
2. archaeological measurement;
3. primary historical record;
4. ethnographic or oral source;
5. secondary synthesis;
6. derived indicator;
7. model inference;
8. synthetic ground truth;
9. scenario assumption;
10. speculative hypothesis.

An item may cite multiple evidence records, but its own type remains explicit. Synthetic data must never be presented as historical evidence. A model inference must not be promoted to an observation because another model repeats it.

### 4.3 Provenance and time

Every sourced observation or claim must carry, where available:
- stable record ID and source URI;
- publisher, author, institution, and source type;
- source publication/version date and retrieval date;
- immutable snapshot digest (SHA-256 or equivalent);
- observation or event time, represented as an interval when uncertain;
- geographic scope, coordinate reference system, and spatial uncertainty;
- units, transformations, denominator, and missing-data treatment;
- extraction code/version and lineage to derived indicators;
- known coverage, selection bias, conflicts of interest, and limitations.

Do not collapse observed time, recorded time, published time, and retrieved time. Historical dates should support ranges, uncertain calendars, alternative chronologies, and competing interpretations.

### 4.4 Confidence is not one magic scalar

Keep separate dimensions for measurement uncertainty, source directness, source independence, temporal fit, spatial fit, sampling coverage, interpretive disagreement, and model uncertainty. Correlated sources count as correlated evidence, not independent corroboration.

Reputation or community attestations may help manage spam, provenance, review workflows, and accountability. They must not determine whether an archaeological interpretation, scientific measurement, or historical statement is true. Dissent and unresolved disagreement must remain queryable.

### 4.5 Empirical and normative records remain separate

Empirical records describe measurements, sources, and inferences. Normative records describe values, rights, goals, principles, or community decisions. Agreement with a norm does not make an empirical statement true; an empirical measurement does not by itself determine a moral or political decision.

CSS must not produce a single civilization-worth score. Dashboards should preserve disaggregated measures and show distributions, trade-offs, uncertainty, and whose interests a metric represents.

## 5. Shared architecture and project boundaries

### Mycelix — provenance, memory, and accountable coordination
- Preserve source-linked claims, observations, interpretations, model manifests, and revision histories.
- Maintain typed support, contradiction, qualification, and derivation relationships.
- Support plural, federated stewardship and access controls.
- Record consent, restrictions, attribution, and community governance for sensitive and Indigenous knowledge.
- Provide auditable workflows for review, corrections, and dissent.
- Keep forecasts and scenarios separate from accepted observations.

Mycelix does not become an oracle that decides historical truth or speaks for humanity.

### Symthaea — cognition, learning, and collective intelligence
- Model bounded mechanisms: learning, memory, social learning, communication, coordination, belief revision, and cultural transmission.
- Test individual and group mechanisms against behavioural or historical data.
- Compare cognitive agents with simple rule-based, statistical, and non-cognitive baselines.
- Report construct validity and limits. HDC similarity, integrated-information proxies, coherence, or other internal metrics are not by themselves evidence of consciousness, moral worth, or legitimate decision authority.

No consciousness score may determine rights, political authority, or human value.

### Symtropy — controlled, physically constrained simulation
- Provide deterministic or reproducibly seeded multi-agent and system-dynamics experiments.
- Represent populations, institutions, resource networks, technologies, infrastructure, and ecological feedbacks at appropriate scales.
- Enforce unit consistency and stock-flow constraints for energy, water, materials, and populations.
- Support replaceable mechanisms so competing theories can be tested.
- Separate simulation models from renderers and UIs.

Visual plausibility is not validation.

### Sol-Atlas — spatial, planetary, and astronomical context
- Represent geospatial observations and infrastructure layers with units, dates, provenance, and uncertainty.
- Combine energy, water, food, transport, materials, climate, land, biodiversity, and population layers where reliable data exist.
- Distinguish observed infrastructure from planned, hypothetical, or simulated infrastructure.
- Support Earth, lunar, Martian, ocean-world, and other planetary contexts only with relevant data and physics.

Static illustrative data or scenario milestones must not appear as observed infrastructure or certain future events.

### Shared exchange contract

Projects exchange versioned records and manifests, not unqualified numbers. Each model result must be traceable to its model ID and commit, dataset snapshot IDs, configuration digest, seeds, time horizon, spatial scope, output digest, and uncertainty summary. The shared API must be versioned before production implementation.

### Existing implementation contracts to compose

CSS is a programme charter and domain map, not a second generic evidence, scenario, or simulation-run protocol. The implementation should build on existing work:

- Mycelix issue #5451 defines the shared evidence plane; #3022 defines canonical statement identity and provenance-family semantics; #3046 and #4778 cover temporal uncertainty/materialization concerns. CSS historical-source adapters should extend these contracts for archaeological context, oral-history transmission, inscriptions, translations, contested chronology, and community stewardship.
- Symtropy PR #369 defines civilizational authority boundaries; #378 defines causal resource/inventory/shipment semantics; #391 defines a deterministic Three Worlds integration lab; #399 defines astrobiology authority and uncertainty. These are open draft PRs, not qualified releases.
- Symtropy PR #1571 defines the Humanity Atlas scenario/run envelope and model-credibility/qualification process. The parent repository PR #3760 exports the existing multiworld simulator's model-qualified technology tree; Sol-Atlas PR #79 consumes the run artifact. CivBench should supply scientific task protocols and scoring on top of those contracts, not duplicate their schemas.
- The existing multiworld simulator is already present in the Luminous Dynamics parent repository at `mycelix-multiworld-sim`. CSS should audit and improve it rather than creating a parallel general-purpose simulator.
- Sol-Atlas issues #17 and #19 already define its Humanity/AI Bootstrap Atlas direction and capability vertical slice. Historical/current civilizational map layers should reuse those semantics and the Humanity Atlas consumer.
- Symthaea issues #5774/#6287, #6688, #7046, #7188, #7193, #7201, #7205, and #8252 already cover capability closure, epistemic humility, institutional evolution, socioeconomic accounting, Goodhart failure, simulation-to-authority boundaries, and cross-tradition transfer. Social-learning experiments should be bounded additions to that work.

All referenced PRs are open drafts at the time of this document's update. Their descriptions distinguish authored tests from exact-head execution and real-world validation. A design document, source file, or queued workflow must not be represented as an implemented/validated capability.

### Existing null result that should shape CSS

The multiworld simulator's PHASE2_FINDINGS.md (2026-04-17) records a null result for a governance-defence intervention: the report states that attacks changed governance-internal variables but did not propagate into the five inputs used by the Civilization Viability Score. The reported treatment effect was indistinguishable from zero under both arithmetic and geometric aggregation.

This should motivate a CSS requirement: every causal or policy claim must identify the state-transition path from intervention to independently meaningful outcomes. Composite metrics alone are insufficient. A metric's lack of response may reveal a missing causal mechanism or an invalid outcome measure; it must not be "fixed" by arbitrarily coupling variables after observing the result. Report null and negative results.

The current multiworld historical-validation module also has a concrete scoring concern: a zero/near-zero observed value is assigned zero absolute percentage error, and metric-specific MAPE values for heterogeneous variables are averaged into one overall score. Parent repository issue #3816 tracks the correction. CSS must not regard the current comparison implementation as evidence of calibration without the source, metric, holdout, and baseline audit.

## 6. CivBench: evaluate before making claims

CivBench is the common benchmark programme. It must be independently runnable, versioned, and designed around falsifiable tasks.

### Track A — Evidence integrity
Test source attribution, temporal validity, entity resolution, provenance preservation, contradiction handling, and uncertainty representation.

**Gate:** 100% of published factual records and model outputs carry required provenance or an explicit machine-readable reason for absence. No silent promotion of inference or scenario to observation.

### Track B — Historical reconstruction and forecasting
Use historical forecast origins. At each origin, expose only evidence available then. Evaluate held-out periods and societies.

**Baselines:** persistence, historical mean, linear trend, simple regression, and appropriate time-series methods.

**Metrics:** Brier and log scores for categorical probabilities; CRPS or log score for continuous distributions; calibration and interval coverage; skill relative to the strongest baseline.

### Track C — Mechanism and intervention tests
Test whether a stated mechanism predicts observed changes under defensible intervention designs. Use placebo interventions, negative controls, alternative specifications, and sensitivity analysis.

A model cannot claim causal superiority from predictive accuracy alone. Identify observational assumptions and unmeasured confounding.

### Track D — Crisis resilience
Test drought, epidemic, conflict, ecological degradation, supply interruption, infrastructure failure, energy shortage, and information disruption.

Report time-to-impact, service loss, recovery time, cascading failures, distribution of harms, adaptation, and ecological consequences. Do not reduce resilience to survival of a central authority or aggregate output.

### Track E — Physical and ecological closure
Test dimensional consistency and conservation for energy, water, material stocks, food flows, population transitions, and infrastructure capacities.

**Gate:** No unexplained conservation violations beyond declared numerical tolerances. Every parameter has a unit, source or explicit assumption, uncertainty range, and applicable regime.

### Track F — Coordination and governance
Test public-goods provision, bargaining, dispute resolution, commons governance, coalition formation, emergency allocation, and treaty compliance under heterogeneous preferences, unequal power, limited information, and adversarial agents.

Report efficiency alongside distribution, rights violations, participation, autonomy, recourse, privacy, exit, and power concentration. No single aggregate welfare metric is sufficient.

### Track G — Technology and cultural transmission
Test learning curves, diffusion, adoption, institutional memory, skill transfer, innovation, dependency formation, and knowledge loss under migration, disaster, and intergenerational gaps.

### Track H — Astrobiology and technosignature observability
Begin with planetary habitability, chemical and energy constraints, biosignature ambiguity, technosignature selection effects, and instrument sensitivity. Test detection pipelines with injected signals, natural confounders, radio-frequency interference, and independent blind verification.

No benchmark may assert that a hypothetical extraterrestrial society exists. Separate life detection, intelligence hypotheses, technology hypotheses, and actual technosignature candidates.

## 7. Model validity and release rules

Every model publication or scenario release includes:
1. intended question, decision context, and excluded uses;
2. conceptual model and mechanism diagram;
3. data inventory, time cutoffs, licences, and provenance;
4. parameter sources, calibration process, identifiability, and uncertainty;
5. verification tests and conservation checks;
6. empirical validation and pattern-oriented comparisons;
7. strong simple baselines and ablations;
8. negative controls, sensitivity analysis, and failure cases;
9. exact commit, dependency lock, seed, configuration digest, and reproduction instructions;
10. a claim table separating observations, inferences, and scenario assumptions.

Validation is a continuous argument about fitness for a particular purpose, not a one-time checkbox. Complex agent models must justify each added mechanism against simpler alternatives.

## 8. Historical and contemporary starting corpus

Start with a diverse, deliberately incomplete corpus:
- Seshat Global History Databank, including Polaris-2026 and replication datasets;
- D-PLACE cultural, linguistic, environmental, and geographic data;
- World Values Survey time series, documented as repeated cross-sectional samples rather than a person-level panel;
- Our World in Data chart datasets and metadata;
- UN Sustainable Development Goal indicator data and metadata;
- Global Carbon Budget datasets;
- archaeological and documentary sources for Mapungubwe, Great Zimbabwe, the Indus Valley, Mesopotamia, Maya societies, imperial China, Andean societies, and other regions;
- oral histories and Indigenous knowledge only under appropriate consent, attribution, access, and stewardship rules.

This is not a representative or complete sample of humanity. Dataset inclusion must include coverage and selection-bias notes.

Useful endpoints:
- https://seshatdatabank.info/data
- https://d-place.org/source
- https://www.worldvaluessurvey.org/AJDocumentationSmpl.jsp?CndWAVE=-1&INID=&SAID=-1
- https://docs.owid.io/projects/etl/api/chart-api/
- https://unstats.un.org/sdgapi/swagger/
- https://globalcarbonbudget.org/datahub/the-latest-gcb-data-2025/

## 9. Extraterrestrial branch: bounded inference

Earth is our one known example of a technological civilization. That is insufficient to estimate a universal distribution of civilizational forms or validate a general theory of technological emergence.

Separate:
- **Observed astronomy:** stellar, planetary, atmospheric, and geophysical measurements.
- **Habitability inference:** model-dependent estimates with explicit priors and selection effects.
- **Biosignature inference:** possible biological explanations, abiotic false positives, and contextual evidence.
- **Intelligence hypotheses:** possible evolutionary mechanisms, clearly marked as hypotheses.
- **Technology hypotheses:** physical feasibility, energetic requirements, and possible observables.
- **Technosignature candidates:** instrument-level data, terrestrial-interference checks, independent follow-up, and reproducible analysis.

Synthetic worlds test model logic; they do not establish extraterrestrial frequencies. Never fit a human civilizational curve and call it a universal evolutionary law.

NASA reports no credible evidence of extraterrestrial life. The NASA Exoplanet Archive is an observational resource, not evidence that any particular planet hosts life. Follow current SETI verification and post-detection guidance, including the IAA update dated 1 June 2026. A decision to reply to a confirmed contact is outside Mycelix's unilateral authority.

Sources:
- https://science.nasa.gov/uap/faqs/
- https://exoplanetarchive.ipac.caltech.edu/
- https://www.seti.org/research/seti-101/protocols-for-an-eti-signal-detection/
- https://iaa-seti-committee.github.io/

## 10. Ethics and governance

- Do not rank cultures, peoples, or civilizations by a single score.
- Do not infer moral worth, consciousness, rights, or legitimate authority from computational metrics.
- Protect personal data and sensitive knowledge; public reproducibility does not justify violating consent or sovereignty.
- Represent disagreements and minority histories rather than collapsing them into a dominant narrative.
- Identify distributional impacts and who bears the risks of policy proposals.
- Keep scenario exploration separate from real-world decision authority.
- Require plural review and transparent accountability for high-impact uses.
- Mycelix can support deliberation and records; it cannot claim to represent humanity or any unconsenting community.

## 11. Initial delivery sequence

**Phase 0 — Research contract**
- Approve the evidence ontology and shared record envelope.
- Inventory actual implemented components in all four repositories.
- Identify licences, source datasets, and reproducible acquisition routes.

**Phase 1 — Evidence-first atlas**
- Import a small, diverse subset of Seshat, D-PLACE, WVS, OWID, and UN data.
- Preserve source snapshots, date semantics, uncertainty, and lineage.
- Display provenance and missingness.

**Phase 2 — One historical mechanism benchmark**
- Select a bounded case with adequate time-series and archaeological evidence.
- Register forecast origin, outcome, baselines, metrics, and holdout before modelling.
- Compare simple statistics, system dynamics, and agent-based alternatives.

**Phase 3 — Physically constrained simulation**
- Implement a small population-food-water-energy-institution model.
- Add conservation tests, parameter sweeps, uncertainty propagation, and deterministic replay.
- Add cognitive and cultural mechanisms only as independently testable modules.

**Phase 4 — Contemporary planetary model**
- Link spatially scoped observations and indicators for a limited region.
- Keep observation, plan, and simulated scenario layers distinct.

**Phase 5 — Coordination experiments**
- Evaluate selected Mycelix governance and resource-sharing mechanisms in controlled, adversarial simulations.
- Report welfare distributions, power concentration, and rights-related failure modes.

**Phase 6 — Extraterrestrial observability**
- Test planetary-context and technosignature-detection models against injected synthetic signals and known interference.
- Publish uncertainty and selection functions, not unsupported civilization counts.

## 12. Definition of success

CSS succeeds when independent researchers can inspect the evidence, reproduce runs, challenge assumptions, compare mechanisms, and understand where a result does and does not apply.

Judge the programme by explanatory discipline, predictive calibration where prediction is possible, physical consistency, transfer tests, reproducibility, transparency, and usefulness to affected communities—not by agent counts, visual realism, civilization rankings, or claims of consciousness.

**The objective is not to predict one inevitable future. It is to understand the constraints and choices that make better futures possible.**
