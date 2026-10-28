# Capability-Economy Comparative Benchmark (CECB) v0.1
**Status:** research specification; not an implemented simulator, qualified benchmark, or policy recommendation.
**Owner:** Mycelix / Civilization OS research
**Date:** 2026-10-28

## 1. Purpose

Test the central Capability Economy hypothesis from the Luminous Dynamics economic-design thesis:

> Under the same physical world, population needs, information limits, and resource envelope, can alternative combinations of ownership, financing, and allocation improve access to essential capabilities without unacceptable loss of productive capacity, resilience, agency, or rights?

The benchmark must be able to show that an alternative **fails**, that conventional arrangements **win under particular conditions**, or that the evidence cannot distinguish the candidates. It is not designed to certify a preferred ideology.

This specification adds an explicit *institutional comparison* experiment. It must compose existing Mycelix work rather than create a parallel economy simulator, accounting engine, allocator, identity system, or trust score.

## 2. Existing work to consume

- [CIV-ECON-001 — socioeconomic protocol profiles and interoperability kernel](https://github.com/Luminous-Dynamics/mycelix/issues/4804): regime/profile boundaries.
- [CIV-ECON-003 — stock-flow and intergenerational accounting](https://github.com/Luminous-Dynamics/mycelix/issues/4808): accounting closure and stocks.
- [CIV-ECON-005 — production, input-output and critical-supply network](https://github.com/Luminous-Dynamics/mycelix/issues/4814): physical dependencies and capacity.
- [CIV-ECON-006 — Civilization Twin](https://github.com/Luminous-Dynamics/mycelix/issues/4816): world state, institutional scenarios, counterfactual semantics, and multi-scale model boundaries.
- [FIN-SYS-004 — capital allocation to real-economy evidence](https://github.com/Luminous-Dynamics/mycelix/issues/3073): distinguish finance from productive outcomes.
- [FIN-SYS-007 — scarce-resource allocation profiles](https://github.com/Luminous-Dynamics/mycelix/issues/4574): price, queue, entitlement, lottery, need-priority and hybrid rules.
- [FIN-ECO-006 — Creditism protocol family](https://github.com/Luminous-Dynamics/mycelix/issues/4771): a named candidate profile, not a default or canonical currency.
- [CIV-IMPACT-001 — incremental value of composed decision systems](https://github.com/Luminous-Dynamics/mycelix/issues/5046): equal-budget evaluation, ablation, and fully loaded costs.
- [NRVCA-001 — real-value and capability accounts](https://github.com/Luminous-Dynamics/mycelix/issues/5374) and [NRVCA-003 — transfers, rents, resource losses and external damages](https://github.com/Luminous-Dynamics/mycelix/issues/5387): multidimensional outcome reporting and accounting boundaries.
- [MYC-FOOD-001 — neighborhood producer cooperative slice](https://github.com/Luminous-Dynamics/mycelix/issues/5001): a later operational vertical once the generic experiment contract is frozen.

**No duplicate authorities:** the Civilization Twin owns the world/simulation semantics; existing finance profiles own financial instruments and allocation rules; NRVCA owns the measurement bridge; the independent evaluator owns hidden truth and scoring. Mycelix records contracts, provenance, authority, and evidence. A recorded claim is not proof that the claim is true.

## 3. Paradigm shifts to evaluate

1. **From transactions to delivered capabilities.** Measure safe, adequate service actually delivered, not merely money moved, loans issued, votes cast, or projects registered.
2. **From a single bundled ideology to separable mechanisms.** Ownership, financing, allocation, service guarantees, and governance are distinct experimental factors. Where feasible, use factorial ablations so that a result can be attributed to a mechanism rather than a bundle.
3. **From credit availability to real-resource feasibility.** Credit can shift purchasing power and timing; it cannot by itself create land, housing, skilled labor, energy, materials, or construction capacity.
4. **From average outcomes to incidence and tails.** Report who gains, who loses, worst-decile outcomes, and whether vulnerable groups carry a disproportionate share of failure.
5. **From model-generated success to adversarial falsification.** Include scarcity, shocks, capture, gaming, stale information, external dependency, and regime-specific failure cases.
6. **From one winner to a conditional mechanism atlas.** The appropriate result may be: “regime A performs better when X holds; regime B performs better under Y; neither is robust under Z.”

These are hypotheses about how to evaluate economic institutions, not claims that any proposed model is already superior.

## 4. First benchmark family: housing access under scarcity

Start with a synthetic, finite-horizon regional housing economy. Housing is a suitable first family because it couples a fixed physical stock, construction and maintenance, household affordability, debt exposure, ownership, and access rules. It must remain synthetic until a separate data-governance and field-pilot plan is approved.

The first fixture should include:
- households with heterogeneous income, savings, household size, accessibility needs, and declared housing urgency;
- existing homes with location, quality, safety, occupancy and maintenance state;
- constrained labor, construction materials, energy, finance and construction lead time;
- rent, purchase, maintenance, vacancy, turnover, and new-build events;
- explicitly modeled financing, default, arrears, reserves and loss-bearing;
- external shocks such as an income drop, material-price spike, construction disruption, or household inflow;
- uncertainty about reported availability, home quality or maintenance needs;
- privacy boundaries separating publicly auditable aggregates from household-level personal information.

The oracle must know the synthetic ground truth; candidate policies must receive only their declared observation view.

### Candidate policy profiles

Implement each as a fully specified, versioned profile. Do not assume a label alone defines a complete economic system.

1. **Market/credit baseline:** price and willingness/ability to pay govern access; conventional debt and equity can finance construction; baseline legal, tax and service arrangements are explicit.
2. **Essential-service floor plus market:** a declared portion of safe housing capacity is made accessible under a public/social-service entitlement; remaining capacity uses market mechanisms. Funding and eligibility rules must be explicit.
3. **Cooperative / commons hybrid:** selected land or housing capacity is held by a cooperative or stewarded entity; access, maintenance, surplus and exit rules are explicit; private enterprise remains permitted elsewhere.
4. **Risk-sharing finance mix:** compare debt, cooperative equity, grants and revenue-/income-contingent claims, with clear loss-bearing and capital-recovery rules. Do not silently grant it free capital.
5. **Bounded mutual-credit profile:** set explicit participant eligibility, limits, reserves, settlement boundaries, default treatment and conversion assumptions. Internal credits are not external purchasing power.
6. **Configured non-price / hybrid allocation:** compare need-priority, queue, lottery, entitlement, market price and hybrid allocation using the same declared housing stock and applicant universe.

The baseline is not a straw man. It should use a strong, transparently configured policy and the same information, planning horizon, physical resources and permitted decision authority. Give each arm comparable implementation effort, computation and evaluation support. If one arm receives a subsidy or grant, account for the grant budget and opportunity cost rather than treating it as free.

For causal attribution, run separable comparisons for (a) financing, (b) ownership/stewardship, (c) access rule, and (d) service-floor policy where feasible. Bundle comparisons may follow, but each bundle must be named and its components frozen. A bundled result cannot establish which component caused the difference.

## 5. Shared-world and fairness contract

For a paired comparison, all candidate arms receive the same:
- initial world state and exogenous event schedule;
- household demand and capability-floor definitions;
- physical resource envelope, construction technology and production constraints;
- decision horizon, allowed actions and information-acquisition budget;
- data freshness, uncertainty, and observation-access rules;
- external funding envelope and its disclosed source, except where funding structure is the declared intervention;
- safety, rights, privacy and non-discrimination constraints;
- scoring code and pre-registered primary estimand.

The *mechanism being tested* may change how finance is structured, how claims are allocated, who owns an asset, and who bears risk. It must not change the hidden ground truth or receive privileged evidence.

Use common random numbers / paired seeds for the same scenario where the simulator supports deterministic replay. Report the scenario as the unit of generalization; repeated seeds alone are not independent real-world replications.

Every scheduled run receives a disposition, including invalid, timed out, failed, abstained, or non-converged runs. No silent removal of unfavourable outcomes.

## 6. Outcome vector — no universal civilization scalar

Keep these dimensions separate. Do not compensate a rights violation with a lower cost or a higher average.

### Essential access
- safe and adequate housed household-months;
- households continuously below the declared capability floor;
- person-days without safe housing and duration of unmet need;
- access by household size, accessibility need, location and declared priority group;
- time from eligible request to suitable housing;
- involuntary displacement, eviction or unplanned moves, where the model specifies them.

### Affordability and financial exposure
- housing cost / disposable income distribution, with components disclosed;
- debt-service / disposable-income distribution;
- share and count of households borrowing to meet essential housing costs;
- arrears, default, debt restructuring, fees and loss severity;
- debt stock, principal outstanding and effective financing cost;
- funding source and who bears downside in stress cases.

Do not infer household distress from a national debt ratio alone. Aggregate debt-service indicators are useful context but can hide household-level dispersion and do not establish why a household borrowed.

### Real productive capacity
- suitable housing units delivered and maintained;
- construction and maintenance capacity utilization;
- time to delivery and completion reliability;
- cost per safely housed household-month, including financing, administration, integration, verification, failures and recovery;
- resource and labor inputs per delivered service unit;
- stock depletion, deferred maintenance, vacancy, quality loss and construction backlog.

### Distribution, ownership and power
- ownership and surplus incidence by stakeholder class;
- concentration of asset ownership and allocation authority;
- access and service-quality inequality;
- related-party payments and unexplained transfers, where observable;
- number and materiality of veto points, verifier/coordinator concentration and switching/exit cost;
- appeal availability, resolution time, and false denial/exclusion rate.

### Resilience
- worst-decile unmet need and tail financial loss;
- days to recover after income, material, labor, supply or coordinator shock;
- service continuity and the number of households pushed below the capability floor;
- reliance on one lender, supplier, allocator or verifier;
- reserve sufficiency and maintenance continuity during a shock.

### Ecology and physical feasibility
- land, water, energy and material use in stated units;
- emissions or other physical externalities only where a declared evidence/method profile supports them;
- constraint violations and unmet maintenance;
- displaced impacts outside the region, where measured.

### Evidence and operational cost
- evidence coverage, freshness, missingness and provenance;
- disagreement between independent sources;
- incorrect availability claims and stale-stock errors;
- dispute/correction burden, verification cost and administrative hours;
- privacy or authority violations (hard failures, not compensable metrics).

Each result includes denominator, unit, time period, distribution, uncertainty, source/model profile, missingness, and claim ceiling. Never add unlike physical units into a pseudo-monetary total. Never count a transfer as net social benefit simply because its recipient gained.

## 7. Accounting and mechanism invariants

The independent checker must reject, at minimum, these invalid inferences:

- credit issued = productive capacity created;
- mortgage / existing-home purchase value = new construction;
- refinancing face value = net new finance to the real economy;
- asset-price appreciation = real production;
- a planned home = available, safe inventory;
- a recorded allocation = fulfilled service;
- a credential or attestation = ground truth;
- a local mutual-credit balance = foreign settlement capacity;
- a transfer = net resource loss or net resource gain without a specified counterfactual;
- average access = fair distribution;
- zero observed failures = zero risk;
- synthetic pass = field validation.

Every financial asset/liability pair must close under the selected accounting profile. Physical stocks cannot increase without a modeled construction/repair/production event and its inputs. Unknown cost, missing evidence and unknown availability must remain unknown rather than silently becoming zero or available.

## 8. Scenario and adversarial corpus

Begin with at least 100 preregistered scenario configurations and 30 paired seeds per configuration for the synthetic research phase. These are engineering targets for the corpus, not evidence of real-world external validity. If the scenario generator cannot support this without changing semantics, shrink the initial corpus transparently and version the protocol.

Cross the following families:

- **Supply:** adequate supply; chronic shortage; construction bottleneck; maintenance backlog; geography/access mismatch.
- **Finance:** stable income; income shock; interest-rate shock; refinancing; grant withdrawal; risk reserve exhaustion; mutual-credit counterparty failure.
- **Demand:** household inflow; demographic change; urgent need surge; heterogeneous accessibility requirements.
- **Information:** complete; stale; missing; conflicting; correlated mirrored reports; strategic false availability; verification delay.
- **Governance:** transparent; low participation; elite capture; conflicted allocator; minority exclusion; delayed appeal; coordinator outage.
- **Market structure:** competitive supply; locally concentrated ownership; monopoly/monopsony; speculative withholding, modelled only with explicit behavioral assumptions.
- **External dependency:** stable imports; supply interruption; currency/settlement shock; externally controlled key input.
- **Time horizon:** operational; multi-year capital/maintenance; transition between regimes. Short runs must not silently stand in for long-run dynamics.

Required negative controls:
1. Debt finance enables a productive, demand-backed construction project and outperforms a cash-constrained alternative.
2. A service-floor policy with insufficient physical housing cannot satisfy all demand.
3. Lower prices that induce demand beyond fixed capacity do not create homes.
4. A non-price queue is gamed by strategic duplicate claims.
5. A cooperative governance process is captured or under-participated.
6. A mutual-credit network improves local liquidity during a shock but cannot settle an external input bill without an external settlement mechanism.
7. Unverified inventory appears available but is not fit for occupancy.
8. A grant shifts costs to the public budget rather than eliminating them.
9. Deferred maintenance creates attractive short-run outcomes but worsens later capacity.
10. A very conservative policy abstains or waits so often that its end-to-end access results deteriorate.

These cases prevent “capability first” from becoming a rule that the desired profile must win every scenario.

## 9. Statistical and qualification protocol

### Evaluation architecture: adapt TEVV, do not invent a new measurement bureaucracy

NIST's 2026 draft TEVV-Athlon framework structures evaluation around assessment objectives, Events and Tools, and Blocks tied to measurement concepts. CECB should borrow this modularity for the software/policy evaluation layer: each scenario event should state which instrument/tool observes it, which metric it informs, and which limitations attach to the observation. This framework is a current draft, not a completed regulatory requirement or proof of the benchmark's validity. Source: https://www.nist.gov/artificial-intelligence/ai-research/tevv-athlon-framework-evaluating-ai-systems

For the institutional/field layer, the OECD's 2026 locally led monitoring, evaluation and learning guidance emphasizes local actors helping define success, evaluation questions and interpretation. A Johannesburg pilot should therefore have affected residents, local operators and domain practitioners co-define relevant outcomes and review unintended effects; this is a governance requirement for the proposed study, not something software can replace. Source: https://www.oecd.org/en/publications/practical-guidelines-for-supporting-locally-led-development_eaecf72b-en/full-report/action-area-locally-led-monitoring-evaluation-and-learning_8a830675.html

### Before confirmatory scoring
Freeze and publish:
- scenario manifest and generator version;
- profile, simulator, policy and scoring-code identities;
- input and artifact digests;
- primary estimand for each domain;
- metric direction, units, exclusions and missing-data rules;
- baselines and policy tuning budget;
- seed schedule, holdout partition and uncertainty method;
- practical-effect threshold and non-compensable guardrails;
- protocol version and any departures from preregistration.

Do not use the scored holdout to tune a policy or rewrite its threshold.

### Separate evidence levels
- **E0 — contract correctness:** schemas, accounting identities, state transitions and negative controls.
- **E1 — synthetic mechanism behavior:** deterministic known-truth scenarios and sensitivity analysis.
- **E2 — historical replay:** source-pinned data with revision and coverage controls.
- **E3 — shadow operation:** predictions or proposals are compared with real outcomes without changing allocation or taking external action.
- **E4 — prospective controlled / stepped-wedge pilot:** an ethics, legal, privacy and stakeholder-approved study with a defined counterfactual.
- **E5 — independent replication:** separately sourced setting and evaluator.

Passing E0 does not imply E1; E1 does not imply E2 or real-world impact. Do not promote benchmark status while required checks are queued or unavailable.

### Success and falsification
There is no universal target such as “the capability economy wins by 10%.” For each real use case, affected stakeholders and evaluators must choose a primary outcome and minimum practical effect **before confirmatory outcomes are exposed**.

### Model adequacy is not a single pass/fail

Recent work on agent-based model validation emphasizes that adequacy depends on the question the model is meant to answer; calibration, verification, empirical validation, sensitivity analysis, and policy counterfactuals have different evidentiary roles. CECB should keep the following claims separate:

1. **Implementation verification:** does the code implement the declared equations, accounting identities, event ordering and constraints?
2. **Mechanism validation:** does a small known-truth example reproduce the declared mechanism, with positive, negative and adversarial controls?
3. **Calibration:** which parameters were fitted to which observed moments, microdata or stylized facts, with what identifiability limits?
4. **Untargeted validation:** does the calibrated model reproduce distributional/temporal regularities that were not used to tune it?
5. **Out-of-sample prediction:** does it beat simple, named baselines on temporally or geographically held-out data?
6. **Historical retrodiction:** can it recover outcomes from a known prior policy or shock when given only the information that would have been available at the time?
7. **Policy counterfactual adequacy:** are the mechanisms governing behavior, capacity, adaptation and institutional response supported well enough for this specific proposed counterfactual?

These are not interchangeable. A model may reproduce aggregate values while encoding the wrong mechanism; multiple parameterizations can produce similar outcomes (equifinality). A model can pass its own fixture while remaining empirically wrong. A calibrated fact must not be re-used as an independent holdout. Report uncertainty in data, parameters, structure, scenarios, and scoring, plus sensitivity to plausible alternative mechanisms. If a counterfactual rests on unvalidated adaptation assumptions, label it exploratory rather than causal evidence.

References for this discipline:
- Grimm et al., “How to keep it adequate: A protocol for ensuring validity in agent-based simulation,” *Environmental Modelling & Software* 159 (2023), 105559: https://doi.org/10.1016/J.ENVSOFT.2022.105559
- “Agent-Based modeling in financial markets: Modeling frameworks, validation challenges, and emerging applications” (2026): https://doi.org/10.3934/nhm.2026043

A candidate is not eligible to be called superior unless it:
1. passes the declared accounting, physical-feasibility, authority, privacy and non-discrimination guardrails;
2. improves the preregistered primary outcome by a practically meaningful margin against the strongest fair baseline, with its registered uncertainty interval excluding no improvement;
3. reports every secondary metric, group-wise outcome, failure and tail result, even when unfavourable;
4. does not obtain the result by moving cost, risk, resource use or harm outside the accounting boundary;
5. replicates in a separately sourced setting before any broad generalization.

If arms trade off outcomes, publish the Pareto frontier and the value choices required to select a policy. Do not bury those choices in an optimizer.

## 10. State-of-the-art comparison: what counts as a fair baseline?

There is no single software product or economic model that is “the SOTA economy.” Compare each layer with the strongest appropriate reference:

- **Economic measurement:** reconcile against the 2025 UN System of National Accounts where applicable; reuse UN SEEA environmental-economic accounting, World Bank wealth accounting, and OECD well-being indicator frameworks rather than relabeling them as new. NRVCA's potential contribution is the evidence-linked bridge and explicit multi-dimensional evaluation, not invention of measurement beyond GDP.
- **Debt exposure:** use the BIS debt-service-ratio methodology and the relevant national statistical authority's series as aggregate context; pilot results require disaggregated household/project measures with consent and privacy controls.
- **Allocation:** compare market/price, queue, entitlement, lottery, need-priority and hybrid mechanisms under equal capacity and explicit objective/authority profiles, as already required by FIN-SYS-007.
- **Mutual credit:** test both stable and turbulent conditions. A 2025 firm-level study found a nuanced resilience effect for member firms; do not cite it as proof that mutual credit universally dominates conventional credit.
- **Governance tooling:** compare the governance task to operational tools such as Loomio, an open-source decision platform built by a worker-owned cooperative. Loomio is a comparator for deliberation and decision records, not an economic regime or a substitute for field outcome evaluation.
- **End-to-end coordination:** compose the best existing domain-specific tools and a strong human/rule-based baseline. Mycelix must show incremental value beyond adding cost and system complexity.

To claim a Mycelix advantage, identify the exact baseline, benchmark task, inputs, versions, budget and uncertainty. “Better than industry” is not a testable claim without specifying what function, data and cost are being compared.

## 11. South Africa / Johannesburg evidence boundary

Use South Africa as a potential pilot context, not as a source of invented thresholds.

Relevant published context:
- The South African Reserve Bank's September 2026 Quarterly Bulletin reports household debt at 61.3% of nominal disposable income for Q2 2026. Source: https://www.reservebank.co.za/en/home/publications/publication-detail-pages/quarterly-bulletins/quarterly-bulletin-publications/2026/september
- The IMF's 2026 South Africa Article IV consultation reports household debt service of 8.5% of disposable income for Q3 2025 in its cited table. Source: https://www.elibrary.imf.org/view/journals/002/2026/034/article-A001-en.xml
- These figures use different periods and are aggregate indicators. They do not establish the distribution of hardship, whether a loan funded an essential need, or a causal case for any proposed institutional design.

A live pilot must define the geography, service, population, consent/authority model, lawful data access, baseline period and comparator with local operators and affected participants. Start with one bounded cooperative or service workflow; do not begin by replacing credit, housing policy, or public service eligibility. Do not ingest sensitive household financial or identity data simply because it is technically obtainable.

## 12. Execution sequence

1. **Freeze the semantic crosswalk:** map each benchmark field to the owning issue/schema above; mark every unresolved field as a gap rather than adding a parallel model.
2. **Create a minimal known-truth housing fixture:** small enough for an independent analytical oracle and manually inspectable.
3. **Implement the independent scorer and negative controls:** candidate code must not read hidden truth; every scheduled case receives a disposition.
4. **Run baseline/ablation analysis:** vary one mechanism at a time before comparing bundled profiles.
5. **Publish sensitivity and Pareto reports:** include counterexamples where each regime wins or fails.
6. **Validate measurement against official accounts and real sourced definitions:** preserve dates, geography, units, revisions, definitions and unclassified residuals.
7. **Only after E0/E1 qualify:** design a locally governed shadow trial, including privacy/legal review, independent evaluation, exit and appeals.
8. **Expand domains only when warranted:** use the same contract for energy, food or transport only after confirming which fields and assumptions transfer and which require a new profile.

## 13. Explicit non-goals and claim ceiling

This specification does not:
- assert that all debt is harmful or that eliminating debt is feasible or desirable;
- assert that cooperative, public, commons, market, mutual-credit or Creditism-style arrangements always win;
- define a universal reputation or “deservingness” score;
- delegate legal entitlement, service denial or final political authority to an AI or ledger;
- create a new currency/token, financing engine, simulator or universal welfare scalar;
- use synthetic outcomes as evidence of real-world impact;
- substitute for policy deliberation, local law, engineering constraints or participant consent.

The intended result is a reusable experiment that can reveal **which institutional mechanisms work, for whom, under what constraints, at what cost, and with what failure boundary**.

## References

1. United Nations, System of National Accounts 2025: https://unstats.un.org/unsd/econstats/national-accounts/sna2025
2. United Nations, System of Environmental-Economic Accounting: https://seea.un.org/
3. OECD, Practical Guidelines for Supporting Locally Led Development — locally led monitoring, evaluation and learning (2026): https://www.oecd.org/en/publications/practical-guidelines-for-supporting-locally-led-development_eaecf72b-en/full-report/action-area-locally-led-monitoring-evaluation-and-learning_8a830675.html
4. BIS, Debt service ratios: https://data.bis.org/topics/DSR
5. Reyns, Hudon & Schoors, “For better or for worse: How Mutual Credit Systems bolster resilience of small firms,” Journal of Economic Behavior & Organization 230 (2025), DOI: https://doi.org/10.1016/j.jebo.2024.106861
6. Loomio Cooperative, about and open-source governance product: https://www.loomio.com/about/
7. South African Reserve Bank, Quarterly Bulletin, September 2026: https://www.reservebank.co.za/en/home/publications/publication-detail-pages/quarterly-bulletins/quarterly-bulletin-publications/2026/september
8. IMF, South Africa: 2025 Article IV Consultation, published 2026: https://www.elibrary.imf.org/view/journals/002/2026/034/article-A001-en.xml
9. NIST, The TEVV-Athlon Framework for Evaluating AI Systems (draft, August 2026): https://www.nist.gov/artificial-intelligence/ai-research/tevv-athlon-framework-evaluating-ai-systems
10. Grimm et al., How to keep it adequate: A protocol for ensuring validity in agent-based simulation, *Environmental Modelling & Software* 159 (2023), 105559: https://doi.org/10.1016/J.ENVSOFT.2022.105559
11. Agent-Based modeling in financial markets: Modeling frameworks, validation challenges, and emerging applications (2026): https://doi.org/10.3934/nhm.2026043

External research supports the selection of reference frameworks and the need for context-dependent tests; it does not validate the proposed Capability Economy architecture.
