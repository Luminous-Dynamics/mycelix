# National Real-Value and Capability Accounts (NRVCA) v0.1

Status: research draft; synthetic fixture only. This is a proposed complementary accounting method, not an official statistical standard and not a replacement for GDP, the System of National Accounts, SEEA, or national statistical authorities.

## Thesis

Measure a nation's economy as a reconciled system of **monetary production, physical stocks and flows, delivered services, distribution, and resilience**. Keep the coordinates separate where they have different units, valuation assumptions, populations, or normative meanings. The first artifact is a traceable report contract—not a grand unified prosperity score.

The intended innovation is operational integration: every important reported value is linked to its definition, period, unit, source, evidence status, coverage, and uncertainty; conventional accounts reconcile to their own identities; derived physical/service views do not silently become official output; and intervention claims are tested against declared counterfactuals.

## Existing state of the art to preserve

NRVCA must compose rather than rebrand existing work:

- **UN System of National Accounts 2025 (SNA 2025):** the reference framework for production, income, expenditure, financial accounts, balance sheets, and international comparability.
- **UN SEEA Central Framework and SEEA Ecosystem Accounting:** environmental flows, physical supply/use, environmental assets, ecosystem extent/condition/services, and links to economic activity.
- **World Bank Changing Wealth of Nations 2024 (CWON):** produced, human, natural, and foreign-asset wealth; an important complementary stock view.
- **OECD Beyond GDP / well-being work:** distribution and well-being dashboards rather than assuming one statistic captures general welfare.
- **Official supply-use/input-output tables, satellite accounts, and macroeconomic nowcasting:** essential baselines for production chains, non-market extensions, and timely estimates.

The specific gap to test is whether a shared evidence and reconciliation contract can link these views to spatially resolved service capabilities and intervention outcomes in a more reproducible and decision-useful way. NRVCA is not novel merely because it includes nature, well-being, or geography; these already have established frameworks.

## Paradigm shifts

1. **One headline number → a non-collapsed outcome vector.** Report GDP/NDP, wealth and asset changes, resource flows, essential service, distribution, resilience, and evidence quality separately. A future composite requires a declared normative profile and sensitivity analysis.
2. **Money spent → useful function delivered.** Distinguish expenditure, installed capacity, available capacity, service actually delivered, quality, reliability, and access. These are different states.
3. **Transactions → bound stock-flow and physical conservation.** Reconcile financial counterparties and balance-sheet changes; reconcile physical flows in consistent units. Each identity term must resolve to the canonical report value it claims to represent, not a detached number copied into an equation. Asset-price gains and borrowing are not new current production.
4. **Industry labels → measured counterfactual harm/benefit.** Do not classify all finance, profit, administration, or intermediation as productive or parasitic by default. Distinguish transfers from real resource costs; estimate market-power, rent-seeking, congestion, pollution, and failure costs only under explicit profiles and evidence.
5. **Point value → evidence frontier.** Record observed, official, derived, estimated, modelled, assumed, and unknown statuses separately. Missingness is not zero; uncertainty is not a permission to invent capacity.
6. **Model success → empirical qualification.** Synthetic correctness, historical fit, out-of-sample prediction, shadow use, prospective causal impact, and independent replication are distinct evidence tiers.

## Report contract

The v0.1 schema defines these components:

- Jurisdiction and period.
- Reporting currency and price basis.
- Source manifest (identity, class, vintage, geographic/temporal coverage, licensing status, limitations, optional digest). Source classes distinguish official_statistics from experimental_statistics; publisher identity is not a claim that a discussion document is official statistical output.
- Conventional accounts (GDP, consumption of fixed capital, NDP, gross capital formation).
- Financial events that must retain their SNA/profile treatment, such as new borrowing, principal repayment, write-off, secondary-market asset transfer, asset revaluation, and inter-sector transfers.
- Canonical stock-flow accounts with signed changes and optional links to the event that causes each financial change.
- Multidimensional outcome vector with explicit unit, evidence status, population/coverage denominator, source references, and optional uncertainty interval.
- Reconciliation checks for accounting identities and physical stock/flow balances. Every reconciliation operand and LHS is bound by JSON Pointer to the canonical value in the report; free-floating numbers cannot qualify an identity.
- Data-quality summary and a policy forbidding a universal scalar or addition across unlike units.
- Explicit claim ceiling.

The JSON Schema enforces shape and basic field domains. Typed claims separately declare claim type, evidence status, scope, statement, and source references. The independent semantic validator checks that references exist, arithmetic identities hold, unknowns remain visible, and the report cannot promote synthetic evidence to observations or causal evaluation. Do not try to infer epistemic status by parsing prose: negated disclaimers create false positives. Schema validation is necessary but not sufficient.

## Economic semantics that must not be blurred

- GDP is a production flow; wealth is a stock. Do not add them.
- A new loan creates paired financial positions under the appropriate accounting profile; principal is not income/output just because cash changes hands.
- Secondary-market trades and asset revaluations are not, by themselves, current production.
- Transfers may matter enormously for distribution but are not automatically net new social resources.
- Intermediate transactions must not be added repeatedly on top of final output.
- Physical quantities are reconciled within a common account and compatible unit; do not sum cubic metres, tonnes, hours, and currency.
- Unpaid care and household work may be represented in a separate satellite account with an explicit valuation method; do not quietly append replacement-cost estimates to GDP.
- Environmental damages, rents, and markups need distinct definitions and counterfactuals. A fee's existence alone does not establish extraction or social loss.
- Regional or national aggregation must declare consolidation, imported content, cross-border externalities, coverage, and price-year conventions.

## Benchmark plan

### Baselines

For all national outputs, retain official country statistical publications and vintage-specific SNA aggregates. For broader views, compare with SEEA, CWON, and OECD-compatible indicators where the definitions/data align. For predictive tasks, compare against simple seasonal/persistence, Bridge, Dynamic Factor, and Lasso/Elastic Net models before complex ML. Night lights and other remote-sensing signals are optional features to ablate, not a ground-truth GDP measure.

This baseline discipline follows recent IMF evidence: the December 2025 IMF working paper found that traditional Bridge/Dynamic Factor approaches often outperformed ML in short national GDP nowcasting series, while a January 2026 IMF working paper reported a useful nightlights + random-forest application in a specific data-limited setting. Those findings support comparison and local validation, not a universal winning model.

For physical/service questions, the baseline is the best available operator/engineering model and a matched intervention scenario. Sol-Atlas assessment conformance is not itself proof of hydraulic, electrical, or delivered-service validity.

### Qualification phases

1. **Contract and synthetic oracle:** known-answer account/physical identities, unit rules, evidence boundaries, and negative controls.
2. **Historical source replay:** source-pinned series, revisions, documentation of gaps, and reproducible reconciliation to official totals.
3. **Held-out evaluation:** rolling-origin and time-vintage-aware forecasting; geography/country holdouts where appropriate; no target leakage from later revisions.
4. **Shadow operation:** compare estimates and proposed interventions to recorded operations without dispatching or changing policy.
5. **Prospective pilot:** preregistered intervention, outcome and cost definitions, plausible comparison group, and safety/privacy review.
6. **Independent replication:** separate dataset/operator/site and external review.

### Required metrics

- Accounting/physical identity closure and classified residuals.
- Coverage, freshness, missingness, source quality, and comparability.
- Forecast MAE/RMSE and interval calibration versus every baseline, with uncertainty and failed runs retained.
- Per-service reliability, unmet demand, quality-adjusted delivered units, fully loaded intervention cost, and recovery/tail loss where meaningful.
- Distributional outcomes by relevant groups/regions when lawful evidence supports it.
- Sensitivity to price basis, source vintages, assumptions, model family, aggregation, and externality valuations.
- Provenance reconstruction and determinism where determinism is claimed.

### Initial hard gates (targets, not measured results)

- 100% of frozen known-answer synthetic accounts reconcile within their declared numerical tolerances.
- Zero unexplained account/physical residuals accepted as qualified.
- Zero missing values silently converted to zero; zero fabricated source records; zero modeled results mislabeled as observations.
- 100% of outcome indicators retain a definition, unit, time scope, evidence state, and source-reference set.
- All negative controls are rejected or explicitly downgraded; no row, period, or failed case is silently removed.
- Official-account bridge discrepancies remain within the source's published rounding/revision tolerance or are classified and disclosed.
- A predictive or intervention improvement must exceed a task-specific, preregistered practical-effect threshold on untouched data while respecting all non-compensable floors. No global “x% better GDP” target is meaningful before the actual task is defined.

## Synthetic fixture

The adjacent fixture is wholly synthetic. Its values are deliberately small and hand-checkable. Its typed claims make the evidence tier explicit. It demonstrates:
- GDP minus consumption of fixed capital equals NDP.
- A financial stock roll-forward reconciles.
- A water stock-flow identity reconciles in a physical unit.
- New borrowing and asset revaluation remain outside the current-production aggregate.
- Water service coverage, unmet demand, unpaid care hours, and a distributional outcome remain separate coordinates.
- The report states its limitations rather than claiming a real South African estimate.

The fixture is not country data, calibration data, or validation evidence. A semantic validator and adversarial/property test corpus are required before v0.1 can be called qualified.

## Adversarial test corpus

The companion file fixtures/nrvca-adversarial-corpus-v0.1.json defines 20 proposed mutation cases against the synthetic golden fixture. It covers accounting misclassification, unknown-to-zero coercion, missing provenance, incompatible physical units, broken identities, duplicate events, currency/profile mismatch, unsupported evidence-tier promotion, scalar collapse, causal overclaim, and missing-period/unknown-register failures. Its status is explicitly specification-only: an independent validator still needs to execute each mutation and assert the expected disposition.

## Additional adversarial tests to add later

1. Count a loan drawdown as GDP.
2. Count a secondary-market asset purchase as new production.
3. Add an asset revaluation to current output.
4. Double-count an intermediate good and final good.
5. Duplicate one physical event under two source IDs.
6. Drop writeoffs or principal repayments from a stock roll-forward.
7. Change the currency, price year, physical unit, period, or jurisdiction without changing the profile.
8. Convert an unknown value into zero or claim missing service capacity is available.
9. Use an observation outside its geographic or valid-time scope.
10. Improve the headline average while violating a protected service floor or making the worst-served group materially worse.
11. Leak revised/late-published GDP values into historical nowcasts.
12. Tune against the holdout or delete failed/negative-result runs.
13. Add unpaid-care valuation to GDP and to the satellite account simultaneously.
14. Count both sides of a transfer as net new social output.

## South Africa first pilot

Start by inventorying available Stats SA national accounts and natural-capital accounts, definitions, update cadence, licenses, and geographic resolution. Begin with water/energy service reliability and a narrowly scoped infrastructure investment comparison. Use public or authorized datasets only. Record every gap; do not estimate national totals by extrapolating from a convenient city pilot without a validated sampling/aggregation method.

Suggested first deliverable after this spec: a source-vintage catalogue and a pinned, low-dimensional water/energy synthetic-to-historical bridge. Keep the historical data pipeline read-only until data rights and statistical mapping are reviewed.

## Preregistered benchmark contract

The current design-only benchmark preregistration and three-track methodology are documented in [benchmark-protocol-v0.1.md](./benchmark-protocol-v0.1.md), with a typed preregistration schema at [nrvca-benchmark-preregistration-v0.1.schema.json](../../../schemas/nrvca-benchmark-preregistration-v0.1.schema.json) and the synthetic/design-only South African account-bridge record at [nrvca-sa-bridge-preregistration-v0.1.json](../../../fixtures/nrvca-sa-bridge-preregistration-v0.1.json). No empirical benchmark has been run by that preregistration. The thresholds are correctness/coverage acceptance rules, not a claim of superiority over current systems.

## South Africa source inventory

The first source-discovery pass is recorded in [South Africa NRVCA Source Catalogue v0.1](./south-africa-source-catalogue-v0.1.md), with a machine-readable companion at [nrvca-sa-source-catalogue-v0.1.json](../../../fixtures/nrvca-sa-source-catalogue-v0.1.json). The inventory distinguishes official GDP statistics from experimental PEFA and sub-national water accounts, and labels DWS planning/monitoring pages as context or candidate evidence rather than claiming their data have already been ingested. All entries remain discovery-only; source bytes, licensing, hashes and transformations are not yet qualified.

## Synthetic follow-on benchmarks

- [Economic decomposition v0.1](./economic-decomposition-v0.1.md) and its [known-answer fixture](../../../fixtures/nrvca-economic-decomposition-golden-v0.1.json) distinguish transfer incidence, modelled deadweight loss, fixed-cost recovery, resource costs of rent capture, physical externality measurement and valuation sensitivity. Nine declared demand/cost-slope settings test whether the toy-model invariants hold.
- [Decision-value benchmark v0.1](./decision-value-benchmark-v0.1.md) and its [synthetic fixture](../../../fixtures/nrvca-decision-value-synthetic-v0.1.json) exhaustively evaluate all 16 allocations in a four-intervention example. A service-only baseline is compared with explicit service/equity/reliability/environmental constraints, four alternative policy profiles, robust interval endpoints for service-weighted reliability, and explicit infeasibility outcomes. The profile is a policy choice; the result is not a universal welfare ranking. The next tier must add correlated and physically admissible scenarios rather than treating interval endpoints as a probability distribution.
- [Correlated scenario stress benchmark v0.1](./correlated-scenario-stress-v0.1.md) and its [synthetic stress fixture](../../../fixtures/nrvca-correlated-stress-synthetic-v0.1.json) evaluate three strategies over five jointly described stress scenarios, with complete matrix coverage, explicit action triggers, scenario-neutral robust-feasibility fractions and worst-case shortfalls. Outcomes are hand-authored synthetic assumptions, not calibrated simulation. The adaptive policy passes 4/5 scenarios and still fails prolonged/repeated disruption; do not hide that failure.
- [Scenario evidence contract v0.1](./scenario-evidence-contract-v0.1.md) and its [machine-readable manifest](../../../fixtures/nrvca-scenario-evidence-manifest-v0.1.json) cross-check all six stressor definitions, distinguish methodology references from empirical evidence, enforce fixture-scoped dependencies and repeated-event history, and prevent unsupported evidence-tier promotion. All current stressors remain synthetic and uncalibrated; digest syntax checks are not exact-byte verification.
- [Pareto stress analysis v0.1](./pareto-stress-analysis-v0.1.md) and its [frozen expected-answer fixture](../../../fixtures/nrvca-pareto-stress-expected-v0.1.json) report all-policy and feasible Pareto fronts plus unit-preserving constraint-violation vectors. The prototype uses no objective weights or cross-unit penalty; the prolonged/repeated case has an empty feasible front. This is analysis of the synthetic fixture, not an empirical policy ranking.
- [NRVCA-003 draft PR #5394](https://github.com/Luminous-Dynamics/mycelix/pull/5394) and [NRVCA-004 draft PR #5400](https://github.com/Luminous-Dynamics/mycelix/pull/5400) contain these prototypes. Static known-answer audits are recorded in the PRs; successful Rust execution must be established separately on each exact head.

## Linked work

- NRVCA-001: https://github.com/Luminous-Dynamics/mycelix/issues/5374
- NRVCA-002: https://github.com/Luminous-Dynamics/mycelix/issues/5382 (source-vintage acquisition and historical reconciliation)
- NRVCA-003: https://github.com/Luminous-Dynamics/mycelix/issues/5387 (transfers, rents, resource costs, deadweight loss and external damages)
- NRVCA-004: https://github.com/Luminous-Dynamics/mycelix/issues/5399 (decision value under explicit service and equity constraints)
- NRVCA-005: https://github.com/Luminous-Dynamics/mycelix/issues/5409 (correlated disruption scenarios and adaptive policy stress tests)
- NRVCA-006: https://github.com/Luminous-Dynamics/mycelix/issues/5417 (machine-checkable scenario evidence, dependencies and repeated-event history)
- NRVCA-007: https://github.com/Luminous-Dynamics/mycelix/issues/5419 (Pareto-front analysis and non-compensable decision-constraint vectors)
- Robust Decision Making under deep uncertainty: https://ppp.worldbank.org/library/investment-decision-making-under-deep-uncertainty
- Mycelix FIN-SYS-003: https://github.com/Luminous-Dynamics/mycelix/issues/3072
- Mycelix CIV-ECON-006: https://github.com/Luminous-Dynamics/mycelix/issues/4816
- Mycelix CIV-IMPACT-001: https://github.com/Luminous-Dynamics/mycelix/issues/5046
- Sol-Atlas evidence-aware spatial-economic resilience analysis: https://github.com/Luminous-Dynamics/sol-atlas-leptos/pull/116
- Sol-Atlas capital assessment adapter: https://github.com/Luminous-Dynamics/sol-atlas-leptos/issues/162

## References

- UN System of National Accounts 2025: https://unstats.un.org/unsd/econstats/national-accounts/sna2025
- UN SEEA Central Framework: https://seea.un.org/en/methodology/seea-central-framework
- UN SEEA Ecosystem Accounting: https://seea.un.org/en/methodology/ecosystem-accounting
- World Bank Changing Wealth of Nations 2024: https://www.worldbank.org/en/publication/the-changing-wealth-of-nations
- World Bank, Investment Decision Making Under Deep Uncertainty: https://ppp.worldbank.org/library/investment-decision-making-under-deep-uncertainty
- Water Utility Climate Alliance / The Water Institute, Beginner's Guide to DMDU for Water Utilities (2026): https://www.cakex.org/documents/beginners-guide-decision-making-under-deep-uncertainty-water-utilities
- OECD Beyond GDP: https://www.oecd.org/en/publications/beyond-gdp_9789264307292-en.html
- IMF GDP Nowcasting Performance of Traditional Econometric Models vs Machine-Learning Algorithms (2025): https://www.imf.org/en/publications/wp/issues/2025/12/05/gdp-nowcasting-performance-of-traditional-econometric-models-vs-machine-learning-572360\n- IMF SNA/BPM Implementation Planning Playbook (July 2026): https://www.imf.org/en/publications/tnm/issues/2026/07/02/sna-bpm-implementation-planning-playbook-575192
- IMF Nowcasting Economic Growth with Machine Learning and Satellite Data (2026): https://www.imf.org/en/publications/wp/issues/2026/01/30/nowcasting-economic-growth-with-machine-learning-and-satellite-data-573623
- Statistics South Africa, National Natural Capital Accounting Strategy: https://www.statssa.gov.za/?page_id=14714
