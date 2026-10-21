# NRVCA economic decomposition v0.1

Status: synthetic method prototype. This is not a measured estimate of any firm, sector, or country, and it is not an official SNA or SEEA extension.

## Why this increment matters

The question “how much economic activity is real value?” is not answerable by subtracting every profit, markup, rent, transfer, pollution estimate, or administrative expense from GDP. Those quantities refer to different economic objects. This prototype makes a narrow falsifiable improvement: preserve the categories and test them on tiny cases where the answers are known.

The fixture at ../../../fixtures/nrvca-economic-decomposition-golden-v0.1.json is synthetic. The integration test implements an independent closed-form oracle for one linear supply-demand example; that oracle is intentionally separate from the production report validator and is not a real-world market estimator.

## Category contract

| Category | What is observed or estimated | What must not be inferred automatically |
|---|---|---|
| Transfer incidence | Counterparty payments and who pays/receives | That a domestic transfer is itself destruction of real resources |
| Productive service | Units, quality, availability, risk-sharing, innovation, coordination, maintenance | That price or profit alone measures delivered social value |
| Markup / rent | Method-dependent estimates under a declared market boundary and cost model | That concentration, profit, or a markup is proof of extraction or welfare loss |
| Rent-seeking resource use | Labor, time, lobbying, legal and administrative effort where measured | That the amount captured as a transfer equals the real resources spent capturing it |
| Deadweight loss | Counterfactual model output under explicit behavioral and market assumptions | That it is an observed transaction, or that it can be added to transfer incidence |
| External damage | Physical emissions, depletion, health/ecosystem outcome, then optional valuation | That one shadow price is a measured fact or that overlapping harms can be summed without reconciliation |
| Non-market service | Delivered units and service-quality measures, with declared valuation method if available | That a missing market price means zero value or licenses a fabricated replacement price |

## Independent toy market oracle

For the benchmark model, inverse demand is P(Q) = a - bQ and marginal cost is MC(Q) = c + dQ, with positive slopes and no externalities. The competitive quantity is (a-c)/(b+d); the monopoly quantity is (a-c)/(2b+d). The test independently computes consumer surplus, producer surplus, total surplus, the price transfer on units still sold, and deadweight loss relative to the competitive outcome.

The purpose is to catch category substitution. In the golden case, the transfer on retained units is not equal to deadweight loss. The loss in consumer surplus decomposes into a producer-surplus gain and deadweight loss under the toy assumptions. These identities are only valid for the declared model; they are not a universal estimator for a real market.

## Acceptance gates for this synthetic tier

- Counterparty entries net to zero for the explicitly consolidated domestic transfer; distributional incidence remains visible.
- The independent market oracle reproduces frozen analytical known answers within tolerance.
- A pure transfer is not counted as resource destruction merely because its amount is large.
- Profit, markup, or capture amount alone cannot manufacture a rent estimate or deadweight-loss estimate.
- Real resources spent on rent-seeking remain distinct from the captured transfer.
- Physical externality quantities remain fixed as shadow-price assumptions vary; valuation sensitivity is retained.
- Missing price or counterfactual inputs trigger unknown/abstention rather than a default zero or unsupported point estimate.
- No universal scalar combines unlike objects, and no synthetic result is claimed as an empirical national account.

## State-of-the-art comparison

This is deliberately not claimed as a new national welfare index. It composes ideas that already exist in national accounts, environmental-economic accounting, competition measurement, and beyond-GDP dashboards. Its measurable contribution at this stage is a testable category contract, traceability and adversarial qualification. A fair comparison must reproduce aligned SNA/SEEA/World Bank/OECD concepts rather than relabel them, and must evaluate the additional decision value separately.

The prototype should not advance beyond synthetic qualification until the production validator is compiling cleanly, source vintages and rights are pinned for real-data work, and the next tests include alternative demand elasticities, entry, fixed costs, quality/innovation, regulated natural monopolies, cross-border incidence, and uncertainty intervals.
