# AC-052: Measurement-Only Real-Resource Observations

## Purpose

AC-052 creates a measurement boundary between raw economic evidence and policy interpretation. It represents real-resource and macroeconomic signals without turning them into a universal composite index or monetary rule.

## Signals

The reference layer currently supports price level/growth, labor capacity/participation, productive-capacity utilization, ecological capacity/pressure, external constraints, import constraints, and foreign-currency constraints.

Each observation retains:

- stable identity;
- subject/entity reference;
- signal type;
- native unit;
- plain decimal value;
- explicit reference period;
- methodology and source references;
- supporting evidence;
- confidence and quality metadata;
- observation timestamp;
- optional supersession and conflict-set references.

## Exact snapshots

EconomicObservationSnapshot binds every observation reference to its exact content fingerprint. Snapshot validation checks both content identity and temporal coherence: an observation recorded after snapshot capture cannot be included as though it were known at capture time.

Snapshot fingerprints are suitable inputs to EconomicPolicyScenario and EconomicPolicyAnalysis, allowing a model result to be reproduced against the same evidence boundary.

## Conflict preservation

Conflicting measurements from different sources remain separate observations. A shared conflict-set reference makes their relationship explicit without choosing a winner at the measurement layer.

## Policy neutrality

This layer does not:

- define an inflation target;
- classify an economy as healthy or unhealthy;
- authorize monetary/fiscal policy;
- convert ecological or labor constraints into automatic action;
- treat one dimension as compensating for a hard breach in another.

Policy engines interpret the observations later under an explicit EconomicPolicyProfile and governance regime.

## Qualification scenarios

The design is intended to support deterministic tests such as:

1. high liquidity with tight productive capacity;
2. low liquidity with slack productive capacity;
3. ecological breach with otherwise healthy financial indicators;
4. stable prices with severe external/import constraint;
5. conflicting observations from distinct sources.

## Research grounding

The layer is compatible with the Economic OS principle of separating measurement from interpretation. Current international digital-infrastructure work also emphasizes reusable, interoperable data-sharing foundations, while current macroeconomic AI research stresses conditional scenarios and material uncertainty rather than point forecasts.

References:

- IMF, AI and Economic Divergence in Asia (2026-08-07)
- OECD, Artificial Intelligence markets: Recent developments and competition issues (2026-07-10)
- World Bank, Global Digital Public Infrastructure Program: From Foundations to Scale (2026-05-06)

## Qualification rule

Repository CI is the qualification source. No local cargo test pass is claimed from an environment without the repository checkout.
