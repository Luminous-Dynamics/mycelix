# AC-059: Economic Scenario Evidence Boundary

## Purpose

AC-059 gives the Economic OS a deterministic identity for conditional economic scenarios and counterfactuals.

A scenario is **evidence-bound model context**, not a forecast, authorization, policy instruction, or truth claim.

The intended flow is:

local observations → exact observation snapshot → scenario definition → independent analyses → governance → authorized action → observed outcome.

## Why this boundary matters

Economic simulations are useful precisely because they can represent multiple conditional worlds. They become dangerous when a scenario is treated as an unqualified prediction or when one model's output becomes a policy authority.

The scenario identity therefore binds:

- exact policy profile and fingerprint;
- exact observation snapshot;
- model/engine identities and versions;
- explicit assumptions;
- optional interventions;
- evaluation horizon;
- uncertainty references;
- retrospective/backtest evidence;
- alternative scenario identities.

The content fingerprint makes the complete scenario definition addressable without treating the hash as authorship, correctness, or legal authority.

## Scenario classes

'Baseline' is a reference state.

'Counterfactual', 'StressTest', and 'Sensitivity' scenarios must identify the baseline they depart from.

'Backtest' is explicitly retrospective and must carry backtest evidence.

This prevents a scenario graph from collapsing distinct epistemic roles into a single generic "forecast" object.

## Exact analysis binding

'EconomicPolicyAnalysis' now carries an 'EconomicScenarioBinding' containing both the scenario reference and its exact content fingerprint.

'EconomicPolicyAnalysis::validate_against_scenario' verifies:

1. scenario reference identity;
2. exact scenario content identity;
3. policy-profile identity;
4. observation-snapshot identity.

An analysis therefore cannot silently retain the name of a scenario while changing the underlying assumptions or input snapshot.

## Governance boundary

Scenario generation and analysis remain advisory.

No scenario object can:

- authorize policy;
- commit economic state;
- settle payments;
- reconcile execution;
- finalize an action.

Those capabilities remain behind the Economic OS governance/execution boundaries established by AC-051 through AC-058.

## Global design implication

The useful global architecture is not "one economic AI." It is a federated observatory in which different jurisdictions and institutions can produce portable, comparable scenario objects while retaining their own policy profiles, authorities, models, and data sovereignty.

This matters as AI adoption itself may create cross-country divergence and as AI market concentration can make a single model/provider an economic dependency. A federated scenario identity makes model pluralism technically visible rather than merely aspirational.

## Research grounding

The IMF's August 2026 work on AI and economic divergence emphasizes that model-based macro simulations should be read as conditional scenario illustrations rather than point forecasts, especially given uncertainty about AI progress and policy response.

The OECD's July 2026 work on AI markets highlights structural concentration in compute, data, chips, and cloud services, reinforcing the need for contestable and plural economic intelligence.

The World Bank's 2026 Global DPI program similarly emphasizes reusable, interoperable digital identity, payments, and secure data-sharing foundations rather than isolated systems.

References:

- IMF, AI and Economic Divergence in Asia (2026-08-07)
- OECD, Artificial Intelligence markets: Recent developments and competition issues (2026-07-10)
- World Bank, Global Digital Public Infrastructure Program: From Foundations to Scale (2026-05-06)

## Qualification rule

Repository CI is the qualification source for this change. No local test pass is claimed from an environment without the repository checkout.
