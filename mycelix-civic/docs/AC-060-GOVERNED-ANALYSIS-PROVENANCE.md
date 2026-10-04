# AC-060: Governed Analysis Provenance and Anti-Herding

## Purpose

AC-060 closes the handoff between advisory economic cognition and governance. A governed decision can now retain exact fingerprints for the analyses it actually considered, while the analyses remain non-authoritative.

## Exact provenance

`GovernedPolicyAdjustment` carries an optional `analysis_evidence` set. Each entry binds an analysis reference to its exact content fingerprint.

Validation resolves each binding against supplied `EconomicPolicyAnalysis` records and fails closed if:

- the analysis is missing;
- its content fingerprint has changed;
- its policy-profile reference differs;
- its policy-profile fingerprint differs.

An empty analysis set remains valid because some policy regimes permit decisions based on non-model evidence.

## Anti-herding measurement

The decision boundary exposes `distinct_analysis_provenance_count`. This groups resolved analyses by model identity, observation snapshot, and scenario identity.

Two differently named analyses that use the same model and the same inputs therefore do not automatically count as independent evidence.

This is intentionally a measurement rather than a constitutional threshold. A policy profile may later require multiple independent analyses for particular decision classes without baking that rule into the Economic OS kernel.

## Why this matters globally

Current IMF work shows that AI-driven economic dynamics can diverge across countries and that model simulations depend materially on country structure and assumptions. Its simulations are explicitly conditional rather than forecasts.

The OECD's July 2026 assessment highlights concentration across compute, data, cloud, chips, and foundation-model infrastructure. A single dominant model or provider therefore represents more than a technical dependency; it can become an economic epistemic dependency.

BIS analysis likewise emphasizes that AI simultaneously shifts supply, demand, and financial conditions, making economic state estimation harder and increasing uncertainty around monetary-policy transmission.

A provenance group is deliberately narrower than an accuracy claim: two independent model groups can still be wrong in the same direction, while repeated wrappers around one model do not become independent merely through naming.

The architecture response is pluralism made machine-checkable:

observations → snapshots → scenarios → competing analyses → governed decision → authorized action → outcome.

## Boundary

Analysis provenance does not prove that an analysis is correct. A content fingerprint is not authorship proof, legal authority, or an accuracy certificate. The governance event remains the authority boundary.

## Qualification rule

Repository CI is the qualification source. No local test pass is claimed from an environment without the repository checkout.
