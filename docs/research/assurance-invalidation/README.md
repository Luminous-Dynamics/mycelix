# LISE-ASSURANCE-INV-50 v1 — public development corpus

**Owner:** [#6219 — LISE-ASSURANCE-INVALIDATION-001](https://github.com/Luminous-Dynamics/mycelix/issues/6219)
**Status:** public synthetic development/calibration set; not a hidden or blinded holdout.

## Files
- `scenarios.v1.json`: 50 scenarios in 25 A/B minimal pairs.
- `oracle.v1.json`: separate 50-row owner-scoped outcome oracle.
- `scenario.schema.json` and `oracle.schema.json`: strict Draft 2020-12 record schemas.

## What it tests
1. Key, registry and trust-root changes.
2. Policy, authority and delegation changes.
3. PEP, route and ACS profile changes.
4. Model, memory, prompt and context subject changes.
5. Observer/source dependency and coverage changes.
6. Bitemporal frontiers and transparency checkpoints.
7. EvidenceLease reusable-horizon expiry.

The oracle preserves the owner vocabularies from #1087 (`EvidenceApplicability`) and #2631 (`CurrentnessAssessment`). The separate horizon field is a benchmark observation of the existing #181 EvidenceLease, not a new lease type. No universal currentness boolean or weighted score is defined.

Pairs include both under-invalidation cases (stale evidence must not remain current) and over-invalidation controls (unrelated claims must remain supported).

## Use
Validate the scenario/oracle record arrays against the schemas; verify 50 stable IDs, 25 A/B pairs, exact one-to-one oracle linkage and family counts. Reviewers should classify scenarios without viewing the public oracle first, then use it for calibration.

## Disclosure and claim ceiling
Both scenarios and oracle are public. This is a development/calibration seed, not a blinded holdout, confirmatory benchmark or independent adjudication. A PASS on this finite corpus would establish only conformance to the named owner-scoped synthetic cases. It would not establish real-world event-feed completeness, continuous system security, or deployment currentness.
