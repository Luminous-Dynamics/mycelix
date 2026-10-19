# Mobility Configuration Contract V1 — Qualification Test Vectors

This document is the human-readable companion to `MOBILITY_CONFIGURATION_CONTRACT_V1.json`.

These vectors qualify **semantic and structural representation only**. They do not establish physical correctness, structural integrity, safety, road legality, seaworthiness, airworthiness, certification, manufacturing conformity, or operational authorization.

For every vector, the contract must preserve the expected distinction and must not support the listed forbidden inference.

| ID | Scenario | Expected outcome | Forbidden inference |
|---|---|---|---|
| MC-CONFIG-001 | same-design-different-artifacts | distinct-physical-artifacts | artifact-identity-collapse |
| MC-CONFIG-002 | same-cad-different-process | manufacturing-lineage-preserved | cad-equality-proves-physical-equivalence |
| MC-CONFIG-003 | component-substitution | substitution-lineage-and-revalidation | compatibility-claim-proves-equivalence |
| MC-CONFIG-004 | failed-test-followed-by-later-test | historical-evidence-preserved | later-pass-erases-failure |
| MC-CONFIG-005 | simulation-equals-measurement | prediction-observation-remain-distinct | numerical-equality-converts-prediction-to-observation |
| MC-CONFIG-006 | negative-inspection-finding | negative-evidence-preserved | only-success-is-authoritative |
| MC-CONFIG-007 | undeclared-dependency | unknown-or-review-required | missing-dependency-means-unaffected |
| MC-CONFIG-008 | private-payload-public-provenance | controlled-payload-with-public-provenance | public-commitment-proves-private-access |
| MC-CONFIG-009 | foreign-engineering-identifier | explicit-binding-or-rejection | foreign-id-silently-becomes-native |
| MC-CONFIG-010 | holochain-hash-as-engineering-id | reject-semantic-substitution | protocol-hash-is-engineering-identity |
| MC-CONFIG-011 | observation-versus-diagnosis | observation-separate-from-interpretation | observation-proves-root-cause |
| MC-CONFIG-012 | repair-lineage | repair-history-and-resulting-state-preserved | repair-erases-history |
| MC-CONFIG-013 | external-authority-disposition | external-authority-remains-attributable | graph-consensus-generates-authority |
| MC-CONFIG-014 | assurance-criticality-metadata | metadata-remains-descriptive | criticality-label-proves-safety |
| MC-CONFIG-015 | multimodal-interface | interface-dependencies-explicit | shared-connector-proves-interoperability |
| MC-CONFIG-016 | configuration-supersession | historical-predecessor-preserved | supersession-deletes-history |
| MC-CONFIG-017 | open-revalidation-obligation | obligation-remains-open | obligation-means-test-passed |
| MC-CONFIG-018 | ground-and-marine-instances | shared-core-profile-specific-divergence | domain-neutrality-forces-identical-engineering |
| MC-CONFIG-019 | software-firmware-change | dependency-aware-change-lineage | unchanged-cad-means-unchanged-configuration |
| MC-CONFIG-020 | retired-physical-artifact | historical-artifact-not-active | historical-existence-means-current-validity |

## Qualification boundary

A passing evaluator means only that the machine-readable corpus satisfies the declared semantic/structural rules. It is not an engineering analysis or certification decision.

The Holochain layer, where later integrated, may validate protocol-level structure and authorship. It must not be treated as an authority that establishes physical truth or regulatory approval.
