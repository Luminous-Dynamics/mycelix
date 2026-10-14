# Mobility Identity and Lineage Algebra v1

This contract separates engineering identity from configuration scope, evidence relationships, and Holochain protocol metadata. It is semantic/provenance-only and does not establish physical safety, physical equivalence, certification, regulatory approval, or engineering correctness.

## Identity kinds

requirement; design_revision; configuration_revision; component_instance; manufacturing_event; physical_artifact; inspection_record; test_record; operational_observation; maintenance_event; change_set; evidence_record.

An IdentityRef has explicit kind, namespace, and id. Empty namespace/id is invalid. Holochain action/entry identifiers are not engineering identities merely because they are unique.

## Lineage relations

- instantiates: physical/component instance -> design revision
- configures: configuration revision -> design revision/component instance
- component_of: component instance/physical artifact -> physical artifact
- manufactured_from: manufacturing event -> design/configuration revision
- inspected_as: inspection record -> physical artifact
- tested_as: test record -> physical artifact
- observed_as: operational observation -> physical artifact
- maintained_as: maintenance event -> physical artifact
- repaired_as: maintenance event -> physical artifact
- replaced_by: component/physical artifact -> component/physical artifact
- supersedes: design/configuration/evidence revision -> same identity class

## Non-equivalences

1. Two artifacts can instantiate the same design without sharing identity.
2. A configuration revision can change while the physical artifact remains the same.
3. Repair does not create a new physical-artifact identity.
4. Component replacement preserves the predecessor component's historical identity.
5. Identical geometry/content does not imply identical manufacturing lineage.
6. Identical manufacturing lineage does not imply identical observed condition.
7. Configuration supersession does not imply physical replacement.
8. Retirement preserves historical identity; it does not erase the record.
9. A foreign identifier must be explicitly bound before it becomes usable in the commons.
10. Holochain hashes and action timestamps remain protocol metadata; they are not engineering validity intervals.

## Relationship to configuration scope

This complements #3727. Scope identifies the configuration context of a relationship; identity identifies the engineering thing. Neither substitutes for the other. Ordinary evidence relationships still obey the scope rules in MOBILITY_EVIDENCE_RELATION_ALGEBRA_V1.

## Qualification boundary

Qualification proves only that identity/lineage distinctions are preserved and invalid substitutions are rejected. It is not a physical test, safety assessment, certification, or regulatory determination.
