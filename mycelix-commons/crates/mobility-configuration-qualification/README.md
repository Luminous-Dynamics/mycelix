# Mobility Configuration Qualification Harness

This is an isolated, non-published Rust harness for **MOBILITY-COMMONS-004**.

It machine-checks the semantic qualification corpus for Mobility Configuration Contract V1. The harness intentionally stays below physical engineering and regulatory authority.

## What a green result means

A green result means:

- the corpus is structurally complete;
- all 20 qualification vectors are present exactly once;
- every vector has an explicit expected semantic outcome;
- every vector names a forbidden inference boundary;
- the qualification status remains semantic-only.

A green result does **not** establish:

- physical correctness or structural integrity;
- safety;
- road legality or seaworthiness;
- airworthiness;
- certification or manufacturing conformity;
- operational authorization.

The harness is deliberately small. It validates the qualification corpus itself rather than attempting to become a universal engineering ontology or safety oracle.

## Independent reconstruction

The corpus is stored separately at:

`docs/mobility/MOBILITY_CONFIGURATION_CONTRACT_V1.json`

The source prose corpus remains:

`docs/mobility/MOBILITY_CONFIGURATION_CONTRACT_V1_TEST_VECTORS.md`

The intended next step is a second, independently reconstructed implementation and then the two physical-domain fixtures described in #3711.
