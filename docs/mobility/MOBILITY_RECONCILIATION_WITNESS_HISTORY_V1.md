# MOBILITY-COMMONS-019: Cross-Layer Witness History and Projection Preservation

This contract qualifies composition across witness identity, witness revision, temporal reconciliation, and evidence projection.

## Canonical lifecycle

`witness-v1 → projection-v1 → witness-v2 supersedes witness-v1 → projection-v1 remains historical → projection-v2 binds only to witness-v2`

A witness identity denotes one semantic witness payload. A materially changed witness requires a new identity and an explicit successor-to-predecessor `Supersedes` lineage edge. Projection references are exact identity bindings; they are not automatically retargeted when a successor appears.

## Required invariants

- Historical projection remains bound to its predecessor.
- Successor projection binds only to its successor.
- Reversed, missing, or mismatched supersession edges are rejected.
- Predecessor remains valid after supersession.
- Claim identities remain distinct from witness identity.
- Three-generation chains preserve every witness/projection binding.
- Protocol identity and timestamps do not become engineering identity or applicability semantics.

## Qualification boundary

The corpus establishes agreement about declared semantic/provenance behavior only. It does not establish physical correctness, physical equivalence, safety, certification, regulatory approval, measurement truth, or authorship truth.

## Independent qualification

Rust and Python consume the same executable corpus and emit `mobility-reconciliation-witness-history-normalized-v1`. The comparator requires exact envelope, cardinality, identifiers, operation, expected result, and actual result.

## Research basis

Holochain documents deterministic/pure validation and requires validation dependencies to be explicitly addressable; unavailable dependencies are unresolved rather than validation failures. citeturn1search0turn1search3

NIST's digital-thread research emphasizes persistent identifiers and traceability across product lifecycle stages, including engineering, manufacturing, quality, and downstream observations. citeturn1search2turn1search5turn1search6
