# Mobility Temporal Reconciliation Witness Contract v1

A reconciliation result is auditable only when the two claims and all classification inputs are explicitly recorded.

## Required fields

- left_claim and right_claim: domain-neutral identity references;
- left_applicability and right_applicability: temporal applicability intervals;
- comparability: explicit comparable/incomparable/unknown disposition;
- compatibility: explicit compatible/incompatible/unknown disposition;
- explicitly_superseded: explicit lineage disposition;
- disputed: explicit dispute marker;
- result: deterministic classification and dispute marker.

The witness validator recomputes the result from these fields and rejects tampered classifications.

## Boundary

A witness establishes reproducibility of a semantic reconciliation classification. It does not establish physical truth, safety, certification, regulatory approval, physical equivalence, or engineering correctness.

A conflict witness means only that the supplied claims were explicitly marked comparable and incompatible and their known applicability intervals overlap. It does not determine which claim is correct.

## Adversarial requirements

1. Missing claim identity is rejected.
2. Same claim on both sides is rejected.
3. Tampered classification is rejected.
4. Tampered dispute marker is rejected.
5. Unknown temporal bounds remain indeterminate.
6. Explicit supersession remains superseded and historical evidence remains addressable.
7. Holochain identifiers cannot silently become native engineering claim identities.
