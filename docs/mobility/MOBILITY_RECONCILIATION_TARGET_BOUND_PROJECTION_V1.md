# MOBILITY-COMMONS-021: Target-Bound EvidenceState Projection

This layer closes a provenance gap left by historical projection preservation: an exact reconciliation witness reference is not sufficient if the resulting `EvidenceState` can still be applied to an unrelated subject.

The qualified projection record therefore carries two distinct identities:

- **witness identity** — the exact temporal reconciliation witness being projected;
- **target identity** — the exact `EvidenceRecord` whose EvidenceState is being projected.

The target is explicit, typed, and must equal the target supplied to validation. A target cannot be the reconciliation witness or either compared claim.

This is intentionally a wrapper around the existing reconciliation projection algebra rather than a rewrite of it. The lower-level projection remains responsible for reconciliation-to-EvidenceState semantics; the target-bound layer is responsible for subject binding.

## Historical rule

A predecessor projection remains bound to its predecessor witness and its target. A successor witness may produce a new target-bound projection for the same target, but existence of the successor cannot retarget or mutate the predecessor projection.

## Adversarial coverage

TBP-001..012 cover:

1. exact target binding;
2. wrong target identity;
3. witness identity used as target;
4. compared claim identity used as target;
5. identical state values with different target identity;
6. historical predecessor target preservation;
7. successor projection for the same target;
8. successor projection cannot retarget the predecessor target;
9. Holochain-shaped target identity;
10. external authority and modality preservation;
11. target binding does not promote epistemic state;
12. explicit witness/target separation.

Qualification is semantic/provenance-only. It does not establish physical correctness, safety, certification, regulatory approval, physical equivalence, measurement truth, or authorship truth.

Holochain validation requires deterministic outcomes from explicitly addressable dependencies; unavailable dependencies remain unresolved rather than becoming an inferred result. Holochain documents these constraints directly. NIST's digital-thread research likewise emphasizes persistent identifiers and traceability across product-lifecycle data.