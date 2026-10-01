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

## MOBILITY-COMMONS-022 scope extension

Target identity does not by itself establish configuration scope. A projection therefore also carries an explicit ConfigurationRevision identity. Validation receives the expected scope separately and requires exact equality.

This prevents a projection from silently crossing configuration revisions while preserving the distinction between target identity, configuration identity, and reconciliation-witness identity. A deliberate configuration transition is represented by a new projection and explicit configuration lineage rather than inferred from timestamps or protocol identifiers.

The executable corpus is extended to TBP-001..018 with wrong-scope, witness-as-scope, Holochain-scope, declaration/supplied-scope mismatch, and explicit-new-scope cases.


## MOBILITY-COMMONS-023 physical-artifact applicability extension

Target binding plus configuration scope still leaves a semantic gap: a configuration revision can be named without identifying the physical artifact instance to which the projection applies.

This extension therefore adds:

- `physical_artifact_scope`, which must be a typed `PhysicalArtifact` identity;
- an explicit `LineageEdge` with relation `AppliesTo`;
- exact equality between the supplied configuration scope and the edge source;
- exact equality between the supplied physical-artifact scope and the edge target.

The only permitted applicability direction is:

`ConfigurationRevision -> PhysicalArtifact`

The binding is provenance/applicability metadata. It does not establish physical equivalence, engineering correctness, safety, certification, regulatory validity, or measurement truth.

The executable corpus is extended to 26 deterministic cases, including reversed-edge, wrong-relation, identity-substitution, Holochain-shaped identity, and target-mismatch probes. The independent Rust and Python evaluators must produce identical normalized results.

This preserves the central boundary: configuration identity and physical-artifact identity remain distinct, and applicability must be explicit rather than inferred.
