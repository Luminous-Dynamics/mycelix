# AC-016 — Adversarial Composition and Invariance Corpus

Status: implementation candidate; execution qualification not yet established.

## Purpose

AC-016 expands the AC-015 municipal adversarial corpus without adding any production API or authority surface. It targets failures that arise only when individually reasonable evidence or transformations are composed.

The governing rule is:

`locally valid inputs + composition != globally safe conclusion`

AC-016 therefore tests global contradiction detection, trust revocation, monetary attribution completeness, and deterministic output invariance.

## Scope

AC-016 adds one integration-test file only. Production source bytes remain identical to AC-015.

### 1. Transitive same-scheme identity contradiction

Two individually valid and independently qualified links are constructed:

- A = B under `ZA-CIPC / SUP-X`
- B = C under `ZA-CIPC / SUP-Y`

Each pair is structurally valid on its own. Their transitive closure would place A, B and C in one equivalence component containing two distinct identifiers under the same registry scheme.

AC-007 must reject that component with `ConflictingIdentifiers` unless an authoritative same-scheme crosswalk is present.

This establishes:

`pairwise qualification != permission for unchecked transitive closure`.

### 2. Challenge invalidates analytical aggregation

A previously corroborated link is moved into `Challenged` with an explicit challenge reference while preserving its historical evidence.

AC-008 may still verify the receipt envelope, but AC-007 must reject the link as not aggregation-eligible.

The expected behavior is:

`challenge -> preserve evidence history + remove current aggregation authority`.

No source node or historical link evidence is deleted.

### 3. Trust-root revocation precedes analysis

A deployment verifier rejects an otherwise structurally valid qualification receipt with stable code `receipt-revoked`.

AC-008 must stop before the equivalence engine is used and return `VerifierRejected`.

This models Xenia/Mycelix authority revocation as an external trust-root decision rather than a civic-types string convention.

### 4. Joint/multi-supplier monetary ambiguity

Two supplier award edges are supplied while the monetary snapshot attributes a joint upstream total to only one supplier.

AC-012 must report `MissingValueForAwardEdge` for the un-attributed supplier rather than assuming an equal split, assigning the total to one participant, or silently dropping that edge.

Together with AC-015's duplicate-award test, this establishes both sides of the rule:

- duplicating one joint total across suppliers is invalid;
- assigning the joint total to only one supplier while leaving another un-attributed is invalid.

A future consortium/joint-award semantic model must therefore be explicit rather than inferred.

### 5. Canonical ordering invariance

The same full AC-014 matrix is computed twice while independently reversing:

- AC-003 input edge order;
- AC-008 qualified identity-link order;
- AC-012 monetary-record order.

The entire `FullProcurementRobustnessMatrix` must remain byte-semantically equal under Rust equality, including:

- robustness envelope;
- exact bounds;
- scenario identifiers and observations;
- scenario lineage;
- equivalence-component/link references;
- monetary record lineage;
- qualification receipt references.

This establishes that source iteration order is not analytical authority.

## Epistemic boundary

AC-016 does not establish that any real supplier, registry, official, or procurement process is corrupt or fraudulent. The fixtures are synthetic adversarial controls.

A contradiction means only that an identity equivalence view is not currently safe to aggregate. A challenge or revocation means only that the current qualification is unusable. Missing monetary attribution means only that value-weighted concentration is not defined under AC-012 v1.

## Qualification gate

AC-016 is qualified only when the exact subject demonstrates:

1. integration tests compile and pass;
2. Rust formatting passes;
3. warnings-denied Clippy passes including integration tests;
4. transitive conflicting identifiers fail specifically at AC-007/AC-008 equivalence construction;
5. challenged links preserve history but lose aggregation eligibility;
6. verifier revocation fails before projection/metric calculation;
7. incomplete multi-supplier attribution fails at AC-012;
8. reversing edges, identity inputs, and value records yields an identical full AC-014 matrix;
9. no production-source file changed relative to AC-015.

A queued or unexecuted workflow is not a PASS.

## Next evidence tranche

After AC-016 executes successfully, prioritize fixture diversity rather than new authority:

- authoritative registry disagreement and later resolution through explicit crosswalk evidence;
- consortium and joint-venture award semantics;
- randomized deterministic permutations and mutation tests;
- real public OCDS release-package shapes with source licensing recorded;
- calibrated red-flag precision/recall studies using labeled synthetic and public benchmark corpora.
