# REGEN-011B3 — Biomass Mass Evidence Firewall v1

Status: preregistration refinement only. This document closes the numeric-evidence gap between PEF measurements and exact biomass reservation/accounting mass.

## 1. Problem

PEF v1 intentionally represents scalar measurements as:

```rust
pub struct Measurement {
    pub value: f64,
    pub unit: String,
}
```

where `unit` is an opaque identifier and UCUM-compatible strings are merely preferred.

REGEN-011B1 separately proposes exact biomass accounting as:

```rust
pub struct MassMg(u64);
```

These are not automatically equivalent representations.

Therefore:

```text
valid admitted PEF mass-like measurement
!= exact canonical MassMg
```

## 2. No implicit scalar reinterpretation

The first biomass core MUST NOT implement a generic conversion such as:

```rust
MassMg((observation.measurement.value * unit_factor) as u64)
```

because that would silently combine:

- floating-point representation;
- unit interpretation;
- rounding/truncation policy;
- range checking;
- uncertainty handling;
- potentially unsupported unit semantics.

No `as u64` cast, saturating conversion, absolute-value conversion, or implicit rounding belongs in the core evidence path.

## 3. Accounting mass is an explicit assertion

The initial biomass core may carry exact accounting mass only as an explicit assertion bound to its derivation/evidence context.

Prefer a shape such as:

```rust
pub struct BiomassMassAssertion {
    pub mass: BiomassMass,
    pub derivation_ref: String,
    pub evidence_bindings: Vec<BiomassEvidenceBinding>,
}
```

`derivation_ref` identifies the exact external or separately qualified procedure that produced the canonical ledger mass.

The biomass core validates structure and scope. It does not claim to reproduce that procedure unless a later normalization theorem is implemented.

## 4. Snapshot correction

The REGEN-011B conceptual snapshot should therefore be interpreted/refined as:

```rust
pub struct BiomassStateSnapshot {
    pub lot_id: BiomassLotId,
    pub snapshot_ref: String,
    pub mass_assertion: BiomassMassAssertion,
    pub evidence: Vec<BiomassEvidenceBinding>,
}
```

rather than treating a bare `quantity: BiomassMass` as self-justifying evidence truth.

The exact accounting mass still drives reservation arithmetic, but its evidence/derivation provenance remains visible.

## 5. Mass-basis evidence role must match

A valid `BiomassMassAssertion` must include at least one mass-role binding consistent with its basis:

```text
AsReceived -> AsReceivedMass evidence/derivation role
DryMatterEquivalent -> DryMatterMass evidence/derivation role
```

A dry-matter accounting mass cannot be justified solely by an as-received mass observation without an explicit separate conversion derivation.

## 6. Evidence role multiplicity

Do not reject all duplicate role categories.

Some biomass propositions legitimately need multiple observations under one high-level role, for example:

- composition measurements for multiple constituents;
- multiple contamination analytes;
- replicate mass observations;
- multiple source-occurrence observations.

The initial uniqueness theorem should therefore be based on exact binding identity rather than `role` alone.

At minimum, duplicate exact tuples such as:

```text
(role, observation_id, phenomenon, expected_class)
```

must be rejected, while distinct observations may share a role when the domain contract permits it.

This supersedes any broad `DuplicateEvidenceRole` invariant in REGEN-011B.

## 7. No exactness inflation

`MassMg` being an exact integer type does not mean the underlying physical mass is known without uncertainty.

```text
exact ledger scalar
!= exact physical truth
```

PEF uncertainty remains PEF uncertainty.

The accounting layer must not erase or reinterpret it as zero merely because reservation arithmetic itself is exact.

## 8. Reservation capacity remains conservative planning state

REGEN-011B2 reservation capacity may consume the exact accounting mass asserted by an eligible assessment.

That capacity remains a planning bound under that assessment, not a statement that the real physical lot mass is known perfectly.

A downstream policy may choose a more conservative reservable mass than the nominal accounting assertion.

The biomass core must permit that narrowing.

## 9. Future automatic normalization requires a separate theorem

If later versions automatically derive `MassMg` from PEF measurements, that work requires a separately reviewed normalization profile defining at least:

- supported source units;
- exact unit-to-milligram scale factors;
- finite/non-negative requirements;
- overflow behavior;
- accepted floating-point representations;
- rounding policy, preferably fail-closed rather than implicit rounding;
- uncertainty treatment;
- treatment of absent measurements;
- treatment of Reported vs Observed vs lineaged Derived products;
- reproducibility/version identity of the normalization algorithm.

Until that theorem exists, arbitrary PEF scalar measurements do not mint exact ledger mass.

## 10. Suggested v1 constructor boundary

Conceptually:

```rust
impl BiomassMassAssertion {
    pub fn new(
        mass: BiomassMass,
        derivation_ref: impl Into<String>,
        evidence_bindings: Vec<BiomassEvidenceBinding>,
    ) -> Result<Self, BiomassError>;
}
```

The constructor validates:

- bounded non-empty derivation reference;
- non-empty evidence binding set;
- exact binding uniqueness;
- at least one basis-compatible mass evidence role;
- every binding is structurally valid.

It does not parse arbitrary PEF units or compare an `f64` scalar to `MassMg` in v1.

## 11. Added error distinctions

The first executable biomass error taxonomy should distinguish at least:

```rust
MissingMassEvidence,
MassEvidenceBasisMismatch,
DuplicateEvidenceBinding,
InvalidMassDerivationReference,
```

A future automatic normalizer would add its own explicit unit/rounding/range errors rather than reusing these structural failures.

## 12. Added regression requirements

The executable campaign gains at least:

1. bare `MassMg` cannot form a snapshot without a mass assertion;
2. empty derivation reference fails;
3. no evidence bindings fails;
4. AsReceived mass with only DryMatterMass role fails;
5. DryMatterEquivalent mass with only AsReceivedMass role fails;
6. exact duplicate evidence binding fails;
7. two distinct composition observations under the same role can coexist;
8. two distinct contamination observations under the same role can coexist;
9. admitted PEF `measurement=None` cannot be converted into mass implicitly;
10. arbitrary PEF `f64 + unit` cannot call a hidden generic `MassMg` converter;
11. exact accounting arithmetic preserves PEF uncertainty rather than replacing it;
12. reservation capacity may narrow below the asserted accounting mass but may never silently exceed it.

## 13. Deliberate non-claims

This refinement does not prove weighing-device calibration, unit correctness, physical mass accuracy, uncertainty sufficiency, feedstock eligibility, ecological sustainability, contamination safety, process safety, execution authority, or physical actuation.

Its proposition is narrow:

> exact biomass ledger mass and PEF scalar evidence remain distinct representations unless an explicit, versioned derivation/normalization theorem binds them.
