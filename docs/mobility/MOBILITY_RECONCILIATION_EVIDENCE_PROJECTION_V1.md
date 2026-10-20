# MOBILITY Evidence Projection V1

## Purpose

This contract binds a temporal reconciliation witness to the existing orthogonal `EvidenceState` tuple without converting reconciliation into engineering truth.

The projection layer is semantic/provenance-only. It does not establish physical safety, certification, regulatory approval, physical equivalence, measurement truth, or engineering correctness.

## Projection effects

### ConflictReferenceOnly

Allowed only for a `Conflicting` reconciliation witness.

It records the witness reference in `conflict_reference` and preserves the existing epistemic disposition. If the witness is disputed, `ConflictDisposition::Disputed` is preserved.

It MUST NOT create `contradiction_reference` or promote the state to `Contradicted`.

### LifecycleSupersession

Allowed only for a `Superseded` witness.

It sets `LifecycleDisposition::Superseded` while leaving epistemic disposition unchanged. Supersession preserves predecessor history; it is not deletion and does not assert that the predecessor was false.

### IndeterminateReference

Allowed only for an `Indeterminate` witness.

With no dependency reference, the resulting epistemic disposition is `Indeterminate`. With an explicit unresolved dependency reference, the resulting disposition is `Unresolved`.

Neither state means failed, unsafe, false, or contradicted.

### NoEpistemicPromotion

Allowed for `Coexistent`, `Sequential`, or `Incomparable`.

The projection records that the reconciliation witness was considered without promoting it into an epistemic conclusion.

- Coexistent does not imply Supported.
- Sequential does not imply causality.
- Incomparable does not imply disagreement.

## Invariants

1. Conflicting != Contradicted.
2. Disputed != Contradicted.
3. Superseded affects lifecycle, not epistemic disposition.
4. Indeterminate != failed.
5. Unresolved != contradicted.
6. External authority provenance is not changed by this projection.
7. Evidence modality is not changed by this projection.
8. A reconciliation witness must validate independently before projection.
9. Projection compatibility is checked against the witness classification.
10. A projection cannot manufacture a safety, certification, regulatory, physical-equivalence, measurement-truth, or engineering-correctness claim.

## Holochain boundary

This contract is intentionally compatible with deterministic validation semantics: validation can depend on addressable evidence, while unavailable dependencies remain unresolved rather than becoming invalid by inference. Holochain's own validation model distinguishes definitive valid/invalid outcomes from unresolved dependencies and requires deterministic validation. The reconciliation projection therefore stores explicit references instead of treating absence of retrievable information as contradiction.

## Qualification boundary

This is a semantic/provenance contract only.
