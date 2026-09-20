# FORGE-007 — Portable review attestations

`mycelix-forge-review` defines the exact decision a reviewer makes over one immutable `ChangeProposalId`, then keeps three trust layers separate.

```text
ReviewStatement
      ↓
FORGE-005A evidence cross-link
      ↓
EvidenceBoundReviewAttestation
      ↓
ReviewSource eligibility in exact AuthorityEpoch
      ↓
StructurallyEligibleReview
```

None of those names imply provider cryptography, trusted time, quorum, or merge authority unless a later layer proves it.

## Review statement identity

A `ReviewStatement` binds:

- exact `ChangeProposalId`;
- reviewer `PrincipalId`;
- `Approve` or `RequestChanges`;
- immutable review-context commitment.

Ordinary comments/discussion are deliberately outside the security-significant decision vocabulary.

The statement does **not** repeat project/authority/repository-policy fields already committed by the proposal. That avoids a second stale-context surface.

Changing the decision or review context changes `ReviewStatementId`.

## Exact authentication action subject

FORGE-005A evidence used for a review must be scoped to `Capability::ReviewSource` and its `action_subject` must equal the exact `ReviewStatementId`.

```text
proposal
  ↓
ReviewStatement(Approve, context X)
  ↓
ReviewStatementId
  ↓
PrincipalAuthenticationRequest.action_subject
```

A signature/authentication over only the proposal id is insufficient because it would not distinguish approval from request-changes or context changes.

## Critical claim boundary

`EvidenceBoundReviewAttestation` proves structural cross-links to FORGE-005A evidence. FORGE-005A provider evidence is intentionally opaque, so this type is **not** named `AuthenticatedReviewAttestation` or `SignedReview`.

A concrete provider verifier such as the Xenia FORGE-005B chain must establish the cryptographic truth of that evidence separately.

Likewise, `StructurallyEligibleReview` proves only that the supplied `AuthorityEpoch` considers the reviewer eligible for `ReviewSource` at a caller-supplied time. It does not establish trusted time or threshold/quorum satisfaction.

## Tests

The tranche covers:

- decision/context mutation changing statement identity;
- exact review-statement action-subject binding;
- wrong authentication capability;
- reviewer substitution;
- proposal mutation invalidating old review;
- structural eligibility inside/outside authority validity;
- evidence-bound outsider remaining ineligible;
- explicit demonstration that arbitrary opaque provider commitments can satisfy structural 005A binding but do not prove provider cryptography;
- statement serde/identity stability.

## Next

The next authority tranche should count only distinct, provider-verified, structurally eligible `Approve` reviews toward the exact `ReviewSource` threshold and must reject duplicates/mixed proposal or authority subjects.
