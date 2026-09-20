# Mycelix Forge authenticated review revisions

This crate authenticates the exact lineage-aware review revision defined by `mycelix-forge-review-state` and carries that subject through Forge's existing Xenia and project-policy trust boundaries.

## Positive chain

```text
ReviewRevisionV1
  -> exact ReviewRevisionId
  -> PrincipalAuthenticationRequest.action_subject
  -> FORGE-005A evidence binding
  -> ReviewSource structural eligibility
  -> Xenia provider verification
  -> exact project-policy provider trust
  -> ProjectPolicyTrustedReviewRevisionV1
```

The security-significant property is that `sequence`, `previous`, `decision`, proposal identity, reviewer identity, and review context all participate in `ReviewRevisionId`. They therefore cannot be attached to an already-authenticated review after the fact.

## Positive types

- `EvidenceBoundReviewRevisionV1` — exact FORGE-005A evidence names the exact review revision.
- `StructurallyEligibleReviewRevisionV1` — the revision author is eligible for `ReviewSource` in the supplied authority epoch at the supplied observation time.
- `ProviderVerifiedReviewRevisionV1` — the exact authentication observation crossed the Xenia provider-crypto boundary.
- `ProjectPolicyTrustedReviewRevisionV1` — the exact Xenia verifier identity is trusted under the exact provider namespace by the exact project policy committed into the proposal.

All positive types are constructed through checked functions; serialized evidence is not itself authority.

## Critical non-claims

A `ProjectPolicyTrustedReviewRevisionV1` does **not** prove:

- that the revision is the globally current review head;
- that no later sibling or successor exists;
- completeness of a supplied revision set;
- trusted wall-clock time;
- approval quorum;
- repository or build correctness;
- merge authorization.

Currentness requires a separate completeness/snapshot theorem from an append-only collaboration-state provider.

## Adversarial cases

Tests cover:

- full exact revision authentication through project-policy trust;
- sibling decision substitution;
- predecessor-lineage substitution;
- provider-observation borrowing across authentication requests;
- cryptographically valid evidence from a verifier not trusted by project policy.

## Qualification

The dedicated workflow pins Rust 1.96 and runs formatting, warnings-denied Clippy, and all-feature tests.