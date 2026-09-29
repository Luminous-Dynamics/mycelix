# mycelix-forge-review-basis-currentness-trust-context

FORGE-007P makes the trust decision for provider-scoped review-basis currentness explicit and local to the exact MergeProtected review-basis finalization.

## Exact theorem

MergeProtectedReviewBasisQuorumV1
+ exact finalization_context
+ ReviewBasisCurrentnessTrustContextV1
+ ProviderVerifiedReviewBasisCurrentnessV1
+ exact provider namespace
+ exact verifier identity
+ exact coverage commitment
    -> MergeProtectedTrustedReviewBasisCurrentnessV1

The critical point is that the existing project-policy authentication trust substrate is not reused for currentness. The currentness verifier is trusted only when the exact MergeProtected quorum explicitly commits to the exact typed currentness trust context through its existing finalization_context.

## Trust context

ReviewBasisCurrentnessTrustContextV1 contains:

- the exact provider namespace;
- the exact currentness verifier identity;
- the exact provider coverage commitment.

Its canonical commitment must equal MergeProtectedReviewBasisQuorumV1.finalization_context().

This makes the governance decision explicit rather than silently treating a verifier as globally or project-policy trusted.

## Positive claim

The positive result binds:

- exact project and proposal;
- exact MergeProtected review-basis quorum;
- exact finalization context;
- exact provider-scoped currentness evidence;
- exact provider namespace;
- exact verifier identity;
- exact provider coverage.

It also requires the currentness observation to be no earlier than the protected quorum observation.

It does not prove:

- trusted wall-clock time;
- global currentness outside the provider's declared scope;
- source/repository/build/M0 correctness;
- merge authorization;
- execution authorization.

The positive type is serializable for evidence export but cannot be deserialized into authority.
