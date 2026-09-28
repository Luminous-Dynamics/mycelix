# mycelix-forge-review-basis-currentness

FORGE-007O creates a narrow provider-scoped currentness evidence boundary for the review basis selected by FORGE-007N.

The theorem is:

    MergeProtectedReviewBasisQuorumV1
    + exact provider-scoped currentness observation
    + independent currentness verifier
        -> ProviderVerifiedReviewBasisCurrentnessV1

The provider observation is deliberately not a global current-state oracle. Its scope is an explicit opaque commitment supplied by the provider and independently accepted by the verifier.

The positive result means only:

    provider verifier accepted that the exact selected review basis was current
    within the exact declared coverage scope at the provider's observation time.

It does not mean:

- globally latest review state;
- absence of observations outside the declared coverage scope;
- trusted wall-clock time;
- merge authorization;
- source/repository/build correctness;
- M0 qualification;
- execution authorization.

The observation cannot be converted into the positive type by deserialization. The exact FORGE-007N quorum evidence commitment is rechecked before provider verification.

The provider namespace is retained explicitly so a later project-policy trust theorem can bind the currentness verifier identity to the exact provider contract rather than treating verifier identity as globally trusted.
