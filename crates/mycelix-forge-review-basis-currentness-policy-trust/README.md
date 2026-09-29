# mycelix-forge-review-basis-currentness-policy-trust

FORGE-007P binds the provider-scoped review-basis currentness claim from FORGE-007O to the exact project policy and provider/verifier trust entry committed by that policy.

## Exact theorem

```text
MergeProtectedReviewBasisQuorumV1
+ ProviderVerifiedReviewBasisCurrentnessV1
+ exact ProjectPolicyStateV1
+ exact AuthenticationProviderTrustPolicyV1
+ project-policy binding
+ exact provider-namespace commitment
+ exact verifier-identity commitment
+ project-policy trust entry
    -> ProjectPolicyTrustedReviewBasisCurrentnessV1
```

The positive result is only a trust join. It does not strengthen the provider's declared coverage scope into a global current-state oracle.

## Why this layer exists

FORGE-007O deliberately leaves the currentness verifier outside project trust. Consuming that evidence directly as merge authority would therefore allow an arbitrary verifier identity to become an implicit root of trust.

FORGE-007P closes that gap by requiring the verifier identity to be admitted by the exact AuthenticationProviderTrustPolicyV1 bound to the exact project policy committed by the protected review-basis quorum.

The provider namespace and verifier identity are committed canonically before the trust lookup. Trust is therefore over an exact provider/verifier pair, not a human-readable name.

## Positive claim

The result proves:

- one exact project and proposal;
- one exact MergeProtectedReviewBasisQuorumV1;
- one exact provider-scoped currentness evidence commitment;
- one exact provider namespace commitment;
- one exact currentness verifier identity commitment;
- the exact project-policy state bound by the review basis;
- the exact provider-trust policy bound by that project policy;
- the currentness verifier is explicitly trusted by that policy.

It does not prove:

- trusted wall-clock time;
- global repository or review currentness;
- absence of observations outside the provider's declared coverage;
- source/build/M0 correctness;
- merge authorization;
- execution authorization.

The resulting type is serializable for evidence export but cannot be deserialized into positive authority.
