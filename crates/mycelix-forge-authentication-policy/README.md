# Mycelix Forge Authentication Policy

This crate binds provider-verified Xenia authentication to the exact authentication-provider trust policy committed by a Forge proposal.

It exists to prevent capability-specific layers from independently reimplementing the same trust join.

```text
EvidenceBoundPrincipalAuthentication
+ ProviderVerifiedXeniaAuthenticationV1
+ exact ProjectPolicyStateV1 committed by proposal
+ exact AuthenticationProviderTrustPolicyV1
+ exact (Xenia namespace, verifier identity) trust membership
    ↓
ProjectPolicyTrustedXeniaAuthenticationV1
```

The theorem is capability-neutral. `ReviewSource`, `Witness`, `MergeProtected`, `QualifyBuild`, `Release`, recovery, and future capabilities retain the exact capability encoded in the authentication request.

## Non-claims

A positive result does **not** establish:

- capability eligibility in `AuthorityEpoch`;
- capability threshold satisfaction;
- trusted wall-clock time;
- review or witness quorum;
- merge/build/release authorization;
- repository or execution correctness.

Those remain separate Forge layers.

## Important failure cases

The join rejects:

- project mismatch;
- authority-epoch mismatch with the proposal;
- non-Xenia provider namespace in the principal binding;
- provider verification for another authentication observation;
- project-policy state not committed by the proposal;
- provider-trust policy not bound by that project-policy state;
- cryptographically valid Xenia evidence from a verifier not trusted by the exact project policy.

The positive type is serializable for evidence export but intentionally not deserializable into authority.
