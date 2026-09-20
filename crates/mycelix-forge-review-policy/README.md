# Mycelix Forge Project-Policy Trusted Review

This crate joins provider-verified review evidence to the authentication-provider trust policy committed by the exact immutable proposal being reviewed.

It exists because these claims are different:

```text
Xenia signatures verify
    != Xenia verifier trusted by this project
    != reviewer eligible
    != review quorum
    != merge authorization
```

## Positive theorem

`ProjectPolicyTrustedReviewV1` exists only when all of the following agree:

- the provider-verified review names the exact proposal;
- the review and proposal bind the same authority epoch and project;
- the supplied `ProjectPolicyStateV1` hashes to the exact project-policy commitment in the proposal;
- that project-policy state binds the exact supplied `AuthenticationProviderTrustPolicyV1`;
- the exact pair `(xenia_provider_namespace, verifier_identity)` is present in that provider-trust policy.

The positive type is serializable for evidence export but not deserializable into authority. Consumers must re-run the trust join.

## Anti-substitution properties

A cryptographically valid review is rejected when:

- its verifier is not admitted by project policy;
- the verifier is admitted under another provider namespace;
- the supplied provider-trust policy is not the one bound by the project-policy state;
- the supplied project-policy state is not the one committed by the proposal;
- the review belongs to an older proposal whose project policy has since changed.

## Verifier rotation

Verifier rotation is proposal-significant:

```text
new trusted verifier set
    ↓
new AuthenticationProviderTrustPolicyV1
    ↓
new ProjectPolicyStateV1
    ↓
new ChangeProposalId
    ↓
old ReviewStatementId no longer applies
```

This prevents a review authenticated under one provider-trust regime from being silently reinterpreted under another.

## Non-claims

`ProjectPolicyTrustedReviewV1` still does not prove:

- trusted wall-clock time;
- that enough distinct approvals exist simultaneously;
- repository correctness;
- build or CI qualification;
- protected-history compliance;
- merge authorization.

Those remain later, separate theorem boundaries.
