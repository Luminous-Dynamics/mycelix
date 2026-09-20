# Mycelix Forge Project Policy

`mycelix-forge-project-policy` defines the portable typed project-policy state used by Forge protocol v1.

The first policy dimension made explicit here is authentication-provider verifier trust. This is intentionally separate from cryptographic verification itself:

```text
provider signature verifies
    != verifier trusted by this project
    != reviewer eligible
    != review quorum
    != merge authorization
```

## Authentication provider trust

`AuthenticationProviderTrustPolicyV1` contains a canonical, duplicate-free set of exact:

```text
(provider_namespace, verifier_identity)
```

pairs.

The provider namespace distinguishes authentication contracts. A verifier identity trusted under one provider namespace is not automatically trusted under another.

Public construction canonicalizes ordering; serialized input must already be in strict canonical order. This prevents multiple accepted wire encodings from silently normalizing to the same authority object.

## Project policy state

`ProjectPolicyStateV1` binds the exact authentication-provider trust policy into a version-linked project-policy lineage:

```text
project
+ sequence
+ previous project-policy commitment
+ exact provider-trust-policy commitment
    ↓
ProjectPolicyStateV1
```

Sequence zero has no predecessor. Every later state must commit its predecessor.

Changing a trusted verifier changes the provider-trust commitment, which changes the project-policy-state commitment.

## Proposal binding

FORGE-006A changes `ChangeProposal::new` so production construction consumes a typed `ProjectPolicyStateV1`, alongside the existing typed `AuthorityEpoch` and `RepositoryPolicyState`.

All three objects must belong to the same project. The proposal derives their commitments itself.

Therefore:

```text
trusted verifier rotation
    ↓
new provider trust policy
    ↓
new project policy state
    ↓
new ChangeProposalId
    ↓
old review statements no longer apply
```

## Non-claims

This crate does not prove that:

- a provider signature is valid;
- a particular review used a project-trusted verifier;
- time is trustworthy;
- a reviewer is eligible;
- review quorum exists;
- repository evidence is correct;
- a proposal may be merged.

Those remain separate Forge theorem boundaries.
