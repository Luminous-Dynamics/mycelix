# FORGE-005A — authenticated-principal contract

This crate defines the provider-neutral binding between an external authentication system and Mycelix Forge authority subjects.

It is deliberately **crypto-free and I/O-free**. A provider adapter such as Xenia supplies cryptographic verification and replay/freshness enforcement; this crate only proves that those external evidence commitments are bound to the exact Forge subjects the verifier intended.

## Exact request subject

A `PrincipalAuthenticationRequest` commits to:

- project identity;
- exact authority epoch;
- stable Forge `PrincipalId`;
- exact provider/logical-identity/key-lineage binding;
- capability scope;
- exact action/change/release subject;
- 32-byte verifier challenge.

Changing any of those fields changes the request commitment.

## Stable identity vs key lineage

`PrincipalBinding` separates:

```text
provider namespace
provider logical identity
Forge PrincipalId
current key-lineage commitment
```

This lets an external provider rotate or recover keys without forcing Forge to reinterpret the logical authority principal as a new person. Old authentication evidence is nevertheless bound to the old key-lineage commitment and cannot be reused against the replacement binding.

## Observation boundary

`AuthenticationObservation` carries three exact commitments:

- the request the provider claims to have verified;
- provider-specific cryptographic evidence;
- freshness/challenge-consumption evidence.

The observation is raw evidence, not self-authenticating authority.

`bind_principal_authentication(...)` produces `EvidenceBoundPrincipalAuthentication` only when:

- project == authority-epoch project;
- authority-epoch commitment == the supplied epoch;
- binding Forge principal == requested principal;
- binding commitment == the supplied binding;
- provider observation == the exact request.

## Explicit non-claims

FORGE-005A does **not** establish:

- signature validity;
- Xenia/DID/provider authenticity;
- one-time challenge consumption;
- principal eligibility for the requested capability;
- threshold/quorum satisfaction;
- review, merge, release, or governance authorization.

Authentication and authorization stay separate. In particular, a cryptographically authenticated principal may still be unauthorized under the active `AuthorityEpoch`.

## Next tranche

FORGE-005B should adapt Xenia's existing typed-signing model to this contract. The expected shape is:

```text
Forge exact authentication request
        ↓
Xenia typed transcript reconstruction
        ↓
Ed25519 AND ML-DSA verification
        ↓
Xenia logical-identity + key-lineage validation
        ↓
single-use challenge evidence
        ↓
AuthenticationObservation
        ↓
EvidenceBoundPrincipalAuthentication
```

No Forge layer should accept caller-supplied arbitrary signing bytes or treat an Xenia session token as Forge authority by itself.
