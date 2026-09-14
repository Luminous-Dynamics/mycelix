# Procedure-Policy Currentness-Provider Identity v0.1

## Purpose

The qualified ADMIN-002 currentness theorem (#804) intentionally receives a `ProcedurePolicyCurrentnessPolicy` from its caller. That policy selects the provider namespace and institutional authority allowed to attest which procedure policy is current.

This crate gives that **provider-selection policy** one canonical semantic identity.

It does not prove that the policy was adopted, is current, came from a trusted source, or grants any authority.

## CPID-001 — Verifier selection is itself policy

A provider namespace, provider institution/rulebook, required capability, accepted roles, and authority-evidence requirements are not neutral implementation details. They define who may make an authoritative closed-world currentness claim.

Therefore:

```text
qualified procedure-policy currentness under supplied verifier policy
!=
proof that the supplied verifier policy is governing
```

The verifier-selection policy must be independently identifiable before later adoption/currentness can be proven.

## CPID-002 — Canonical identity commits the complete target scope

The identity commits:

- currentness-policy protocol version;
- target institution;
- optional target jurisdiction;
- exact target rulebook ID/version/digest; and
- exact procedure profile.

Changing the administrative scope changes the identity.

## CPID-003 — Canonical identity commits the complete provider-selection scope

The identity commits:

- provider namespace;
- provider-authority institution;
- optional provider-authority jurisdiction;
- exact provider-authority rulebook ID/version/digest;
- required provider capability;
- canonical accepted-provider-role set; and
- canonical provider-authority evidence-requirement set.

Changing who may attest currentness changes the identity.

## CPID-004 — Set-like vectors have set semantics

The ordering of accepted provider roles is non-semantic.

The ordering of provider-authority evidence requirements is non-semantic.

Within each `EvidenceRequirement`, accepted issuer ordering is non-semantic.

Canonical identity therefore sorts these sets before hashing.

## CPID-005 — Duplicate semantic evidence is rejected

A duplicate accepted issuer inside one evidence requirement is rejected rather than silently deduplicated.

Two evidence requirements that become identical after canonical issuer ordering are rejected rather than becoming two encodings of one semantic requirement.

Existing #804 duplicate provider-role rejection remains authoritative.

## CPID-006 — Constructor bypass cannot create an identity

The inherited institutional ID wrappers are publicly constructible. Until generic institutional-core typed-ID hardening is fully converged, this identity layer independently validates every identity-bearing institution, jurisdiction, rulebook ID, procedure profile, capability, role, and accepted issuer string.

Malformed direct wrapper construction must fail identity qualification.

This local defense may become redundant after the generic core repair converges, but retaining it is harmless defense in depth.

## CPID-007 — Identity is not adoption/currentness

A `QualifiedCurrentnessProviderPolicyIdentity` proves only:

```text
these exact provider-selection semantics
→
this exact digest/profile
```

It does not prove:

- immutable record authenticity;
- institutional adoption;
- policy currentness/revocation state;
- provider verifier origin;
- procedure-policy currentness;
- administrative decision authority; or
- external-effect authority.

## CPID-008 — No recursive bootstrap

This crate does not ask #804 to prove its own verifier-selection policy current.

The next trust-root layer must use an independently anchored adoption/currentness path, such as the shared policy-evidence waist plus a governance/authority-rooted currentness theorem. A verifier-selection policy may not bootstrap itself by selecting the provider that declares it current.

## CPID-009 — Language-neutral identity profile

`PROFILE_V1.md` is normative. Rust is one implementation.

Any change to field inclusion, ordering, framing, optional-value encoding, set canonicalization, integer encoding, or hash/domain separation requires a new profile/domain.

## CPID-010 — No runtime/effect path

Pure Rust only. No HDK/Holochain, DHT, persistence, network, filesystem, ambient time, provider discovery, currentness lookup, authority evaluation, decision issuance, actuator, or external-effect path is introduced.
