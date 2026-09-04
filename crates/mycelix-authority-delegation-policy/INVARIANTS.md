# Mycelix Authority Delegation Policy v0.1 — Normative Invariants

Status: **pure semantic qualification kernel; not a runtime authority source**

This crate answers one narrow question:

> Does one exact, currently fresh institutional parent grant currently possess an independently adopted, currently fresh policy that permits this bounded delegation scope?

It does not create grants, resolve current DHT state, verify cryptography itself, persist policy, or execute effects.

## 1. Possession is not delegability

A parent grant containing capability `X` does not imply that the holder may delegate `X`.

Delegation authority exists only through an explicit `DelegationPolicy` whose institutional source and exact policy proof are independently verified.

## 2. Exact parent authority

A policy binds:

- exact parent grant ID;
- exact canonical parent grant digest under `mycelix-authority-grant-v1-blake3-framed-semantic`;
- exact parent freshness generation;
- exact parent holder as delegator;
- exact institution;
- exact jurisdiction; and
- exact rulebook ID/version/digest.

A same-ID parent with changed authority bytes is another policy domain.

A parent generation advance makes the old policy stale until an explicitly re-bound policy is adopted.

## 3. Policy can only attenuate parent authority

`delegable_roles` must be a subset of parent roles.

`delegable_capabilities` must be a subset of parent capabilities.

The policy may narrow delegate identity, lifetime, role/capability scope, and re-delegation permission. It may never expand parent authority.

## 4. Delegate scope is explicit

`DelegateScope::AnyPrincipal` is an explicit policy decision.

An empty allow-list is not interpreted as “anyone.”

`DelegateScope::AllowList` must contain one or more unique, valid principals and is bounded to 256 entries in v0.1.

## 5. Re-delegation is explicit authority

`allow_redelegation` is semantic and identity-bearing.

A child created under a policy with `allow_redelegation = false` must not later appear as the parent of another current delegation edge.

The lineage layer is responsible for enforcing this on adjacent edges.

## 6. Child lifetime is bounded

Every child delegation checked against a qualified policy must:

- have a non-zero valid interval;
- be issued no later than the qualification time;
- still be active at qualification time;
- begin no earlier than policy validity;
- end no later than policy validity; and
- have duration no greater than `max_child_lifetime_ms`.

The policy itself must fit entirely inside the parent grant lifetime.

## 7. Policy source cannot be circular delegation

A v0.1 `DelegationPolicy` may not use `AuthoritySourceKind::Delegation` as the source that makes delegation itself lawful.

This prevents a delegation from directly bootstrapping the policy that authorizes delegation.

Other source kinds remain subject to independent host/institutional verification.

## 8. Semantic policy identity is canonical

`DelegationPolicy.identity_digest()` commits:

- registered delegation-policy profile;
- registered canonical parent-grant profile;
- exact policy ID;
- exact institution/jurisdiction/rulebook;
- exact parent ID/digest/generation;
- exact delegator;
- canonical role set;
- canonical capability set;
- exact delegate scope;
- re-delegation permission;
- child lifetime ceiling;
- policy validity interval;
- exact institutional source kind/reference/proof; and
- exact policy proof reference.

Role, capability, and allow-list ordering is non-semantic. Exact duplicates are invalid rather than silently normalized.

## 9. Policy freshness is independently revocable

Expiry is not sufficient current-authority control.

A `VerifiedDelegationPolicy` must carry current generation-bound freshness for the exact policy semantic identity.

The v0.1 freshness category is `AuthoritySubjectKind::Delegation`; the exact identity profile distinguishes a delegation policy from a delegation attestation, so the two cannot collide.

A revoked or superseded policy cannot qualify even while its immutable policy object and cryptographic proof remain historically valid.

## 10. Parent freshness and policy freshness are composed

Current delegation authority requires the exact closed set:

- current parent-grant freshness; and
- current delegation-policy freshness.

`qualify_current_freshness` must positively qualify that exact set under the registered freshness bundle profile.

Verifier invocation timestamps are not current-authority identity.

## 11. Re-verification is not re-authorization

Refreshing proof/verification metadata for unchanged parent and policy generations must preserve the resulting `DelegationAuthorityRef`.

A policy-generation change must change it.

A parent-generation change must invalidate the old semantic policy because the policy binds the exact parent generation.

## 12. Current authority ref prevents resurrection

`DelegationAuthorityRef` binds:

- exact semantic policy ID/digest/profile;
- exact parent generation;
- exact policy freshness generation;
- exact current delegation-authority digest; and
- exact current delegation-authority profile.

The current delegation-authority digest commits the semantic policy digest plus the exact current parent+policy freshness bundle.

If a policy is revoked and later re-authorized under a new generation, the old authority reference does not become current again.

A later delegation attestation must therefore bind the exact current `DelegationAuthorityRef` under which it was issued.

## 13. Qualified authority is not application data

`QualifiedDelegationAuthority` has private fields and intentionally does not implement `Deserialize`.

Callers must obtain it through `qualify_delegation_authority`.

The qualified object may validate a proposed child scope; it does not itself create a child grant or delegation attestation.

## 14. Historical validity remains separate

Current policy revocation must block new delegation while preserving the ability to prove that an older delegation was valid under the policy generation effective at its event time.

Historical verification therefore requires as-of generation resolution and must not call the current-authority path as a substitute.

## 15. Advisory signals cannot create delegation authority

Phi, reputation, stake, model recommendations, consciousness metrics, and other advisory scores are not inputs.

They cannot adopt a delegation policy, select a generation, expand policy scope, restore revoked authority, or authorize re-delegation.

## 16. Runtime follow-on remains fail-closed

A future runtime provider must independently establish:

- exact parent grant record and grant proof;
- exact canonical parent identity;
- one authoritative current parent generation/state;
- exact delegation policy record;
- exact policy proof;
- exact policy institutional-source proof; and
- one authoritative current policy generation/state.

Missing provider, ambiguous current state, revoked subject, invalid proof, mismatched generation, or expired freshness lease must deny.

This crate does **not** enable external effects.