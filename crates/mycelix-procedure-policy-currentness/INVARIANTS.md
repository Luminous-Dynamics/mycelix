# Procedure Policy Currentness v0.1 Invariants

This crate composes qualified canonical policy identity with explicit provider authority and an explicit closed-world currentness claim.

## Core theorem

`canonical policy identity != provider-qualification profile != authorized provider != current governing policy != administrative decision authority != external effect authority`

A positive `QualifiedCurrentProcedurePolicy` proves only:

1. the policy's semantic identity was independently qualified;
2. the exact provider claim targets that identity and procedure scope;
3. the provider principal/grant is explicitly authorized under the supplied frozen currentness profile;
4. the policy is effective at the queried `as_of_ms`; and
5. the provider's authoritative namespace is explicitly closed through `as_of_ms`.

The token grants no administrative decision authority and no external-effect authority.

## Currentness-policy adoption boundary

`ProcedurePolicyCurrentnessPolicy` is a qualification profile, not a self-authorizing constitution.

This crate proves that a provider is authorized **under that exact supplied profile**. It does not independently prove that the target institution adopted that profile as its governing provider-selection rule.

A later administrative/constitutional composition layer must capture or otherwise authoritatively bind the exact currentness-policy profile before consequential decisions rely on it. This prevents the caller from choosing a convenient provider rule and then treating successful qualification as proof that the institution selected that rule.

Therefore:

`qualified under policy P != institution adopted policy P`.

## No local-absence inference

There is intentionally no API of the form:

`[] -> no challenge / no newer policy / current policy`

Currentness requires a positive provider claim containing `closed_through_ms`. A receipt closed through time `T` cannot establish currentness at `T + 1`.

`last record observed locally != closed-world currentness`.

## Target authority != provider authority

The currentness policy binds two independent scopes:

- target administrative institution/jurisdiction/rulebook/procedure profile; and
- provider-authority institution/jurisdiction/rulebook/capability/roles/evidence.

They may be different. A records office, registry service, or shared public infrastructure provider can be authorized to attest which policy is current without acquiring authority to decide the underlying administrative case.

## Provider claim bindings

A candidate `ProcedurePolicyCurrentnessClaim` binds:

- exact target institution;
- exact target jurisdiction;
- exact target rulebook;
- exact procedure profile;
- canonical policy digest/profile;
- publication reference;
- provider namespace;
- provider principal;
- exact provider authority-grant ID;
- non-zero provider generation;
- non-zero provider-state digest;
- effective interval;
- closed-through time;
- claim issuance time; and
- host-verified currentness proof reference.

The pure kernel does not implement signature/transport verification. Hosts must verify grant/proof references against their exact corresponding objects before treating those references as authentic evidence.

## Temporal rules

- `effective_from_ms > 0`.
- `effective_until_ms`, when present, is strictly greater than `effective_from_ms`.
- effective intervals are half-open: `[from, until)`.
- `closed_through_ms > 0`.
- `issued_at_ms > 0`.
- `closed_through_ms <= issued_at_ms`.
- currentness at `as_of_ms` requires `as_of_ms <= closed_through_ms`.
- currentness at `as_of_ms` also requires the policy effective interval to contain `as_of_ms`.
- provider authority is evaluated at claim issuance time.
- authority evidence observed after claim issuance fails closed.

A currentness receipt may therefore prove a historical `as_of_ms` when issued later by a provider that is authorized at issuance and whose authoritative state is closed through that historical time.

## Exact identity rule

The provider claim must bind the exact canonical digest/profile produced by the qualified policy-identity parent. Content-address substitution and identity-profile substitution fail closed.

The policy locator carried by the identity token may differ from `publication_ref`; mirrors and authoritative registry record locations are provenance, not semantic identity.

## Provider authority

Provider authority is evaluated with `mycelix-institutional-core::evaluate_authority` against the frozen currentness policy.

The exact provider grant ID and holder must equal the provider claim. Required provider capability, role constraints, authority rulebook, jurisdiction, institution, and evidence requirements are evaluated separately from the target administrative rulebook.

Advisory scores, reputation, models, local cache state, or bare registry presence cannot satisfy provider authority.

## Generation boundary

`provider_generation` is bound as evidence and must be non-zero, but v0.1 does not infer maximality merely from the integer. Closed-world maximality/currentness comes from the provider's externally verified currentness proof over its exact namespace/state and `closed_through_ms`.

Thus:

`largest generation seen locally != authoritative latest generation`.

## Positive-token boundary

`QualifiedCurrentProcedurePolicy` is deliberately not Clone/Serialize/Deserialize. Persisted currentness must be reconstructed from:

`qualified content identity + frozen provider policy + exact provider claim + provider authority + authority evidence + as-of time`.

## Parent preservation

This tranche is rooted directly at qualified policy-identity exact head `47c6738ddba752f5ba366d6f775902acaad2f422`. CI pins the parent identity crate's manifest, implementation, invariant specification, and normative language-neutral profile blobs.

Currentness is additive composition. It must not rewrite the canonical identity theorem beneath it.
