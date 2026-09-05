# Authority-State Bootstrap Root v0.1 — Invariants

## Purpose

The operational authority-state coverage plane cannot be allowed to prove its own currentness.

A runtime dependency of:

`challenge -> current coverage policy -> challenge`

is a trust-bootstrap cycle, not a valid security architecture.

This crate introduces one deliberately narrow root whose current authority comes from the already-qualified DNA-bound constitutional plane.

## Root of currentness

A positive `QualifiedAuthorityStateBootstrapRoot` requires all of the following:

1. an exact `AuthorityStateBootstrapRootManifest`;
2. an independently verified **current** `ConstitutionStatement` from the DNA-bound constitution/transition plane;
3. exact recomputation of the constitutional statement digest;
4. exact network, institution, constitution version, rulebook ID/version/digest/profile agreement;
5. independently verified rulebook adoption of the exact root-manifest digest/profile; and
6. a current verification window for both constitution and root adoption.

No authority-freshness receipt is an input to bootstrap-root qualification.

Therefore the bootstrap root does not recursively ask the operational freshness plane whether the bootstrap root is current.

## Constitutional epoch binding

The root manifest commits the exact current constitutional statement digest and version.

If the constitution advances from N to N+1, the previous root does not remain current merely because:

- the same logical source is retained;
- the same source key is retained;
- the same coverage policy bytes are retained; or
- the same rulebook ID text is retained.

The new current statement must independently adopt the applicable root.

Historical verification may retain the old root/constitution evidence. It cannot convert that old root into live current authority.

## Narrow control-plane scope

The bootstrap coverage policy may cover only:

- `AuthorityCoveragePolicy` (subject code 7);
- `CoverageTrustContextPolicy` (subject code 8); and
- `WitnessTrustPolicy` (subject code 9) when witness quorum is used.

Operational authority classes are forbidden from the bootstrap root, including:

- AuthorityGrant;
- SigningPolicy;
- ThresholdAuthorization;
- ExecutorDesignation;
- EffectSafetyPolicy; and
- Delegation.

This prevents the constitutional bootstrap layer from becoming a universal super-authority.

Once control-plane policy currentness is established, those current operational policies may qualify ordinary authority subjects through the normal #91/#94/#96/#74 path.

## Embedded root policy semantics

The exact root `AuthorityCoveragePolicy` and exact root `CoverageTrustContextPolicy` are identity-bearing parts of the root manifest.

The context policy must bind the exact embedded coverage-policy digest/profile.

Both policies must:

- use the root control-plane namespace;
- be semantically valid at the root effective epoch;
- remain semantically active at current qualification time; and
- name the same root adoption authority as the manifest.

DirectSource forbids a witness-trust root.

WitnessQuorum requires an exact witness-trust policy and verifier and requires `WitnessTrustPolicy` to be included in the root control-plane subject set.

## Probe challenges are evidence, not authority

A future bootstrap probe challenge is a read-only evidence-collection primitive.

Generating unpredictable challenge entropy does **not** grant:

- governance capability;
- execution authority;
- voting authority;
- mutation authority; or
- permission for an external effect.

Challenge generation may be operationally capability/rate-limit protected, but institutional `AuthorityGrant` freshness must not be a prerequisite for generating the probe needed to discover freshness.

The challenge becomes useful only when later source/witness coverage and root qualification prove that its referenced trust context is acceptable.

## Adoption evidence is an adapter boundary

`VerifiedBootstrapRootAdoption` is evidence-shaped host input, not proof by deserialization.

A runtime adapter must construct it only after independently verifying that the exact current rulebook adopted:

- the exact root manifest digest/profile;
- under the exact current constitutional statement;
- with the exact adoption reference and proof reference.

Provider outage, decode failure, stale evidence, wrong statement, wrong rulebook, wrong root digest, or mismatched adoption reference must deny.

## Identity vs dynamic verification

The stable root-manifest identity commits semantic authority.

The qualified-root identity commits:

- exact root manifest;
- exact current constitutional statement;
- exact root coverage policy; and
- exact root coverage-context policy.

Dynamic verification timestamps and lease horizons are not semantic root identity. Re-verifying the exact same root under the exact same constitutional epoch may refresh those metadata without inventing a different root.

A constitutional or semantic root change does change authority identity.

## No currentness-by-observation shortcut

This layer does not permit:

- latest timestamp wins;
- highest observed generation wins;
- DHT absence means no revocation;
- record arrival order;
- author prestige;
- Phi/reputation/model score; or
- challenge issuer identity

to establish bootstrap currentness.

The root comes from the current constitution plus exact rulebook adoption only.

## Historical/live separation

Later constitutional change or control-plane policy revocation must stop new live authority while preserving historical auditability.

Historical root evidence cannot satisfy a new live challenge and cannot be converted into a current freshness receipt.

## Containment

This crate is pure Rust.

It performs no Holochain calls, persistence, source queries, DHT reads, challenge generation, signature verification, lifecycle claims, or external effects.

No production DNA is changed by this tranche.

## Pre-production blockers

Before the authority-state runtime is provisioned:

1. probe challenge semantics in #99 must be separated from institutional authority;
2. #103/#105 must stop requiring current issuer-grant freshness merely to mint a freshness probe;
3. a runtime provider must verify the current constitution and exact root-manifest adoption;
4. root-covered control-plane freshness must be proven before operational policy use;
5. source refusal/partition must remain fail-closed; and
6. adversarial bootstrap-cycle, old-root replay, constitutional-advance, hidden-revocation and historical/live tests must pass.

Related: #109, #105, #103, #100, #99, #98, #96, #94, #91, #74, #73.
