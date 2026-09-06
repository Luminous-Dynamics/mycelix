# Authority-State Bootstrap Root v0.1 — Normative Invariants

Status: **pure qualification implementation repaired; runtime verifier boundaries still unprovisioned**

## 1. Purpose

The operational authority-state coverage plane cannot prove its own currentness.

A runtime dependency of:

`challenge -> current coverage policy -> challenge`

is a trust-bootstrap cycle, not a valid security architecture.

This crate introduces one deliberately narrow root whose live semantic authority comes from the already-qualified constitutional plane plus exact rulebook adoption of one exact root manifest.

## 2. Root of currentness

A positive `QualifiedAuthorityStateBootstrapRoot` requires all of the following:

1. an exact `AuthorityStateBootstrapRootManifest`;
2. an independently verified **current** `ConstitutionStatement` receipt;
3. exact recomputation of the constitutional statement digest under `STATEMENT_PROFILE`;
4. exact network, institution, constitution ID/version, rulebook ID/version/digest/profile agreement;
5. independently verified rulebook adoption of the exact root-manifest digest/profile, adoption reference and proof reference; and
6. a current conservative verification window for both constitution and root adoption.

No `VerifiedAuthorityFreshness` receipt is an input to bootstrap-root qualification.

Therefore the bootstrap root does not recursively ask the operational freshness plane whether the bootstrap root is current.

## 3. Constitutional epoch binding

The root manifest commits the exact current constitutional statement digest/profile and version.

If the constitution advances from N to N+1, the previous root does not remain current merely because:

- the same logical source is retained;
- the same source key is retained;
- the same coverage policy bytes are retained; or
- the same rulebook ID text is retained.

The new current statement must independently adopt the applicable root.

Historical verification may retain the old root/constitution evidence. It cannot convert that old root into live current authority.

## 4. Narrow control-plane scope

The bootstrap coverage policy must cover exactly:

- `AuthorityCoveragePolicy` (subject code 7);
- `CoverageTrustContextPolicy` (subject code 8); and
- `WitnessTrustPolicy` (subject code 9) exactly when witness quorum is used.

Operational authority classes are forbidden from the bootstrap root, including:

- AuthorityGrant;
- SigningPolicy;
- ThresholdAuthorization;
- ExecutorDesignation;
- EffectSafetyPolicy; and
- Delegation.

This prevents the constitutional bootstrap layer from becoming a universal super-authority.

Once control-plane policy currentness is established, those current operational policies may qualify ordinary authority subjects through the normal #91/#94/#96/#74 path.

## 5. Embedded root policy semantics

The exact root `AuthorityCoveragePolicy` and exact root `CoverageTrustContextPolicy` are identity-bearing parts of the root manifest.

The coverage policy commits the exact logical authoritative source reference and exact source verification identity/profile. The context policy binds the exact embedded coverage-policy digest/profile.

Both policies must:

- use the root control-plane namespace where applicable;
- be semantically valid at the root effective epoch;
- remain semantically active at current qualification time;
- name the same root adoption authority as the manifest; and
- belong to the exact manifest institution/rulebook context.

DirectSource forbids a witness-trust root.

WitnessQuorum requires an exact witness-trust policy and verifier and requires `WitnessTrustPolicy` in the root control-plane subject set.

## 6. Adoption evidence is an adapter boundary

`VerifiedBootstrapRootAdoption` is evidence-shaped host input, not proof by deserialization.

A runtime adapter must construct it only after independently verifying that the exact current rulebook adopted:

- the exact root manifest digest/profile;
- under the exact current constitutional statement digest/profile;
- under the exact rulebook ID/version/digest/profile;
- with the exact adoption authority, adoption reference and proof reference.

Provider outage, decode failure, stale evidence, wrong statement, wrong rulebook, wrong root digest, or mismatched adoption reference/proof must deny.

## 7. Current constitution evidence is an adapter boundary

`VerifiedCurrentConstitutionReceipt` is also evidence-shaped host input, not proof by deserialization.

It carries the exact `ConstitutionStatement`, its recomputed statement digest/profile, a verification reference and bounded live window. It also carries the deployment `dna_hash` for the separate runtime-DNA binding layer.

The semantic bootstrap-root digest deliberately does not absorb `dna_hash`; #123-style deployment binding remains a separate necessary condition before a Holochain runtime treats semantic authority as live in one concrete DNA.

A runtime MUST obtain the current-constitution receipt from an independent constitution/current-head verifier. A generic root provider must not be allowed to invent this positive receipt merely because its fields deserialize.

## 8. Validity is monotone

The qualified root's `valid_until_ms` is the minimum of:

- current-constitution receipt horizon;
- root-adoption verifier horizon;
- manifest lifetime;
- embedded coverage-policy lifetime; and
- embedded context-policy lifetime.

Its `verified_at_ms` is the maximum of current-constitution verification, root-adoption verification and root effective time.

No projection or composer may widen those horizons.

## 9. Probe challenges are evidence, not authority

A bootstrap probe challenge is a read-only evidence-collection primitive.

Generating unpredictable challenge entropy does **not** grant governance capability, execution authority, voting authority, mutation authority, or permission for an external effect.

Challenge generation may be operationally capability/rate-limit protected, but institutional `AuthorityGrant` freshness must not be a prerequisite for generating the probe needed to discover freshness.

## 10. No currentness-by-observation shortcut

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

## 11. Historical/live separation

Later constitutional change or control-plane policy revocation must stop new live authority while preserving historical auditability.

Historical root evidence cannot satisfy a new live challenge and cannot be converted into a current freshness receipt.

## 12. Positive result is non-deserializable

`QualifiedAuthorityStateBootstrapRoot` derives `Serialize` but not `Deserialize`.

Runtime adapters may transport candidate manifest/constitution/adoption evidence, but must call `qualify_bootstrap_root` locally before obtaining the positive result.

## 13. Containment

This crate is pure Rust.

It performs no Holochain calls, persistence, source queries, DHT reads, challenge generation, signature verification, lifecycle claims, or external effects.

No production DNA is changed by this tranche.

## 14. Implementation-integrity repair

The originally opened #111 branch accidentally committed the constitutional lineage implementation into this crate path even though its Cargo metadata, invariants and workflow described the bootstrap-root API.

The active stack must contain the actual bootstrap implementation and MUST NOT contain a second `ConstitutionGenesisManifest`, `project_verified_lineage`, or local `Digest32` definition in this crate.

The bootstrap workflow statically freezes that distinction in addition to compiling the direct downstream #115/#116 consumers.

## 15. Remaining runtime blocker

The pure kernel can cross-check evidence but cannot cryptographically establish that a deserialized current-constitution/adoption receipt came from the correct verifier.

Before provisioning, the runtime must split current-constitution verification and root-adoption verification into independent role-specific boundaries. One generic root provider must not be able to fabricate both receipts and thereby become the bootstrap oracle.

No external effects are enabled.
