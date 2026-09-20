# PSI-002B2C r2 — Registry Currentness Structural Core

Status: **corrected structural source / not compile-qualified / not currentness**

This r2 replaces superseded #2354 before qualification. It remains a sibling of provider-authenticity/provenance work and depends only on the B2A registry-subject core.

## Corrections from r1

### Provider independence means provider namespace

R1 counted `(provider_namespace, verifier_identity)` pairs toward `minimum_distinct_providers`. That could allow one provider namespace with multiple verifier keys to amplify an independence quorum.

R2 defines independence as:

```text
one provider namespace = at most one independent observation
```

A repeated provider namespace fails closed even when the verifier key differs.

This is conservative. A later profile may explicitly model independently operated subproviders, but it must do so with a new named independence theorem rather than by counting keys.

### Evidence class is part of the requirement

`CurrentnessRequirementV1` now binds `required_evidence_class` in addition to the free-form evidence profile.

An observation whose evidence class differs from the exact requirement yields `UnsupportedProfile / EvidenceClassMismatch`.

Thus a weak provider observation cannot be mixed with a signed checkpoint merely because both use the same profile string.

### Explicit wire IDs

`HeadEvidenceClass` no longer enters commitments through Rust `Debug` formatting. It has explicit versioned wire identities:

```text
append-only-head-witness-v1
signed-head-checkpoint-v1
transparency-consistency-proof-v1
provider-observed-head-v1
quorum-head-agreement-v1
```

Both requirement and observation commitments bind these explicit wire IDs.

## Structural-only authority

Even unanimous matching observations can produce only:

```text
StructurallyConsistentLatestClaim
```

and still report:

```text
registry_current = false
completeness_established = false
provider_evidence_verified = false
contact_discovery_composition_qualified = false
```

## Fail-closed behavior

The evaluator detects/refuses:

- empty evidence;
- malformed observations;
- service/registry-epoch mismatch;
- evidence-profile mismatch;
- evidence-class mismatch;
- repeated provider namespace / quorum amplification;
- divergent head claims and same-sequence forks;
- candidate behind the agreed claim;
- candidate ahead of available claims;
- insufficient independent provider namespaces;
- missing/mismatched clock profile;
- future observations;
- observations outside an exact max-age window.

## Clock boundary

Clock-profile names are only structural identities.

```text
clock profile matches
!= clock trusted
```

Trusted time requires a later evidence adapter.

## Source tests

The committed 11-case corpus covers explicit wire IDs, structural-only agreement, same-provider multi-key amplification, evidence-class mismatch, empty evidence, fork/conflict, candidate behind, candidate ahead, insufficient independent provider namespaces, stale observations and future observations.

## Qualification boundary

```text
source exists
!= source compiles
!= tests pass
!= provider evidence verified
!= clock trusted
!= completeness established
!= registry current
```

A fresh exact-source qualifier is required. Superseded #2355 must not be executed.
