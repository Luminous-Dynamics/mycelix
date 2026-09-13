# Mycelix Public Election Verifier Resource Envelope v0.1

Status: **ELECT-013 bounded verification-resource contract; not an archive parser or cryptographic verifier**

Parent stack: ADV-002 above the hosted-qualified ELECT-001..012 / ADV-001 foundation.

Profile identifier: `mycelix-public-election-verifier-resource-envelope-v1`

## Purpose

ELECT-011 made verification portable and offline, but portability is not enough if a syntactically valid evidence package can force a verifier to allocate or expand unbounded data.

The target theorem is:

> Before expensive parsing or verification begins, the election publishes a bounded resource policy and each verifier proves that its declared safe capability dominates that policy. Observed package resources are then checked against the same envelope with overflow-safe arithmetic. Exceeding a valid envelope blocks certification as indeterminate; malformed or inconsistent resource evidence fails hard.

This tranche deliberately does **not** choose one universal byte limit for every election.

## Policy versus verifier capability

Two different objects are required.

`ElectionVerificationResourcePolicyV1` is intended to be frozen by the election and expresses the maximum workload the election may legitimately require.

`VerifierResourceCapabilityV1` is bound to a specific verifier release and expresses the maximum workload that implementation can safely process.

The verifier may proceed only when:

```text
verifier capability >= election policy
```

for every resource axis.

A verifier with insufficient capability does not silently skip work, truncate evidence, or downgrade a stage. It returns a certification-blocking indeterminate resource gate.

## Resource axes

The v1 policy covers:

- artifact count;
- maximum bytes for one artifact;
- total declared artifact bytes;
- canonical path length;
- interoperability profile count;
- manifest bytes;
- container bytes;
- expanded bytes;
- container entry count; and
- nested-container depth.

The policy is also constrained by the already-qualified ELECT-011 structural maxima for artifact count, path length, and interoperability profile count. An election cannot use ELECT-013 to relax the parent package contract.

## Preflight before parser authority

`EvidencePackagePreflightHeaderV1` binds only the information needed to make an early resource decision:

- package root digest;
- resource-policy digest;
- manifest byte length; and
- container byte length.

A concrete package encoding should make this bounded preflight information available before allocating or decoding the full manifest.

The current Rust type is a logical contract, not a frozen wire encoding. A later package-format specification must define the exact bounded preamble bytes and canonicalization.

Malformed header relationships fail hard. For example, a manifest cannot claim to be larger than the containing package.

A valid but oversized manifest/container yields:

```text
BlockIndeterminate(ResourceLimitKind)
```

rather than process exhaustion or partial verification.

## Manifest resource accounting

`evaluate_manifest_resources(...)` first replays ELECT-011 manifest validation and then derives resource observations directly from the manifest:

- exact artifact count;
- checked sum of declared artifact bytes;
- largest single artifact;
- longest canonical path; and
- interoperability profile count.

The byte sum uses checked arithmetic. Integer overflow is a malformed-evidence failure, not an indeterminate capacity result.

This distinction is intentional:

```text
honest workload > supported envelope  -> indeterminate / certification blocked
inconsistent or overflowing evidence  -> fail / certification blocked
```

## Container/expansion observation

`ContainerResourceObservationV1` binds the observed package root and policy digest plus:

- observed container bytes;
- container entry count;
- expanded bytes;
- maximum nested-container depth; and
- evidence for the observation.

The observation must agree exactly with the preflight header's package root, policy digest, and container byte length.

The resource-consistency theorem additionally requires:

```text
container entries >= manifest artifacts
expanded bytes    >= total declared artifact bytes
```

so a resource summary cannot under-report the data that the manifest itself claims exists.

## Authority/binding boundary

ELECT-013 validates a supplied resource policy and checks resource observations against it. **This tranche does not yet prove that the frozen election constitution/certification evidence commits to the exact resource-policy bytes used by the verifier.**

In particular, the preflight header carries a `resource_policy_digest`, but this v0.1 logical contract does not recompute that digest from a frozen canonical policy encoding or make the resource policy a new required ELECT-011 package artifact.

Therefore the following remains a required follow-on theorem before deployment claims:

```text
frozen election / certification evidence
        commits to exact resource-policy digest
                    |
                    v
bounded package preflight
        names the same digest
                    |
                    v
standalone verifier recomputes digest
        from canonical resource-policy bytes
```

Until that exists, ELECT-013 qualifies the **resource decision semantics**, not authoritative election-policy anchoring.

## Certification semantics

ELECT-013 introduces a resource gate, not a shortcut around verification.

`Proceed` means only that the resource sub-gate permits the verifier to continue. It does not mean package integrity or election verification passed.

`BlockIndeterminate(limit)` means the verifier cannot safely complete the required work under the frozen envelope/capability relation. Certification must remain blocked.

Malformed resource evidence is a hard validation failure.

The following are therefore not equivalent:

```text
resource gate proceeded       != election evidence verified
resource limit exceeded       != evidence disproven
resource metadata malformed   != verifier too small
verifier cannot process       != verifier may skip the stage
```

## Why no universal byte ceiling

A municipal pilot and a national election can legitimately have radically different evidence volumes. A fixed global ceiling would either be dangerously high for small verifiers or unrealistically low for large elections.

The safer model is:

```text
frozen election requirement
          +
verifier-specific safe capability
          +
exact observed resource evidence
          =
deterministic proceed / block decision
```

Concrete deployments may still publish recommended implementation ceilings and operational profiles.

## Deliberate non-claims

ELECT-013 does not yet implement:

- authoritative constitution/certification binding of the exact resource-policy digest;
- canonical resource-policy wire encoding/digest recomputation;
- archive extraction;
- symlink defense;
- duplicate archive-entry detection;
- decompressor sandboxing;
- compression-ratio limits beyond the explicit expanded-byte bound;
- streaming parser implementation;
- memory allocator limits;
- CPU/instruction metering;
- cryptographic proof-work metering;
- operating-system sandbox limits; or
- legal election certification.

The contract gives those implementations a deterministic policy surface and fail-closed result semantics.

## Next adversarial tranche

After ELECT-013 qualifies, the adversarial corpus should add resource attacks including:

- single-artifact oversize;
- aggregate declared-byte oversize;
- checked-sum overflow;
- oversized manifest preflight;
- oversized container preflight;
- expanded archive bomb;
- too many container entries;
- excessive nested-container depth;
- verifier capability below the frozen election policy; and
- resource summaries that under-report manifest evidence.

Those become ADV-003 executable attacks rather than remaining prose-only concerns.

After ADV-003, add a narrow resource-policy anchoring tranche that binds canonical resource-policy bytes/digest into frozen election or certification evidence and requires the package preflight to name that exact commitment.
