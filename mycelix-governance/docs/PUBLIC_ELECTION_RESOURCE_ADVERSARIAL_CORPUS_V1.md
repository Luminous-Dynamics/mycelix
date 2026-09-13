# Mycelix Public Election Resource Adversarial Corpus v0.1

Status: **ADV-003 resource-exhaustion adversarial corpus; depends on qualified ELECT-013**

Profile identifier: `mycelix-public-election-resource-adversarial-corpus-v1`

## Purpose

ELECT-013 defines a bounded resource gate for offline election verification. ADV-003 converts that contract into executable attacks so resource-exhaustion safety cannot remain a documentation-only promise.

The core distinction is:

```text
valid workload beyond safe capacity
    -> BlockIndeterminate(limit)
    -> certification blocked

malformed / inconsistent resource evidence
    -> exact hard failure
    -> certification blocked
```

A verifier must never turn resource pressure into skipped verification, silent truncation, partial parsing, or a false PASS.

## Complete resource-axis coverage

ADV-003 requires at least one executable capacity attack for every ELECT-013 `ResourceLimitKind`:

1. artifact count;
2. single-artifact bytes;
3. total declared artifact bytes;
4. canonical path bytes;
5. interoperability profile count;
6. manifest bytes;
7. container bytes;
8. container entries;
9. expanded bytes; and
10. nested-container depth.

An explicit coverage test iterates the required resource axes and fails if any axis loses its attack.

## Sixteen attacks

The initial corpus includes:

- `ArtifactCountBudgetExceeded`;
- `SingleArtifactBudgetExceeded`;
- `AggregateArtifactBudgetExceeded`;
- `CanonicalPathBudgetExceeded`;
- `InteroperabilityProfileBudgetExceeded`;
- `ManifestPreflightBudgetExceeded`;
- `ContainerPreflightBudgetExceeded`;
- `ContainerEntryBudgetExceeded`;
- `ExpandedByteBudgetExceeded`;
- `NestedContainerDepthExceeded`;
- `VerifierCapabilityBelowElectionPolicy`;
- `DeclaredArtifactByteSumOverflow`;
- `ManifestLargerThanContainer`;
- `ContainerLengthObservationMismatch`;
- `ExpandedBytesUnderReported`; and
- `ElectionPolicyRelaxesParentArtifactCap`.

Every case carries an exact expected disposition/finding and is certification-blocking.

## Capacity versus malformed evidence

Capacity attacks intentionally produce `BlockIndeterminate(ResourceLimitKind)`.

Examples:

```text
expanded archive > frozen envelope
    -> BlockIndeterminate(ExpandedBytes)

verifier capability < election requirement
    -> BlockIndeterminate(TotalArtifactBytes)
```

Malformed evidence uses exact typed failures instead:

```text
u64 declared-byte sum overflow
    -> DeclaredArtifactBytesOverflow

manifest length > containing container length
    -> ManifestExceedsContainer

observed container length != preflight length
    -> ContainerByteLengthMismatch

expanded bytes < bytes declared by manifest
    -> ExpandedBytesBelowDeclaredArtifactBytes

election policy tries to exceed ELECT-011 artifact cap
    -> ArtifactLimitExceedsPackageContract
```

That distinction prevents an attacker from laundering dishonest metadata into an innocent-looking “verifier too small” result.

## Positive control

ADV-003 also executes a normal resource path that must continue to:

- validate the frozen resource policy;
- validate verifier capability;
- pass capability dominance;
- pass bounded preflight;
- pass manifest resource accounting;
- pass container resource accounting; and
- pass manifest/container consistency.

This prevents an always-block verifier from satisfying the corpus.

## Relationship to election theorems

Resource attacks primarily protect:

- **RecoverableVerification** — preserved election evidence must remain practically verifiable rather than being an unbounded denial-of-service object;
- **EvidenceBeforeCertification** — resource inability may never be silently converted into certification; and
- **EvidenceContinuity** — resource observations must remain consistent with the evidence package they describe.

Passing ADV-003 does not qualify ballot cryptography or other protocol-dependent election theorems.

## Deliberate non-claims

This corpus does not yet execute a real archive decompressor, allocator sandbox, CPU meter, proof verifier, operating-system sandbox, or hostile filesystem. Therefore it does not establish resistance to every implementation-specific decompression bomb, parser exploit, memory exhaustion, CPU exhaustion, or filesystem attack.

Those become appropriate after the logical ELECT-013 resource contract is integrated into a concrete standalone verifier implementation.

## Next work

After ADV-003, the next resource hardening should focus on implementation-level isolation:

- canonical archive format and bounded preamble bytes;
- streaming extraction;
- symlink/hardlink/device-file rejection;
- duplicate-entry and path-normalization rejection;
- memory and instruction budgets;
- sandboxed proof verification; and
- hostile-package fuzzing/property testing.

That should remain separate from choosing the actual public-election ballot cryptography.
