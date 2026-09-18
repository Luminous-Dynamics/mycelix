# Mycelix Forge FORGE-004A — Repository Verification Contract

**Status:** implementation candidate  
**Depends on:** FORGE-003 authority subjects  
**Next:** FORGE-004B gittuf v0.16 adapter

## Purpose

FORGE-004A freezes the provider-neutral contract between Mycelix Forge and a repository-security verifier. It deliberately does **not** embed gittuf, GitHub, GitLab, Radicle, or a Git implementation into the portable protocol.

The contract answers a narrower question:

> What exact repository state, policy state, verification semantics, and evidence commitments must an adapter prove before Forge may consume the observation?

This preserves the M0 rule that an external verifier is an interoperability component, not the root of project identity or authority.

## Why split FORGE-004

The original FORGE-004 milestone combined the portable semantic boundary with the concrete gittuf integration. Splitting it into **004A contract** and **004B adapter** prevents third-party API details from becoming accidental Forge protocol semantics.

The numbering of later milestones is unchanged:

- FORGE-005 remains the Mycelix/Xenia identity bridge.
- FORGE-006 remains exact-subject change proposals.
- FORGE-007 remains signed review attestations.
- FORGE-008 remains SLSA Source provenance.

## Protocol subjects

`mycelix-forge-repository` introduces:

- `GitObjectAlgorithm` and `GitObjectId` — algorithm-qualified Git object identities supporting SHA-1 and SHA-256 repository formats;
- `RepositoryRef` — conservative, fully-qualified reference names;
- `RepositoryTip` — exact ref + object state;
- `RepositoryAdoption` — the explicit point from which Forge verification begins;
- `RepositoryPolicyState` — project-bound, monotonically sequenced external repository-policy state;
- `RepositoryVerificationRequest` — exact protected-source transition and policy/authority context;
- `VerificationCapability` / `VerificationProfile` — semantics an adapter must actually establish;
- `AdapterObservation` — evidence-bearing adapter output that is still only an observation;
- `StructurallyQualifiedRepositoryVerification` — positive runtime result that the observation matches the exact request and required semantics.

## Adoption boundary

Forge MUST NOT imply that an imported repository has been protocol-verified since its first commit.

An adoption statement binds:

- exact `ProjectIdentity`;
- exact baseline protected ref and Git object;
- exact authority epoch commitment;
- exact project-policy commitment;
- exact external repository-policy commitment;
- adoption time.

Therefore:

```text
historical Git repository
        │
        ▼
exact adoption boundary
        │
        ▼
Forge-verifiable history from that point forward
```

History before the boundary may be inspected, but it is not retroactively represented as Forge-qualified history.

## External policy monotonicity

A repository policy state is not accepted merely because its bytes or signatures are valid.

Every non-genesis `RepositoryPolicyState` binds:

- one exact project;
- sequence `N`;
- exact predecessor digest for sequence `N-1`;
- exact external policy digest.

Successor validation requires both contiguous sequence and predecessor equality. This prevents a generic rollback class where an older still-valid policy is replayed after a newer accepted state exists.

`valid signature != current accepted policy state`

## Exact verification request

A `RepositoryVerificationRequest` binds:

- project identity;
- adoption commitment;
- protected ref;
- exact `from` Git object;
- exact `to` Git object;
- authority epoch commitment;
- project-policy commitment;
- repository-policy-state commitment;
- repository-policy sequence.

A no-op source transition is rejected.

The request is the subject that FORGE-004B must translate into gittuf verification operations.

## Capability honesty

Adapters do not return a single unqualified `true` value. They declare the semantics actually established by one observation.

Current capability vocabulary:

1. `RefTipBinding`
2. `FullHistory`
3. `ProtectedRewriteDetection`
4. `PolicyLineageMonotonic`
5. `AuthorizationAttestations`
6. `OfflineEvidence`

The M0 protected-source profile requires:

- reference-tip binding;
- full-history verification;
- protected-rewrite detection;
- monotonic policy lineage;
- offline evidence.

An adapter that only verifies the latest state therefore cannot satisfy the M0 profile while claiming equivalence to full-history verification.

## Evidence commitments

Capability declarations have structural evidence requirements:

- claiming `FullHistory` requires a `history_commitment`;
- claiming `OfflineEvidence` requires an `evidence_commitment`.

Deserialization re-runs these checks, so malformed wire data cannot bypass the constructor and claim capabilities without their required commitments.

The concrete meaning and construction of gittuf RSL/policy commitments belongs in FORGE-004B.

## gittuf v0.16 mapping direction

Current gittuf exposes full reference verification and a `LatestOnly` mode. Its full verification path evaluates policy/RSL history and separately checks that the local ref tip equals the expected RSL tip.

FORGE-004B should therefore map:

```text
RepositoryVerificationRequest
        │
        ├─ protected ref ───────────────> gittuf VerifyRef / full path
        ├─ expected target ─────────────> local tip equality
        ├─ repository policy state ─────> gittuf policy state commitment
        ├─ history commitment ──────────> RSL history commitment
        └─ evidence commitment ─────────> portable local evidence bundle
```

For the M0 protected-source profile, gittuf `LatestOnly` MUST NOT be represented as `FullHistory`.

## Frozen v1 vectors

The crate freezes an end-to-end fixture using the FORGE-002 project-identity fixture:

- adoption SHA-256: `b999dc307fddc2dc27942a914016ce0dce3daceb205ffd9f34ffdfefdbb4f263`
- repository-policy-state SHA-256: `e13cadc61b053d665aa7fcaf12ab26ab63bfa8009fac650ed25f48c288f4e500`
- verification-request SHA-256: `d3a9aa5c5b5328712bd04b84608438bcc722380b3d5dbd8f1e052acff232c2fc`

Independent implementations must reproduce these exact commitments for the fixture.

## Adversarial gates

FORGE-004A tests require at least:

- malformed Git object lengths fail closed;
- unsafe/ambiguous ref names fail closed;
- policy rollback, sequence gaps, and wrong predecessors fail;
- external policy state cannot cross project identities;
- no-op transitions fail;
- latest-only semantics cannot satisfy the M0 profile;
- one-byte target mutation invalidates adapter applicability;
- deserialization cannot claim full-history semantics without a history commitment;
- canonical v1 vectors remain frozen.

## Claim boundary

FORGE-004A establishes **protocol subjects and structural adapter semantics only**.

It does not establish:

- that gittuf verification has actually executed;
- signature authenticity;
- Xenia/DID principal resolution;
- repository network currentness;
- authorized authority transitions;
- review authorization;
- build/source provenance;
- release authority;
- transparency witnessing.

The concrete gittuf adapter must produce the evidence that satisfies this contract rather than being trusted merely because its implementation is named `gittuf`.
