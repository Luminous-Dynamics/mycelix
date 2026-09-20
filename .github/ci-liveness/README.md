# GitHub REST → EVIDENCE-CI Core Adapter V1

This directory is a provider adapter for the pure Rust liveness/conjunction core in
`crates/mycelix-evidence-ci-core` at exact semantic-core head:

`4190f855eb0f3c03a7a6b0decee84dd7edba07b4`

## Canonical entrypoint

External callers MUST enter through:

`github_rest_adapter_integrity.py::adapt`

The other Python modules are internal layers and are not independently admissible
provider receipts:

1. `github_rest_adapter.py` — pinned raw REST projection core
2. `github_rest_adapter_policy.py` — exact Rust-V1 structural compatibility policy
3. `github_rest_adapter_integrity.py` — canonical receipt-integrity guard

Calling layer 1 or layer 2 directly bypasses later invariants and MUST NOT be treated
as equivalent to the canonical V1 adapter.

## Exact internal identities

Projection core Git blob:

`603fd2b77bc588701dbcdae376ae31f5b49ce13b`

Projection implementation commitment:

`68fdfa639868b9300aad36cfa84dede8b2056cc280cd223d9e2d97a76cefa1bf`

Compatibility-policy Git blob:

`2bfbaba7696347cce016cd2344b7127af873c9d8`

Compatibility-policy implementation commitment:

`754030d5cd6557975c34032d3433167043c570818db25feae90d812ddb51ccb2`

Receipt-integrity Git blob:

`1e063f678e19574d796d22ecc1eb6d7805700edd`

Receipt-integrity implementation commitment:

`cb252abe2a45d7309e165b7b2352a236f591daf35eb9e7fa21f051636156a8e5`

Any revision of a pinned layer requires a reviewed successor of the outer layer that
pins the new exact identity. Caller-supplied implementation identities are not trusted.

## Layer theorem

The intended composition is:

```text
already-fetched GitHub REST bytes
        ↓
exact projection core
        ↓
exact Rust-V1 compatibility policy
        ↓
exact receipt-integrity guard
        ↓
normalized RequiredJobManifestV1 + WorkflowRunObservationV1
        ↓
Rust semantic core (separate theorem)
```

The Python stack does not derive `RunLivenessV1`, conjunctive PASS/FAIL, or failover
eligibility. Those semantics remain owned by the Rust core.

The integrity guard independently verifies both domain-separated inner commitments.
A compatibility layer may reject provider evidence, but canonical V1 MUST NOT silently
rewrite committed projection evidence and retain the old projection commitment.

For example, the pinned projection core maps GitHub `stale` into a tuple that the
exact Rust V1 core rejects. The compatibility policy can identify that mismatch, but
the canonical integrity entrypoint rejects any rewrite whose visible projection no
longer matches `adapter_observation_commitment_sha256`.

## Frozen vectors

`frozen_vectors.json` contains provider/core boundary vectors including:

- AMSAP-004A R1 terminal cancelled/no-runner/no-theorem-start;
- AMSAP-004A R2 queued/no-runner/no-theorem-start.

`expected_core_liveness` fields are contract expectations for the pinned Rust core.
They are NOT an executed Rust integration PASS unless a separate exact-toolchain
execution receipt proves that run.

## Claim ceiling

Even canonical adapter success means only deterministic local normalization and
commitment/contract checks over caller-supplied bytes.

It does NOT prove:

```text
GitHub API response authenticity
runner identity or runner attestation
theorem PASS/FAIL
qualification PASS/FAIL
failover authority
rerun authority
dispatch authority
merge authority
deployment authority
```

Required output nonclaims remain false/null through the canonical stack.

## Side effects

V1 has no GitHub client, network fetch, workflow dispatch, rerun, cancellation,
repository mutation, or runner-control capability. REST payloads are supplied by the
caller.
