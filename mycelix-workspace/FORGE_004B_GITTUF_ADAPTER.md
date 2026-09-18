# Mycelix Forge FORGE-004B — gittuf v0.16 Local Full-Verifier Adapter

**Status:** implementation candidate  
**Depends on:** FORGE-004A repository-verification contract  
**Pinned semantic target:** gittuf `0.16.0`

## Purpose

FORGE-004B maps one local gittuf protected-reference verification into FORGE-004A without making gittuf a protocol dependency or root of trust.

The adapter is a Rust process boundary. It verifies the external executable self-reports gittuf `0.16.0`, runs `gittuf verify-ref <protected-ref>` with developer/debug modes forced off, consumes only the command exit status, and independently reads the exact Git roots used to construct a Forge receipt. Human-formatted verification output is not parsed.

FORGE-004B is a **local, cache-free full-verification** tranche. Portable offline replay is deferred to FORGE-004C.

## Process-boundary rationale

Importing gittuf's Go package directly would couple the Forge adapter to gittuf's Go 1.26 toolchain and dependency closure. The process boundary instead keeps the portable Rust protocol crates independent, makes the upstream verifier replaceable, and gives later Nix/Spore packaging an explicit executable boundary to pin.

`gittuf version` is semantic version gating, not executable provenance. Binary/store-path provenance is a separate later claim.

## Full-history semantics

gittuf v0.16's `verify-ref` CLI selects `VerifyRefFull` unless `--latest-only` is supplied. The adapter therefore emits exactly:

```text
gittuf verify-ref <protected-ref>
```

and its command-runner tests reject `--latest-only` and developer-only `--from-entry`.

A deeper upstream audit found that `VerifyRefFull` may resume from `refs/local/gittuf/persistent-cache` when that local cache exists. Because Mycelix's `FullHistory` capability must not silently inherit an unsigned/local optimization as a trust input, FORGE-004B requires the persistent-cache ref to be absent immediately before and immediately after verification. If present, verification fails closed as `PersistentCachePresent`.

This gives the 004B claim unambiguous from-first-entry semantics under gittuf v0.16's ordinary uncached path.

## Exact repository snapshot

Before and after verification the adapter records:

- Git object format (`sha1` or `sha256`);
- protected ref tip;
- `refs/gittuf/reference-state-log` tip;
- `refs/gittuf/policy` tip;
- optional `refs/gittuf/attestations` tip.

Any difference fails as `RepositoryChangedDuringVerification`.

This narrows normal check/read races but does not claim immunity to an adversary able to mutate and perfectly restore state inside the observation interval. Stronger filesystem/store-path isolation is a deployment-hardening concern.

## External-policy mapping

FORGE-004A intentionally leaves `RepositoryPolicyState.policy_digest` adapter-defined. FORGE-004B freezes the gittuf mapping as a domain-separated Forge digest over the algorithm-qualified active `refs/gittuf/policy` tip.

`GittufInvocation::from_request` requires the supplied full `RepositoryPolicyState` to hash to the exact state digest and sequence named by the request.

The live adapter then requires that state's `policy_digest` to equal the commitment derived from the actual gittuf policy ref.

## Receipt integrity

`GittufLocalReceipt` contains:

- schema and adapter identity;
- exact Forge request digest;
- the complete translated `RepositoryPolicyState`;
- that state's exact digest;
- protected ref name;
- stable repository snapshot;
- history commitment;
- policy-lineage commitment.

On deserialization it revalidates:

1. schema and adapter version;
2. Git object-format agreement for all roots;
3. embedded policy-state digest;
4. policy-state `policy_digest` ↔ actual policy-ref mapping;
5. history commitment ↔ RSL tip;
6. policy-lineage commitment ↔ RSL/policy roots and policy sequence.

A receipt cannot become Forge evidence by itself. `into_observation_for(request)` requires the exact `RepositoryVerificationRequest` and rechecks request digest, policy-state digest/sequence, ref name, and target tip.

`GittufInvocation` intentionally has no serde wire format.

## Evidence commitments

### History

A domain-separated Forge digest over the exact RSL tip. Because the RSL tip is a Git commit identity, it commits to the reachable RSL commit graph under Git's object model.

### Policy lineage

A domain-separated Forge digest over:

- exact RSL tip;
- exact active policy tip;
- exact Forge repository-policy sequence.

### Local receipt

A deterministic, path-independent canonical commitment over the validated receipt fields. Identical repository state at different filesystem paths produces the same receipt commitment.

## Capabilities

FORGE-004B asserts only:

- `RefTipBinding`;
- `FullHistory`;
- `ProtectedRewriteDetection`;
- `PolicyLineageMonotonic`.

It does **not** assert:

- `OfflineEvidence`;
- `AuthorizationAttestations`.

FORGE-004C must carry the replayable Git object closure before `OfflineEvidence` is allowed. Exact review/merge authorization is a later tranche.

## Adversarial gates

Qualification tests cover at least:

- exact gittuf `0.16.0` gating;
- absence of `--latest-only` and `--from-entry`;
- developer mode forced off;
- persistent-cache presence fails closed;
- Git SHA-1/SHA-256 object-format agreement;
- exact protected-tip equality;
- mandatory RSL and active policy refs;
- optional attestations ref;
- before/after metadata equality;
- external policy-subject equality;
- path-independent receipt commitment;
- receipt deserialization rejects policy-state tampering;
- receipt deserialization rejects policy-subject tampering;
- receipt cannot qualify a different Forge request;
- local receipts cannot claim `OfflineEvidence`.

## Claim boundary

FORGE-004B does not establish executable provenance, repository network currentness, portable offline replay, immunity to all hostile filesystem races, review/merge authorization, Xenia/DID authenticity, authorized Forge authority transitions, SLSA provenance, release authorization, or transparency witnessing.
