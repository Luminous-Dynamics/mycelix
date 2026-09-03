# Content Fabric Hard Policy Gate v1

Status: **CF-06A contract**

## Purpose

CF-06A creates a strict boundary between Content Fabric evidence and optimization.

The optimizer MUST NOT receive raw CF-05A `SnapshotServiceCandidateV1` values. It receives a `PolicyQualifiedPoolV1` only after hard policy has been applied and pool feasibility has been established.

The intended flow is:

```text
CF-05 append-only evidence
        |
        v
CF-05A deterministic projection
        |
        v
CF-06A hard policy gate
        |
        +--> rejected candidates + reasons
        |
        v
PolicyQualifiedPoolV1
        |
        v
CF-06 optimizer / Symthaea proposal
        |
        v
CF-06A selection validation
        |
        v
later executor authority boundary
```

Preference scoring never runs on candidates already known to violate hard policy.

## Two levels of hard policy

### Per-candidate qualification

A candidate can be rejected for:

- not actually being temporally live at the supplied projection time;
- wrong digest or size for the exact placement target;
- missing/conflicting provider policy evidence;
- insufficiently assured jurisdiction evidence;
- forbidden or non-allowed storage jurisdiction;
- missing required client-side encryption;
- missing/weak provider-at-rest encryption evidence;
- missing/weak/insufficient retention capability;
- missing/weak/ambiguous required failure-domain evidence;
- contradiction between independently attested failure-domain facts and the provider's own CF-05 claim.

### Pool feasibility

Even individually acceptable providers do not imply a feasible placement.

The surviving set must contain at least `minimum_replicas` candidates and enough distinct sufficiently assured values for every dimension in `FailureDomainPolicyV1`.

For example:

```text
minimum_replicas = 3
operator >= 3
site >= 2
```

requires a pool containing at least three candidates, at least three distinct accepted operator values, and at least two distinct accepted site values.

## Selection validation

A feasible pool can still contain policy-bad subsets. Example: a pool has three nodes across two sites, but an optimizer selects two nodes from the same site.

Therefore `PolicyQualifiedPoolV1::validate_selection()` MUST be applied to a planner-selected subset before it can be accepted by any executor.

The selection validator rejects:

- duplicate candidate IDs;
- unknown candidates;
- too few replicas;
- insufficient distinct values for any required failure-domain dimension.

Optimization is recommendation; hard-policy validation remains authoritative.

## Assurance boundary

`PolicyAssuranceV1` is ordered:

```text
SelfClaimed < ProviderSigned < IndependentlyAttested
```

Strict mode requires `IndependentlyAttested` provider facts.

The policy crate does not verify signatures, TPM quotes, Xenia evidence, organizational attestations, or external registries. A dedicated upstream verifier must map raw evidence into these neutral assurance labels.

**Critical rule:** CF-05 self-claimed failure-domain metadata MUST NOT be upgraded merely because it is signed by the provider. Provider-authored data is still not independent evidence.

## Jurisdiction

Jurisdiction is a hard filter whenever the storage intent contains an allowed or forbidden jurisdiction set.

Only facts meeting the configured assurance threshold count. Every accepted possible storage jurisdiction for the candidate must satisfy `PlacementRequirementsV1::jurisdiction_allowed()`.

No optimizer score can override a jurisdiction rejection.

## Encryption

Client-side encryption and provider-at-rest encryption are intentionally different facts.

`ClientSide` is established locally by `PlacementTargetV1::client_side_encrypted`, because the exact bytes named by the digest must already be encrypted before untrusted storage.

`ProviderAtRest` requires sufficiently assured provider policy evidence.

`ClientSideAndProviderAtRest` requires both.

## Retention

Retention is evaluated at the explicit CF-05A projection timestamp.

- `BestEffort` requires no provider retention guarantee.
- `MinimumSeconds(n)` requires evidence extending through at least `evaluation_time + n`.
- `Until(t)` requires evidence through `t` when `t` is still in the future.
- `Indefinite` requires evidence of indefinite-retention capability.

Overflow while deriving a retention deadline fails the pool closed.

## Snapshot quality

Strict mode requires:

- `SnapshotCoverageV1::QueriedIndexesComplete`; and
- zero CF-05A projection issues.

This still does **not** claim global Holochain finality. It means the caller supplied the best local queried-index view under the CF-05A contract.

## Determinism

The gate performs no I/O and reads no system clock.

Provider-policy evidence is normalized by advertisement action. Identical duplicates collapse; conflicting evidence for the same advertisement causes that advertisement to fail closed.

Candidates, rejections, pool failures, and accepted facts are canonically ordered so input vector ordering cannot influence policy results.

## Non-authority

CF-06A cannot:

- score candidates;
- choose a provider for economic or performance reasons;
- create a placement lease;
- authorize an Iroh read;
- mutate CAS state;
- pay a provider;
- claim that content bytes are correct;
- mint independent attestations.

Its only authority is to say **no** to policy-invalid inputs and to certify that a candidate pool/subset satisfies the supplied hard constraints.
