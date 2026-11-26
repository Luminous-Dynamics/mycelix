# Craft Evidence Lineage V1

- **Work item:** CRAFT-EVIDENCE-LINEAGE-001 (#6761)
- **Status:** Reference core; integration and field qualification pending
- **Version:** 1.0-draft
- **Date:** 2026-11-26

## Purpose

Evidence and decisions are often treated as valid forever once reviewed. Real systems change: a source is corrected, a policy is superseded, consent is withdrawn, an issuer revokes a credential, a measurement is invalidated, or an authority decision expires.

A digest binds a review to exact bytes. It does not guarantee that downstream decisions remain justified after an upstream artifact changes.

Evidence Lineage V1 computes the impact of such changes across an explicit, version-pinned dependency graph. It is a pure reference core: it reports which artifacts remain current, require revalidation, or are no longer usable. It does not modify prior decisions or perform re-approval.

## Contract

Each artifact reference binds:
- a stable artifact identifier;
- a monotonically increasing artifact revision;
- an externally computed 32-byte digest.

Each node records an artifact kind and explicit dependencies. A dependency contains an exact artifact reference, a typed relation, and one impact policy:

- **RevalidateOnChange:** the dependent artifact must be reviewed and replaced before it is relied on again.
- **InvalidateOnChange:** the dependent artifact is unusable until replaced by a newly qualified artifact.
- **Informational:** the relation is useful for navigation but must not grant, remove, or imply validity.

The implementation rejects missing dependencies, incorrect digests, duplicate artifact revisions, duplicate dependency references and cyclic graphs.

## Change events

A change event contains a unique event identifier, a host-assigned sequence, an exact target artifact reference, a reason and a timestamp.

Supported reasons:
- Corrected
- Superseded
- Revoked
- Expired
- Withdrawn

Corrected and Superseded events must identify an existing higher revision of the same logical artifact. Revocation, expiry and withdrawal events cannot smuggle in a replacement. Exact duplicate event replay is idempotent. Distinct events may target the same immutable artifact revision; they are ordered by the host-assigned sequence, and the latest event determines the current disposition while every admitted event remains in the replay receipt. Reusing an event ID with different content, reusing a sequence number or targeting an unknown revision fails closed.

The host must authenticate the event issuer, verify signatures, canonicalize and digest event payloads, validate legal or organizational authority, establish effective time and apply jurisdiction-specific rules before admitting an event. This crate only evaluates the consequences of admitted events.

## State semantics

| State | Meaning |
| --- | --- |
| Current | No admitted event or non-informational dependency makes this exact artifact version stale |
| NeedsRevalidation | At least one dependency changed under a revalidation policy |
| Invalidated | At least one dependency changed under an invalidation policy |
| Superseded | This exact version was corrected or superseded |
| Revoked | This exact version was revoked |
| Expired | This exact version expired |
| Withdrawn | This exact version was withdrawn |

An artifact with a terminal event keeps that terminal state. Dependents are evaluated in parent-before-dependent order, so impact propagates transitively. Informational edges are ignored for validity. Independent observations remain current when they have no declared dependency on the changed artifact.

A stale artifact is **not** made current by deleting an event or editing its record. Recovery requires a new artifact revision with newly pinned dependencies and fresh assessment or review evidence. The old revision remains available for audit.

## Integration boundary

Candidate dependency relations for the existing Craft cores:

| Producer | Downstream artifact | Suggested dependency relation |
| --- | --- | --- |
| Need Intake sources and assumptions | Need proposal claim | SourcedFrom / DerivedFrom |
| Federation agreement | Foreign credential/evidence recognition | Authorizes / GovernedBy |
| Claim Support result | Decision material element | SemanticallySupportedBy |
| Decision Closure result | Project/implementation authorization | Ratifies / Authorizes |
| Project Formation terms | Participant offers and work packages | Implements / GovernedBy |
| Field measurement | Verification and outcome report | Measures / Verifies |

The relation and impact policy must be selected by the owning domain contract. There is no default transitive trust and no generic assumption that every link is invalidating.

The core does not establish:
- truth or semantic entailment of a source;
- identity or independence of reviewers;
- correctness of external digests or event signatures;
- legal authority to revoke or approve;
- permission to execute work, transfer money or publish private material.

## Threat model

Assume an adversary may attempt to:
- reuse a claim after its source is corrected;
- replay an old support or approval result;
- target a revision with a different digest;
- reuse event IDs with altered payloads;
- provide duplicate sequence numbers;
- omit a dependency edge;
- create a dependency cycle;
- turn an informational link into an authorization edge;
- rely on an expired credential or withdrawn consent;
- make a fresh-looking revision depend on superseded evidence.

The pure core detects structural inconsistencies and propagates impact only across explicitly declared edges. Missing or inaccurate dependency declarations remain a host-governance and integration risk.

## Conformance

The unit suite covers direct and transitive impact, corrected-source replacement, exact-digest mismatch, missing and duplicate dependencies, cycles, event replay, conflicting identifiers, duplicate sequences, replacement validity and independent observations. A four-node directed-graph sweep enumerates all 4,096 possible non-self edge configurations and compares cycle classification to an independent depth-first-search oracle.

See the Evidence Lineage Benchmark Protocol for matched baselines, metrics and qualification targets.
