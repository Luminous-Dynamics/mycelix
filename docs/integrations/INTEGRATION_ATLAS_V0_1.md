# Integration Atlas v0.1

Parent program: INT-ECOSYSTEM-000 / #2871. Contract issue: INT-ATLAS-001 / #2888.

## Purpose

Define a deterministic, non-authoritative index over external-ecosystem integration status. The atlas is presentation/discovery evidence only; all semantic capability and authority remains owned by the domain theorem referenced by each entry.

## Constitutional separations

- atlas entry != provider capability
- `Qualified(ref)` text != qualification unless the referenced domain receipt independently verifies
- provider-wide status != capability status
- read/observation != write/effect authority
- source authentication != semantic truth
- current provider profile != all historical profiles
- UI presentation != authorization

## Closed status vocabulary

`Unavailable`, `Designed`, `SourceCandidate`, `ImplementedUnqualified`, `QualificationQueued`, `Qualified`, `Superseded`, `Conflict`, `Unknown`.

`Draft`, `Queued`, `Skipped`, `Cancelled`, source review, mergeability, and implementation existence are never aliases for `Qualified`.

## Entry coordinates

Every entry binds:

- `domain_family`
- `provider_family`
- exact `provider_profile`
- domain-owned `capability_kind`
- optional descriptive `atlas_dimension`
- `status`
- exact evidence/source refs
- explicit claim ceiling
- optional supersession/conflict refs

`atlas_dimension` is UI grouping only. v0.1 dimensions are:

`IdentityLink`, `ReadObservation`, `HistoricalSync`, `ContentImport`, `ContentExport`, `ReplicaStorage`, `RealtimeSubscription`, `RelationshipObservation`, `ExternalEffect`, `EffectReconciliation`.

No consumer may use an atlas dimension as an authorization/capability type.

## Positive-reference firewall

The atlas serializes references to positive domain evidence; it does not deserialize or construct the positive domain type itself. A document containing `status = Qualified` with an unresolvable, mismatched, stale, or conflicting exact reference is invalid as a positive atlas projection.

## Conflict and supersession

For one exact cell, incompatible positive refs produce `Conflict`. Input order never selects a winner. A superseded ref remains historical evidence but cannot become current-ingestion support unless the owning domain theorem explicitly allows that profile for the requested operation.

## Determinism

Canonical generation sorts by:

1. `domain_family`
2. `provider_family`
3. `provider_profile`
4. `capability_kind`
5. `atlas_dimension`

Arrays used as sets are sorted/deduplicated. Duplicate entries are accepted only when byte-semantic content is identical; conflicting duplicates fail closed.

## Initial programs

The v0.1 registry covers the eight programs established under #2871:

- Steam / #2872, profile corpus #2880
- Matrix + Discord / #2873, profile corpus #2881
- GitHub + GitLab / #2874, profile corpus #2883
- WebDAV + Nextcloud + S3 / #2875, profile corpus #2882
- ActivityPub + ATProto / #2876, profile corpus #2884
- ORCID + OpenAlex + Crossref + DataCite / #2877, profile corpus #2885
- OpenStreetMap / #2878, profile corpus #2886
- Matter + Home Assistant / #2879, profile corpus #2887

FIN-PROVIDER-CAP-001/#2823 is comparison evidence only; Finance does not depend on this atlas.

## UI projection

Leptos may render a matrix such as:

`Provider/Profile | Identity | Read | Sync | Import | Export | Effects | Reconcile`

but every cell retains exact status text and evidence links. No overall score, rating, tier, readiness badge, or provider-wide green state is defined.

## Fixture corpus

`fixtures/INT_ATLAS_001_V0_1.json` contains only synthetic/descriptive cases. It deliberately includes no real qualified integration capability. Its positive-looking adversarial cases are expected to fail or remain non-positive unless an exact independently verified domain reference is supplied.

## Claim ceiling

A future PASS establishes deterministic indexing/projection only. It does not qualify an integration, authenticate an account/provider, establish currentness, authorize effects, establish provider truth, or grant Symthaea autonomous authority.
