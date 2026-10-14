# EPI-013T — OSINT tool/source-adapter capability and limitation profile corpus v0.1

Status: FROZEN PROTOCOL/CORPUS CANDIDATE — NOT EXECUTED / NOT QUALIFIED / NOT PASS

Parent: EPI-013 / #2763

## Purpose

Freeze a domain-neutral semantic profile for OSINT/research tools and source adapters before any concrete connector becomes canonical methodology.

The profile records what a tool can observe, what view/corpus it covers, what transformations it applies, what it discloses, and what it cannot establish.

It does not authorize execution.

## Core laws

```text
tool exists != tool available now
same task label != same observation semantics
selected tool != authorized tool execution
provider rank != credibility
TopKOnly != exhaustive coverage
AuthenticatedAccountView != public view
adapter output != source-native bytes
read-oriented != no disclosure
valid historical profile != current tool behavior
```

## Profile identity

Profile:

`mycelix:epi-osint-tool-profile:v1`

Authority:

`ObservationContractOnly`

No profile field may mint collection, credential-use, network, OPSEC, target-admission, lease, write/action, or EPI-admission authority.

## Required semantic dimensions

Each `InvestigationToolProfileV1` binds, as applicable:

- profile/tool/provider identity;
- implementation/version/currentness refs;
- capability classes;
- supported query/input classes;
- produced artifact/evidence classes;
- coverage semantics;
- freshness/currentness semantics;
- pagination/result-limit semantics;
- authentication/view class;
- provider ranking semantics;
- transformation/normalization semantics;
- disclosure surfaces;
- retention/provider-history surfaces;
- side-effect class;
- reproducibility/determinism class;
- known limitations;
- policy/jurisdiction/ethical constraint refs;
- qualification refs.

## Initial capability vocabulary

The fixture exercises:

- `WebSearch`
- `ScientificLiteratureSearch`
- `CodeForgeSearch`
- `CtiFeedLookup`
- `WebArchiveLookup`
- `BrowserRenderedCapture`
- `MediaProvenanceInspection`

Capability is descriptive only.

## Coverage vocabulary

At minimum preserve:

- `ExactFiniteCorpus`
- `DeclaredProviderCorpus`
- `ProviderRankedSubset`
- `TopKOnly`
- `TimeBoundWindow`
- `GeographicSubset`
- `AuthenticatedAccountView`
- `PersonalizedView`
- `BestEffortSearch`
- `CoverageUnknown`

Coverage may be narrowed further by an execution/search receipt.

## View identity

The fixture distinguishes anonymous/public and authenticated/account-specific views.

```text
authenticated result set != public result set
```

Credentials are never embedded in this profile.

## Ranking semantics

Provider ranking is retained as source-provided ordering metadata.

```text
provider rank 1
!= most credible
!= most independent
!= most relevant under Symthaea profile
!= true
```

## Transformation semantics

Profiles may declare transformations such as:

- HTML/text extraction;
- OCR;
- translation;
- media re-encoding;
- resizing;
- provider-side deduplication;
- generated summaries;
- metadata stripping.

Material transforms should later bind EPI-004 derivation profiles.

## Disclosure surfaces

A profile may declare surfaces including:

- query/search terms;
- target identifiers;
- DNS/transport metadata;
- headers/body;
- account/provider identity;
- IP/network origin;
- uploaded media/files;
- remote-model prompts;
- provider analytics/telemetry;
- stored search history;
- retained result/capture data.

These feed OPSEC intent construction; they do not decide OPSEC policy.

## Side effects

V1 distinguishes:

- `ObservationOnly`
- `ReadWithRemoteDisclosure`
- `StateCreating`

The first two remain non-write semantics but may still disclose sensitive queries/content.

## Tool drift/currentness

A profile binds an exact implementation/provider behavior version and a currentness state.

`HistoricalKnownProfile` may remain valid evidence for a past investigation without implying the tool is currently available or unchanged.

## Known limitations

The fixture uses typed limitation classes including:

- `ResultCap`
- `PaginationCap`
- `TemporalCoverageLimit`
- `GeographicCoverageLimit`
- `IndexingDelay`
- `DeletedContentBlindSpot`
- `AuthenticationRequired`
- `PersonalizationPossible`
- `ProviderRankingOpaque`
- `ArchiveIncomplete`
- `MetadataStripped`
- `MediaReencoded`
- `RateLimited`
- `CoverageUnknown`

## Synthetic profiles

The machine-readable fixture contains synthetic providers only:

- `T_WEB_PUBLIC_TOPK` — anonymous web search, ranked Top-K subset;
- `T_WEB_AUTH_VIEW` — same nominal WebSearch capability but authenticated/personalized view;
- `T_SCI_FINITE` — exact finite synthetic scholarly corpus;
- `T_CODE_BEST_EFFORT` — best-effort code-forge search with indexing delay;
- `T_CTI_WINDOW` — time-bounded CTI feed lookup;
- `T_ARCHIVE_PARTIAL` — incomplete archive lookup;
- `T_BROWSER_CAPTURE` — browser-rendered capture with DNS/TLS/HTTP/telemetry disclosure surfaces;
- `T_MEDIA_PROV` — deterministic local media-provenance inspection with no network disclosure.

## Required deterministic semantics

1. `T_WEB_PUBLIC_TOPK` and `T_WEB_AUTH_VIEW` share a capability class but are not interchangeable observation profiles.
2. Top-K may never be interpreted as exhaustive coverage.
3. authenticated/personalized view remains distinct from anonymous/public view.
4. provider ranking remains separate from credibility/truth.
5. `T_SCI_FINITE` may support exact finite-corpus coverage only for its declared corpus commitment.
6. `T_CODE_BEST_EFFORT` preserves indexing-delay limitation.
7. `T_CTI_WINDOW` preserves time-window bounds.
8. `T_ARCHIVE_PARTIAL` cannot establish absence outside its archive coverage.
9. browser capture's read-oriented behavior still exposes multiple OPSEC surfaces.
10. local media provenance inspection can be `ObservationOnly` with zero network disclosure, but its output still does not establish media truth.
11. stale/historical profile cannot silently satisfy a `CurrentQualified` requirement.
12. selected profile cannot become execution authority.

## Metamorphic ratchets

Future qualification must prove, at minimum:

- changing Top-K to exact finite corpus changes coverage semantics;
- switching public to authenticated view changes profile identity;
- deleting a declared transform invalidates transform ancestry claims;
- making a profile stale prevents current-tool claims;
- adding an upload surface changes OPSEC-disclosure semantics;
- changing provider ranking does not change credibility fields because none exist in this profile;
- changing capability order only leaves set-semantic capability identity unchanged if the canonical profile says it is a set;
- changing result cap changes the profile/receipt identity;
- changing side-effect class from ObservationOnly to StateCreating cannot preserve the old authority ceiling.

## Symthaea boundary

A future `ToolSelectionCandidateV1` may reason over these profiles, but:

```text
ToolSelectionCandidate
!= connector authority
!= OPSEC permit
!= credential authority
!= target admission
```

## Investigation-capsule boundary

EPI-012 may record the exact tool profile proposed and/or actually used, together with execution/search receipt refs and observed truncation/coverage.

Methodology changes therefore remain distinguishable from evidence changes.

## Nonclaims

EPI-013T does not establish tool correctness, provider honesty, current availability, exhaustive world coverage, privacy compliance, safe credential use, legal permission, source truth, source independence, collection authority, or action authority.
