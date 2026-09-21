# PRAX-DNA-001A: Adaptive DNA v2 Migration Matrix

Status: **planning / authority only**

This document freezes the intended migration semantics for the Praxis adaptive Holochain surface. It does **not** authorize or contain product DNA changes.

## 1. Authority boundary

No adaptive DNA v2 materialization is authorized until all required authority/evidence gates are satisfied.

At minimum:

1. the semantic source stack is reviewed and executable-qualified at the exact heads selected for materialization;
2. repository issue #299 has a closed inventory theorem for all Holochain-bearing Cargo workspaces;
3. PR #305 (or its reviewed successor) has moved beyond measurement-only inventory and Praxis is classified under exactly one explicit migration authority;
4. the isolated Praxis Holochain workspace has its own resolved dependency/source qualification;
5. the exact parent Praxis DNA, toolchain, WASM artifacts, conductor/runtime identity, and migration subject are frozen before installed-DNA evidence begins.

Current authority theorem:

```text
semantic contract exists
!= source stack executable-qualified
!= workspace migration authority closed
!= DNA materialization authorized
!= installed-DNA semantics qualified
```

If #299/#305 is not closed, this document may be refined but no product entry/link schema should be materialized from it.

## 2. Semantic roots

The v2 adaptive DNA must consume the semantic contracts frozen by the Praxis source stack rather than recreating competing zome-local ontologies.

Relevant subjects include:

- #2467 — learning observation/admission/advisory-estimate authority root;
- #2508 — provenance-complete attempt observation contract;
- #2513 — deterministic advisory BKT projection;
- #2516 / #2540 — evidence-bound learning analysis and legacy PoL trust quarantine;
- #2541 — learner-authored goal intent vs derived progress;
- #2546 — exact-input expiring recommendations;
- #2550 — path plan vs path progress;
- #2569 — session observations vs inferred/period analytics;
- #2575 — learner preference intent vs private profile projection/disclosure;
- #2581 — private calibration aggregate vs policy-qualified content calibration disclosure.

The core invariant across all migrated entries is:

```text
source intent / source observation
!= policy admission
!= derived projection
!= disclosure
!= credential decision
!= runtime authorization
```

## 3. Legacy entry migration matrix

| Legacy entry | Legacy visibility | v2 semantic replacement | v2 default visibility | Lifecycle / integrity policy | Historical migration rule |
| --- | --- | --- | --- | --- | --- |
| `LearnerProfile` | public | `LearnerPreferenceIntent` + `LearnerProfileProjection` + optional `LearnerProfileDisclosureProjection` | preference + full projection private; minimized disclosure explicitly shareable | intent is versioned learner-authored state; profile projection is immutable derived receipt with expiry; disclosures are immutable audience-bound receipts | do not copy the old mixed public object into authored intent; recover only provenance-supported preferences, keep ambiguous/inferred legacy fields historical, recompute projections from complete inputs |
| `LearningStyleAssessment` | private | provenance-complete `LearningEvidenceEvent` / assessment observation | private | append-only source observation; author/subject binding; explicit assistance/provenance; no in-place update | preserve legacy assessment separately unless source metadata is complete; do not synthesize missing task/evaluator/assistance lineage |
| `SkillMastery` | private | `AttemptEvidence` + admitted evidence + `AdvisoryCapabilityEstimate` / deterministic BKT receipt | private | attempt source append-only; estimates are immutable/recomputable derived receipts; no threshold acquires credential authority | historical mutable `SkillMastery` remains legacy projection/cache; never reinterpret it as provenance-complete source evidence |
| `DifficultyCalibration` | public | `PrivateCalibrationAggregate` + `ContentDifficultyCalibrationProjection` + `ContentDifficultyCalibrationDisclosure` | aggregate/projection private; only policy-qualified disclosure shareable | private aggregate immutable over exact inputs; calibration immutable derived projection; public/shareable disclosure requires release policy + release admission | historical public calibration remains compatibility data; do not invent event lineage, cohort admission, analyzer version, or privacy/DP guarantees |
| `Recommendation` | private | `RecommendationProjection` | private | immutable derived receipt; freshness derived from generation/expiry and dependency versions; no persisted `is_valid` | legacy recommendation remains historical output; do not invent exact dependency receipts/reason provenance |
| `LearningGoal` | public | `GoalIntent` + `GoalProgressProjection` | private | intent is learner-authored versioned state; progress is immutable/recomputable policy receipt; positive state is profile-relative | do not treat old public progress or `is_completed` as evidence; retain historical object and migrate intent only where authorship is defensible |
| `SessionAnalytics` | private | `SessionObservationSummary` + `SessionAnalysisProjection` | private | session summary derived from exact source events; inferred analysis is a separate immutable analyzer receipt | do not promote legacy focus/flow/`mastery_gained`/unlock fields into complete provenance; preserve as legacy analytics |
| `AggregatedAnalytics` | private | `PeriodAnalyticsProjection` | private | immutable aggregate receipt over exact session/evidence/projection inputs | do not reinterpret `skills_mastered` or `mastery_improvement` as universal capability truth; recompute under named profiles/estimators |
| `AdaptivePath` | private | `AdaptivePathPlan` + `AdaptivePathProgressProjection` | private | every adaptation produces a new plan version; progress is a separate policy receipt; navigation counters are derived | do not synthesize exact completion evidence from old `is_completed`, `current_step`, or `completed_steps` |

## 4. New v2 entry classes

The adaptive v2 DNA should distinguish entry classes by semantic authority rather than by UI feature.

### 4.1 Source observations

Examples:

- provenance-complete attempt evidence;
- assessment/learning observations where Praxis is the authoritative producer.

Required behavior:

```text
create   -> validate + author/subject bind
update   -> reject
rewrite  -> reject
```

Deletion/redaction must be an explicit lifecycle rather than an accidental generic delete permission. If legal/privacy requirements require removal semantics, model a separate redaction/revocation/tombstone record and qualify what remains observable.

Persistence means only that a structurally valid observation was authored/stored. It does not mean the evidence is admitted or true.

### 4.2 Learner-authored intent

Examples:

- `LearnerPreferenceIntent`;
- `GoalIntent`;
- learner-authored path plan versions where applicable.

Required behavior:

- declared learner must bind to the authoring agent for learner-authored flows;
- revisions produce a new explicit version/supersession record rather than rewriting historical intent;
- source intent remains distinct from evidence-derived progress;
- private by default.

### 4.3 Derived projections

Examples:

- advisory BKT capability estimate;
- learning-analysis projection;
- learner-profile projection;
- goal-progress projection;
- recommendation projection;
- adaptive-path progress projection;
- session-analysis / period-analytics projection;
- content calibration projection.

Required behavior:

- exact named/versioned producer, model, policy, or estimator provenance;
- exact input event/projection references;
- immutable/recomputable receipt semantics;
- expiry/freshness where time-sensitive;
- cache persistence does not turn the projection into source evidence;
- no implicit credential/trust/authorization authority.

### 4.4 Disclosure / release receipts

Examples:

- minimized learner-profile disclosure;
- content-calibration disclosure.

Required behavior:

- explicit audience or release policy;
- exact source projection commitment;
- minimized fields;
- immutable disclosure receipt;
- explicit expiry/revocation semantics;
- no automatic widening from private projection to public DHT object.

## 5. Visibility matrix

The v2 default should be privacy-preserving.

| Semantic subject | Default |
| --- | --- |
| attempt/source learning evidence | private |
| assessment observation | private |
| learner preference intent | private |
| learner profile projection | private |
| goal intent | private |
| goal progress | private |
| recommendation | private |
| path plan/progress | private |
| session observation/analysis | private |
| period analytics | private |
| private calibration aggregate | private |
| full calibration projection | private |
| BKT/capability projection | private |
| learning analysis | private |
| minimized disclosure/release receipt | shareable only under its explicit disclosure/release contract |

There must be no generic rule equivalent to:

```text
useful aggregate or projection
=> public
```

## 6. Link migration matrix

The legacy link set must not survive unchanged merely for query convenience.

| Legacy link | v2 default | Rationale / replacement |
| --- | --- | --- |
| `LearnerToProfile` | remove/default-deny for full profile | full preference/profile state is private; source-chain-local lookup or private index; share only minimized disclosure |
| `LearnerToAssessments` | remove/default-deny publicly | private source evidence must not acquire public metadata side channel |
| `LearnerToMasteries` | remove/default-deny publicly | legacy mastery state is superseded by private evidence/projections |
| `LearnerToRecommendations` | remove/default-deny publicly | recommendation existence can reveal inferred weakness/goals |
| `LearnerToGoals` | remove/default-deny publicly | goals are private intent by default |
| `LearnerToSessions` | remove/default-deny publicly | session timing/activity is sensitive telemetry |
| `LearnerToAggregates` | remove/default-deny publicly | period analytics are private learner state |
| `LearnerToPaths` | remove/default-deny publicly | path existence/structure reveals plans and inferred gaps |
| `SkillToMasteries` | remove/default-deny publicly | learner-skill state must not become globally enumerable metadata |
| `ContentToCalibration` | allow only to privacy-qualified minimized calibration disclosure when sharing is intended | exact private aggregate/projection must not be linked publicly |
| `GoalToRecommendations` | remove/default-deny publicly | both sides are private learner state; local/private discovery only |

Private state should prefer source-chain-local discovery or a privacy-preserving local index.

### 6.1 Any retained/shareable link must validate

For every allowed link type, integrity validation must check:

- exact link type;
- expected base type/identity;
- expected target type/identity;
- subject/content correspondence;
- tag schema/version when tags are used;
- whether the target has a valid disclosure/release contract;
- create-link author policy;
- delete-link authority (original author or separately explicit policy).

No v2 link operation should be accepted via a generic `RegisterCreateLink { .. } => Valid` or `RegisterDeleteLink { .. } => Valid` fallback.

## 7. Integrity dispatcher policy

The legacy dispatcher currently applies specialized entry validation to only `LearnerProfile` and `SkillMastery`; most other entry variants and generic register update/delete/link operations fall through to `Valid`.

Adaptive v2 must replace this with an explicit matrix.

Conceptually:

```text
StoreEntry/Create
    -> dispatch every app entry to exact create policy

StoreEntry/Update
    -> dispatch every mutable/versioned app entry to exact update/supersession policy
    -> reject source-evidence mutation

RegisterUpdate
    -> verify original + new entry classes, subject identity, author authority,
       and allowed version transition

RegisterDelete
    -> explicit per-entry redaction/deletion policy
    -> no generic Valid

RegisterCreateLink
    -> exact base/target/tag/link-type validator

RegisterDeleteLink
    -> exact link deletion authority validator
```

Unknown/unhandled application entry or link semantics must fail closed rather than inherit authority from a wildcard branch.

System operations such as agent activity/store-record must be handled according to Holochain's callback semantics, but they must not be used as a bypass around app-entry validation.

## 8. Entry operation matrix

### Attempt / source learning evidence

- create: yes, after structural/provenance validation + author/subject binding;
- update: no;
- delete: only through explicitly designed redaction/revocation lifecycle;
- public links: no by default.

### Learner preference intent / goal intent

- create initial version: learner author only;
- revise: new version/supersession semantics; no historical rewrite;
- delete: explicit learner-controlled archival/redaction policy;
- public links: no by default.

### Derived projections

- create: authorized producer acting for exact learner/input scope;
- update: prefer new immutable projection version/receipt; do not mutate provenance basis;
- delete: cache cleanup may be allowed only if it cannot erase source evidence or consequential decision history; define separately;
- public links: no by default.

### Disclosures

- create: source learner/authorized release policy must consent/admit;
- update: no semantic widening in place; create a new disclosure;
- revoke: separate revocation/supersession receipt;
- links: only within explicit audience/release semantics.

### Content calibration release

- exact aggregate + projection private;
- shareable disclosure only after exact release-policy admission;
- threshold-only release must not be labeled DP;
- DP release must retain mechanism/version/epsilon/delta/noise-parameter provenance.

## 9. Coordinator/API migration

### 9.1 `record_attempt`

Legacy behavior:

```text
caller correct: bool
-> mutate SkillMastery
-> return mastery-looking state
```

Migration:

1. add provenance-complete attempt-recording API;
2. persist private append-only attempt source evidence;
3. apply named/versioned admission separately;
4. recompute advisory BKT projection from exact admitted inputs;
5. return source receipt + advisory projection references without credential authority.

The historical API may remain temporarily as a compatibility adapter producing `LegacyIncomplete` attempt semantics. It must not invent missing task, dimension, assistance, evaluator, or trace data.

### 9.2 Mastery/read APIs

New APIs should use `AdvisoryCapabilityEstimate` / BKT projection vocabulary. Legacy `SkillMastery` reads may remain compatibility-only until clients migrate.

### 9.3 Goals

- write/read `GoalIntent` versions separately from `GoalProgressProjection`;
- no API should update a goal source object merely because progress changed.

### 9.4 Recommendations

- return `RecommendationProjection` with exact dependencies and expiry;
- no mutable `is_valid` write operation;
- clients compute freshness and dependency staleness.

### 9.5 Paths

- plan revisions and progress receipts are separate;
- adaptation creates a new plan version;
- current/complete counters are derived.

### 9.6 Analytics

- descriptive session summary is separate from inferred analysis;
- period metrics must preserve estimator/profile relativity;
- no API response should relabel an estimator threshold as universal mastery.

### 9.7 Learner profile

- authored preferences are private intent;
- inferred profile is private projection;
- sharing uses a minimized audience-bound disclosure.

### 9.8 Difficulty calibration

- exact learner-attempt aggregate private;
- calibration projection private;
- only release-policy-qualified minimized disclosure is shareable/publicly indexable.

## 10. Legacy coexistence / dual-read rules

A migration period may require legacy + v2 entries to coexist.

Rules:

1. legacy entries retain explicit `legacy` semantics;
2. reading legacy data does not silently upgrade its provenance or authority;
3. v2 clients prefer v2 entries when available;
4. old records are not rewritten to look v2-complete;
5. migration adapters may preserve historical display values but must expose incompleteness;
6. credential/trust/authorization decisions must not consume legacy mutable summaries as though they were v2 evidence.

Suggested client state:

```text
V2Complete
V2DerivedFromCompleteInputs
LegacyIncomplete
Unavailable
```

not a single ambiguous “valid” flag.

## 11. Privacy adversarial qualification matrix

Installed-DNA qualification must include at least:

### Source evidence

- learner/author mismatch rejected;
- complete attempt with unknown assistance rejected;
- complete attempt missing task/dimension/producer-version/trace rejected;
- source attempt update rejected;
- arbitrary source deletion rejected unless explicit redaction path is used;
- private attempt not publicly discoverable through learner, skill, task, or generic links.

### Goals/profile/path/recommendations

- private goal not publicly discoverable;
- private full learner profile not publicly discoverable;
- private recommendation not publicly discoverable;
- private adaptive path not publicly discoverable;
- stale projection/disclosure cannot be represented as fresh through mutation;
- disclosure cannot widen audience or signal set by in-place update.

### Analytics

- session/period analytics remain private;
- BKT threshold cannot produce an unqualified `mastered` entry;
- `skills_mastered` / `mastery_gained` legacy fields do not become v2 authority;
- provenance padding is rejected.

### Calibration

- private calibration aggregate/event lineage is not DHT-enumerable;
- cohort below release threshold cannot create a valid disclosure;
- disclosure aggregate digest mismatch rejected;
- thresholded release cannot claim DP mechanism metadata;
- malformed DP mechanism/version/epsilon/delta/noise digest rejected.

### Links

- arbitrary base/target pair rejected;
- wrong target entry type rejected;
- private target cannot gain a public discovery link by generic link API;
- unauthorized link deletion rejected;
- malformed/unknown tag schema rejected.

## 12. Semantic adversarial qualification

The installed-DNA test suite must prove the non-equivalences survive persistence:

```text
private persistence != admission
admission != capability truth
capability estimate != credential
recommendation != obligation
completion-under-profile != universal completion
analytics != source evidence
calibration disclosure != learner evidence
```

Tests should inspect both returned coordinator values and stored/queryable state so a coordinator-only check cannot hide an integrity bypass.

## 13. Migration tranche sequence

### PRAX-DNA-001A — migration matrix

This document only.

- no entry/link/schema changes;
- no Holochain dependency changes;
- no product materialization authorization.

### PRAX-DNA-001B — entry schema and visibility materialization

**Blocked until migration authority closes.**

Intended scope:

- introduce v2 private/source/derived/disclosure entry types from reviewed semantic contracts;
- freeze exact entry visibility;
- keep legacy entry types readable during compatibility window;
- no broad coordinator rewrites yet.

### PRAX-DNA-001C — fail-closed integrity lifecycle

**Blocked until 001B + migration authority qualification.**

- per-entry create/update/delete policy;
- author/subject binding;
- source immutability;
- explicit supersession/redaction semantics;
- strict link validators;
- eliminate generic app-operation `Valid` fallthroughs.

### PRAX-DNA-001D — coordinator migration / dual read

- provenance-complete attempt API;
- advisory-estimate API;
- v2 goal/profile/recommendation/path/analytics/calibration APIs;
- compatibility adapters for old callers;
- explicit legacy-incomplete responses;
- no invented provenance.

### PRAX-DNA-001E — installed-DNA qualification

Freeze and seal:

- exact source commit/tree;
- exact isolated Praxis Cargo manifests + lock/source graph;
- Rust/HDK/HDI/Holochain/conductor versions;
- integrity/coordinator WASM hashes;
- DNA manifest/hash;
- installed app/conductor identity;
- adversarial create/update/delete/link results;
- private-state non-discoverability evidence;
- legacy dual-read behavior;
- source tree cleanliness after qualification.

Only this tranche may establish installed-DNA semantic PASS for its exact subject.

## 14. Evidence lineage / environment drift

The qualification process must follow the same evidence-lineage discipline used elsewhere in the repository:

```text
environment drift before evidence
-> reprepare

environment drift after evidence begins
-> do not mix lineages
```

A toolchain, dependency graph, WASM, DNA manifest, conductor, or runtime change after evidence collection begins requires a fresh qualification root.

## 15. Retention-model follow-up boundary

The current adaptive integrity module still contains legacy floating-point retention/forgetting/retrievability helpers that consume legacy `mastery` and `confidence` inputs.

Those functions are **not** silently blessed by this migration matrix.

Create a separate future subject such as `PRAX-RETENTION-001` to define:

- exact input BKT/capability projection references;
- model ID/version/parameter digest;
- explicit forecast `generated_at` and forecast horizons;
- native/WASM determinism policy;
- model calibration/validation status;
- non-credential/non-authorization authority.

Until then, legacy retention outputs remain advisory legacy analytics.

## 16. Exit gate for DNA materialization

`PRAX-DNA-001B` must not begin product materialization until all of the following are true for an exact repository subject:

```text
semantic stack selected and source-frozen
AND executable qualification accepted
AND closed Holochain workspace inventory
AND Praxis assigned exactly one migration authority
AND isolated Praxis dependency/source graph qualified
AND exact parent DNA/runtime/toolchain frozen
```

Anything weaker is planning evidence, not product migration authority.
