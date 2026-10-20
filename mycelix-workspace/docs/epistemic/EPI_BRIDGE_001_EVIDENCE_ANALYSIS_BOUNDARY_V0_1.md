# EPI-BRIDGE-001 — Mycelix ↔ Symthaea Evidence-Analysis Boundary v0.1

Status: architecture freeze candidate

Tracks: #2627, #2625

## Purpose

Freeze the first transport-neutral boundary between the Mycelix epistemic-evidence substrate and Symthaea research/reasoning before either project imports the other's internal types or gains authority through a bridge implementation.

The bridge exists to preserve evidence identity, lineage, uncertainty, dependency state, and analysis ancestry while keeping semantic ownership explicit.

## Ownership theorem

```text
Mycelix
= semantic evidence / claims / policy root

Symthaea
= research / reasoning / hypothesis / investigation engine
```

Therefore:

```text
Mycelix evidence object
!= Symthaea scientific evidence admitted

Symthaea analysis
!= Mycelix canonical relation admitted

bridge transport success
!= semantic validity
!= factual truth
!= action authority
```

Neither repository's internal Rust module layout is the protocol.

## Architectural shape

```text
Mycelix internal EPI graph
        |
        v
EvidenceExportBundleV1
        |
        | transport-neutral interchange
        v
ImportedEvidenceBundleV1
        |
        v
Symthaea bounded analysis
        |
        v
AnalysisCandidateBundleV1
        |
        | transport-neutral interchange
        v
Mycelix candidate import
        |
        v
explicit EPI admission / assessment
```

There is intentionally no direct:

```text
Symthaea output -> canonical Mycelix fact
```

and no direct:

```text
Mycelix evidence -> Symthaea grammar/curriculum/action authority
```

## Bridge profile identity

Every bridge object MUST bind an explicit versioned profile identity.

Changing any load-bearing semantic field, canonical framing rule, supported object family, qualification-state encoding, completeness vocabulary, or authority ceiling requires a new bridge profile.

Serde/JSON field order is not protocol identity.

A future executable profile should define canonical bytes independently of whichever transport encoding carries them.

## EvidenceExportBundleV1

The Mycelix -> Symthaea direction exports a closed evidence subject under an exact frontier/cutoff.

A first profile should be able to bind references to:

- artifact snapshots and exact selectors;
- assertion occurrences;
- claim and hypothesis identities;
- evidence relations and relation assessments;
- known dependency edges and dependency assessments;
- source-reliability, information, uncertainty, and calibration assessments where present;
- derivation/provenance activities;
- currentness and source limitations;
- redaction/omission declarations;
- purpose/export-policy references where required;
- producer implementation/profile identity;
- exact export frontier/cutoff identity.

The bundle should retain native EPI identities and qualification states rather than flattening them into prose.

### Export authority ceiling

```text
object exported
!= object independently revalidated
!= evidence complete
!= source authentic
!= proposition true
```

## Scoped completeness

The bridge MUST NOT imply completeness merely because serialization succeeded.

A future closed vocabulary should distinguish states such as:

```text
CompleteForDeclaredQueryUnderFrontier
PartialDeclared
RedactedDeclared
DependencyClosureIncomplete
UnknownCompleteness
```

The exact names may change before code implementation.

A stronger completeness state requires a declared scope and a qualified theorem about that scope.

```text
bundle contains N records
!= all relevant evidence included
```

## Frontier semantics

Every export binds the exact frontier/cutoff analyzed.

```text
analysis produced at T2
using frontier F1
!= all evidence available before T2 was included
```

Later evidence creates a new analysis lineage.

Historical analysis is never silently rewritten as though it saw future evidence.

Keep distinct:

- source publication time;
- capture time;
- event time;
- export time;
- analysis time;
- trusted/notarized time;
- currentness.

## ImportedEvidenceBundleV1

Symthaea imports bridge material as external evidence input with an explicit non-authority ceiling.

The imported object MUST NOT itself grant:

- scientific disposition;
- persistent macro/grammar admission;
- curriculum mutation;
- verified internal-memory status;
- tool capability;
- runtime action authority;
- Mycelix write authority.

Current RES-EPI and RES-SEC hardening remains independently load-bearing.

```text
EPI-qualified identity/provenance
!= Symthaea scientific admission
```

## External content remains untrusted data

Raw source content carried inside or referenced by the bridge remains untrusted payload.

```text
artifact text
!= system instruction
!= policy
!= capability
!= tool authorization
```

Prompt delimiters and content warnings are defense in depth, not the authority theorem.

Any future Symthaea ingestion path must preserve the external-content plane separately from trusted system/control metadata.

## AnalysisCandidateBundleV1

Symthaea -> Mycelix output is candidate-scoped.

Candidate families may include:

- assertion candidates;
- support/contradiction/context relation candidates;
- dependency-edge candidates;
- entity-resolution candidates;
- competing hypotheses;
- contradiction findings;
- source-discovery suggestions;
- next-best-information proposals;
- bounded temporal inferences;
- uncertainty/limitations;
- exact model/tool/profile ancestry;
- input EvidenceExportBundle commitment;
- derivation/analysis receipt references.

Use authority-bounded terms such as:

```text
Candidate
Proposed
ObservedUnderProfile
NoRelationDetectedWithinScope
```

rather than unqualified truth labels.

```text
AnalysisCandidateBundleV1
!= canonical EPI relation
!= adjudicated claim
!= scientific disposition
```

## Candidate import

Schema-valid candidate bytes never mutate the canonical EPI graph by themselves.

Required conceptual sequence:

```text
candidate bytes accepted by bounded transport
        |
bridge profile validated
        |
input ancestry validated
        |
purpose/policy admission checked
        |
candidate semantics validated
        |
explicit admission decision
        |
optional canonical EPI object created
```

Every authority transition must be visible and receipt-bearing where applicable.

Evaluator disagreement creates parallel candidate/assessment records rather than latest-write-wins mutation.

## Confidence non-collapse

The bridge preserves EPI-006 and RES-EPI-001R2 separation.

Never map by numeric coincidence:

```text
Symthaea conjecture_score = 0.8
-/-> Mycelix evaluator confidence = 0.8
```

unless an explicit mapping/calibration profile establishes that interpretation.

Keep separately named:

- model/internal score;
- research provenance quality;
- source reliability;
- information assessment;
- relation/evaluator confidence;
- measurement uncertainty;
- model uncertainty;
- calibration evidence;
- scientific disposition.

No bridge-level universal confidence scalar exists in v0.1.

## Dependency non-strengthening

The bridge preserves EPI-005 scoped dependency semantics.

```text
NoDependencyDetectedWithinScope
!= Independent
```

Symthaea may propose a new dependency edge or dependency assessment candidate with evidence/derivation.

It may not rewrite imported dependency state as a stronger fact merely because no dependency was found during its analysis.

Likewise:

```text
three observed dependency groups
!= three independent sources
```

without a stronger declared profile.

## Entity-resolution ceiling

Entity resolution is candidate analysis only unless a separate profile/admission theorem establishes more.

```text
same username
!= same account owner

same device/infrastructure pattern
!= same person

behavioral similarity
!= identity

shared relationships
!= coordination
```

Identity merges must remain reversible until independently admitted, and consequential attribution requires a stronger review/policy boundary.

## Derivation ancestry

Every Symthaea analysis candidate must be able to retain the exact analysis ancestry needed to explain how it was produced.

At minimum, future executable profiles should support references to:

- exact input export commitment;
- model/tool identity;
- analysis profile/version;
- configuration/parameters;
- prompt/template artifact identity where material;
- relevant retrieval/query plan identity;
- deterministic seed where applicable;
- produced candidate identities;
- known limitations.

```text
model generated relation
!= relation independently verified
```

## Redaction and protected evidence

The bridge must support explicit omission/redaction rather than pretending protected evidence does not exist.

Possible future declarations include:

- content omitted by policy;
- selector redacted;
- metadata-only reference;
- restricted-review evidence exists;
- dependency closure incomplete due to access restriction.

A public bundle may preserve commitments/references without exposing protected plaintext.

## Purpose limitation

Person-centered or protected evidence should later bind a purpose/export-policy profile.

Public availability does not itself grant unlimited aggregation, retention, deanonymization, or redistribution authority.

The bridge must not become an implicit generalized people-tracking permission layer.

## Xenia boundary

A later Xenia integration may cryptographically bind:

- export bundle commitments;
- acquisition receipts;
- derivation transcripts;
- analyst/admission decisions;
- purpose/delegation scopes;
- redaction/access decisions.

But:

```text
signature valid
!= content true
!= source honest
!= analysis correct
!= relation admitted
```

Xenia owns cryptographic integrity and authorization facts, not epistemic truth.

## Sol-Atlas boundary

Sol-Atlas is a rendering/sense-making consumer.

A rendering projection should preserve enough lineage to answer:

```text
Why is this edge shown?
What exact evidence bears on it?
Who/what evaluated it?
Was it machine-proposed or admitted?
What contradicts it?
What dependencies are known?
What remains unresolved?
What frontier/cutoff was analyzed?
```

Visual prominence, graph proximity, clustering, or animation MUST NOT strengthen stored epistemic authority.

## Transport boundary

This document does not select JSON, CBOR, MessagePack, Holochain serialization, HTTP, IPC, or another transport.

Concrete transports must separately satisfy EPI-001H2-style hostile-input resource bounds.

```text
transport-valid
!= bridge-semantic-valid
!= EPI-admitted
```

## First executable milestone

The first implementation should intentionally support only a tiny closed slice:

```text
one ArtifactSnapshot + exact selector
+ one AssertionOccurrence
+ one Claim
+ one candidate EvidenceRelation
+ dependency status Unknown
        |
        v
EvidenceExportBundleV1
        |
        v
Symthaea read-only import
        |
        v
one candidate relation assessment
with exact input/model/profile ancestry
        |
        v
Mycelix candidate import only
```

No crawler, autonomous graph mutation, broad Holochain coupling, or automatic admission is required to qualify the bridge.

## Qualification corpus for the first executable child

At minimum require:

1. export/import round-trip preserves every supported native EPI identity and qualification state;
2. frontier substitution changes identity or rejects admission;
3. omitted/redacted material remains explicitly declared;
4. imported evidence cannot mint grammar, curriculum, memory, tool, or action authority;
5. Symthaea result is candidate-only and cannot deserialize as canonical EPI relation;
6. candidate import requires an explicit admission transition;
7. equal numeric scores under different profiles remain non-interchangeable;
8. `NoDependencyDetectedWithinScope` cannot become `Independent`;
9. model/tool/profile substitution changes analysis identity;
10. hostile artifact text cannot alter authority metadata;
11. later evidence creates a new analysis lineage rather than rewriting historical analysis;
12. signature/transport validity alone cannot establish semantic truth.

## Migration rules

1. No qualification inheritance across repositories.
2. No internal crate layout becomes the protocol by convenience.
3. No universal confidence/truth scalar is introduced at the bridge.
4. No source-count independence shortcut.
5. No candidate-to-canonical promotion without explicit admission.
6. No imported evidence grants Symthaea persistent authority by type aliasing.
7. No UI projection strengthens candidate semantics.
8. Null, contradiction, unknown, redacted, stale, superseded, and insufficient states remain first-class.

## Nonclaims

This architecture freeze does not establish:

- a stable executable wire format;
- production transport safety;
- source authenticity;
- factual truth;
- semantic entailment;
- source independence;
- scientific qualification;
- privacy/legal compliance;
- prompt-injection immunity;
- Mycelix write authority;
- Symthaea grammar/curriculum authority;
- tool/action authority;
- deployment readiness.
