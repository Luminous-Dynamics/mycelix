# EPI-000 — Shared Epistemic Evidence Constitution v0.1

Status: architecture / semantic boundary only

Tracks: #2611

## 1. Purpose

Mycelix and Symthaea already contain several independently useful epistemic systems: the Knowledge/LEM claim model, Symthaea web-research verification, scientific-method evidence boundaries, STEW provenance, domain-specific qualification receipts, and evidence-aware application workflows.

The immediate risk is not lack of features. It is semantic collapse between neighboring concepts that use similar names or numeric ranges while carrying different authority.

EPI-000 freezes a common cross-domain epistemic boundary before additional OSINT, research, media, cyber-threat, civic, scientific, or archival ingestion power is added.

The intended architecture is a semantic waist:

```text
Web / Science / CTI / Civic / Media / Archive adapters
                         |
                         v
               Epistemic Evidence Waist
                  /        |        \
             Mycelix    Symthaea   Sol-Atlas
              store       reason      render
```

This constitution does not create that runtime. It constrains later implementations so they compose instead of inventing contradictory notions of truth, confidence, provenance, independence, or authority.

## 2. Governing non-equivalences

The canonical evidence layer MUST preserve the following distinctions:

```text
artifact exists
!= artifact authentic
!= observation established
!= assertion made
!= assertion entailed by artifact
!= claim supported
!= claim true
!= source reliable
!= information credible
!= evidence independent
!= social consensus
!= normative endorsement
!= material importance
!= action authority
```

Additional mandatory separations:

```text
source reliability != information credibility
information credibility != claim stance
claim stance != entailment
assertion provenance != factual truth
support count != independent corroboration
unique URL != independent evidence
unique domain != independent evidence
new wording != independent evidence
normative agreement != empirical support
material importance != epistemic strength
same numeric range != same epistemic quantity
high confidence != calibrated probability
consensus != proof
receipt exists != receipt authentic
receipt authentic != proposition true
scientific disposition != runtime authorization
```

No canonical API may weaken these into a single boolean or scalar without naming the exact projection profile and preserving the original dimensions.

## 3. Legacy Knowledge / LEM boundary

The existing `mycelix-claim-types` and Knowledge zomes remain compatibility surfaces while migration is explicit.

Their current empirical, normative, materiality, and confidence coordinates MAY continue to be displayed and transported for backward compatibility.

A legacy composite score MUST be treated only as a named compatibility/display projection.

```text
legacy composite score
!= canonical truth score
!= source reliability
!= evidence independence
!= scientific qualification
!= action authority
```

The future evidence core MUST NOT depend on a universal scalar truth/evidence score.

## 4. Canonical semantic planes

Later EPI tranches should expose separate types for at least the following planes.

### 4.1 Artifact plane

Represents exact captured or referenced material.

Candidate concepts:

- `ArtifactId`
- content commitment
- source locator
- media/content type
- capture profile
- capture tool identity
- asserted acquisition time
- exact source snapshot identity

```text
URL known != bytes captured
bytes captured != publisher authenticated
publisher authenticated != statement true
```

### 4.2 Selector plane

Evidence should be able to target the exact portion of an artifact from which an assertion or observation was derived.

Candidate selectors include:

- exact byte range;
- text quote + context;
- text position;
- JSON pointer / structured-field selector;
- DOM/semantic selector;
- image/video/audio region or time span;
- source-code path + line/object identity.

Selectors MUST bind to an exact artifact identity. A mutable URL alone is insufficient evidence identity.

### 4.3 Observation plane

An observation records what an admitted acquisition/measurement process produced under one declared profile.

```text
observation recorded
!= phenomenon objectively true
!= measurement calibrated
!= interpretation established
```

Observations MAY carry uncertainty and calibration references but MUST NOT manufacture stronger claims automatically.

### 4.4 Assertion plane

An assertion records that an identified source/artifact states or expresses a proposition.

```text
source asserted X
!= source supports X independently
!= X is true
```

This distinction is mandatory for web research, media analysis, OSINT, scientific literature, and threat-intelligence ingestion.

### 4.5 Claim / hypothesis plane

Claims and hypotheses are proposition identities suitable for comparison, support/contradiction relations, falsifiers, revisions, and competing explanations.

Claim identity MUST remain separate from:

- source identity;
- wording/paraphrase identity;
- evaluator identity;
- confidence/uncertainty;
- evidence relation identity.

### 4.6 Evidence-relation plane

A closed v1 vocabulary should distinguish at least:

```text
AssertedBySource
SupportsCandidate
ContradictsCandidate
Contextualizes
DerivedFrom
NoRelationEstablished
```

Later stronger profiles MAY add independently qualified entailment/refutation semantics, but assertion-level provenance MUST NOT be silently promoted into semantic entailment.

A relation SHOULD bind:

- exact claim/hypothesis subject;
- exact artifact + selector;
- relation profile/version;
- evaluator identity/version;
- derivation/activity identity;
- uncertainty/limitations;
- evidence references.

Multiple evaluators MAY disagree. Such disagreement MUST remain representable.

```text
evaluator A says support
+ evaluator B says contradiction
!= latest evaluator wins
```

### 4.7 Dependency / independence plane

Evidence independence is a graph property under an explicit scope, not a source counter.

The system MUST be able to represent observed derivation such as:

```text
press release P
    -> article A
    -> article B
    -> blog C
```

and preserve that four URLs may constitute one observed information lineage.

Candidate scoped dispositions should avoid universal overclaiming, e.g.:

```text
DependencyObserved
DeclaredDisjointWithinScope
NoDependencyDetectedWithinScope
IndependenceIndeterminate
```

The vocabulary SHOULD avoid a bare universal `Independent=true`.

### 4.8 Assessment plane

Source reliability, information assessment, claim/evidence relation strength, model fit, scientific disposition, and uncertainty are different dimensions.

They MUST NOT be collapsed solely because they share `[0,1]` representations.

If numeric values are exposed, each value MUST carry named semantics/profile identity.

## 5. Provenance ownership boundary

STEW-006 remains the owner of provenance between stewarded representations and knowledge artifacts in the stewardship domain.

EPI provenance owns the derivation history of epistemic production and evaluation, such as:

```text
artifact capture
-> extraction
-> assertion candidate
-> relation evaluation
-> dependency analysis
-> investigation report
```

The two graphs MAY be bridged explicitly when one relationship has both stewardship and epistemic meaning.

```text
STEW provenance edge
!= EPI evidence relation
```

Neither graph may silently subsume the other's authority.

## 6. Symthaea boundary

Symthaea may consume EPI artifacts and relations for reasoning, hypothesis generation, contradiction search, evidence dependency analysis, experiment planning, and reporting.

EPI evidence does not automatically authorize Symthaea to:

- execute tools;
- mutate external systems;
- publish/disclose protected data;
- promote persistent scientific grammar;
- classify a scientific theorem as qualified;
- grant Mycelix governance/finance/health/civic authority.

```text
evidence admission
!= reasoning authority for every purpose
!= action capability
```

A later cross-repository profile MUST bind any stronger capability explicitly.

## 7. Untrusted external-content boundary

Every externally acquired document, webpage, repository artifact, PDF, media file, message, archive, or data feed is untrusted content by default.

Its embedded text/instructions are evidence payload, not control-plane instructions.

```text
retrieved text says "run command"
!= command authorized

source marked reliable
!= source may issue instructions

claim assessed high-confidence
!= tool capability granted
```

Parsing/extraction MUST occur across a semantic boundary that cannot promote content-plane strings into system/developer/user authority or runtime capabilities.

## 8. Conflict, negative evidence, and absence

The canonical model MUST preserve:

- supporting evidence;
- contradicting evidence;
- null results;
- negative results;
- unresolved relations;
- stale/superseded evidence;
- insufficient evidence;
- dependency conflicts;
- source-identity conflicts;
- evaluator disagreement;
- retractions/corrections where modeled by the source domain.

Absence of detected contradiction is not positive support.

```text
no contradiction found
!= corroborated

no dependency found
!= independent

no error observed
!= valid
```

## 9. Time and currentness

Capture time, publication time, event time, verification time, trusted time, and currentness are distinct.

Later profiles MUST name which time semantics they bind.

```text
published at T
!= captured at T
!= true at T
!= current now
```

No ambient host clock may silently promote historical evidence to current evidence in authority-bearing decisions.

## 10. External standards and interchange

External standards should be supported through adapters where useful, without becoming the canonical authority model.

Candidate mappings include:

- W3C PROV for entities/activities/agents/derivation;
- W3C Web Annotation selectors for exact source fragments;
- STIX/TAXII and MISP for cyber-threat intelligence;
- CASE/UCO for forensic evidence;
- WARC/WACZ for reproducible web capture;
- C2PA for media provenance inspection;
- SCITT-style signed statements/transparency receipts.

For every adapter:

```text
standard-valid object
!= proposition true
!= source authorized
!= Mycelix/Symthaea authority
```

## 11. Receipt hierarchy

Later implementations may define evidence receipts such as:

```text
CaptureReceipt
ExtractionReceipt
RelationEvaluationReceipt
DependencyAnalysisReceipt
InvestigationCapsule
```

Each receipt MUST state its authority ceiling.

Example:

```text
CaptureReceipt
= exact capture operation evidence
!= authentic publisher
!= truthful content
!= independent corroboration
```

Receipt authenticity, if later added through signatures/transparency systems, remains separate from proposition truth.

## 12. Initial implementation train

This constitution freezes the following dependency-gated sequence:

```text
EPI-000  constitution / ownership boundary         <- this document
EPI-001  dependency-light evidence identities
EPI-002  artifact snapshot + exact selector
EPI-003  evidence relation algebra
EPI-004  derivation/provenance + interchange mapping
EPI-005  dependency / independence graph
EPI-006  reliability / information / uncertainty profiles
EPI-007  legacy Knowledge / LEM projection
```

Candidate later tranches:

```text
EPI-010  Holochain materialization profile
EPI-011  protected evidence profile
EPI-012  investigation capsule
EPI-020  WARC/WACZ adapter
EPI-021  CTI adapter family
EPI-022  media provenance adapter
EPI-023  transparency receipt adapter
```

No implementation tranche inherits qualification from this architecture document.

## 13. EPI-001 requirements

The first code tranche should remain deliberately small and dependency-light.

Candidate identities:

```text
ArtifactId
ObservationId
AssertionId
ClaimId
HypothesisId
SourceId
DerivationId
EvidenceRelationId
AssessmentId
```

EPI-001 SHOULD establish only:

- bounded/versioned identities;
- deterministic/canonical identity framing;
- role separation between identity types;
- serialization validation;
- exact profile identity;
- explicit non-authority.

It SHOULD NOT add:

- Holochain;
- network access;
- scraping;
- AI evaluation;
- scoring;
- source reputation;
- evidence relations;
- truth classification;
- action authority.

## 14. Migration rules

Every implementation child MUST obey:

1. No qualification inheritance.
2. No semantic weakening during generalization.
3. No universal scalar truth score in the canonical core.
4. No flag inflation for `verified`, `independent`, `causal`, `novel`, `safe`, or `current` where scoped typed states are required.
5. No source-count-to-independence shortcut.
6. No claim-source-assertion-to-entailment shortcut.
7. No confidence-number fusion without a declared and justified model.
8. No evidence-to-action shortcut.
9. No latest-write-wins conflict resolution for epistemic disagreement.
10. No hidden provenance for learned models, transforms, embeddings, grammars, tools, or prior investigations when they materially influence downstream evidence.
11. Legacy data remains historical/compatibility data unless exact provenance required for a stronger semantic class is actually present.
12. Null, negative, contradictory, inconclusive, stale, and superseded results remain first-class evidence.

## 15. Nonclaims

EPI-000 does not establish:

- factual truth;
- source authenticity;
- source reliability;
- information credibility;
- semantic entailment;
- scientific validity;
- independent corroboration;
- calibrated uncertainty;
- legal evidentiary status;
- provenance completeness;
- archival durability;
- protected-data authorization;
- runtime tool authority;
- Holochain schema validity;
- OSINT completeness;
- production readiness.

It establishes only the semantic and authority boundaries that later exact subjects must preserve and independently qualify.