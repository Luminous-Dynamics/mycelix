# EPI-012T — Evidence-bearing Investigation Capsule v0.1

Status: protocol/corpus subject only. **NOT EXECUTED / NOT QUALIFIED / NOT PASS.**

Parent: EPI-012 / #2762.

## Purpose

Freeze the first Mycelix-side investigation record for the synthetic reservoir investigation while preserving Symthaea as the reasoning owner and OPSEC/web systems as collection-authority owners.

The capsule records **what happened and what was visible**, not what is true.

```text
InvestigationCapsule
!= reasoning engine
!= search planner
!= truth
!= collection authority
!= scientific disposition
```

## Authority ceiling

The machine-readable authority is:

```text
RecordOnly
```

with:

```text
reasoning_authority = false
collection_authority = false
action_authority = false
```

No field in the fixture is a network/tool capability.

## Frozen fixture

Path:

`docs/architecture/fixtures/EPI_012_INVESTIGATION_CAPSULE_V0_1.json`

Git blob at authoring:

`0d11fb28ce351ae58bc026629d82bd03c6edf197`

No independent SHA-256 claim is made by this document; a qualifier should derive one from the exact committed bytes.

The fixture is fully synthetic and contains no real person-centered data.

## Frontier history

The capsule preserves two explicit evidence frontiers:

```text
F1 -> F2
```

F1 contains the original station report and two downstream reports.

F2 adds:

- a maintenance/calibration record;
- explicit shared-lineage evidence for the three reporting artifacts;
- two search records;
- an invalidated assumption state;
- a later Symthaea candidate analysis;
- the deterministic disconfirmation/Pareto planner trace.

Critical theorem:

```text
F2 exists
!= F1 was rewritten
```

Historical analyses/assumptions remain addressable under the frontier where they were produced.

## Hypotheses remain propositions, not truth fields

The fixture retains:

```text
H1 real level change
H2 sensor/calibration fault
H3 shared upstream reporting lineage
HU insufficient evidence
```

No hypothesis has a `True`, `False`, or `Proven` state.

Symthaea candidate objects remain explicitly:

```text
CandidateUnadmitted
```

until a separate Mycelix EPI admission process acts on them.

## Assumption history

The fixture records one load-bearing assumption:

```text
ASSUMP1@F1
DeclaredWorkingAssumption
```

followed by:

```text
ASSUMP1@F2
InvalidatedWithinProfile
supersedes ASSUMP1@F1
```

The first record is retained.

```text
assumption invalidated later
!= old investigation secretly knew that earlier
```

## Dependency-aware evidence

At F2, AR1/AR2/AR3 are explicitly bound to one observed lineage group:

```text
DEP:G1
ObservedSharedLineageGroup
artifact_refs = {AR1, AR2, AR3}
```

The fixture explicitly forbids promoting that record into `IndependentCorroboration`.

```text
3 artifacts
+ 1 observed lineage group
!= 3 independent observations
```

## Search / negative evidence

The capsule preserves two different negative-search semantics.

S1:

```text
result_count = 0
coverage = UnknownCoverage
finding = NoMatchObservedUnderSearchProfile
```

Permitted:

```text
UnresolvedDueToUnknownCoverage
```

Forbidden:

```text
AbsentFromWorld
```

S2:

```text
result_count = 0
coverage = ExhaustiveWithinDeclaredFiniteCorpus
finding = AbsentFromExactFiniteCorpusCommitment
```

Permitted:

```text
AbsentWithinExactFiniteCorpus
```

Forbidden:

```text
DidNotOccurInWorld
```

## Symthaea planner trace

The F2 capsule references the exact synthetic disconfirmation/Pareto planning subject from Symthaea:

```text
repo    = Luminous-Dynamics/symthaea
head    = 7b1bfbc376e3efe7ed197bd846c0d2d3ec2b8da5
fixture = 09331ce91386f2151e3681f66eb0c341f04aac89
profile = symthaea:next-information:pareto-front:v1
```

The recorded planning trace preserves:

- H1 as `PreferredWithinProfile`, not true;
- disconfirmation candidates D1/D2/D4;
- policy-eligible Pareto front D1/D2/D3;
- D4 as analytically useful but privacy-blocked;
- D5 as dominated by D2 under the exact named profile;
- `PriorFalsifierSearchUnknownCoverage` as a limitation;
- `execution_authority = false`.

Critical theorem:

```text
planner result recorded
!= search authorized
!= OPSEC block removed
!= target admitted
!= connector capability created
```

A domination witness is historical/profile-relative. If proposal coordinates or policy state change later, the old witness remains part of old analysis history and cannot silently become the new current result.

## Protected omissions

The fixture does not embed the protected D4 detail.

Instead it records:

```text
ProtectedInformationNotEmbedded
+ opaque commitment ref
```

This prevents absence of raw protected material from being interpreted as negative evidence.

```text
omitted/protected
!= absent in source world
```

## Presentation boundary

The fixture references a synthetic Sol-Atlas projection:

```text
ATLAS:F2
authority = RenderingOnly
```

The presentation layer may explain:

- which frontier is shown;
- which assumption changed;
- which dependency group exists;
- why a next-information proposal was suggested;
- which proposal was blocked;
- why D5 was dominated under the profile.

Rendering cannot upgrade semantic authority.

## Required invariants

The fixture freezes at least:

```text
F2 does not rewrite F1
invalidated assumption remains historical evidence
Symthaea candidate remains candidate until explicit EPI admission
UnknownCoverage zero-result does not establish absence
finite-corpus absence remains corpus-scoped
planner Pareto front does not grant collection authority
blocked planner proposal remains blocked in capsule history
domination witness is profile-relative historical analysis
capsule integrity does not imply source truth
capsule presence cannot reconstruct search/network/action authority
protected omission is explicit rather than inferred as negative evidence
rendering projection does not upgrade semantic authority
```

## Future qualification

A future EPI-012T qualifier should independently parse the committed fixture and prove:

1. F1/F2 ordering and no historical rewrite;
2. exact assumption supersession;
3. exact dependency-group membership;
4. coherent negative-search interpretations;
5. candidate-only Symthaea import state;
6. exact planner-trace ancestry to the frozen Symthaea subject;
7. blocked D4 remains blocked;
8. D5 domination witness remains tied to the exact profile;
9. protected omission stays explicit;
10. no field/API can be interpreted as live collection or action authority.

## Nonclaims

This subject does not establish investigation completeness, source authenticity, factual truth, source independence, search completeness, privacy compliance, safe publication, legal admissibility, scientific proof, identity, collection authority, or production readiness.
