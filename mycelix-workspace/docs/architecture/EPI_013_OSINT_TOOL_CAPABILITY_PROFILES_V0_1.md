# EPI-013T — OSINT tool capability and limitation corpus v0.1

## Purpose

Freeze a deterministic, language-independent semantic corpus for EPI-013 / #2763 before implementing runtime OSINT adapters or allowing Symthaea to select them operationally.

Exact Mycelix subject base:

`main@a85369699099d4c7524e502e531735eed4ab36f4`

This subject is documentation/test-vector only under the active `mycelix-workspace` semantic root. It does not revive the older `mycelix-knowledge` truth-engine model and introduces no runtime adapter.

## Governing theorem

```text
tool available != fit for question
tool selected != authorized to execute
request succeeded != coverage complete
returned result != true
tool says no results != evidence absent
read-only API != no outbound disclosure
tool output != canonical EPI evidence
profile limitations != actual run state
```

A tool profile describes declared capability, limitations, coverage semantics and disclosure surfaces. It is not an execution permit or an observation receipt.

## ToolCapabilityProfileV1

The synthetic corpus requires each profile to bind:

- exact profile, tool, provider and version identity;
- input and output classes;
- declared capabilities;
- authentication requirement;
- network requirement;
- disclosure surfaces;
- coverage model and coverage preconditions;
- known limitations and blind spots;
- transformations / derivations;
- source-native versus derived-output status;
- archive support;
- retention/cache behavior;
- privacy and OPSEC references;
- resource/rate behavior;
- reproducibility expectations;
- explicit absence of execution authority.

## Frozen synthetic profiles

### Ranked Top-K discovery

A public-search-like profile returns ranked result metadata and locators.

```text
TopK success
!= provider index exhaustively searched
!= web exhaustively searched
```

Provider ranking is a transformation and provider indexing/filtering remain explicit blind spots.

### Paginated advisory-style API

A structured provider can expose pagination, but completion requires the exact profile preconditions, including a stable snapshot/equivalent and every page fetched without error.

```text
last page observed under profile
!= records absent from every other corpus
!= world completeness
```

### Exact finite local archive

A no-network local reader can establish exhaustive search only inside an exact committed finite corpus when every declared entry is scanned.

```text
absent from exact committed archive
!= absent now
!= absent from world
```

### OCR/media extractor

OCR is modeled as a derivation from an exact captured artifact.

```text
captured image
  -> OCR(profile/version)
  -> derived text

OCR text != source-native text
OCR text != source truth
```

Changing the OCR version changes derivation ancestry.

### Authenticated read provider

An authenticated read-only API explicitly exposes query/account/network/timing disclosure surfaces.

```text
read-only
!= no disclosure
```

The provider may also retain request metadata and account-scoped visibility may differ from a public view.

### Degraded/truncated provider

A successful transport with timeout/truncation remains `DegradedPartial`.

```text
request returned some results
+ response truncated or timed out
!= normal profile coverage
```

A degraded run cannot inherit a stronger nominal capability profile's completion theorem.

## Profile versus execution receipt

This distinction is load-bearing.

```text
ToolCapabilityProfileV1
= what the named tool/profile declares it can observe and under what limits

ToolExecutionReceiptV1 (future)
= what this exact run actually did and observed
```

The future receipt must bind actual request/profile identity, bounds, pagination, errors, timeouts, truncation, account/provider view, actual coverage/degradation, disclosure and retention state.

Therefore:

```text
profile says pagination supported
+ run timed out
!= pagination exhausted
```

and:

```text
profile limitations documented
!= actual run limitations recorded
```

## Exact fixture

Schema:

`mycelix:epi-013-tool-capability-corpus:v0.1`

Profile:

`mycelix:epi-osint-tool-profile:synthetic:v0.1`

Authority:

`ProfileDescriptionOnly`

Profiles: 6 synthetic profiles.

Exact compact UTF-8 JSON SHA-256:

`d499b680d6461dbffed9b4ea053d3758ee4bd4e7d3e26658d26334acddc35cf7`

The digest identifies the authored fixture bytes only.

## Required invariants

1. Top-K success does not establish exhaustiveness.
2. Pagination exhaustion remains scoped to the exact provider/profile.
3. Finite local archive absence remains exact-corpus scoped.
4. OCR text remains derived rather than source-native.
5. Authenticated read-only providers still have disclosure surfaces.
6. Degraded success cannot inherit normal complete coverage.
7. Tool profiles do not mint execution authority.
8. Tool output still requires separate EPI admission.
9. Profile metadata cannot substitute for actual execution/run evidence.
10. Symthaea tool-selection candidates remain non-executable.

## Metamorphic requirements

- increasing Top-K does not upgrade coverage to exhaustive;
- omitting a final page invalidates pagination-exhaustion claims;
- changing a finite-corpus commitment changes the scope of any corpus-absence claim;
- changing an OCR engine/profile version changes derivation ancestry;
- marking a remote provider `NoNetwork` is incoherent;
- assigning complete coverage to a timeout/truncated run rejects;
- hiding authenticated-provider disclosure surfaces changes profile/OPSEC semantics and requires explicit review.

## Symthaea bridge direction

Symthaea may later consume these profiles read-only when proposing the next information request:

```text
DiscriminatingObservationCandidate
  + available ToolCapabilityProfile refs
  -> ToolSelectionCandidate
```

The selection should preserve dimensions such as coverage fit, expected blind spots, transformation risk, disclosure/OPSEC cost, reproducibility and resource burden rather than one universal tool score.

```text
ToolSelectionCandidate
!= ToolExecutionReceipt
!= OPSEC permit
!= target admission
!= network/browser authority
```

## Authority ceiling

This corpus has maximum authority:

`ProfileDescriptionOnly`

It introduces no network, browser, filesystem, credential, account, action, evidence-admission, truth or scientific authority.

## Nonclaims

This corpus does not rank real tools, establish provider truth, establish complete web coverage, prove privacy or legal compliance, authenticate sources, admit tool outputs into canonical EPI state, or authorize execution.

## State

**FROZEN CORPUS CANDIDATE / NOT IMPLEMENTED / NOT EXECUTED / NOT PASS.**
