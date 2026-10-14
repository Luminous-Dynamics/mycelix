# EPI-INTEG-001T v0.2 — Closed-world investigation loop manifest

Status: **FROZEN DESIGN/CORPUS / NOT EXECUTED / NOT QUALIFIED / NOT PASS**

Owner: EPI-INTEG-001 / #2637.

## Purpose

Extend the original closed-world evidence round-trip fixture with one deterministic, zero-network investigation loop spanning the current Mycelix and Symthaea OSINT subjects without copying semantic ownership across repositories.

The manifest tests whether identities, authority ceilings, coverage limits, blocked proposals, methodology choices, and historical frontier state survive the handoffs.

```text
successful investigation-loop fixture
!= source truth
!= search completeness
!= tool correctness
!= collection authority
!= production readiness
```

## Exact repository subjects

### Symthaea implementation chain

- RES-OSINT-001A-r2 / PR #5448
  - head `10e75ee462456023d7084a19db75b104aefccad1`
  - tree `663bb5c7e2600923c6ce9facdf87ec451bb3c762`
  - role: bounded investigation vocabulary + secret-safe diagnostics
- RES-OSINT-001B-r2 / PR #5450
  - head `8f3707f612fc77e6ef62cb4f91f4fcf9552b4009`
  - tree `08f8a067a2abd12bec559a201d99a8e78797cb4f`
  - role: deterministic dependency + negative-search analysis

### Symthaea frozen reasoning corpora

- RES-OSINT-001C / PR #5437
  - head `7b1bfbc376e3efe7ed197bd846c0d2d3ec2b8da5`
  - fixture blob `09331ce91386f2151e3681f66eb0c341f04aac89`
  - profile `symthaea:next-information:pareto-front:v1`
- RES-OSINT-QUERY-001T / PR #5444
  - head `d58e6655d01643062a833f9e7c98b5eefadb06e4`
  - fixture blob `edf685ff4ed1e05a859963babad53c66bbf6b07b`
  - profile `symthaea:osint-query-strategy:v1`
- RES-OSINT-TOOL-001T / PR #5441
  - head `9959bdc3d2f998d3265d46f7cbd6b3818cb18b4b`
  - fixture blob `7f55f73e69477594cfff2e2a5a93c9454dc5e19e`

### Mycelix semantic corpora

- EPI-012T / PR #2781
  - head `f3b1328ba9ffeba9712fd458742a19b0bd29f827`
  - fixture blob `0d11fb28ce351ae58bc026629d82bd03c6edf197`
  - role: RecordOnly investigation capsule
- EPI-013T / PR #2784
  - head `3ce0c6ceeda41419512185c2a938f391d1592e3f`
  - fixture blob `2536cfbba7402c19a62417904b2c732fbfb4ac88`
  - profile `mycelix:epi-osint-tool-profile:v1`

### Required safety gate

RES-SEC-001B / Symthaea PR #5416 remains an upstream safety gate for live research admission. Its current subject is `81892bd9bc009df2c9036db58b2603521ebcdcdc`; queued/unexecuted does not satisfy this gate.

## Synthetic investigation

Use the existing reservoir scenario and preserve four live hypotheses:

- `H1` physical level change;
- `H2` calibration/sensor fault;
- `H3` apparent corroboration caused by shared reporting lineage;
- `HU` insufficient evidence.

The fixture starts from frontier `F2`, where three reports are already known to share one observed lineage group, an incomplete falsifier search returned zero results, and a maintenance record challenges the initial independence assumption.

## Required deterministic loop

### 1. Investigation state

001A-r2 must preserve the live hypothesis set, explicit insufficiency alternative, append-only assumption history, explicit unknowns, proposal-only search plans, and candidate-only authority.

Default diagnostics must not expose exact investigation reference strings; exact values remain available only through explicit accessors.

### 2. Dependency / negative-search analysis

001B-r2 must preserve:

```text
asserting artifacts = 3
observed shared-lineage groups = 1
```

with exact artifact membership.

The prior zero-result search with `UnknownCoverage` must remain:

`UnresolvedDueToUnknownCoverage`

and may not support H1.

### 3. Disconfirmation / next-information planning

Under `symthaea:next-information:pareto-front:v1`:

- disconfirmation candidates for H1 = `{D1,D2,D4}`;
- policy-eligible disconfirmation candidates = `{D1,D2}`;
- eligible Pareto front = `{D1,D2,D3}`;
- D4 remains analytically useful but privacy-blocked;
- D5 remains dominated by D2 under the exact profile.

`Pareto-nondominated != authorized`.

### 4. Anti-confirmation query strategy

The query-strategy stage must include at least:

- neutral/descriptive family;
- H1 disconfirmation family;
- H2 alternative-explanation family;
- H3 dependency-lineage family.

Wording-only clones do not increase methodological diversity.

Protected query candidate Q7 remains visible but blocked. Query commitments never become disclosure permission or search attempts.

### 5. Tool methodology selection

Tool selection consumes the exact EPI-013T profile corpus.

For a public discovery task, authenticated/personalized view may be excluded even if it advertises the same broad search capability.

For a request equivalent to “prove this never existed anywhere,” the expected result is:

`NoToolProfileCanEstablishRequestedCoverage`

not a weakened question or a false exhaustive claim.

Method fit remains distinct from OPSEC/collection permission.

### 6. Record-only Mycelix capsule

EPI-012T records the exact reasoning ancestry without re-running or upgrading it.

The capsule must preserve separately:

- methodology selected by Symthaea;
- methodology actually executed.

For v0.2 the latter is explicitly `NotExecuted`.

```text
selected methodology
!= executed methodology
!= authorized methodology
```

The capsule also retains F1->F2 history, assumption invalidation, dependency membership, search coverage, blocked D4, dominated D5, and protected omissions.

## Authority matrix

| Stage | Maximum authority |
|---|---|
| 001A-r2 investigation core | `CandidateAnalysisOnly` |
| 001B-r2 analyzer | `CandidateAnalysisOnly` |
| 001C planner | `ProposalOnly` |
| QUERY-001 | `ProposalOnly` |
| TOOL-001 | `ProposalOnly` |
| EPI-013 tool profile | `ObservationContractOnly` |
| EPI-012 capsule | `RecordOnly` |

No stage creates network, browser, credential, OPSEC, target-admission, durable-lease, evidence-admission, or action authority.

## Negative vectors

Qualification must reject or visibly change state for at least:

1. change a dependency-group member while retaining the old group count;
2. promote `UnknownCoverage` zero results to H1 support;
3. remove `HU`;
4. remove all H1-disconfirmation candidates and strengthen H1;
5. include a privacy-blocked D4 in the executable/eligible set;
6. treat wording-only query clones as methodological diversity;
7. turn Q7 query commitment into a disclosure permit;
8. substitute authenticated search for selected anonymous/public methodology without recording divergence;
9. select a partial archive profile for a world-absence claim and call it exhaustive;
10. record `NotExecuted` methodology as executed;
11. reconstruct execution authority from EPI-012 capsule fields;
12. emit a sensitive investigation ref through default diagnostics;
13. change any pinned repository head/fixture without changing manifest identity/version.

## Qualification rule

This v0.2 manifest is only a conformance target. Individual repository subjects must qualify under their own contracts; no repository may inherit another repository's PASS.

A future end-to-end PASS requires independently checking the pinned subject identities, replaying the deterministic transformations, preserving every authority ceiling, and emitting a receipt over the exact manifest and participating subject identities.

Queued, skipped, cancelled, design-only, source-review-only, partial, identity-mismatched, or receipt-less remains **NOT PASS**.
