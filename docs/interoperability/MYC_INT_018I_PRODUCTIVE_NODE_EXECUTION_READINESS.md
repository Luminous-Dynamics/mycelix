# MYC-INT-018I — Productive-Node Execution Readiness Gate

Status: dependency-readiness architecture only. Tracks #3237. Child of MYC-INT-018H / PR #3236.

Observed dependency state: 2026-09-27.

## 1. Purpose

Freeze the exact boundary between **architecture/fixtures that may safely continue now** and **executable qualified integration that must wait for upstream evidence**.

The productive-node program now has enough architecture. Its main assurance risk is implementation outrunning shared identity, evidence, translation, currentness, and analysis-protocol owners.

```text
designed
!= implemented
!= qualified

queued
!= PASS
```

This document is intentionally conservative: it prevents local substitutes from becoming accidental permanent infrastructure.

## 2. Readiness vocabulary

Use these states for this document only:

- `QualifiedReusablePrimitive` — exact hosted evidence observed for the narrow theorem being reused;
- `SourceImplementedUnqualified` — source implementation exists but exact qualification is unresolved;
- `BootstrapPending` — construction/bootstrap exists but final product subject is not qualified;
- `DesignedNotImplemented` — issue/spec exists without an executable qualified owner;
- `DomainOwned` — no generic primitive should replace the domain theorem;
- `Designed` — productive-node architecture/profile only;
- `FixtureReadyForSchemaRefinement` — synthetic cases may be refined, but executable qualification is premature;
- `BlockedForQualifiedExecutableReuse` — may be referenced conceptually, but cannot support a qualified product claim today.

These states are not global repository labels.

## 3. Dependency matrix

| Dependency | Current owner | Observed state | Productive-node disposition |
|---|---|---|---|
| `SchemaRef` / `SemanticRef` | MYC-INT-002A / PR #3136; qualifier #3137 | exact qualifier job still queued | `BlockedForQualifiedExecutableReuse` |
| EPI `ObservationId` / `SourceId` / `AssessmentId` | EPI-001 / PR #2614 | bootstrap qualification pending / NOT PASS | `BootstrapPending` |
| translation receipts | MYC-INT-016A / #3130 | design issue only in this audit | `DesignedNotImplemented` |
| `EvidenceLease` | PR #181 | exact hosted run completed successfully | `QualifiedReusablePrimitive` |
| generic/domain currentness | #2688 convergence architecture | generic currentness deliberately rejected | `DomainOwned` |
| Symthaea `AnalysisRequest -> AnalysisArtifact` | Symthaea PR #6205; qualifier design #6207 | source implemented, no qualifier PASS identified | `BlockedForQualifiedCrossSystemReuse` |
| 018E environment refs | PR #3230 | docs/profile only | `Designed` |
| 018F physical observations | PR #3232 | docs/profile only | `Designed` |
| 018G derived assessments | PR #3234 | docs/profile only | `Designed` |
| 018H corpus | PR #3236 | synthetic architecture + manifest | `FixtureReadyForSchemaRefinement` |

## 4. Semantic-reference gate

MYC-INT-002A / PR #3136 owns the current interoperability `SchemaRef` / `SemanticRef` subject.

Its never-merge exact qualifier is PR #3137, pinned to subject head:

`280e9b576c52b5838130c9875a35ad215908fe98`

and qualifier head:

`576844e5b7830efc677267bf2387481cb9ad593d`.

Observed workflow run:

`36299016752`

Current observed job state on 2026-09-27:

```text
status = queued
conclusion = null
```

Therefore:

```text
SchemaRef/SemanticRef source exists
!= exact qualified semantic-reference theorem
```

018E/F/G may continue to use these names architecturally, but executable qualification must wait for exact PASS or a later explicitly qualified successor.

## 5. EPI role-identity gate

EPI-001 / PR #2614 defines role-safe evidence identities including:

- `ObservationId`;
- `SourceId`;
- `AssessmentId`;
- Artifact/Assertion/Claim/Hypothesis/Derivation/EvidenceRelation identities.

The current PR explicitly describes itself as a bootstrap construction subject and states:

```text
SOURCE PREPARED
BOOTSTRAP QUALIFICATION PENDING
NOT PASS
```

It also plans a fresh direct-child final product after exact lock material is promoted.

Therefore 018F/G/H must **not** create local substitutes such as:

```text
ProductiveNodeObservationId
ProductiveNodeSourceId
ProductiveNodeAssessmentId
```

That would fork the identity substrate immediately before EPI convergence.

## 6. Translation-receipt gate

MYC-INT-016A / #3130 owns the intended provenance-preserving translation-receipt semantics:

- source object/schema/version;
- destination schema/version;
- translator/adapter identity;
- losses/omissions;
- resulting object ref;
- source provenance retained.

The current audit identified this as a design issue, not an exact qualified executable owner.

Therefore 018E/H may require translation receipts in architecture/fixtures but should not implement a second local receipt system.

```text
translation requirement known
!= translation implementation owner qualified
```

## 7. EvidenceLease — qualified reusable primitive

PR #181 exact repaired head:

`58fa357e53d7e529362c5f766965498ee557d6ce`

Hosted run:

`34794620487`

Observed job state:

```text
status = completed
conclusion = success
```

with completed successful steps for formatting, tests, Clippy, and lease authority audit.

This is the one dependency in this audit that can be treated as a qualified reusable primitive within its exact theorem.

The ceiling remains load-bearing:

```text
EvidenceLease
!= source authenticity
!= semantic/domain currentness
!= actor authority
!= execution authority
!= external effect
```

018F/H may reuse its lease algebra where the exact theorem applies. They may not turn it into generic currentness.

## 8. Currentness remains domain-owned

MYC-TIME-CONVERGE-001 / #2688 freezes the architecture decision:

```text
DO NOT create a second generic lease algebra
DO NOT create a universal CurrentnessAdmission
```

The reusable composition is instead:

```text
semantic temporal/profile refs
+ source/time evidence
+ EvidenceLease where applicable
+ domain-specific currentness theorem
```

Productive-node implication:

- fast-changing pump state has a domain/profile currentness rule;
- energy interval records are historical interval facts;
- water samples do not automatically represent present water state;
- crop/environment identities may have different lifecycle semantics;
- calibration evidence has its own validity theorem.

No 018F type should mint a universal `current=true` token.

## 9. Symthaea analysis-protocol gate

Symthaea PR #6205 owns the first source implementation of the transport-neutral:

```text
AnalysisRequest
-> AnalysisArtifact
```

read-only protocol.

Exact source head:

`1bb8665575ab5b48e114ddb689d43cc38df9d4e9`

The PR explicitly reports:

```text
SourceImplemented only
```

and says no compile/test/Clippy PASS is claimed before an exact qualifier executes.

Qualifier design is Symthaea #6207 (`SYM-INT-001BQ`).

The current audit did not identify a completed exact qualifier run.

Therefore:

```text
Symthaea protocol source exists
!= qualified cross-system analysis bridge
```

018H may contain synthetic analysis-shaped fixture cases. It must not claim the Mycelix↔Symthaea bridge is qualified until the exact analysis-protocol qualifier succeeds.

## 10. 018E/F/G implementation posture

The candidate structs in 018E/F/G are **semantic sketches**, not permission to freeze Rust APIs now.

Do not copy them verbatim into a new crate before upstream convergence.

Instead, once prerequisites qualify:

1. inspect exact qualified `SchemaRef` / `SemanticRef` API;
2. inspect final qualified EPI role-identity API;
3. inspect translation-receipt owner;
4. map 018E/F/G fields onto those exact owners;
5. delete candidate fields that duplicate upstream data;
6. add only genuinely missing productive-node relations/profiles.

The preferred result is a thin adapter layer, not a new foundational crate family.

## 11. Safe work before gates clear

The following may proceed without making qualified executable claims:

- architecture/docs;
- synthetic fixture refinement;
- fixture schema sketches;
- profile namespace planning;
- deterministic evaluator pseudocode;
- source-domain census;
- unit/profile registry research;
- hidden-oracle design;
- import/export fixture design;
- adversarial/negative test-vector design;
- physical H1 bill-of-materials planning that does not depend on unqualified authority semantics;
- non-authoritative local experiments clearly marked experimental.

## 12. Work that should wait

Do not yet:

- freeze duplicate `SemanticRef` / EPI identity implementations;
- create generic productive-node currentness;
- create a second translation-receipt subsystem;
- claim water/energy convenience fields are verified evidence;
- claim Symthaea analysis integration is qualified;
- call 018H executable or PASS;
- let a physical-control path depend on candidate 018E/F/G types as authority-bearing protocol.

## 13. Executable 018H entry gate

Before opening a qualified executable 018H evaluator product subject, require at least:

### G1 — semantic identity
Exact `SchemaRef` / `SemanticRef` qualifier PASS, or an explicitly qualified successor owner.

### G2 — epistemic role identity
Final role-safe EPI identity subject PASS, or an explicitly qualified converged successor.

### G3 — translation receipts
Executable owner resolved for fixture cases requiring translated external/native schemas.

### G4 — profile reconciliation
018E/F/G candidate fields reconciled against the exact qualified identity/evidence APIs.

### G5 — Symthaea bridge
Exact read-only analysis protocol PASS before any cross-system analysis fixture is used as evidence of bridge qualification.

### G6 — currentness mapping
Every positive live/current fixture identifies its actual domain/profile currentness theorem. No generic substitute.

### G7 — authority lineage
Decision/authorization/effect cases reuse the existing Mycelix authority/effect owners rather than defining productive-node-local authority.

## 14. What can become executable first

Once G1–G4 clear, the first executable 018H tranche should remain intentionally narrow:

```text
parse frozen synthetic fixtures
-> revalidate exact identities/profile refs
-> assert role separation
-> assert required provenance/translation refs
-> assert currentness ceilings
-> assert authority-negative controls
-> emit deterministic conformance disposition
```

It should **not** initially:

- talk to sensors;
- operate a greenhouse;
- call external services;
- issue commands;
- score sustainability;
- decide governance policy.

This creates a small theorem surface suitable for exact qualification.

## 15. Physical integration gate

H1/H2 physical work may collect experimental sensor data earlier, but promotion into the qualified productive-node evidence chain should wait until the observation/profile adapters are executable-qualified.

A useful separation is:

```text
physical bench experiment
!= qualified protocol integration
```

Raw experimental data can be retained and later imported through a qualified adapter with explicit provenance.

## 16. No deadlock requirement

These gates must not prevent useful work.

If an upstream qualifier is delayed by runner capacity, continue with work that does not strengthen claims:

- fixtures;
- test cases;
- hardware planning;
- simulator scenarios;
- comparison methodology;
- export formats;
- adapter mapping tables.

But preserve:

```text
runner delay
!= permission to self-qualify downstream semantics
```

## 17. Dependency-change rule

If any upstream owner changes profile, bytes, identity semantics, or authority ceiling before qualification, 018E/F/G/H must be reconciled against the new exact subject.

Do not preserve a stale candidate mapping merely to avoid updating fixtures.

```text
upstream semantic change
-> mapping review
-> new fixture/profile identity where material
```

## 18. Readiness summary

Current state on 2026-09-27:

```text
018C productive-node architecture       DESIGNED
018D domain-boundary census             DESIGNED
018E production-environment refs        DESIGNED
018F physical-observation profile       DESIGNED
018G derived-assessment profile         DESIGNED
018H cross-domain synthetic corpus      FIXTURE-READY / UNQUALIFIED

EvidenceLease                          QUALIFIED within narrow ceiling
SemanticRef exact qualifier             QUEUED / NOT PASS
EPI role identities                     BOOTSTRAP PENDING / NOT PASS
Translation receipts                    DESIGNED / no qualified executable owner in this audit
Symthaea read-only analysis protocol    SOURCE IMPLEMENTED / NOT QUALIFIED
```

## 19. Preferred continuation

```text
keep refining 018H fixtures safely
+
resolve upstream qualifiers
        |
        v
reconcile exact qualified owners into 018E/F/G
        |
        v
freeze executable fixture schema
        |
        v
implement deterministic evaluator
        |
        v
exact-head qualifier
        |
        v
H1 wet bench / H2 crop rack integration
```

## 20. Nonclaims

MYC-INT-018I does not establish that upstream work will pass, that the productive-node architecture is production-ready, that physical integration is safe, or that any farming/governance/economic model is preferable.

It establishes only the current assurance boundary so downstream implementation can remain truthful about what is qualified and what is not.
