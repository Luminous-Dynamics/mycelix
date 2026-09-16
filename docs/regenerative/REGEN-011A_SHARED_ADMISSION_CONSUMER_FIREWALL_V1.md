# REGEN-011A — Biomass Shared-Admission Consumer Firewall v1

Status: preregistration only. This document freezes how a future executable biomass/feedstock core may consume the qualified REGEN-019 shared PEF-admission theorem without duplicating it or accidentally upgrading structural evidence admission into sustainability, rights, safety, suitability, recommendation, or execution authority.

## 1. Purpose

REGEN-011 already separates resource occurrence, ecological allocation, rights, lot formation, custody, material state, process admissibility, reservation, execution, and output qualification.

REGEN-011A freezes the evidence-consumer boundary that should be used once REGEN-019D qualifies the shared admission crate.

Core theorem:

```text
qualified shared PEF admission
+ biomass-specific context validation
= reviewable biomass evidence use
```

not:

```text
PEF evidence admitted
= biomass sustainably available
= rights-cleared
= uncontaminated
= process-suitable
= reserved
= executable
```

## 2. Upstream gate

Executable REGEN-011 biomass MUST NOT consume `mycelix-regenerative-admission` as qualified infrastructure until REGEN-019D has earned its own exact-head ProductFrozen PASS.

A REGEN-019C preparation PASS is not sufficient.

A queued REGEN-019D run is not sufficient.

REGEN-010B/010C soil PASSes are behavioral predecessors, not substitutes for REGEN-019D shared-kernel qualification.

## 3. Dependency direction

The initial executable dependency direction should be:

```text
mycelix-core-types
        ^
mycelix-regenerative-admission
        ^
mycelix-regenerative-biomass
```

`mycelix-regenerative-biomass` may also depend on the narrow regenerative identity/core types it actually owns and, where separately justified, the adopted quality-profile contract.

It MUST NOT depend on `mycelix-regenerative-evidence` merely to reuse soil behavior.

It MUST NOT import Holochain, Marketplace, Manufacturing, Symthaea, Climate, Finance, networking, storage, or actuator runtimes into the dependency-light core.

## 4. No second raw-vs-lineaged validator

The biomass core MUST NOT introduce a parallel generic provenance gate equivalent to:

```text
match evidence_class {
    Reported | Observed => ...,
    Derived | Inferred | Forecast | Scenario => require_lineage(...),
}
```

for ordinary PEF admission.

That theorem belongs to `mycelix-regenerative-admission`.

Biomass may inspect evidence class only where the biomass proposition itself materially distinguishes classes after shared admission.

## 5. Required call order

For one biomass evidence binding, the intended sequence is:

```text
caller-supplied resolution
-> construct exact EvidenceExpectation
-> admit_pef_evidence(...)
-> biomass-specific subject/role/context validation
-> later spatial/currentness/specimen/quality propositions where applicable
```

The biomass-specific layer does not run first and then attempt to excuse malformed provenance afterward.

Invalid PEF structure/provenance fails before biomass interpretation.

## 6. Shared admission proves only shared admission

A successful `admit_pef_evidence()` result proves only that the exact supplied PEF candidate satisfied:

- the owning PEF validator;
- the raw-vs-computed provenance rule;
- exact expected observation ID;
- exact expected phenomenon;
- optional exact expected evidence class.

It does not prove:

- that the evidence is current enough;
- that a point lies inside a biomass source area;
- that sampling is representative;
- that a specimen belongs to the lot;
- that custody is authentic;
- that rights are valid/current;
- that a contaminant is absent;
- that ecological removal is allowed;
- that a quantity is on the intended mass basis;
- that a feedstock is process-admissible.

## 7. Exact evidence use remains contextual

The same canonical PEF observation may be validly admitted against one expectation and irrelevant to another biomass proposition.

Therefore:

```text
observation admitted once
!= universally admitted for every biomass role
```

Each consequential biomass use must bind the exact expectation/context it relies on rather than caching an ambient `trusted=true` label.

## 8. No confidence-to-eligibility shortcut

Evidence confidence, producer identity completeness, model type, number of sources, or lineage depth MUST NOT automatically promote a biomass item to ecological, legal, contamination, or process eligibility.

Those propositions retain their own rules.

## 9. Resource occurrence evidence

Evidence that biomass exists in some region or inventory is `ResourceOccurrence` evidence only.

It does not establish:

```text
physically accessible quantity
or ecologically allocable quantity
or rights-cleared quantity
or formed lot quantity
```

Regional forecasts/scenarios may be admitted as properly lineaged PEF products while remaining forecasts/scenarios.

They do not become observed stock.

## 10. Ecological allocation evidence

Ecological retention remains a hard eligibility boundary owned by REGEN-011 semantics.

Shared PEF admission may validate the evidence used by an ecological allocation assessment, but:

```text
valid evidence inputs
!= correct ecological allocation conclusion
```

An allocation result should preserve its own method/profile, assumptions, evidence snapshot, uncertainty, and explicit unresolved state.

## 11. Rights/custody evidence

PEF admission does not establish legal or customary rights.

Rights/custody references may point to separately authoritative records, but biomass must preserve:

```text
ownership
!= custody
!= access right
!= harvest/removal right
!= transfer right
!= processing authority
```

The shared admission crate must not become a rights verifier.

## 12. Material-state evidence

A stable `BiomassLotId` does not imply stable moisture, composition, contamination state, quantity, or custody state.

Material-state evidence must bind the exact lot and relevant state snapshot.

A newly admitted observation does not rewrite an earlier snapshot; it may support a new snapshot/evaluation.

## 13. Quantity basis

The biomass core must preserve the REGEN-011 basis firewall:

```text
AsReceivedMass
!= DryMatterEquivalent
```

A dry-matter conversion is a computed proposition. Its PEF output must pass the shared lineaged-admission rule, while the biomass layer separately proves that the conversion is applied to the intended lot/state/input basis.

Valid lineage does not prove the conversion model is scientifically adequate for every feedstock.

## 14. Contamination boundary

Historical/source hazard indicators and analytical contaminant measurements are different evidence roles.

Shared admission may validate either PEF representation, but biomass MUST NOT infer:

```text
no contamination observation
=> uncontaminated
```

or:

```text
reported clean history
=> analytical safety
```

REGEN-016 remains the contamination evidence/profile authority.

## 15. Spatial boundary

REGEN-019A owns the missing bridge between semantic subject identity and exact spatial/sampling-frame propositions.

Biomass MUST NOT treat PEF `SpatialExtent` as proof that an observation lies within a biomass source area or lot-formation boundary unless the explicit spatial relation theorem exists.

```text
spatial support present
!= source-area containment
```

## 16. Currentness boundary

REGEN-019B owns evidence snapshot/currentness semantics.

Biomass MUST NOT introduce a generic `fresh=true` property or substitute ingestion/retrieval time for unknown observation time.

Shared admission and currentness remain separate:

```text
valid PEF evidence
!= current-enough biomass evidence
```

## 17. Specimen boundary

Where laboratory evidence depends on a physical specimen, REGEN-018 specimen lineage remains separate from shared PEF admission.

A valid analytical PEF result does not itself prove that the tested specimen was collected from, remained bound to, or still represents the declared biomass lot.

## 18. Process-admission boundary

A `FeedstockAssessment` may use admitted PEF evidence plus biomass-specific context, quality and process-profile rules.

It remains distinct from a process reservation:

```text
FeedstockAssessment
!= ProcessInputReservation
```

And reservation remains distinct from execution:

```text
ProcessInputReservation
!= ProcessExecution
```

Neither the shared admission crate nor the biomass evidence core contains actuator authority.

## 19. Error propagation

The first executable biomass implementation should preserve shared admission failures as recognizable causes rather than flattening every failure into `InvalidFeedstock`.

At minimum reviewers should be able to distinguish:

- malformed/invalid PEF evidence;
- computed evidence missing lineage;
- observation ID substitution;
- phenomenon substitution;
- evidence-class substitution;
- biomass-specific semantic failures.

This keeps the failure layer visible.

## 20. Consumer regression target

A first biomass consumer campaign should include at least:

1. raw `Observed` evidence passes shared admission then biomass role validation;
2. raw `Reported` evidence passes where the exact biomass role permits it;
3. bare `Derived` fails through the shared admission error;
4. bare `Inferred` fails through the shared admission error;
5. bare `Forecast` fails through the shared admission error;
6. bare `Scenario` fails through the shared admission error;
7. valid lineaged computed evidence reaches biomass semantic checks;
8. ID substitution fails before biomass interpretation;
9. phenomenon substitution fails before biomass interpretation;
10. class substitution fails before biomass interpretation;
11. unknown observation time remains unknown;
12. `measurement=None` remains absent rather than acquiring a fabricated scalar;
13. spatial support is not treated as source-area containment;
14. valid lineage is not treated as complete producer reproducibility;
15. successful shared admission still cannot manufacture ecological eligibility;
16. successful shared admission still cannot manufacture rights/custody;
17. successful shared admission still cannot manufacture contamination safety;
18. successful shared admission still cannot manufacture process reservation/execution authority.

## 21. No ambient admission cache

The initial core SHOULD avoid an unscoped global cache of “trusted/admitted observation IDs.”

Admission is relative to exact expectation and exact candidate bytes/state.

If later caching is introduced for performance, cache keys and invalidation semantics require a separate theorem so one successful expectation cannot bleed into another.

## 22. Product qualification independence

Even after REGEN-019D passes, executable REGEN-011 must earn its own exact ProductHead qualification.

```text
REGEN-019D PASS
!= REGEN-011 PASS
```

The biomass campaign should bind the exact shared-admission dependency identity/lock graph it actually executed.

## 23. No shared-kernel widening in the consumer

If biomass discovers a missing generic admission semantic, it should not patch around it locally and later call that equivalent.

The options are:

1. prove the requirement is biomass-specific and keep it in biomass; or
2. revise/requalify the shared admission theorem in a new REGEN-019 product lineage.

This prevents local workaround drift.

## 24. Deliberate non-claims

REGEN-011A establishes no current biomass availability, ecological sustainability, title/right, custody authenticity, specimen authenticity, contamination safety, process suitability, agronomic suitability, treatment efficacy, climate/carbon claim, marketability, economic optimality, resilience superiority, governance authority, process-execution authority, or physical actuation.

Its proposition is deliberately narrow:

> once REGEN-019D qualifies the shared PEF-admission kernel, biomass must consume that theorem as a narrow precondition and independently establish every biomass-specific proposition that follows it.
