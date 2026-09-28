# COS -> ProductiveLoopV1 Refinement Contract

**Status:** Executable reference refinement  
**Parents:** #3332, #3333, #3334  
**Purpose:** demonstrate that COS evidence boundaries can refine into ProductiveLoopV1 semantics without collapsing claim ceilings.

## Refinement rule

A COS record is admissible to a ProductiveLoopV1 obligation only through an explicit evidence binding. Existence of a manufacturing or hydroponic source object is never sufficient by itself.

| ProductiveLoopV1 obligation | COS evidence boundary | Required binding |
|---|---|---|
| planned input != consumed input | ProductionPlan vs MaterialConsumption | plan→consumption |
| declared work != observed work | WorkAssignment vs ObservedWorkEvent | assignment→observed work |
| useful output != qualified output | OutputDisposition vs QualityEvidence | output qualification |
| one success != general capability | Outcome history vs capability evidence | general-capability evidence |
| capability != availability | capability/requirement vs AvailabilityObservation | requirement→availability |
| later success != erased failure | Failure/Loss vs later Outcome | failure-history preservation |
| external dependency != local capability | Material/evidence origin | explicit foreign recognition |
| productive-loop closure != N2 | ProductiveLoopV1 closure | no N2 transition |

## Cross-domain test

The same refinement contract is instantiated for:

- manufacturing;
- H2 hydroponics.

The tests intentionally use identical semantic obligations. Only the claim ceiling changes.

This is an important architectural test: if a boundary is genuinely substrate-level, changing the productive domain should not require weakening the invariant.

## Qualification

The current implementation is **ReferenceModelOnly** until:

1. the executable harness passes in CI;
2. each refinement is mapped to an actual source-owned production path;
3. bounded implementation evidence is produced;
4. formal proof artifacts are linked to the concrete refinement;
5. pilot observations establish domain-specific real-world behavior.

## Nonclaims

Passing these tests does not establish:

- manufacturing qualification;
- crop efficacy;
- food safety;
- economic/ecological performance;
- N2;
- Integral validation;
- general productivity.

The purpose is narrower: prove that the evidence-separation semantics survive domain substitution.
