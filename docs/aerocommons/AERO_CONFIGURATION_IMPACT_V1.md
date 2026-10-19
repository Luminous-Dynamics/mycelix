# AeroCommons Configuration Impact Contract v1

## Status

Draft protocol contract for the AeroCommons evidence and configuration lineage layer.

This document defines the semantics of propagating a configuration change through engineering evidence. It does not determine airworthiness, certification status, or engineering acceptability.

## 1. Purpose

A physical-system change must produce an explicit, inspectable impact set.

Given a predecessor configuration, a proposed ChangeSet, the configuration graph, and requirements/evidence attached to affected artifacts, the system should derive:
1. directly affected artifacts;
2. transitively affected artifacts;
3. evidence that remains valid;
4. evidence that becomes conditionally valid;
5. evidence that is invalidated;
6. evidence requiring human review;
7. analyses, inspections, or tests that may need repetition;
8. unresolved dependencies that prevent a complete impact determination.

The output is a decision-support artifact, not a certification decision.

## 2. Core principle

> No evidence is silently inherited across a configuration boundary.

An unchanged claim may remain applicable only when its declared validity domain and dependency set still cover the new configuration.

A change-impact engine must prefer an explicit unknown or requires_review result over an unsupported inference.

## 3. Configuration identity

Every configuration is content-addressed from a canonical representation.

A configuration contains: configuration identifier; parent configuration identifiers; artifact references; requirement references; applicable constraints; toolchain/provenance references; declared environment; lifecycle state.

Human-readable version numbers are labels, not identity.

## 4. ChangeSet

A ChangeSet describes a transition:
- change_id
- predecessor_configuration
- proposed_configuration
- changed_artifacts
- changed_parameters
- changed_requirements
- reason
- author
- created_at
- supersedes
- review_state

Changes must be machine-readable at the finest granularity available. A statement such as 'wing updated' is insufficient when the source system can identify a changed surface, feature, dimension, material, or interface.

## 5. Dependency graph

Evidence MUST declare its dependencies where practical.

A dependency may target an artifact, sub-artifact, parameter, requirement, material batch, manufacturing process, inspection method, analysis model, simulation model, toolchain, environmental condition, or predecessor evidence.

The graph is directional:
requirement -> design -> analysis -> manufacture -> inspection -> test -> observation

and may also carry feedback edges:
observation -> prediction_error -> analysis/design

## 6. Impact classes

### Unaffected
The evidence dependencies and validity domain are unchanged by the ChangeSet.

### ConditionallyValid
The evidence remains potentially applicable, but an explicit condition must be satisfied.

### Invalidated
A declared dependency changed in a way that makes the evidence inapplicable. The engine MUST record the dependency and rule that caused invalidation.

### RequiresReview
The system cannot establish applicability without engineering judgment or additional evidence. This is the preferred result for unknown dependency semantics.

### Unknown
The system lacks enough structured information to classify the evidence. Unknown MUST NOT be silently converted to Unaffected.

## 7. Propagation rules

1. If a changed artifact is a direct evidence subject, classify the evidence as RequiresReview unless an explicit validity rule proves applicability.
2. If an evidence input is changed, classify the evidence as Invalidated unless its validity domain explicitly covers the new input.
3. If an upstream evidence record is invalidated, downstream derived evidence that depends on it becomes RequiresReview or Invalidated according to its declared dependency semantics.
4. If only metadata unrelated to the claim changes, the evidence may remain Unaffected.
5. If the engine encounters an undeclared dependency, it MUST emit Unknown and a missing-dependency diagnostic.
6. A human attestation cannot make missing physical evidence disappear.
7. Consensus among agents cannot override a failed physical test.
8. A simulation result remains a prediction; it does not become an observation merely because multiple agents reproduce it.

## 8. Negative evidence

Failed analyses, failed tests, rejected inspections, and disputed claims are first-class records.

A change-impact result MUST preserve negative evidence and its relationship to the configuration that produced it.

Deletion is not an acceptable way to resolve a failed result.

## 9. Revalidation obligations

The impact result should emit explicit obligations:
- rerun analysis
- rerun simulation
- repeat inspection
- repeat test
- obtain independent reproduction
- obtain qualified engineering review
- update manufacturing instructions
- update maintenance information
- evaluate certification/conformity implications

An obligation is a workflow output. It is not proof that the obligation has been satisfied.

## 10. Independence

Independent reproduction must remain distinct from the original evidence.

Two records produced by the same toolchain, operator, dataset, or physical artifact MUST NOT automatically be treated as independent merely because they have different record identifiers.

Independence should be represented as evidence with explicit relationship metadata.

## 11. Safety boundary

AeroCommons MUST NOT expose an API whose semantics imply automatic airworthiness approval, automatic certification, automatic release to flight, automatic substitution for a qualified engineer or inspector, or automatic acceptance of a manufacturing deviation.

The protocol can establish provenance and surface obligations. Authority remains with the applicable engineering, manufacturing, inspection, and regulatory processes.

## 12. Example

Configuration C1 contains artifact A1. Evidence E1 establishes a load-test result for A1. A ChangeSet modifies parameter A1.thickness.

If E1 has no validity envelope for thickness changes, the result is RequiresReview or Invalidated according to the declared evidence semantics, with a diagnostic identifying A1.thickness as the changed dependency and an obligation to determine the required analysis/test/review.

If the new thickness is explicitly within a declared validity envelope and all other dependencies remain satisfied, the engine may classify E1 as ConditionallyValid rather than silently carrying it forward.

## 13. Conformance requirements

A conforming implementation MUST:
- preserve predecessor configuration identity;
- preserve ChangeSet identity;
- preserve evidence identity;
- produce deterministic results for identical inputs;
- report unknown dependencies;
- retain negative evidence;
- distinguish prediction from observation;
- distinguish evidence status from engineering authority;
- provide machine-readable reasons for each impact classification.

## 14. Relationship to existing standards

AeroCommons should interoperate with established product-definition and quality-information ecosystems rather than invent replacement CAD, PMI, inspection, or manufacturing formats.

Candidate interoperability surfaces include STEP/AP242, QIF, and related model-based enterprise workflows.

The AeroCommons layer adds agent-centric provenance, lifecycle attestations, dispute state, and configuration/evidence lineage around those artifacts.