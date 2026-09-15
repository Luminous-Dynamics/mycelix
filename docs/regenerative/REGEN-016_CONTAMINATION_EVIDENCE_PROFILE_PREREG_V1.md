# REGEN-016 — Contamination Evidence Profile Preregistration v1

Status: architecture preregistration only. No universal safety threshold, certification, agronomic recommendation, legal determination, carbon authority, or physical-action authority.

## 1. Purpose

Freeze the evidence semantics for contamination/hazard assessment across regenerative feedstocks, biochar, compost, combined amendments, soils, and related material subjects without inventing a second generic measurement system or silently converting incomplete analytical evidence into a safety claim.

Core theorems:

```text
not detected
!= zero concentration
```

```text
analytes tested
!= all hazards absent
```

```text
passes adopted profile
!= universally safe
```

## 2. Evidence ownership

Generic analytical values, units, uncertainty, spatial/temporal support, source references, and provenance remain PEF-owned.

REGEN-016 adds contamination-domain binding context around PEF observations rather than copying scalar measurement fields.

A contamination profile SHOULD bind:

- exact material/site subject identity;
- matrix/material class;
- analyte or hazard key;
- exact PEF observation ID;
- expected PEF phenomenon;
- analytical qualifier;
- sample/sample-group reference where known;
- sampling-method reference where known;
- analytical/laboratory-method reference where known;
- detection/quantification-limit evidence references where applicable;
- optional adopted quality-profile evaluation reference.

## 3. Matrix identity

Results are interpreted in the declared matrix/context.

Examples include feedstock, biochar, compost, combined amendment, soil, water, or another explicitly declared matrix.

```text
result in feedstock
!= result in output biochar
```

and:

```text
result in one sample matrix
!= result in another matrix
```

Transformation may concentrate, dilute, destroy, create, mobilize, or redistribute constituents. Input evidence cannot automatically stand in for output evidence.

## 4. Analytical qualifier

The profile SHOULD preserve a qualifier concept at least equivalent to:

```text
Quantified
DetectedBelowQuantification
NotDetectedAtDeclaredDetectionLimit
ReportedWithoutQuantitativeResult
NotAssessed
OtherDeclared
```

This qualifier does not replace the referenced PEF observation; it records how the analytical result is being used.

## 5. Non-detect firewall

`NotDetectedAtDeclaredDetectionLimit` MUST NOT be encoded as a numeric zero merely to satisfy an algorithm.

A consequential non-detect interpretation SHOULD bind the applicable detection-limit evidence or method reference.

```text
ND at limit L
!= concentration = 0
```

If the detection limit is unknown, that uncertainty remains explicit.

## 6. Quantification-limit firewall

A detected signal below a declared quantification limit remains distinct from a quantified concentration.

```text
detected < LOQ
!= precise quantified value
```

Any later numerical substitution used for statistical analysis is a derived/model assumption and should carry explicit lineage rather than rewriting the raw analytical result.

## 7. Tested-panel completeness

The profile SHOULD preserve the exact analyte/hazard set actually assessed.

```text
no failure among tested analytes
!= no untested contamination
```

A consuming quality profile may define a required panel. Conformance is possible only relative to that adopted panel/profile and evidence state.

## 8. Source-history hazards vs measurements

REGEN-011 provenance may carry source-history flags such as unknown prior use, treated material history, industrial exposure, or other concern.

Those flags are not analytical measurements.

```text
no known hazardous history
!= analytically clean
```

and:

```text
hazardous source history
!= measured concentration
```

Both forms of evidence may matter and remain separately visible.

## 9. Sampling representativeness

A laboratory result pertains to the sampled material/support described by its evidence.

```text
one sample
!= entire heterogeneous batch
```

Sampling method, sample count/group, batch subdivision, and representativeness remain explicit where evidence exists. The generic profile does not manufacture representativeness.

## 10. Laboratory/method boundary

A method or laboratory reference does not by itself prove method validity, accreditation, competence, chain-of-custody authenticity, or fitness for the decision threshold.

Those are separate claims/evidence sources.

## 11. Unit and conversion discipline

Units remain PEF-owned. A threshold comparison requires compatible units/bases.

```text
unit conversion
= Derived evidence with lineage
```

not an invisible in-place rewrite.

Dry/as-received or other material-basis conversions likewise require explicit evidence and lineage when material to interpretation.

## 12. Adopted quality profiles

REGEN-003 provides the structural mechanism for an exact adopted quality-profile revision.

REGEN-016 may evaluate evidence against such a profile, but the result means only:

```text
given evidence E
+ adopted profile revision P
-> bounded conformance result under P
```

It does not make P universal or prove legal/safety suitability outside its declared scope.

## 13. Unknown and incomplete evidence

The result model SHOULD be capable of preserving outcomes such as:

```text
ConformsUnderProfile
FailsUnderProfile
InsufficientEvidence
NotApplicable
NotEvaluated
```

Exact names may change at implementation, but `InsufficientEvidence` MUST NOT collapse into pass.

## 14. Failed/adverse evidence

A failed contaminant result remains part of the subject's evidence history.

Re-testing MAY create later evidence; it MUST NOT silently erase the previous result. Supersession, sample differences, remediation, or reprocessing should be explicit.

## 15. Biological vs chemical hazards

The generic contamination profile may reference different hazard families, but it MUST NOT assume that evidence appropriate for one family proves another family safe.

```text
chemical panel pass
!= biological/pathogen safety
```

and vice versa.

Specialized safety profiles may extend the evidence requirements.

## 16. PEF provenance admission

REGEN-016 inherits the qualified REGEN-010 evidence boundary:

- raw `Reported | Observed` evidence is admitted only after PEF validation;
- computed `Derived | Inferred | Forecast | Scenario` evidence requires validated PEF-2 lineage;
- exact observation ID and expected phenomenon match;
- missing measurement remains missing;
- valid lineage is not equivalent to scientific validity.

## 17. Batch/material relation

A contamination assessment MUST bind the exact subject/sample context it evaluates.

```text
parent batch pass
!= child blend pass
```

```text
input material pass
!= transformed output pass
```

REGEN-012/013/014 lineage determines which material subject exists; REGEN-016 provides contamination evidence about that subject.

## 18. Agronomic and food-system boundary

```text
contamination-profile conformance
!= agronomic suitability
!= food/feed safety in every use
```

REGEN-017 or specialized downstream profiles own context-specific suitability propositions.

## 19. Climate/carbon boundary

```text
contamination evidence
!= carbon removal
!= carbon-credit eligibility
```

Climate/MRV remains downstream.

## 20. Proposed executable layering

A future dependency-light contamination crate/profile should compose:

```text
qualified REGEN-010 evidence waist
+ exact regenerative material/site subject identity
+ optional qualified REGEN-003 adopted profile
+ qualified batch/material lineage where applicable
```

It SHOULD NOT directly depend on Symthaea, Climate, Marketplace, Finance, Holochain networking, or physical-control runtimes.

## 21. Qualification target

Future implementation should demonstrate at least:

1. exact subject + analyte/phenomenon binding;
2. explicit non-detect vs zero distinction;
3. explicit below-quantification distinction;
4. required-limit evidence handling;
5. incomplete panel does not become universal pass;
6. unit/basis mismatch fails closed unless explicit derived conversion is supplied;
7. PEF raw-vs-lineaged preservation;
8. sample/batch substitution rejection;
9. failed/insufficient evidence preservation;
10. exact adopted-profile revision binding where evaluated;
11. ProductFrozen dependency qualification per REGEN-008;
12. exact ProductHead and clean checkout evidence.

## 22. Deliberate non-claims

REGEN-016 preregistration establishes no universal contaminant limits, material safety, food/feed safety, legal compliance, laboratory competence, sampling representativeness, agronomic suitability, crop outcome, carbon removal, certification, governance authority, market authority, process execution authority, or physical actuation.
