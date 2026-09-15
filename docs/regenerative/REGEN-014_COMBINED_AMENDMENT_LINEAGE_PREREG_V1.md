# REGEN-014 — Combined Amendment Lineage Preregistration v1

Status: architecture preregistration only. No production recipe, quality threshold, agronomic recommendation, carbon authority, governance authority, or physical-action authority.

## 1. Purpose

Freeze the evidence semantics for a combined/co-processed regenerative amendment before implementation.

The central theorem is:

```text
qualified input A
+ qualified input B
!= qualified combined output
```

A co-composted or otherwise combined amendment is a new material subject with its own identity, transformation lineage, sampling evidence, and downstream qualification state.

## 2. Identity

REGEN-014 reuses the existing `CoCompostedAmendmentBatchId` from REGEN-002 for the output subject.

Input subjects may include exact `BiocharBatchId`, `CompostBatchId`, biomass/material lots, or other explicitly identified inputs allowed by a later profile.

```text
input identities
!= output identity
```

## 3. Input quantity semantics

Each input contribution SHOULD bind:

- exact source subject;
- quantity actually allocated/consumed;
- exact quantity basis;
- source batch/lot lineage reference;
- unresolved quantity when applicable.

At minimum, `AsReceivedMass` and `DryMatterEquivalent` remain distinct.

```text
100 kg as received
!= 100 kg dry matter
```

A conversion between quantity bases is a Derived evidence product with provenance; it is not an implicit arithmetic coercion.

## 4. Transformation boundary

The combined-batch record SHOULD preserve the declared transformation/process identity and evidence about the process where available.

Generic measurements remain PEF-owned. REGEN-014 binds roles/context to PEF evidence rather than copying value/unit/uncertainty fields.

The preregistration intentionally defines no universal ratio, duration, temperature, moisture, turning, inoculation, or maturation recipe.

## 5. Mass-accounting visibility

Where compatible quantity evidence exists:

```text
identified inputs
=
identified combined output
+ identified side outputs/losses
+ unresolved residual
```

`unresolved residual` is permitted and remains explicit.

```text
unmeasured != zero
```

The model MUST NOT force residual to zero merely to close the ledger.

## 6. No claim inheritance

The combined material starts with no automatic inheritance of the strongest input claims.

Examples:

```text
biochar conforms to profile X
+ compost conforms to profile Y
!= combined amendment conforms to X or Y
```

```text
input A suitable for site S
+ input B suitable for site S
!= combined output suitable for site S
```

```text
one input has carbon-accounting eligibility
!= combined output has carbon-accounting eligibility
```

Any combined-output claim requires its own applicable evidence/profile/methodology.

## 7. Heterogeneity and sampling

Mixture identity does not prove uniform composition.

```text
one sample
!= whole-batch homogeneity
```

A sampling plan, sample-group identity, spatial/lot support, and analytical method may be referenced where relevant, but presence of those references does not prove representativeness or laboratory competence.

Later qualification profiles may declare sampling requirements; the generic lineage layer does not hard-code them.

## 8. Process-state language

Terms such as `charged`, `conditioned`, `mature`, `stabilized`, or `activated` MUST NOT become intrinsic truth merely because they appear in a recipe or operator label.

Where such a state is consequential, it should be represented by an adopted profile and supporting evidence.

```text
process label
!= demonstrated material state
```

## 9. PEF inheritance

REGEN-014 follows the qualified REGEN-010 evidence waist:

- `Reported | Observed` may enter as raw validated PEF evidence;
- `Derived | Inferred | Forecast | Scenario` require validated PEF-2 lineage;
- exact observation ID and expected phenomenon must match;
- missing measurement remains missing;
- unknown temporal support remains unknown;
- valid spatial support does not prove batch representativeness.

## 10. Split / merge / reprocessing

Any later split, merge, dilution, enrichment, or reprocessing that changes material identity requires explicit child/output identity and lineage.

```text
parent qualification
!= child qualification
```

A child may reference parent evidence but must not silently inherit a stronger conclusion than the child evidence supports.

## 11. Quality-profile boundary

REGEN-003 may provide a qualified adopted-profile reference where a combined material is evaluated against an applicable standard.

That remains downstream of material identity:

```text
combined batch exists
-> evidence collected
-> adopted profile evaluated
-> bounded conformance conclusion
```

not:

```text
combined batch exists
-> conforming
```

## 12. Agronomic boundary

```text
combined-output conformance
!= agronomic suitability
!= field efficacy
```

A site/crop/soil-specific assessment or REGEN-015 trial remains a separate proposition.

## 13. Climate boundary

```text
combined amendment lineage
!= durable carbon removal
!= carbon-credit eligibility
```

Climate/MRV authority remains downstream and must bind its own methodology and evidence.

## 14. Proposed later executable layering

A future dependency-light `mycelix-regenerative-amendment` crate SHOULD compose:

```text
REGEN-002 identities
+ qualified REGEN-010 evidence waist
+ qualified REGEN-012 biochar lineage
+ qualified REGEN-013 compost lineage
+ optional qualified REGEN-003 quality-profile references
```

It SHOULD NOT directly depend on Symthaea, Climate, Finance, Marketplace, Holochain networking, or physical-control runtimes.

## 15. Qualification target

A future executable campaign should demonstrate at least:

1. exact input/output identity preservation;
2. partial-input quantity accounting;
3. quantity-basis mismatch rejection;
4. explicit unresolved residual support;
5. no automatic input-claim inheritance;
6. PEF raw-vs-lineaged evidence preservation;
7. split/merge/reprocessing lineage;
8. null/unknown state preservation;
9. serde/top-level revalidation where enabled;
10. ProductFrozen dependency qualification per REGEN-008;
11. exact ProductHead and clean-checkout evidence.

## 16. Deliberate non-claims

REGEN-014 preregistration establishes no process recipe, material uniformity, maturity/stability, contaminant safety, agronomic suitability, application rate, crop response, carbon removal, legal compliance, market value, governance authority, process execution authority, or physical actuation.
