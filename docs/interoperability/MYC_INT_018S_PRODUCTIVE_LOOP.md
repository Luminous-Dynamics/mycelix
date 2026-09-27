# MYC-INT-018S — ProductiveLoopV1 and Hydroponic H2 Conformer

Status: design/fixture only. Tracks #3303. Child of MYC-INT-018Q/R.

## Purpose

Close the intentional gap between an instrumented process and a productive loop without widening H1 or hard-coding hydroponics as the only path to N1→N2 evidence.

```text
instrumented process
!= productive loop

productive loop
!= whole-node productivity

useful output observed
!= safe / edible / marketable output
!= food independence
```

018S freezes:

- a generic `ProductiveLoopV1` contract;
- a planned `HydroponicProductiveLoopH2V1` conformer that reuses H1 evidence by reference.

## Generic productive-loop contract

A productive loop binds one exact useful-output process under an explicit observation window. It preserves five different evidence families:

1. input/material observations;
2. work observations;
3. process observations;
4. useful-output + loss observations;
5. outcome feedback.

The loop also retains external dependencies/imports, currentness/coverage, conflicts/unknowns and correction/supersession lineage.

### Inputs

```text
recipe / plan / BOM
!= material actually consumed
```

Observed input records require quantity/unit/profile and source/provenance. Estimates remain estimates.

### Work

```text
work performed
!= compensation owed
!= ITC credit issued
!= governance standing
```

ProductiveLoopV1 only needs evidence that work occurred under the declared profile. Compensation/accounting policy stays elsewhere.

### Process

Domain-specific process evidence stays externally owned. The generic loop references it; it does not rename or duplicate the source observations.

### Output

A useful-output observation requires an exact output subject/profile, quantity + unit/profile, event/window evidence and disposition.

For hydroponics:

```text
harvested mass/count
!= edible
!= food-safe
!= nutritionally adequate
!= marketable
```

Those claims require their own evidence/profile.

### Losses

Rejected, failed, spoiled, lost or otherwise unusable output must remain visible. Yield/share metrics require explicit denominators and coverage.

### Outcome feedback

The loop records whether the intended useful output was actually obtained, plus defects/rejections, post-cycle inspection, deviations/unknowns and any required recovery/adaptation.

No universal satisfaction score is required.

## External dependencies

A locally produced output must not erase imported inputs. ProductiveLoopV1 keeps categories such as seeds/genetics, feedstocks, water, electricity, parts, calibration materials, consumables, specialist knowledge and applicable external services explicit.

```text
local output
!= zero external dependency
```

## N1→N2 maturation boundary

018S is shaped around the three frozen 007O requirements:

- `at-least-one-real-productive-loop`;
- `work-material-observations`;
- `outcome-feedback`.

A physical ProductiveLoop run may contribute evidence for all three when the exact profile is satisfied.

But:

```text
one evidence-complete ProductiveLoop
!= N2 automatically established
```

The separate 007Q maturation transition record still evaluates transition evidence and preserves missing/conflicting information.

## Hydroponic H2 conformer

H2 reuses H1 as its process-observation substrate:

```text
H1a/H1b/H1c sensing/process evidence
        +
H2 productive-loop evidence
        =
HydroponicProductiveLoopH2V1
```

H2 must reference H1 evidence identities rather than minting duplicate pH/EC/flow/etc. channels.

### H2 stages

- `H2a` — crop/lot/input/work preregistration;
- `H2b` — cultivation/process observation window;
- `H2c` — harvest/useful-output observation;
- `H2d` — losses/outcome feedback/post-cycle review.

The first fixture deliberately leaves the crop/cultivar profile unbound. A crop profile should be selected only after separate agronomic/hardware research.

## Generalized conformers

The generic contract should also support future profiles such as:

```text
fabrication
material + work + process + manufactured output + inspection

repair
repair demand + parts/work + restored capability + outcome

water treatment
raw input + process/work + qualified output + losses/outcome

energy service
energy/resource input + operations + delivered service + outages/losses
```

Each keeps domain-specific safety/quality/certification outside ProductiveLoopV1.

## Evidence identity

ProductiveLoop evidence does not become authority.

```text
productive loop evidence
!= governance standing
!= federation membership
!= certification
!= actuation authority
```

Symthaea may analyze or predict loop behavior but:

```text
AnalysisArtifact
!= material observation
!= work observation
!= output observation
```

## Synthetic/physical boundary

The H2 fixture is `SyntheticPlannedShowcase` only.

Physical execution requires new evidence-bearing run objects. Do not mutate this planned fixture into physical evidence after the fact.

## Nonclaims

018S does not establish crop performance, food safety, edible output, commercial viability, economic independence, ecological sustainability, N2 transition, or whole-node productivity.
