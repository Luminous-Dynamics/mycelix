# MYC-INT-018V — H2 Lettuce Cultivar Admission

Status: research/admission fixture only. Tracks #3311. Parent: MYC-INT-018U / PR #3310.

## Purpose

Compare concrete lettuce cultivars against the H2 reference profile without turning supplier claims into local performance claims or declaring a scalar winner.

Canonical fixture:

`docs/interoperability/fixtures/MYC_INT_018V_H2_LETTUCE_CULTIVAR_ADMISSION.json`

## Current South Africa candidate set

The first census records five cultivars currently described by Rijk Zwaan South Africa as suitable for hydroponic production:

- Sandalina RZ;
- Kireve RZ;
- Extranet RZ;
- Klee RZ;
- Station RZ.

These records preserve supplier-stated characteristics and source URLs. They do not establish local stock, DWC qualification or measured yield.

## Admission model

Allowed dispositions:

- `AdmittedForH2ReferenceProfile`;
- `AdmittedWithDeclaredLimitations`;
- `MoreEvidenceRequired`;
- `IncompatibleWithH2ReferenceProfile`;
- `Deferred`.

Every initial candidate remains `MoreEvidenceRequired`.

There is deliberately no weighted overall score.

## Evidence dimensions

A cultivar record keeps separate:

- exact cultivar and supplier identity;
- hydroponic suitability claim;
- cultivation-season claim;
- South Africa catalogue/source evidence;
- architecture/head/leaf form;
- bolting/heat notes;
- tipburn notes;
- resistance codes as supplier claims;
- DWC-specific evidence if any;
- cycle-duration evidence under exact conditions if any;
- exact seed-lot provenance once procurement begins;
- geometry/spacing compatibility;
- known unknowns.

## Important boundaries

```text
hydroponic-suitable
!= DWC-qualified in our geometry

high-yielding supplier description
!= measured H2 yield

year-round supplier description
!= climate-independent performance

South Africa catalogue listing
!= exact seed lot in stock

resistance code
!= complete disease immunity
```

## Seed-lot identity

The eventual physical H2 run must bind the delivered seed lot or package generation separately from cultivar identity.

```text
same cultivar name
+ different seed lot / treatment / source
!= same physical evidence subject
```

The run record should retain supplier/lot/batch identifiers when available, treatment/coating status, pack date/expiry where supplied, and deviations from the admission record.

## Geometry boundary

Before admission for the actual H2 system, check mature canopy/head dimensions, raft spacing, root-zone spacing, air-flow access, light uniformity, service access and intended harvest method.

A reasonable cultivar may still be incompatible with the chosen small showcase geometry.

## Nonclaims

018V does not select a winning cultivar, establish seed availability, DWC performance, harvest duration, food safety, marketability, economic viability, N2 maturity or food independence.
