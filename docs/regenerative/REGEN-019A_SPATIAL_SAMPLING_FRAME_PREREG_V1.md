# REGEN-019A — Spatial Subject / Sampling-Frame Preregistration v1

Status: preregistration only. This document supersedes the accidentally colliding REGEN-020 draft in #1238. It defines evidence semantics for relating regenerative semantic subjects to versioned spatial references and sampling frames. It creates no GIS authority, cadastral title, land right, representativeness theorem, geofence authority, or physical-action authority.

## Purpose

REGEN-002 gives `RegenerativeSiteId` and `SoilPlotId` stable semantic identity. PEF observations may carry point, bounding-box, or region spatial support. Qualified REGEN-010B deliberately does not infer plot containment or sample representativeness because semantic identity is not geometry.

REGEN-019A freezes the missing spatial bridge while preserving the original program numbering in which REGEN-020 remains reserved for nutrient stock-and-flow.

Core theorem:

```text
semantic subject identity
+ exact spatial revision
+ explicit coordinate reference
+ evidence-backed spatial relation
= reviewable spatial proposition
```

not:

```text
SoilPlotId
= immutable geometry
= cadastral parcel
= ownership
= sampling frame
= representative sample
```

## Stable subject != stable boundary

A site or plot identity may persist while its interpreted physical boundary changes. Every consuming assessment must bind the exact spatial revision used. Boundary correction, split, merge, resurvey, operational redefinition, or source replacement creates a new revision rather than silently rewriting historical evidence.

A minimal future reference may bind `subject_ref`, `geometry_ref`, `geometry_revision_ref`, `crs_ref`, optional content digest, source reference, and declared validity. Exact field names are not frozen here.

## Logical reference != immutable geometry

A stable geometry reference is not proof that the referenced bytes are immutable. Where an owning GIS or registry provides a content digest or immutable object identity, the spatial revision should bind it. Otherwise the proposition remains weaker.

REGEN-019A does not create a universal canonical geometry encoding or a parallel GIS engine.

## CRS and transform discipline

Coordinates without a compatible explicit coordinate reference system are not comparable. PEF WGS84 semantics remain PEF-owned. External geometries may use other CRSs. Material coordinate transforms are derived operations whose transform/implementation identity should be preserved rather than unexplained in-place rewrites.

## Spatial relation is derived evidence

Statements such as `observation support lies within plot revision R` are derived propositions. A later result should bind exact observation support, spatial revision, relation operator, geometry/transform implementation identity, result, and material tolerance/uncertainty policy.

`within`, `contains`, `covers`, `intersects`, and boundary semantics are not interchangeable.

```text
point inside plot
!= complete measurement support inside plot
```

A bounding box may intersect without being contained. Boundary and uncertainty cases stay explicit.

## Sampling frame != plot geometry

A sampling frame is the declared population/support from which a design draws samples. It may bind geometry revision, strata, exclusions, grid/transect rules, temporal window, protocol revision, and intended inferential population.

Changing exclusions, strata, target population, or selection design creates a new sampling-frame revision.

No generic `representative = true` belongs in this contract. Representativeness depends on study design, selection, missingness, heterogeneity, temporal scope, exclusions, measurement process, and analytical assumptions.

## Rights firewall

```text
inside geometry
!= parcel identity
!= ownership
!= lease/access right
!= right to sample
!= right to cultivate
!= action authority
```

Operational plot geometry is also not automatically a watershed, habitat boundary, soil-map unit, hydrologic boundary, contaminant transport boundary, or ecological community boundary.

## Specimen and trial composition

Recommended direction:

```text
SoilPlotId
-> spatial revision R
-> sampling-frame revision F
-> REGEN-018 specimen collection
-> PEF observation/product
-> downstream trial / contamination / suitability interpretation
```

REGEN-015 trial arms may bind exact treatment-area revisions separately from endpoint sampling frames. `treatment applied within area A != endpoint samples representative of all A`.

## Privacy and sovereignty

Exact geometry may be sensitive. The core contract must not require public disclosure. Deployments may use access-controlled geometry, coarse public representations, selective disclosure, commitments/proofs, or local-only relation evaluation with exported evidence identity.

## Initial implementation boundary

Prefer a narrow module of the shared regenerative evidence substrate or a dependency-light sibling. It should own only references/revisions/relation-result semantics, not geometry storage, cadastral authority, networking, Holochain publication, Symthaea, Finance, Marketplace, Climate authority, or physical control.

## Qualification target

A later ProductFrozen campaign should exercise at least:

1. semantic subject identity distinct from geometry revision;
2. historical bindings survive later geometry revisions;
3. missing/incompatible CRS prevents unsupported comparison;
4. logical geometry refs are not mislabeled immutable without content identity;
5. relation operator identity is preserved;
6. `within`/`intersects`/boundary semantics remain distinct;
7. point containment does not imply support containment;
8. sampling frame remains distinct from plot geometry;
9. sampling-frame revision is explicit;
10. no representativeness boolean is synthesized;
11. no ownership/right is inferred from geometry;
12. bounded references revalidate on deserialization;
13. unknown uncertainty/tolerance is not silently replaced;
14. no action-authority field exists.

## Deliberate non-claims

REGEN-019A establishes no cadastral title, ownership, survey-grade accuracy, ecological boundary, sampling representativeness, statistical validity, specimen authenticity, agronomic suitability, treatment efficacy, contamination safety, climate/carbon claim, geofence authorization, governance authority, or physical actuation.

Its proposition is deliberately narrow: bind stable regenerative subjects and sampling designs to exact versioned spatial references strongly enough that later spatial claims are explicit rather than inferred from names or current GIS state.
