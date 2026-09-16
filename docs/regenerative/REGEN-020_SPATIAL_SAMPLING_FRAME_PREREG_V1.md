# REGEN-020 — Spatial Subject / Sampling-Frame Preregistration v1

Status: preregistration only. This document defines evidence semantics for relating regenerative semantic subjects to versioned spatial references and sampling frames. It creates no cadastral title, land right, representativeness theorem, geofence authority, or physical-action authority.

## 1. Purpose

REGEN-002 gives `RegenerativeSiteId` and `SoilPlotId` stable semantic identity. PEF observations can carry `Point`, `BoundingBox`, or `RegionId` spatial support. REGEN-010 deliberately refuses to infer plot containment or sample representativeness because the plot identity itself does not contain geometry.

REGEN-020 freezes the missing bridge.

Core theorem:

```text
semantic subject identity
+ exact spatial revision
+ compatible coordinate reference
+ evidence-backed spatial relation
= reviewable spatial proposition
```

not:

```text
SoilPlotId
= cadastral parcel
= immutable geometry
= ownership
= representative sampling frame
```

## 2. No new GIS engine

REGEN-020 is not a geometry database, cadastral system, municipal GIS, survey engine, or land registry.

Existing authoritative/operational systems SHOULD remain the owners of detailed geometry where appropriate.

REGEN records bounded references and evidence needed to say which exact spatial representation a later proposition used.

## 3. Stable subject != stable boundary

A semantic site/plot identity may persist while its interpreted physical boundary changes over time.

Therefore:

```text
subject_id
!= geometry revision
```

A consuming assessment MUST be able to bind the exact geometry revision it used rather than silently dereferencing whatever geometry happens to be current later.

## 4. Proposed spatial revision concept

A future dependency-light contract may contain a shape conceptually like:

```text
SpatialSubjectRevision {
    subject_ref,
    geometry_ref,
    geometry_revision_ref,
    crs_ref,
    geometry_content_digest?,
    source_ref,
    recorded_at?,
    declared_validity?
}
```

Exact naming is not frozen here.

The geometry may live in an external GIS, file, Holochain record, institutional database, or another owning system.

## 5. Logical reference != immutable bytes

A stable `geometry_ref` by itself is not proof that the referenced geometry bytes are immutable.

```text
same geometry_ref
!= same geometry bytes
```

Where the owning system exposes an algorithm-qualified content digest or immutable object identity, the spatial revision SHOULD bind it.

Absent such a commitment, the evidence claim must remain weaker.

REGEN-020 does not invent a universal canonical geometry encoding in v1.

## 6. Coordinate reference system must be explicit

Arbitrary geometry MUST NOT be interpreted under an implicit CRS.

PEF `GeoPoint` currently has explicit WGS84 decimal-degree semantics. An external plot geometry may use another CRS.

```text
coordinates without compatible CRS
!= comparable geometry
```

If a transform is required, the transformed geometry/point should be a derived product with the transformation identity/provenance preserved rather than an unexplained in-place rewrite.

## 7. Geometry revisioning

Boundary correction, split, merge, resurvey, changed operational plot definition, or source replacement SHOULD produce an explicit new spatial revision.

A later revision does not erase the geometry used by an earlier observation, trial, or suitability assessment.

```text
assessment at revision R1
!= assessment at current revision R2
```

Historical evidence remains bound to its original spatial proposition.

## 8. Geometry != cadastral parcel

A regenerative operational plot may coincide with, overlap, subdivide, aggregate, or ignore cadastral parcel boundaries.

REGEN-020 MUST NOT treat spatial coincidence as legal identity.

```text
geometry overlap
!= parcel identity
!= ownership
!= lease
!= access right
!= right to sample
!= right to cultivate
```

Legal/rights evidence belongs to its owning domain.

## 9. Geometry != ecological boundary

An operational plot boundary does not automatically define:

- watershed boundaries;
- habitat boundaries;
- soil-map units;
- hydrologic connectivity;
- root-zone influence;
- contaminant transport boundaries;
- ecological community boundaries.

Those are separate scientific propositions.

## 10. Spatial relation is a derived proposition

A statement such as:

```text
PEF point P lies within plot geometry revision R
```

should be treated as a derived result with inputs/provenance, not as an intrinsic property of either input.

A future result may bind:

```text
observation/spatial-support identity
plot spatial-revision identity
relation operator
geometry implementation/version
result
uncertainty/tolerance policy if applicable
```

The relation operation might be `within`, `covers`, `intersects`, `contains`, or another precisely defined operator. These are not interchangeable.

## 11. Point containment != support containment

An observation point may lie inside a plot while the physical support of the measurement extends outside it.

Likewise a PEF bounding box may intersect a plot without being contained by it.

```text
point inside
!= full measurement support inside
```

The consuming profile must use the relation appropriate to the evidence support.

## 12. Boundary semantics matter

A point exactly on a polygon boundary may be classified differently by `within`, `contains`, and `covers` semantics.

REGEN-020 MUST NOT hide those distinctions behind one universal `inside=true` field.

The relation operator/version belongs in the evidence proposition.

## 13. Complex geometry remains possible

The reference contract MUST NOT assume every subject is one simple polygon.

Owning geometry systems may represent:

- multipolygons;
- holes/exclusions;
- disjoint management areas;
- antimeridian-crossing extents;
- other valid geometries.

REGEN-020 need not implement those geometry algorithms itself; it must preserve enough identity to know what exact geometry was evaluated.

## 14. Spatial uncertainty remains evidence

GNSS error, survey uncertainty, digitization error, coarse raster resolution, uncertain boundaries, and coordinate transforms may matter to the proposition.

A consuming policy MUST NOT convert an uncertain near-boundary case into definite containment merely because both objects have coordinates.

REGEN-020 does not define one universal tolerance.

## 15. Sampling frame is separate from plot geometry

A sampling frame is the declared population/support from which a sampling design draws observations.

It may include:

- exact geometry revision;
- strata/zones;
- eligible locations;
- exclusions;
- grid/transect/reference design;
- temporal window;
- sampling protocol revision;
- intended inferential population.

Therefore:

```text
plot geometry
!= sampling frame
```

A study may intentionally sample only a subset/stratum of a plot.

## 16. Sampling-frame revision

A sampling frame SHOULD have an exact revision/reference distinct from the stable plot identity.

Changing exclusions, strata, eligibility, grid definition, or target population creates a new frame revision.

A later analysis must bind the exact frame against which its representativeness/statistical claims are made.

## 17. No generic representativeness boolean

REGEN-020 MUST NOT define a universal:

```text
representative = true
```

Representativeness depends on study design, target population, selection mechanism, missingness, spatial heterogeneity, temporal scope, exclusions, measurement process, and analytical assumptions.

At most, REGEN-020 supplies evidence needed by a later design-specific theorem.

## 18. Specimen integration

REGEN-018 specimen collection may bind collection spatial support to an exact sampling-frame and/or spatial-subject revision.

Recommended direction:

```text
SoilPlotId
-> spatial revision R
-> sampling frame F
-> specimen collection event S
-> PEF observation/product O
```

This allows a reviewer to determine what was claimed without turning specimen location into representativeness automatically.

## 19. Soil evidence integration

REGEN-010 currently binds one `SoilPlotId` plus PEF observations and correctly makes no containment claim.

A later explicit spatial bridge may add:

```text
soil binding
+ observation spatial support
+ plot spatial revision
+ derived spatial relation
```

without modifying the PEF observation payload or rewriting the original REGEN-010 record.

## 20. Field-trial integration

REGEN-015 treatment/control arms may bind exact spatial revisions and sampling frames where field location matters.

Treatment-area identity, measurement sampling frame, and inferential population should remain distinct.

```text
treatment applied within area A
!= endpoint specimens representative of all A
```

## 21. Suitability integration

REGEN-017 contextual suitability may bind an exact spatial revision/evidence snapshot.

A suitability result for one geometry revision is not automatically current after plot boundary or sampling-frame changes.

## 22. Privacy and sovereignty

Exact field/site geometry can be sensitive for private farms, endangered species, indigenous/community resources, critical infrastructure, or vulnerable populations.

The core contract MUST NOT require public disclosure of full geometry.

A deployment may use:

- access-controlled geometry;
- coarse public representations;
- selective disclosure;
- commitments/proofs where later supported;
- local-only evaluation with exported result/evidence identity.

```text
spatial verifiability
!= mandatory public location disclosure
```

## 23. Region identifiers

PEF permits `SpatialExtent::RegionId`.

A region identifier may resolve through an owning geographic registry. Its presence does not itself establish immutable geometry.

The same logical region name/versioning problem applies:

```text
RegionId
!= immutable polygon bytes
```

Where exact geometry matters, bind the exact registry revision/content identity used.

## 24. Geometry algorithms are versioned evidence dependencies

Containment/intersection results can depend on geometry engine, precision model, repair rules, CRS transform, and tolerance policy.

A derived spatial relation SHOULD preserve sufficient implementation/configuration identity for review/reproduction when material.

```text
same input labels
+ different geometry semantics
may != same relation result
```

## 25. No geofencing or actuator authority

A spatial relation such as `within plot` creates no right to perform an action there.

```text
inside authorized-looking geometry
!= legal authority
!= safety authority
!= actuator capability
```

Physical-control systems remain separately governed.

## 26. Proposed implementation boundary

A later dependency-light module/crate might be approximately:

```text
mycelix-regenerative-spatial
```

or a narrow module of a shared regenerative evidence crate.

Its initial core SHOULD own references/revisions/relation-result semantics, not a full GIS stack.

Likely dependencies are limited to regenerative subject identities, PEF evidence/provenance, and optional serialization.

Adapters may connect to established GIS/cadastral/municipal systems.

## 27. Qualification target

Executable REGEN-020 should follow the qualified shared evidence foundation and use REGEN-008 ProductFrozen semantics where practical.

A later test campaign SHOULD include at least:

1. semantic subject ID remains distinct from geometry revision;
2. geometry revision changes do not rewrite historical bindings;
3. missing CRS prevents unsupported comparison;
4. logical geometry reference without content identity is not labeled immutable;
5. relation operator identity is preserved;
6. `within` / `intersects` / boundary semantics are not collapsed;
7. point containment is not treated as support containment;
8. sampling frame remains distinct from plot geometry;
9. sampling-frame revision is explicit;
10. no representativeness boolean is synthesized;
11. no ownership/right is inferred from geometry;
12. serialization revalidates bounded references/revisions;
13. unknown spatial uncertainty/tolerance is not silently replaced;
14. no action-authority field exists in the core contract.

## 28. Deliberate non-claims

REGEN-020 establishes no:

- cadastral/legal parcel identity;
- ownership/title/lease/access right;
- survey-grade accuracy;
- ecological boundary;
- sampling representativeness;
- statistical validity;
- specimen authenticity;
- agronomic suitability;
- treatment efficacy;
- contamination safety;
- climate/carbon claim;
- geofence authorization;
- governance authority;
- physical actuation.

Its proposition is intentionally narrow:

> bind regenerative semantic subjects and sampling designs to exact, versioned spatial references strongly enough that later containment and representativeness claims can be made explicitly instead of being inferred from names or current GIS state.
