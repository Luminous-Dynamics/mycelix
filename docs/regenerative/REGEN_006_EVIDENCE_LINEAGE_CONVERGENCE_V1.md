# REGEN-006 — Regenerative Evidence-Lineage Convergence Plan v1

Status: architecture / Git-lineage integration plan; no runtime or scientific authority.

Program: #940

Parent constitution: REGEN-000 / #935

## 1. Purpose

REGEN requires both:

1. typed regenerative subject identity; and
2. the existing Mycelix planetary environmental observation / provenance stack.

Those capabilities currently live in separate active Git lineages. REGEN-006 freezes how they should converge without duplicating observation schemas, rewriting qualified evidence history, or treating a source-level merge as scientific qualification.

The governing rule is:

```text
REGEN subject identity
+ PEF environmental evidence / lineage
= composable regenerative evidence subject
```

not:

```text
REGEN creates another EnvironmentalObservation
```

## 2. Current independent lineages

At the time this profile is frozen, the relevant exact subjects are:

### Planetary evidence

PEF-1 / Mycelix #445

```text
head = 54d57e5f6163f6e4a248e62da1a14cb6bbb7ccd7
```

PEF-1 owns `EnvironmentalObservation`, including evidence class, measurement/unit, uncertainty, spatial support, temporal support and external evidence references.

### Planetary lineage

PEF-2 / Mycelix #468

```text
head = 2c097613d5295840653531ba67c78fe74ee3cd10
```

PEF-2 is stacked on PEF-1 and owns provenance DAGs and `LineagedObservation` binding for computed products.

### Regenerative identity

REGEN-002 / Mycelix #936

```text
head = 9cecd04aad840af7465278c6d3c932c305b8774d
```

REGEN-002 is stacked on REGEN-000 and owns typed `regen:v1:<kind>:<token>` semantic identities.

### Regenerative identity vectors

REGEN-004A / Mycelix #939

```text
head = c6a192c2df18d26ee7980c8562ce40de41845442
fixture_sha256 = 23fd4603e283b271a33d566c6f1678cd05e22fdd522d1cb55b3fd0264eadf9ef
```

REGEN-004A freezes the language-neutral subject-ID vector corpus.

### Quality-profile adoption

REGEN-003 / Mycelix #937

```text
head = e2046e7c6c8f5f3777d76109c884d5423eeb4f7d
```

REGEN-003 is a sibling child of REGEN-002. It is not required for the first soil observation binding, but later feedstock/batch qualification will need it.

## 3. Qualification before convergence

A branch being mergeable is not evidence that its theorem is qualified.

Before producing the first runtime evidence-convergence base, require at minimum:

```text
PEF parent evidence status known
+ REGEN-002 exact-head qualification known
+ REGEN-004A exact-head vector qualification known
+ paired Symthaea REGEN-004B compatibility status known
```

A queued or stale workflow is not PASS.

REGEN-006 does not require every later REGEN architecture document to merge before code convergence; it requires the exact implementation contracts the child will consume.

## 4. Preferred Git convergence: preserve both ancestries

Do not manually copy/retype PEF types into a REGEN branch.

Do not manually copy/retype REGEN identity code into the PEF crate.

The preferred convergence is an explicit two-parent Git merge/integration commit after both parents are qualified enough to be used:

```text
PEF-2 exact head
       \
        +--> REGEN evidence integration base
       /
REGEN-004A exact head
```

The integration commit should:

- have PEF-2 as one parent;
- have REGEN-004A as the other parent;
- use a tree that is the exact union of the two parent trees except for any separately reviewed conflict resolution;
- record every conflict resolution explicitly;
- prove that REGEN-002 production files and golden-vector files retain their exact parent Git blob identities where no conflict exists;
- prove that PEF-1/PEF-2 files retain their exact parent Git blob identities where no conflict exists.

This preserves ancestry rather than creating a third hand-copied implementation lineage.

## 5. Why not cherry-pick/retype by default

A selective copy can reproduce source bytes, but it weakens ancestry visibility and creates more opportunities for accidental drift.

When two active lines have non-overlapping paths, a true integration merge is preferable because reviewers can establish:

```text
both exact parents
+ exact merged tree
+ zero hidden conflict resolution
```

If branch policy or repository constraints make a two-parent integration impractical, a selective-integration commit may be used only if it proves exact child blob identity against both frozen parents and documents the weaker ancestry shape.

## 6. Expected path overlap

The lineages are intentionally designed to minimize collision.

PEF-1/2 primarily modify:

```text
crates/mycelix-core-types/**
```

REGEN-002/004A primarily add:

```text
crates/mycelix-regenerative-core/**
.github/workflows/regenerative-core-ci.yml
```

REGEN-000 adds:

```text
docs/regenerative/REGEN_000_REGENERATIVE_COMMONS_CONSTITUTION_V0_1.md
```

Therefore the first convergence is expected to be additive rather than a semantic conflict. That expectation MUST still be checked against the live exact parent trees immediately before integration.

## 7. First post-convergence crate

REGEN-010 SHOULD introduce a narrow dependency-light crate rather than modifying the core observation schema directly.

Candidate location:

```text
crates/mycelix-regenerative-evidence
```

Initial dependency boundary:

```text
mycelix-regenerative-core
+ mycelix-core-types
```

No Holochain, Finance, Marketplace, Climate-credit, Symthaea runtime, database, network or physical-control dependency is required for the first soil evidence profile.

## 8. REGEN-010 data ownership

REGEN-010 should reference or compose canonical PEF observations. It should not copy generic measurement semantics into new fields.

Prefer:

```text
SoilSiteObservationProfile
    site_id: RegenerativeSiteId
    plot_id: SoilPlotId
    observation_refs: [...]
```

where each referenced observation retains its own:

- observation ID;
- evidence class;
- measurement / unit when present;
- uncertainty;
- spatial support;
- temporal support;
- evidence references;
- PEF lineage when derived/computed.

Avoid:

```text
struct SoilProfile {
    ph: f64,
    carbon: f64,
    moisture: f64,
    confidence: f64,
    provenance: String,
}
```

because that would silently create a second environmental evidence schema.

## 9. Profile role versus measurement payload

REGEN-010 may add domain roles for observations, for example:

```text
soil_ph
soil_organic_carbon
soil_texture
bulk_density
water_retention
plant_available_nitrogen
plant_available_phosphorus
potassium
contaminant
context
```

These roles classify how an existing observation is used in a regenerative profile. They do not alter the observation's evidence class or numerical semantics.

```text
role in soil profile
!= measurement payload
```

## 10. Observation-class firewall

The binding layer should be able to require or constrain expected evidence classes for particular uses without relabeling the underlying observation.

Examples:

- a direct laboratory result may be `Observed`;
- a computed soil index may be `Derived`;
- a spatial interpolation may be `Inferred`;
- a future moisture estimate may be `Forecast`;
- a hypothetical treatment condition may be `Scenario`.

A Scenario observation MUST remain Scenario when included in a planning profile.

## 11. Spatial and temporal consistency

A soil profile may bind multiple observations only if the application explicitly handles their spatial and temporal support.

REGEN-010 should not silently assume that measurements from different plots, depths, dates, seasons, or spatial extents describe one interchangeable current soil state.

The first profile may remain conservative and simply preserve each observation's support rather than implementing spatial/temporal reconciliation.

## 12. Depth and sampling semantics

PEF's generic spatial extent does not, by itself, encode soil depth or sampling design.

REGEN-010 MAY add regenerative-domain context for:

- sample depth interval;
- composite vs point sample;
- replicate/sample group identity;
- sampling method reference;
- laboratory method reference.

Those domain semantics should reference the PEF observation rather than replace it.

If depth semantics are not yet modeled, they must remain explicit unknown/not represented rather than being inferred from a plot ID.

## 13. Lineage binding

A derived/inferred/forecast/scenario soil product SHOULD be able to carry PEF-2 `LineagedObservation` evidence.

REGEN-010 does not need to redefine provenance DAGs.

The strongest initial composition should look like:

```text
REGEN subject identity
+ exact PEF observation
+ optional exact PEF lineage for computed product
+ regenerative role/context
```

## 14. No implicit currentness

A valid observation and valid lineage do not imply that the observation is current enough for a specific decision.

Currentness/freshness belongs to the consuming policy/profile.

REGEN-010 should preserve observation time support and avoid embedding one universal freshness threshold.

## 15. Quality-profile convergence is later and separate

REGEN-003 does not need to be merged into the first soil evidence base unless REGEN-010 actually consumes adopted quality-profile semantics.

The preferred layering is:

```text
PEF + REGEN identity
    -> REGEN-010 soil evidence

REGEN-003 quality-profile adoption
    -> later explicit convergence
    -> REGEN-011/012/016 quality-sensitive feedstock/batch layers
```

This keeps soil observation semantics independent from one standards/adoption framework.

## 16. Carbon/climate separation during convergence

Do not import Climate credit/project authority into the evidence foundation merely because some soil observations concern carbon.

A soil organic-carbon observation remains an environmental observation.

```text
soil carbon observation
!= carbon-removal claim
!= carbon-credit authority
```

Climate integration remains REGEN-070+.

## 17. Symthaea separation during convergence

Do not introduce a Symthaea runtime dependency into the Mycelix evidence crate.

Symthaea should consume narrow DTOs/references through a later bridge after the Mycelix evidence contract is stable.

```text
Mycelix evidence core
    != Symthaea model runtime
```

This prevents model dependencies from becoming prerequisites for recording physical observations.

## 18. Holochain separation during convergence

The first evidence contract SHOULD remain useful without Holochain.

A later Holochain zome may persist/share the records, but Holochain storage/network semantics should not become part of the core evidence proposition.

```text
valid evidence record
!= DHT publication
```

## 19. Qualification plan for the integration base

A future integration PR should prove at least:

1. both parent SHAs are exact and frozen;
2. both parents are ancestors of the integration commit;
3. expected parent file/blob identities are preserved;
4. no unexpected overlapping path was resolved silently;
5. PEF focused tests still pass;
6. REGEN core/golden-vector tests still pass;
7. repository diff/tree hygiene is clean;
8. the integration commit itself introduces no new scientific or action authority.

A green merge-only qualification proves compatibility of those exact source lines. It does not prove the scientific validity of later soil profiles.

## 20. Qualification plan for REGEN-010

REGEN-010 should add focused tests for at least:

- exact REGEN site/plot identity preservation;
- observation ID uniqueness within a profile;
- explicit domain role for every bound observation;
- duplicate observation rejection;
- optional expected-class binding and class mismatch rejection;
- Scenario preservation;
- profile output contains no generic replacement measurement fields;
- no authority-bearing field;
- serde/transport validation if serde is enabled;
- PEF observation validation remains authoritative for the nested observation.

## 21. No soil-health master score

REGEN-010 must not introduce one canonical `soil_health: f64` as the evidence foundation.

Soil state is plural and context-dependent. A later model may compute an explicit derived index with PEF lineage, but that index is one derived observation, not a replacement for its inputs.

## 22. Planned Git shape

Conceptually:

```text
PEF-1 #445
    |
PEF-2 #468 ------------------\
                               \
                                +--> REGEN evidence integration base
                               /
REGEN-000 #935               /
    |                        /
REGEN-002 #936              /
    |                       /
REGEN-004A #939 -----------/
                                |
                                +--> REGEN-010 soil-site observation profile
```

REGEN-003 remains a sibling branch until a quality-sensitive child needs it.

## 23. Change management

The exact SHAs above describe the current frozen plan inputs. If any parent head changes before convergence:

- do not pretend the old SHA still represents the new branch;
- review the new exact diff;
- update the convergence record/PR description;
- re-run parent qualification as required;
- bind the integration commit to the actual parent SHAs used.

## 24. Deliberate non-claims

REGEN-006 does not merge the lineages, create soil records, establish agronomic validity, authenticate evidence, define sampling best practices, define currentness, create Holochain state, invoke Symthaea, authorize field action, or create Climate/carbon authority.

It freezes the convergence method so later implementation cannot solve integration pressure by duplicating or weakening the existing evidence architecture.
