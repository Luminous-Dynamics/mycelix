# REGEN-010 — Soil Observation Profile Preregistration v1

Status: preregistered executable design; implementation gated on REGEN-009 qualification. No agronomic recommendation or physical-action authority.

## 1. Purpose

REGEN-010 is the first domain-specific evidence layer in the regenerative-resilience program.

Its job is deliberately narrow:

> bind canonical regenerative site/plot identities to canonical planetary environmental observations, while preserving soil-specific sampling context and the evidence class/provenance already owned by PEF.

It MUST NOT create a second generic measurement format.

The central theorem is:

```text
soil evidence profile
= regenerative subject context
+ references to canonical PEF observations
+ soil-specific sampling/use context
```

not:

```text
soil evidence profile
= copied pH/carbon/moisture/value/unit/confidence schema
```

## 2. Parent gate

Implementation MUST target a qualified successor of REGEN-009, whose source-level convergence contains:

- PEF-1 environmental observation primitives;
- PEF-2 evidence lineage / lineaged-product primitives;
- REGEN subject identities;
- REGEN subject-ID golden vectors.

The initial preregistration is stacked on REGEN-009 integration subject:

`94e9adba5a761b2d673ad207f4ce245e7fcafac9`

This preregistration does not treat the currently queued integration workflows as PASS.

## 3. Planned crate boundary

Preferred initial crate:

```text
crates/mycelix-regenerative-evidence
```

Initial direct dependency boundary SHOULD remain approximately:

```text
mycelix-regenerative-core
mycelix-core-types
optional serde
```

It SHOULD NOT directly depend on:

- Holochain/HDK;
- Symthaea;
- Climate;
- Finance;
- Marketplace;
- Water/Energy runtime crates;
- databases;
- network clients;
- physical-control systems.

Persistence, networking, economics, modeling, and action remain downstream concerns.

## 4. Identity boundary

The soil profile binds:

- one `RegenerativeSiteId`;
- one `SoilPlotId`;
- zero or more canonical observation bindings subject to profile minimums defined by the consuming protocol.

The profile asserts only that the listed plot is being used in the context of the listed site for this evidence artifact.

It does NOT prove:

- land ownership;
- cadastral/legal parcel status;
- plot geometry;
- right to sample;
- right to cultivate;
- site/plot governance authority.

Those facts must resolve through their owning domains/evidence systems where needed.

## 5. No master soil-health primitive

REGEN-010 MUST NOT introduce:

```text
soil_health: f64
```

as a canonical primitive.

Soil condition remains plural.

A future soil-health index may exist only as an explicitly derived/inferred observation with its own PEF evidence class and PEF-2 lineage.

```text
plural source observations
    -> explicit producer lineage
    -> derived soil-health observation
```

The derived index never replaces its source observations.

## 6. Observation-reference contract

Each soil binding SHOULD contain only the minimum additional facts needed to interpret one canonical PEF observation in a soil profile.

Conceptual shape:

```text
SoilObservationBinding {
    observation_id
    role
    expected_evidence_class?
    sample_context?
    sampling_method_ref?
    laboratory_method_ref?
}
```

The PEF observation remains authoritative for:

- scalar measurement value;
- unit;
- evidence class;
- uncertainty;
- spatial support;
- temporal support;
- external evidence references;
- optional contextual epistemic classification.

REGEN-010 MUST NOT duplicate those fields merely for convenience.

## 7. Canonical observation role key

Rather than freezing a large closed agronomy enum prematurely, v1 SHOULD use a bounded canonical namespaced role key.

Conceptual textual form:

```text
soil:<role>
```

Examples MAY include:

```text
soil:ph
soil:organic-carbon
soil:total-carbon
soil:bulk-density
soil:electrical-conductivity
soil:gravimetric-water-content
soil:volumetric-water-content
soil:total-nitrogen
soil:nitrate-nitrogen
soil:ammonium-nitrogen
soil:available-phosphorus
soil:exchangeable-potassium
soil:texture-sand-fraction
soil:texture-silt-fraction
soil:texture-clay-fraction
```

These are role identifiers, not units, laboratory methods, target ranges, safety thresholds, or agronomic recommendations.

The grammar SHOULD be canonical lowercase ASCII, bounded, and reject whitespace, case folding, empty segments, and path-like ambiguity.

A later standards profile may define which role keys are required for a particular trial or decision.

## 8. Role meaning does not override measurement semantics

A binding such as:

```text
role = soil:ph
```

does not itself prove that the referenced PEF observation:

- uses an appropriate pH method;
- uses an appropriate soil-to-solution ratio;
- is current;
- is representative of the plot;
- was produced by an accredited laboratory;
- is suitable for a given crop decision.

Those propositions require separate evidence/profile semantics.

## 9. Sampling context

PEF spatial support describes where an observation applies geographically.

REGEN-010 may add soil-specific sampling context not owned by the generic PEF payload.

Planned v1 context:

```text
SoilSampleContext {
    depth_interval?
    sampling_support
    sample_group_ref?
}
```

### 9.1 Depth interval

A depth interval SHOULD use exact nonnegative integer millimetres:

```text
top_mm
bottom_mm_exclusive
```

with:

```text
bottom_mm_exclusive > top_mm
```

Integer millimetres avoid floating-point canonicalization problems while remaining more precise than typical field protocols require.

Absence means depth was not supplied/applicable; it MUST NOT silently mean surface soil.

### 9.2 Sampling support

Planned values:

```text
Unspecified
Point
Composite { subsample_count }
```

`Unspecified` is explicit uncertainty about sample construction, not evidence of point sampling.

A composite sample's `subsample_count` must be positive and bounded.

The profile does not infer spatial representativeness from composite status alone.

### 9.3 Sample-group reference

An optional bounded opaque reference may associate observations produced from the same physical sample/composite.

This supports facts such as:

```text
pH observation
organic-carbon observation
nitrogen observation
```

having been measured from the same submitted sample without claiming that the analyses share the same method or uncertainty.

## 10. Method references

Two optional opaque method references may be preserved:

- `sampling_method_ref` — field collection procedure/protocol;
- `laboratory_method_ref` — analytical laboratory method/procedure.

These SHOULD be bounded nonempty text/reference identities.

Presence proves only that the profile declares a method reference.

```text
method_ref present
!= method retrieved
!= method correctly followed
!= method scientifically appropriate
!= laboratory accredited
```

Later lineages may bind exact method documents/digests where stronger reproducibility is required.

## 11. Evidence-class preservation

Every referenced observation already has a mandatory PEF `EvidenceClass`.

REGEN-010 MUST preserve it.

Examples:

```text
direct lab/instrument result -> may be Observed
unit conversion -> Derived
spatial interpolation -> Inferred
future soil-moisture estimate -> Forecast
hypothetical amendment response -> Scenario
```

A binding MUST NOT relabel a `Scenario` prediction as an `Observed` soil fact.

## 12. Optional expected-class binding

A profile MAY bind an expected evidence class for a particular observation reference.

This is useful where a protocol requires, for example, an actual observed baseline rather than a model estimate.

Conceptual rule:

```text
expected_class = Some(Observed)
actual PEF class = Inferred
=> resolved-profile validation fails
```

`expected_class = None` means the profile does not add a class restriction; it does not mean class is unknown, because PEF class remains mandatory.

## 13. Structural validation vs resolved validation

REGEN-010 SHOULD expose two distinct validation propositions.

### 13.1 Structural validation

Checks only the profile bytes/shape:

- supported schema revision;
- valid REGEN site/plot IDs;
- bounded canonical role keys;
- valid depth intervals;
- bounded sample context/method refs;
- unique observation IDs;
- bounded number of bindings;
- canonical ordering if ordering is part of wire identity.

### 13.2 Resolved validation

Requires resolving every observation ID to an actual `EnvironmentalObservation`.

It then checks at least:

- every reference resolves;
- every nested PEF observation validates under PEF rules;
- resolved observation ID exactly matches the requested reference;
- optional expected evidence class matches;
- no resolved observation is silently replaced by another ID/alias.

This separation preserves:

```text
well-formed reference set
!= evidence successfully resolved and validated
```

## 14. Resolver boundary

The dependency-light crate SHOULD avoid owning storage/network lookup.

Resolved validation may accept a caller-provided resolver abstraction/callback conceptually equivalent to:

```text
observation_id -> Option<&EnvironmentalObservation>
```

or validate against an explicitly supplied collection.

The core crate MUST NOT make hidden network calls.

## 15. Observation-ID uniqueness

Within one profile, the same canonical observation ID SHOULD NOT appear more than once.

Because the current PEF observation contains one scalar measurement/phenomenon, reusing one ID under several soil roles risks semantic laundering.

If one physical sample produces multiple analyses, those analyses should normally be separate observations linked by `sample_group_ref`, not one observation relabeled several ways.

## 16. Multiple observations per role are valid

The same role MAY appear more than once with different observation IDs.

This is necessary for:

- replicates;
- multiple depths;
- repeated dates;
- multiple sampling locations;
- independent laboratories;
- control/verification measurements.

REGEN-010 MUST NOT collapse these into one value automatically.

Aggregation is a later explicit derived observation with lineage.

## 17. Canonical ordering

For deterministic transport and review, v1 SHOULD define one canonical binding order.

Recommended order:

```text
(role key, observation_id)
```

Constructors may sort into canonical order; direct/deserialized noncanonical ordering should be revalidated or rejected according to the final wire contract.

Canonical ordering proves only deterministic representation, not scientific priority.

## 18. Bounded cardinality

The profile SHOULD have an explicit maximum binding count to protect parsing/storage surfaces.

A reasonable initial engineering bound can be generous (for example 1024 bindings) while remaining finite.

This bound is a protocol safety limit, not a statement about how many measurements are scientifically sufficient.

## 19. Temporal/currentness boundary

PEF temporal support remains authoritative.

REGEN-010 MUST NOT define one universal freshness window such as:

```text
soil observations valid for one year
```

Freshness depends on:

- property;
- soil dynamics;
- management change;
- decision context;
- adopted protocol.

A later consuming quality/trial/policy profile may define freshness requirements explicitly.

## 20. Spatial-representativeness boundary

A PEF point/bounding-box/region plus a soil sample context does not prove the observation is representative of the entire plot.

```text
sample located in plot
!= representative of plot
```

Representativeness may require a sampling protocol, design evidence, replicate structure, or later statistical analysis.

REGEN-010 MUST preserve that non-claim.

## 21. Units remain PEF-owned

REGEN-010 MUST NOT hard-code one canonical unit for every soil role into the binding layer.

A later standards/profile validator may require compatible units for a decision, but the raw observation continues to carry the actual unit supplied by PEF.

This prevents destructive normalization such as converting measurements while losing the transformation lineage.

If conversion is needed:

```text
source observation
-> deterministic conversion lineage
-> Derived PEF observation
-> soil profile may reference derived observation
```

## 22. Texture fractions

If sand/silt/clay fractions are represented as separate observations, REGEN-010 itself SHOULD NOT silently normalize them to sum to one or 100%.

A later profile/derived validator may assess consistency and produce an explicit result.

```text
three measurements present
!= texture classification valid
```

## 23. Nutrient-method dependence

Roles such as `soil:available-phosphorus` are method-dependent.

The role name alone MUST NOT erase that dependency.

A consuming protocol may require an exact `laboratory_method_ref` or lineage before accepting such an observation for a specific inference.

This is why method evidence remains separate from role identity.

## 24. Contaminants deliberately deferred

REGEN-010 may reference soil contaminant observations if a role key is supplied, but it MUST NOT define universal contamination/safety thresholds.

Threshold/profile semantics belong to REGEN-016 and adopted quality/legal profiles.

```text
contaminant measured
!= safe
!= unsafe
```

without an applicable decision profile.

## 25. Soil carbon firewall

A soil-carbon observation in REGEN-010 is environmental evidence only.

```text
soil organic carbon observed
!= carbon removal
!= additionality
!= permanence
!= project attribution
!= credit eligibility
```

Climate-domain authority remains downstream.

## 26. Agronomic firewall

Likewise:

```text
pH / carbon / nutrient observation
!= amendment recommendation
!= crop recommendation
!= safe application rate
```

REGEN-010 binds evidence; it does not prescribe treatment.

## 27. Symthaea boundary

Symthaea may later consume a resolved soil profile for analysis/modeling.

REGEN-010 MUST NOT contain Symthaea-specific model state or confidence fields.

```text
Mycelix soil evidence
-> narrow adapter
-> Symthaea model input
```

not:

```text
Mycelix evidence type embeds Symthaea runtime
```

## 28. Holochain boundary

The dependency-light contract is not itself a Holochain entry type.

A later persistence adapter may store/publish it after validating the same core invariants.

```text
valid core profile
!= committed to DHT
```

and:

```text
DHT entry exists
!= scientifically valid profile
```

## 29. Serde boundary

If serde is enabled, raw transport structures MUST be revalidated after deserialization before downstream code treats them as structurally valid.

Prefer a pattern similar to REGEN-003:

```text
RawSoilObservationProfile
    -> validate
    -> ValidatedSoilObservationProfile
```

The validated wrapper SHOULD not be directly constructible from untrusted bytes without running the validation path.

## 30. Planned errors

The first implementation SHOULD expose typed structural/resolution failures rather than booleans, including at least:

- unsupported schema revision;
- empty profile;
- too many bindings;
- malformed role key;
- duplicate observation reference;
- noncanonical ordering;
- invalid depth interval;
- zero/oversized composite subsample count;
- malformed/oversized method or sample-group reference;
- unresolved observation ID;
- nested invalid PEF observation;
- resolved ID mismatch;
- expected evidence-class mismatch.

Error fields should be documented because qualification denies warnings.

## 31. Planned authored test corpus

The implementation campaign SHOULD include at least:

1. valid observed pH binding;
2. valid repeated same-role observations at different depths;
3. valid composite sample with explicit count;
4. valid `Scenario` observation retained as scenario;
5. duplicate observation-ID rejection;
6. malformed role-key rejection;
7. invalid depth interval rejection;
8. zero composite-count rejection;
9. oversized method-ref rejection;
10. structural validation without resolution;
11. successful resolved validation;
12. missing observation resolution failure;
13. nested PEF validation failure;
14. requested/resolved observation-ID mismatch rejection;
15. expected-class mismatch rejection;
16. serde round-trip followed by required revalidation;
17. canonical ordering behavior;
18. explicit proof that the REGEN type contains no raw `value`, `unit`, `uncertainty`, or duplicate generic provenance fields.

## 32. Mutation/adversarial targets

After the baseline passes, stronger qualification SHOULD attempt to detect mutations such as:

- disabling duplicate-reference rejection;
- treating missing depth as zero depth;
- accepting `bottom <= top`;
- ignoring evidence-class mismatch;
- accepting unresolved IDs;
- skipping nested PEF validation;
- aliasing one resolved observation under a different requested ID;
- converting `Scenario` to `Observed`;
- weakening role-key canonicalization.

These are later evidence classes; authored unit tests alone do not claim mutation adequacy.

## 33. Qualification requirements

When implementation begins, the REGEN-010 lane SHOULD follow REGEN-007/008:

- exact ProductHead checkout + observed SHA assertion;
- pinned Rust/Cargo toolchain;
- ProductFrozen dependency graph or explicit reviewed exception;
- rustfmt;
- dependency-light tests;
- serde tests;
- strict all-target Clippy `-D warnings`;
- focused compatibility test against the exact PEF/REGEN integration parent;
- clean checkout;
- machine-readable receipt where practical.

The scientific meaning of actual soil measurements is not qualified by these software tests.

## 34. No physical side effects

REGEN-010 contains no capability to:

- operate a sampler;
- start irrigation;
- control a pyrolyzer;
- dose fertilizer/amendment;
- command machinery;
- write to a device controller.

It is evidence-binding infrastructure only.

## 35. Promotion gate to REGEN-011+

Feedstock/batch/trial layers SHOULD NOT use ad-hoc soil measurements once REGEN-010 is established.

They should consume canonical observations/profile bindings or explicitly justify another authoritative domain type.

This makes REGEN-010 the narrow soil-evidence waist rather than a parallel agronomy database.

## 36. Deliberate non-claims

REGEN-010 does not establish:

- soil health;
- agronomic sufficiency;
- crop suitability;
- amendment efficacy;
- fertilizer need;
- contamination safety;
- carbon removal;
- carbon-credit eligibility;
- legal land status;
- sampling representativeness;
- laboratory competence;
- freshness/currentness for a decision;
- community/governance authority;
- physical-action authority.

It preregisters the smallest evidence contract needed to bind soil-specific context to the existing Mycelix planetary-evidence lineage without duplicating that lineage.
