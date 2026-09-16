# REGEN-019 — Shared Evidence Admission Kernel Preregistration v1

Status: preregistration only. No executable product is authorized by this document. It defines a domain-neutral validation boundary for consuming existing PEF observations/products without duplicating PEF measurement or provenance semantics.

## 1. Purpose

REGEN-010 establishes the first concrete regenerative consumer of PEF evidence. REGEN-011 biomass, REGEN-015 field trials, REGEN-016 contamination, REGEN-018 specimens, and later domains need the same provenance admission rule.

That rule should not be copied independently into every domain crate.

The central theorem is:

```text
one qualified PEF admission kernel
+ domain-specific expectation/context validation
= composable regenerative evidence use
```

not:

```text
N regenerative domains
= N subtly different raw/computed provenance validators
```

## 2. Sequence / no retroactive widening

REGEN-019 MUST NOT change what an already-qualified REGEN-010B ProductHead proved.

Recommended sequence:

```text
REGEN-010B
-> qualify exact current soil-evidence ProductFrozen bytes
-> new successor ProductHead
-> add shared admission API + missing adversarial coverage
-> separately qualify successor
-> downstream domains consume the qualified shared API
```

A refactor or API extraction is a product mutation and therefore creates a new qualification lineage.

## 3. Existing PEF classes remain authoritative

The exact PEF evidence classes remain:

```text
Reported
Observed
Derived
Inferred
Forecast
Scenario
```

REGEN-019 groups them only for provenance admission:

```text
raw = Reported | Observed
computed = Derived | Inferred | Forecast | Scenario
```

This grouping is not a confidence, quality, authority, or truth ranking.

Every class remains distinct.

## 4. Proposed pure admission shape

A future implementation may expose an API conceptually like:

```text
ResolvedEvidence<'a> {
    Raw(&'a EnvironmentalObservation),
    Lineaged(&'a LineagedObservation),
}

AdmittedObservation<'a> {
    observation: &'a EnvironmentalObservation,
    provenance_form: Raw | Lineaged,
}

admit_resolved(resolved) -> Result<AdmittedObservation, EvidenceAdmissionError>
```

Exact naming may vary after review.

The function MUST be:

- deterministic for the supplied value;
- pure validation with no hidden network/database lookup;
- authority-free;
- domain-neutral;
- dependent on owning PEF validators rather than copied internal validation.

## 5. Raw admission theorem

For `Raw(observation)`:

1. call `EnvironmentalObservation::validate()`;
2. require class exactly `Reported` or `Observed`;
3. reject `Derived`, `Inferred`, `Forecast`, and `Scenario` as bare observations.

```text
computed class
=> validated computation lineage required
```

## 6. Lineaged admission theorem

For `Lineaged(product)`:

1. call `LineagedObservation::validate()`;
2. use its validated nested observation;
3. preserve the exact computed evidence class;
4. never reinterpret lineage validity as model/scientific validity.

PEF already requires exact observation/output-ID binding and rejects computation laundered as `Reported` or `Observed`.

REGEN-019 should compose that theorem, not fork it.

## 7. Generic expectation matching

A small domain-neutral expectation check MAY bind:

```text
requested_observation_id
expected_phenomenon
expected_class?
```

and require exact equality against the admitted observation.

No case folding, normalization, alias expansion, or phenomenon inference occurs implicitly.

```text
right ID + wrong phenomenon -> reject
right ID + wrong expected class -> reject
```

A domain may add stricter expectations later.

## 8. Domain semantics remain outside the kernel

The shared kernel MUST NOT decide:

- that `soil_ph` is appropriate for a soil acidity role;
- that a biomass observation proves sustainable availability;
- that a contaminant result proves safety;
- that a specimen is representative;
- that a trial endpoint proves treatment effect;
- that a result is current enough for a decision;
- that a model is sufficiently reproducible;
- that any action is authorized.

Those belong to consuming contracts/policies.

## 9. No duplicate measurement schema

REGEN-019 MUST NOT define replacements for PEF-owned:

- scalar values;
- units;
- uncertainty;
- spatial extent;
- temporal extent;
- external evidence references;
- epistemic classification;
- computation lineage.

A regenerative domain references/adopts those validated values; it does not copy them into a second canonical truth format.

## 10. Unknown remains unknown

The kernel preserves PEF semantics such as:

```text
measurement = None
!= zero

TemporalExtent::Unspecified
!= ingestion time

Uncertainty::Unspecified
!= exact certainty
```

No default/sentinel manufacture belongs in admission.

## 11. Spatial support remains evidence support

PEF `Point`, `BoundingBox`, or `RegionId` support MUST NOT be promoted by the shared kernel into:

- plot containment;
- lot representativeness;
- jurisdiction;
- ownership;
- ecological coverage;
- population representativeness.

Geometry/representativeness requires an explicit downstream theorem.

## 12. Valid lineage != reproducibility completeness

A valid `LineagedObservation` may carry incomplete producer capsule metadata.

The shared API MUST preserve the distinction:

```text
valid lineage
!= complete reproducibility capsule
!= deterministic reproduction
!= scientific validity
```

A downstream profile may explicitly require complete producer code/config/environment identities.

## 13. Observation ID != immutable content commitment

Exact observation-ID equality prevents resolver substitution by identifier, but it does not prove immutable bytes.

```text
logical ID equality
!= cryptographic content identity
```

Where the owning evidence system provides a canonical content digest, that identity may be bound separately. REGEN-019 does not invent a canonical encoding/hashing theorem in v1.

## 14. Error taxonomy

A future shared error type SHOULD preserve distinctions useful to callers, such as:

- invalid raw PEF observation;
- computed class supplied without lineage;
- invalid lineaged product;
- requested/resolved observation-ID mismatch;
- phenomenon mismatch;
- evidence-class mismatch.

It SHOULD NOT collapse all failures into one boolean because downstream evidence/reporting needs to distinguish missing lineage from wrong identity or malformed nested evidence.

The error type itself conveys no authority.

## 15. Ownership of resolution

The kernel validates evidence supplied by the caller.

It does not own persistence or lookup.

A resolver may be backed by memory, files, Holochain, a database, an institutional system, or another adapter, but the dependency-light core accepts values/references rather than initiating hidden I/O.

```text
resolution mechanism
!= evidence validity
```

## 16. Adversarial completeness target

The first executable shared admission campaign SHOULD explicitly exercise all six PEF evidence classes and the major boundary cases:

1. raw `Reported` accepted when structurally valid;
2. raw `Observed` accepted when structurally valid;
3. bare `Derived` rejected;
4. bare `Inferred` rejected;
5. bare `Forecast` rejected;
6. bare `Scenario` rejected;
7. lineaged `Derived` admitted;
8. lineaged `Inferred` admitted;
9. lineaged `Forecast` admitted;
10. lineaged `Scenario` admitted;
11. lineaged `Reported` rejected;
12. lineaged `Observed` rejected;
13. nested invalid observation rejected;
14. nested invalid lineage rejected;
15. exact observation-ID mismatch rejected;
16. exact phenomenon mismatch rejected;
17. explicit expected-class mismatch rejected;
18. `measurement=None` remains absent;
19. `TemporalExtent::Unspecified` remains unspecified;
20. valid lineage without complete capsule remains valid lineage but is not labeled reproducibility-complete.

This is the natural place to close the adversarial gaps already recorded for the post-010B successor.

## 17. Soil compatibility

REGEN-010 soil bindings should become thin domain users of the shared theorem after a qualified successor exists.

Conceptually:

```text
shared admit_resolved(...)
-> exact generic expectation checks
-> soil binding context checks
```

Soil-specific depth, point/composite support, sample-group, sampling-method, and laboratory-method references remain in the soil contract.

## 18. Biomass compatibility

REGEN-011 SHOULD consume the shared admission theorem for observations supporting:

- material state;
- moisture / dry-matter conversion;
- composition evidence;
- ecological allocation evidence;
- source-history or condition evidence where appropriate.

It MUST NOT copy the raw-vs-lineaged class logic into a divergent biomass implementation.

## 19. Specimen compatibility

REGEN-018 should use the shared kernel when binding analytical PEF observations/products to exact specimen references.

Specimen collection/custody/transformation semantics remain outside admission.

## 20. Contamination compatibility

REGEN-016 uses the shared kernel to establish that an analytical observation/product is structurally/provenance-valid before interpreting detection/quantification semantics against an exact contamination profile.

```text
PEF admission
!= contamination conformance
!= universal safety
```

## 21. Trial compatibility

REGEN-015 may use the shared kernel for measured/derived trial endpoints while keeping protocol, endpoint registration, missingness, exclusions, analysis lineage, and causal interpretation separate.

## 22. Suggested crate/module direction

The already-created crate name `mycelix-regenerative-evidence` is broad enough to host the domain-neutral admission kernel.

A likely future structure is approximately:

```text
mycelix-regenerative-evidence
  admission.rs        # domain-neutral PEF admission
  expectation.rs      # exact generic ID/phenomenon/class matching
  soil.rs             # REGEN-010 soil binding
```

This is a design target, not authorization to refactor the currently frozen 010B bytes.

If review instead favors a separate tiny crate, that decision must preserve acyclic dependency direction and avoid making soil a dependency of biomass merely to reuse a generic helper.

## 23. Dependency boundary

The shared admission kernel should depend only on the owning PEF/core types it validates plus minimal standard-library support.

No direct dependency belongs here on:

- Holochain/HDK;
- Symthaea;
- Climate;
- Finance;
- Marketplace;
- Manufacturing runtime;
- databases;
- network clients;
- physical-control systems.

## 24. Qualification discipline

Any executable successor implementing this contract creates a new ProductHead and MUST earn its own qualification.

Recommended profile:

- REGEN-008 ProductFrozen Cargo graph;
- exact-head verification;
- explicit feature campaigns;
- all-class adversarial corpus above;
- strict Clippy;
- clean checkout;
- explicit system-closure classification;
- REGEN-Q001 machine-readable receipt once composed cleanly.

A green refactor test does not transfer the prior 010B qualification automatically.

## 25. Deliberate non-claims

REGEN-019 establishes no:

- truth of any real observation;
- scientific validity of any model;
- currentness/sufficiency for a decision;
- sampling representativeness;
- material safety;
- agronomic suitability;
- ecological sustainability;
- legal rights;
- carbon removal/credit;
- economic value;
- governance authority;
- process execution authority;
- physical actuation.

Its proposition is deliberately narrow:

> provide one qualified, reusable regenerative boundary for admitting PEF raw/computed evidence so every downstream domain does not reimplement and potentially weaken the provenance firewall.
