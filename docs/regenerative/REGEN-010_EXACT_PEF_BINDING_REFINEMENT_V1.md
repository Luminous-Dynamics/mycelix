# REGEN-010 — Exact PEF Binding Refinement v1

Status: preregistration refinement. This document narrows the executable REGEN-010 design against the exact PEF-1/PEF-2 interfaces present in the current REGEN-009 parent. It adds no agronomic recommendation or physical-action authority.

## 1. Why this refinement exists

The original REGEN-010 preregistration deliberately froze the domain boundary before implementation. Inspection of the actual PEF interfaces reveals several details that should be made normative before code exists:

1. `EnvironmentalObservation.id` is a bounded opaque `String`, not a typed/canonical global observation identifier.
2. `EvidenceClass::Reported` is a distinct raw-evidence class alongside `Observed`.
3. `EnvironmentalObservation::validate()` validates the observation payload and requires at least one evidence reference, but a non-raw class can still exist as a bare `EnvironmentalObservation` without a provenance DAG.
4. PEF-2 already provides `LineagedObservation`, which binds one non-raw observation to an exact `EvidenceLineage`, checks exact output-ID equality, and rejects a computation relabeled as `Observed` or `Reported`.

REGEN-010 SHOULD compose those existing semantics instead of recreating or weakening them.

## 2. Exact PEF evidence classes

The current PEF roster is:

```text
Reported
Observed
Derived
Inferred
Forecast
Scenario
```

REGEN-010 groups them only for provenance admission:

```text
raw classes      = Reported | Observed
computed classes = Derived | Inferred | Forecast | Scenario
```

This grouping is not a confidence ranking.

```text
Reported != Observed
Derived != Inferred
Forecast != Scenario
```

No class may be silently promoted or collapsed.

## 3. Observation-reference semantics

A REGEN soil binding MUST treat `observation_id` as the exact PEF observation ID string.

It MUST NOT invent a stricter global grammar such as lowercase-only IDs, case folding, path normalization, or a new `regen:` observation prefix.

Structural validation SHOULD mirror the applicable PEF ID safety boundary:

```text
nonblank after trim
UTF-8 byte length <= PEF MAX_ID_BYTES
```

Resolution uses exact byte/string equality:

```text
requested_observation_id == resolved_observation.id
```

No normalization is performed before equality.

Important non-claim:

```text
same observation ID
!= cryptographic commitment to immutable observation bytes
```

REGEN-010 MUST NOT pretend that ID equality is content addressing. A future canonical observation commitment may strengthen this once an exact canonical encoding is adopted.

## 4. Resolved-evidence union

The executable design SHOULD resolve to a provenance-aware union conceptually equivalent to:

```text
ResolvedSoilEvidence<'a> {
    Raw(&'a EnvironmentalObservation),
    Lineaged(&'a LineagedObservation),
}
```

The storage/network mechanism remains caller-owned. This is a validation input shape, not a persistence API.

## 5. Raw evidence admission

`ResolvedSoilEvidence::Raw` is valid only when the resolved observation class is:

```text
Reported | Observed
```

The observation MUST pass `EnvironmentalObservation::validate()`.

A bare `EnvironmentalObservation` carrying:

```text
Derived | Inferred | Forecast | Scenario
```

MUST be rejected by REGEN-010 resolved validation even if the PEF payload itself is structurally valid.

This is an intentional REGEN strengthening:

```text
computed class
=> provenance lineage required
```

## 6. Computed evidence admission

`ResolvedSoilEvidence::Lineaged` MUST pass `LineagedObservation::validate()`.

That existing PEF-2 validator already proves, structurally:

- nested observation validation;
- nested lineage validation;
- exact `observation.id == lineage.output_observation_id`;
- a computed lineage cannot retain `Reported` or `Observed` class.

REGEN-010 SHOULD depend on that theorem rather than duplicating it.

The lineaged observation class is therefore expected to be one of:

```text
Derived | Inferred | Forecast | Scenario
```

## 7. Evidence-class laundering firewall

Resolved validation MUST fail for all of the following:

```text
Raw(Derived)
Raw(Inferred)
Raw(Forecast)
Raw(Scenario)
Lineaged(Observed)
Lineaged(Reported)
```

The last two are already rejected by `LineagedObservation::validate()`; REGEN-010 should retain explicit regression coverage so later adapter changes cannot bypass the PEF-2 wrapper.

## 8. Expected evidence class

The original optional `expected_evidence_class` binding remains useful.

After provenance-aware resolution:

```text
expected_evidence_class = Some(X)
resolved observation class != X
=> reject
```

Example:

```text
protocol requires Observed baseline
resolved input = Reported
=> reject
```

`Reported` is not silently accepted as `Observed` merely because both are raw classes.

## 9. Phenomenon binding

The current PEF observation also carries an opaque `phenomenon: String`.

A soil role such as:

```text
soil:ph
```

MUST NOT itself be treated as proof that a resolved observation measures soil pH.

To prevent reference substitution, the executable binding SHOULD additionally commit to the expected PEF phenomenon string:

```text
SoilObservationBinding {
    observation_id,
    role,
    expected_phenomenon,
    expected_evidence_class?,
    sample_context?,
    sampling_method_ref?,
    laboratory_method_ref?,
}
```

Resolved validation requires exact equality:

```text
binding.expected_phenomenon == observation.phenomenon
```

This proves only that the resolved PEF payload matches the phenomenon identity the binding declared.

It does NOT prove that the phenomenon name is scientifically correct for the role. Role-to-phenomenon admissibility belongs to a later adopted soil/profile vocabulary.

## 10. Measurement presence remains explicit

`EnvironmentalObservation.measurement` is optional by design.

REGEN-010 MUST NOT manufacture a scalar when it is absent.

```text
measurement = None
!= zero
!= missing-value sentinel
```

Some future soil-role profiles may require a scalar measurement (for example a particular pH or nutrient protocol). That requirement belongs to the consuming profile, not the generic binding waist.

## 11. Evidence references are already mandatory at PEF level

`EnvironmentalObservation::validate()` requires:

```text
1 <= evidence.len() <= MAX_EVIDENCE_REFS
```

and validates/deduplicates each `ExternalEvidenceRef`.

REGEN-010 SHOULD call the PEF validator and MUST NOT duplicate the evidence vector into its own schema.

This gives every resolved soil observation at least one declared source-artifact reference without claiming that the external source is truthful, accredited, or scientifically sufficient.

## 12. Temporal semantics

The current PEF `TemporalExtent` is:

```text
Unspecified
Instant(i64)
Interval { start, end }
```

and explicitly permits `Unspecified` rather than substituting ingestion time for unknown observation time.

REGEN-010 MUST preserve that property.

```text
unknown observation time
!= ingestion time
```

No generic soil freshness window is introduced.

## 13. Spatial semantics

PEF spatial support is one of:

```text
Point
BoundingBox
RegionId
```

REGEN-010 does not own plot geometry in v1, therefore it MUST NOT claim that the PEF spatial support lies inside, covers, or represents the referenced `SoilPlotId`.

```text
plot binding + spatial observation
!= geometric containment proof
!= representativeness proof
```

A later geometry/cadastral bridge may add those propositions explicitly.

## 14. Uncertainty and units remain PEF-owned

The existing PEF payload owns:

- `Measurement { value, unit }`;
- `Uncertainty::{Unspecified, Interval, StandardDeviation}`.

REGEN-010 MUST reference them through the resolved observation, never duplicate or silently normalize them.

A unit conversion remains:

```text
source observation
-> EvidenceLineage with DeterministicTransform
-> Derived EnvironmentalObservation
-> LineagedObservation
-> REGEN soil binding
```

not an in-place mutation of source evidence.

## 15. Reproducibility capsule boundary

`ProducerIdentity` may carry code/configuration/environment digests and exposes `has_complete_capsule()`.

REGEN-010 requires valid lineage for computed evidence, but v1 SHOULD NOT automatically equate a valid lineage with a complete reproducibility capsule.

```text
valid lineage
!= complete capsule
!= deterministic reproduction
!= scientific validity
```

A consuming trial/quality protocol may later require complete producer capsules for particular evidence roles.

## 16. Revised resolved-validation algorithm

Conceptually:

```text
for each SoilObservationBinding:
    validate binding structure
    resolved = caller.resolve(binding.observation_id)
    require resolved exists

    match resolved:
        Raw(observation):
            observation.validate()
            require class in {Reported, Observed}

        Lineaged(product):
            product.validate()
            observation = product.observation
            require class in {Derived, Inferred, Forecast, Scenario}

    require observation.id == binding.observation_id
    require observation.phenomenon == binding.expected_phenomenon

    if expected_evidence_class exists:
        require observation.class == expected_evidence_class
```

Then profile-level duplicate/order/cardinality rules are evaluated.

## 17. Revised adversarial corpus

In addition to the original preregistered cases, the executable campaign SHOULD include:

1. `Reported` raw observation accepted when not prohibited by the consuming profile;
2. expected `Observed` rejects resolved `Reported`;
3. bare `Derived` observation rejected for missing lineage;
4. bare `Inferred` observation rejected for missing lineage;
5. bare `Forecast` observation rejected for missing lineage;
6. bare `Scenario` observation rejected for missing lineage;
7. lineaged computed observation accepted when PEF-2 validation succeeds;
8. lineaged `Observed` laundering rejected;
9. lineaged `Reported` laundering rejected;
10. requested/resolved ID exact mismatch rejected;
11. expected/resolved phenomenon mismatch rejected;
12. `TemporalExtent::Unspecified` remains representable and is not replaced by ingestion time;
13. absent scalar measurement remains absent;
14. spatial support is not interpreted as plot containment;
15. valid lineage without complete producer capsule is not mislabeled reproducible.

## 18. Revised implementation boundary

The first executable crate remains:

```text
crates/mycelix-regenerative-evidence
```

with approximately:

```text
mycelix-regenerative-core
mycelix-core-types
optional serde
```

No Holochain, network lookup, database, Symthaea runtime, climate authority, marketplace authority, finance authority, or physical-control dependency is introduced.

## 19. Gate remains unchanged

This refinement does not authorize implementation ahead of REGEN-009 qualification.

The implementation parent MUST still be a qualified REGEN-009 ProductHead or an explicitly qualified successor.

## 20. Deliberate non-claims

This refinement proves no real soil observation true and establishes no soil health, agronomic sufficiency, amendment efficacy, contamination safety, carbon removal, land right, laboratory competence, sampling representativeness, governance authority, or physical-action authority.

Its purpose is narrower:

> make REGEN-010 inherit the exact PEF evidence/provenance semantics already present in the codebase, including the raw-vs-computed provenance firewall, instead of accidentally weakening them at the soil boundary.
