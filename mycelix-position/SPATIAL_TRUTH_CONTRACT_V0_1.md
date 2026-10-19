# Mycelix Position — Spatial Truth Contract v0.1

Status: architecture contract / non-authoritative design boundary

Tracks: #413

## Purpose

`mycelix-position` is the domain-independent spatial truth substrate for Mycelix.

It owns the semantics required to answer:

- where is a subject, and in which reference system?
- when was that spatial state observed, received, authored, derived, or predicted?
- how uncertain is the state?
- which sensor/source/agent produced the evidence?
- which transformations and fusion steps produced an estimate?
- which spatial relationship or transition occurred?
- what may a caller learn about the spatial state?

It does **not** own transport journeys, cargo, custody, carriers, logistics contracts, routing decisions, or transport-specific events.

The dependency invariant is:

```text
mycelix-transport -> mycelix-position
mycelix-position -X-> mycelix-transport
```

The same Position substrate must remain reusable by Transport, Robotics, Emergency, Sol Atlas, Pulse, Personal, agriculture, infrastructure, autonomous systems, and space systems.

## Existing strengths to preserve

The current pure Rust positioning library is already correctly separated from Holochain and contains reusable mathematical machinery for ranging, trilateration, dilution-of-precision analysis, Kalman filtering, dead reckoning, fusion, navigation runtime behavior, and Earth/Moon/Mars navigation.

Preserve this rule:

```text
mathematical positioning primitives
        !=
trust / DHT / authority semantics
```

The Holochain layer may consume the math library, but the mathematical core must not require Holochain.

## Spatial truth is not a scalar score

A single quality/trust number must never substitute for the underlying evidence.

A qualified spatial state should preserve separate dimensions such as:

```text
geometric uncertainty
+ measurement uncertainty
+ source identity
+ source/failure-domain independence
+ observation age/freshness
+ reference-system identity
+ fusion/derivation lineage
+ clock/time uncertainty
+ integrity/authenticity evidence
+ policy/authorization state
```

A 0–100 quality score may remain as a derived presentation aid. It is not authority and must not erase the components above.

## Immediate numeric invariants

All spatial numeric inputs must fail closed unless finite.

This includes, at minimum:

- latitude / longitude / altitude;
- Cartesian coordinates;
- range and sigma;
- velocity and acceleration;
- bearing/orientation values;
- covariance elements;
- sensor calibration values.

`NaN`, `+Inf`, and `-Inf` are invalid spatial facts.

Covariance must additionally be validated for:

- exact expected shape;
- finite values;
- symmetry within an explicit tolerance;
- non-negative diagonal entries;
- positive semidefiniteness within an explicit numerical tolerance.

Prefer fixed-size covariance types where dimensionality is fixed rather than `Vec<f64>`.

## Reference-system identity is mandatory

Bare coordinates are not globally meaningful.

Every reusable spatial state must bind an explicit reference-system identity. Position already operates across Earth, Moon, Mars, body-local, and local-navigation contexts, so a geodetic tuple without body/frame identity is insufficient.

The contract should support at least:

```text
Earth-fixed geodetic / Earth-centered
local tangent frame
subject/body frame
inertial frame
Moon-fixed / Moon inertial
Mars-fixed / Mars inertial
named external CRS/reference-system identifier
```

Where an external standard identifier exists, adapters should preserve it rather than silently normalizing away its identity.

OGC GeoPose is a useful interoperability target for position + orientation exchange because it explicitly models poses relative to reference frames, including non-Earth astronomical systems. OGC WKT CRS / ISO 19111-compatible identifiers are useful for coordinate-reference-system descriptions.

## Subject identity must be typed

A free-form `node_id: String` is not sufficient as the universal identity of a moving subject.

Introduce a domain-neutral subject reference that can distinguish namespaces and prevent accidental collisions without importing Transport types into Position.

Conceptually:

```text
SpatialSubjectRef {
    namespace,
    subject_id,
}
```

Examples may refer to a vehicle, cargo unit, robot, device, person, anchor, infrastructure asset, satellite, or synthetic simulation entity without Position needing to understand that domain.

## Observation, estimate, and prediction are different epistemic objects

Prediction must never masquerade as observation.

The public model should distinguish at least:

```text
Observed
Reported
Attested
Derived
Fused
Predicted
```

Prefer type separation where practical:

```text
PositionObservation
PositionEstimate
PredictedPosition
```

A transport arrival detector may consume observed/fused state according to policy. It must not silently treat a prediction as proof that a vehicle entered a port.

## Source provenance is richer than Local / Peer / Infrastructure

Preserve both *who supplied evidence* and *how the evidence was produced*.

A source description should be able to represent:

```text
GNSS
AIS
ADS-B
radar
satellite observation
UWB
LoRa
Wi-Fi RTT
cellular
IMU
visual odometry
acoustic
rail telemetry
fleet telematics
manual survey
authority report
cooperative estimate
synthetic prediction
```

It should additionally carry source identity and, when known, correlation/failure-domain metadata so that five feeds ultimately derived from one provider are not counted as five independent witnesses.

## Time is part of uncertainty

Do not collapse every time field into one wall-clock timestamp.

The spatial evidence model should distinguish, where applicable:

```text
observed_at
received_at
authored_at
processed_at
```

and bind the time basis / clock domain plus uncertainty or interval semantics.

A future trusted-time layer may qualify stronger claims. Raw provider timestamps, Holochain action timestamps, local wall clocks, and reception timestamps are not interchangeable.

The safest generic observation-time representation is an interval or nominal time plus asymmetric uncertainty rather than an unexplained integer timestamp.

## Recommended core state shape

Illustrative only; exact Rust ownership belongs to implementation PRs:

```text
PositionObservation {
    subject: SpatialSubjectRef,
    observed_time: ObservationTime,
    received_time: Option<ReceiptTime>,
    pose_or_position: SpatialState,
    uncertainty: SpatialUncertainty,
    source: ObservationSource,
    provenance: ProvenanceRef,
    epistemic_class: Observed | Reported | Attested,
}

PositionEstimate {
    subject: SpatialSubjectRef,
    state: SpatialState,
    uncertainty: SpatialUncertainty,
    reference_system: SpatialReferenceSystem,
    estimate_time: ObservationTime,
    contributors: bounded contributor commitments,
    algorithm_profile: AlgorithmProfileId,
    derivation_provenance: ProvenanceRef,
    epistemic_class: Derived | Fused,
}

PredictedPosition {
    subject: SpatialSubjectRef,
    predicted_for: ObservationTime,
    state: SpatialState,
    uncertainty: SpatialUncertainty,
    model_profile: ModelProfileId,
    prediction_provenance: ProvenanceRef,
}
```

## Orientation and kinematics are first-class

Transport and robotics both need more than point location.

The shared model should support optional:

- velocity;
- acceleration;
- heading/course;
- orientation;
- angular velocity;
- turn rate;
- climb/descent rate.

OGC GeoPose should be treated as an interoperability mapping target, not copied blindly into the internal authority model.

## Trajectories and moving features

A coordinate stream should become a structured moving-feature history, not an unbounded list of unrelated points.

Introduce bounded trajectory primitives and segment identities. Align external interfaces with OGC Moving Features where useful so Mycelix can exchange trajectories, directions, speeds, and time-varying attributes without inventing an incompatible API.

A trajectory must retain its provenance and reference-system semantics.

## Spatial geometry and relations

Add first-class domain-independent primitives for:

```text
Point
BoundingArea
Polygon
Corridor
RouteGeometry
Geofence
AltitudeBand
DepthBand
Trajectory
JurisdictionRegion
InfrastructureLocation
```

and derived relations/events such as:

```text
INSIDE
NEAR
CONNECTED_TO
APPROACHING
DEPARTING
CROSSED
ENTERED
EXITED
ALONG
INTERSECTS
```

These are spatial facts. Transport may map them into domain events such as `ARRIVED_AT_TERMINAL` or `DEPARTED_PORT`.

## Spatial-event causality

Do not emit important spatial events from a single threshold crossing without hysteresis or uncertainty handling.

A geofence engine should account for:

- positional covariance;
- boundary uncertainty;
- minimum dwell time;
- entry/exit hysteresis;
- out-of-order observations;
- duplicated observations;
- clock uncertainty;
- impossible jumps;
- late corrections.

When evidence is insufficient, emit an indeterminate state rather than forcing `inside = true/false`.

## Streaming boundary

Raw live telemetry does not belong in Holochain.

The intended pipeline is:

```text
provider / local sensor stream
        ↓
edge ingestion
        ↓
canonical normalization
        ↓
validation + deduplication + ordering
        ↓
hot state / time-series storage
        ↓
fusion + trajectories + spatial events
        ↓
significant evidence event / checkpoint
        ↓
cryptographic provenance
        ↓
Mycelix / Holochain trust and evidence plane
```

Holochain stores durable trust/evidence/control facts. Specialized streaming/time-series systems store the high-volume telemetry history according to policy.

## Event identity, replay, and correction

Every normalized observation should have a stable event identity suitable for deduplication and audit.

The ingestion contract should explicitly define:

- provider/source event identity where available;
- canonical Mycelix observation identity;
- duplicate handling;
- out-of-order handling;
- retractions/corrections without destructive history rewriting;
- deterministic replay rules;
- watermark/finality semantics for derived trajectory windows.

A corrected provider report must not silently rewrite the evidence that produced an earlier decision.

## Fusion safety

The math library already contains covariance intersection. Expand the surrounding policy rather than inventing a single global trust score.

Recommended defenses include:

- innovation / Mahalanobis gating;
- robust outlier rejection;
- modality-aware sanity checks;
- source correlation/failure-domain tracking;
- covariance intersection when cross-correlation is unknown;
- impossible-motion detection;
- clock-skew detection;
- bounded contributor sets and deterministic contributor ordering for audit;
- independent corroboration rules that distinguish independent sensors from duplicated feeds.

A source reputation signal may influence admission policy. It must not directly overwrite covariance or convert a weak measurement into precise mathematical evidence.

## External observation interoperability

Position adapters should map external observations into the canonical model while preserving source semantics.

Useful standards-facing adapters include:

- OGC SensorThings API for heterogeneous IoT observations;
- OGC SensorML for sensor/process metadata;
- OGC GeoPose for pose exchange;
- OGC Moving Features for moving-object trajectory exchange;
- transport-specific AIS, ADS-B, rail, fleet-telematics, DCSA, ONE Record, and EPCIS adapters above/beside this spatial core.

The internal model is not required to serialize identically to any one standard. Mappings must be explicit and loss-aware.

## Privacy and disclosure are Position concerns

Possessing a location does not imply permission to reveal it.

The disclosure model should support at least:

```text
Exact
Approximate
Delayed
RegionOnly
ProofOfPresence
ProofOfRoute
Aggregated
Private
```

with policy scoped by subject, purpose, recipient/capability, precision, delay, retention, and expiry.

The architecture must preserve:

```text
public infrastructure
    != public people
    != public cargo
    != public vehicle telemetry
```

Later Xenia/ZK integrations may prove bounded spatial predicates without publishing exact trajectories, but raw exact history must remain protected regardless of whether a proof system exists.

## Authorization is policy, not geometry

Position primitives must not hard-code one universal social/trust tier for every deployment.

Replace fixed operation thresholds with an authorization-policy boundary. Existing Mycelix consciousness/trust tiers can remain one policy implementation, but ports, railways, humanitarian systems, robot swarms, private facilities, regulated aviation systems, and public receive-only sensors must be able to supply different authorization semantics without forking spatial primitives.

Authorization answers whether an operation is permitted. It does not alter the geometric truth of a measurement.

## Transport contract

All transport-mode telemetry adapters normalize into Position first:

```text
AIS              -> vessel PositionObservation
ADS-B            -> aircraft PositionObservation
GNSS / ELD       -> road PositionObservation
rail telemetry   -> train/wagon PositionObservation
UWB / Wi-Fi RTT  -> cargo/yard PositionObservation
```

Transport may consume a policy-qualified spatial output such as:

```text
VerifiedSpatialState {
    subject,
    state,
    uncertainty,
    reference_system,
    time,
    provenance,
}
```

and derive:

```text
ARRIVED
DEPARTED
TRANSFERRED
LOADED
UNLOADED
DELAYED
DELIVERED
```

Transport must not own GPS filtering, reference-frame conversion, generic geofencing, sensor fusion, or spatial privacy semantics.

## No spatial evidence -> actuation authority shortcut

A verified position proves only the qualified spatial statement.

It does not by itself authorize:

- route changes;
- vehicle actuation;
- cargo release;
- enforcement;
- financial settlement;
- emergency dispatch;
- autonomous interception.

Those effects require independent domain authority and policy.

## Migration sequence

### P-0 — this architecture contract

Freeze ownership and non-goals before schema expansion.

### P-1 — numeric and covariance hardening

Reject non-finite values, add fixed covariance invariants, and regression-test all affected shared validators.

### P-2 — reference-system/body identity

Eliminate ambiguous bare-coordinate states.

### P-3 — epistemic/source provenance model

Separate observation, attestation, derivation, fusion, and prediction. Expand source identity/failure-domain semantics.

### P-4 — time semantics

Separate observation/receipt/authorship time and model uncertainty/time basis explicitly.

### P-5 — trajectory/geofence/spatial-event primitives

Add bounded moving-feature histories and uncertainty-aware spatial transitions.

### P-6 — privacy/disclosure policy

Make exact trajectory disclosure capability-controlled and purpose/precision/time bounded.

### P-7 — authorization-policy decoupling

Move fixed consciousness thresholds behind a policy interface without weakening existing deployments.

### T-1 — transport core

Introduce transport semantics that depend on Position outputs only.

### T-2+ — live adapters and higher layers

AIS, ADS-B, rail, road, cargo/yard sensing, standards mappings, event correlation, Symthaea state estimation/forecasting, and Sol Atlas visualization.

## Qualification requirements

Each implementation PR should add focused tests before claiming a stronger spatial theorem.

At minimum, regression coverage should include:

- NaN/Inf rejection for every numeric entry path;
- malformed covariance rejection;
- cross-reference-system confusion rejection;
- prediction-as-observation rejection;
- source-identity substitution rejection;
- duplicated/correlated-source handling;
- out-of-order and duplicate observation replay;
- geofence hysteresis and uncertainty-boundary cases;
- disclosure downgrade/expiry behavior;
- transport cannot import or construct internal Position authority by bypass.

## Non-goals

This contract does not establish a globally authoritative location oracle.

It does not claim that every observation is honest, every source is independent, every reference transformation is qualified, every clock is trusted, or every fused estimate is correct.

The purpose of the architecture is the opposite: preserve those distinctions so later layers can make narrow, auditable claims without laundering ambiguity into authority.
