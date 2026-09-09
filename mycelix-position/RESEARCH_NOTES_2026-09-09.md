# Mycelix Position — Standards and Qualification Research (2026-09-09)

Status: non-authoritative research notes

Related: #413, #414, #415, #416, #417

## Purpose

This note records external interoperability research and code-level qualification findings discovered while promoting `mycelix-position` into the common spatial-truth substrate.

It does not establish runtime authority. Normative implementation contracts belong in focused follow-up issues/PRs.

## External standards alignment

### OGC Observations, Measurements, and Samples (OMS / ISO 19156:2023)

OMS gives us a strong conceptual boundary for the *act of observation* and its result. Position should preserve the distinction between:

- the feature/subject being observed;
- the observation act;
- the procedure/sensor/process;
- the observed property;
- the result;
- sampling context where relevant.

This reinforces the proposed separation between `PositionObservation`, `PositionEstimate`, and `PredictedPosition` rather than treating every coordinate-like value as one generic position.

### OGC API - Connected Systems 1.0

Connected Systems is especially relevant to the planned live-data edge plane because it bridges system/deployment metadata and dynamic observations/commands. It supports systems, procedures, deployments, datastreams, observations, control streams, commands, system events, streaming, and historical access.

Recommendation:

- treat Connected Systems as a first-class adapter/conformance target for heterogeneous live sensing;
- do **not** make Position's internal authority model identical to the OGC API representation;
- preserve an explicit non-actuation boundary: observation qualification never implies command authority.

This is a stronger general live-sensor interoperability target than inventing one custom endpoint per IoT modality.

### OGC SensorThings API / SensorML

SensorThings remains useful for broad IoT compatibility and observation retrieval. SensorML is useful for sensor/process descriptions and post-measurement transformations.

Recommendation:

- map SensorThings/Connected Systems observations into canonical Position observations;
- map SensorML/Connected Systems procedures into versioned `ObservationProcedure` / `SensorProfile` metadata;
- retain the source standard/version and lossiness of each mapping in provenance.

### OGC GeoPose 1.0

GeoPose explicitly separates poses and reference frames and supports earth-anchored and other astronomical coordinate systems. Its logical model also includes frame transforms and composite chains/graphs.

Recommendation:

- use GeoPose as an interoperability target for position + orientation + frame graphs;
- adopt the conceptual distinction between outer/inner frames and explicit transforms;
- do not inherit GeoPose's single Unix-time assumption as Mycelix's complete trusted-time model;
- preserve explicit external frame identifiers and transform provenance.

### NASA/JPL NAIF SPICE

SPICE demonstrates why extraterrestrial positioning needs named, versioned, time-dependent reference frames and transformation kernels. It distinguishes inertial, body-fixed, mission/instrument, and parameterized dynamic frames.

Recommendation:

- P-2 reference-system work should support externally identified frame authorities, including future SPICE/NAIF mappings;
- a non-Earth `body` identifier alone is insufficient for high-precision space state;
- frame transforms may be time-dependent and therefore require an epoch/time scale and transform-profile provenance.

### OGC Moving Features 1.0

Moving Features models time-varying positions/attributes and provides an API for querying and exchanging trajectories.

Recommendation:

- model trajectories as bounded, versioned moving-feature histories rather than unrelated point lists;
- expose an optional Moving Features-compatible API mapping above the canonical Position trajectory store;
- keep evidence lineage and uncertainty semantics that Moving Features does not itself supply.

### GS1 EPCIS/CBV 2.0

EPCIS is explicitly a business-event visibility standard: what, where, when, why, and how, including chain of custody and sensor data.

Recommendation:

- Position supplies qualified spatial observations/relations;
- Transport maps these into EPCIS-compatible business events where relevant;
- do not place EPCIS business semantics inside Position.

### DCSA Track & Trace 2.2

DCSA defines interoperable container-shipping track-and-trace events, APIs, information models, and event naming across shipment phases.

Recommendation:

- preserve DCSA events as transport/logistics semantics;
- Position may provide/verify the underlying vessel/container spatial evidence;
- Transport owns DCSA mapping and correlation.

### IATA ONE Record

ONE Record defines an air-cargo data model, JSON-LD exchange APIs, and a federated security model for a shared shipment record.

Recommendation:

- Transport owns ONE Record logistics entities/events;
- Position provides qualified spatial references and observation provenance;
- avoid duplicating ONE Record's shipment model in Position.

### IETF SCITT (RFC 9943)

SCITT provides a useful architecture analogy for signed statements, append-only transparency services, and receipts demonstrating registration of statements.

Recommendation:

- do not replace Holochain with SCITT;
- consider SCITT-compatible export/receipt profiles for external audit ecosystems;
- preserve the same conceptual split between an issuer statement and a transparency/registration receipt.

## Code-level findings

### 1. DHT `PositionEstimateEntry` is currently a claim, not a verified derivation

The current coordinator accepts a caller-supplied `PositionEstimateEntry`, replaces only `computed_by`, and writes it. Despite module documentation saying the zome computes positions from stored ranges/anchors, the endpoint does not bind the result to:

- range measurement action hashes;
- anchor action hashes/certifications;
- a reference system;
- an algorithm profile/version;
- deterministic parameters;
- a derivation commitment;
- an observation-time basis.

Therefore the current entry should be treated epistemically as a self-authored spatial estimate claim, not as proof that Mycelix recomputed or verified the estimate.

### 2. Tier comments are not runtime authority

The anchor/ranging coordinators contain comments saying Participant/Steward tiers are required, while anchor registration explicitly states that standalone deployment currently allows all registrations and certification still contains a TODO. The Position estimate endpoint likewise documents a Citizen requirement without enforcing one there.

Authorization must be moved behind a real policy boundary, and comments/constants must never be counted as enforced authority.

### 3. Anchor certification graph is incomplete

The shared model declares `AnchorToCertifications`, but `certify_anchor` currently creates the certification entry without linking it to the referenced anchor. The integrity path does not establish that the referenced anchor exists or bind the certification to a particular anchor action/version.

`AnchorNode.certified_by` is also immutable and initialized to `None`, so it is not a durable representation of later certifications.

A v2 certification should reference the exact anchor action/commitment being certified and expose certifications through validated links/indexes.

### 4. Observation time and DHT authoring time are conflated

Current coordinator-created `registered_at`, `measured_at`, and similar fields use local wall-clock time, while a modified coordinator can construct an entry with a different embedded timestamp unless integrity binds it.

For v2:

- `observed_at` belongs to evidence and may come from a sensor/provider clock;
- `received_at` belongs to ingestion;
- `authored_at` should normally derive from the Holochain action timestamp rather than a duplicate caller field;
- clock/time-scale/uncertainty metadata must stay explicit.

### 5. `get_latest_position` is latest publication, not latest observation

The query chooses the latest record by Holochain action timestamp. This is valid for "latest authored/publication" semantics but not necessarily "latest observed state" semantics.

Name/query contracts should explicitly distinguish:

- latest authored claim;
- latest observed state;
- latest qualified estimate;
- latest valid-at-time state.

`get_position_history` also documents newest-first behavior but currently does not sort its results.

### 6. Measurement routing's `HighConfidenceOnly` is not confidence-based

The current router treats a fixed modality allowlist (UWB, Wi-Fi RTT, GPS) as `HighConfidenceOnly`. Confidence is contextual; UWB under NLOS, spoofed GNSS, or malformed Wi-Fi RTT can be poor evidence, while another modality can be excellent.

Rename this policy or make routing consume qualified measurement uncertainty, health, freshness, procedure profile, and source status.

### 7. Clock reversal is silently treated as zero age

`fix_age_s(fix_us, current_us)` returns zero when the current timestamp is before the fix timestamp. This hides clock skew/future-dated evidence.

Return an explicit clock-order error/indeterminate state instead of making future evidence appear maximally fresh.

### 8. `confidence_from_sigma` invents a domain-specific scalar

`100 * exp(-sigma/10m)` has no universal probabilistic meaning and folds a dimensional standard deviation into an arbitrary percentage scale.

Keep uncertainty in physical/statistical units. If a UI needs a score, make the profile/version explicit and never feed it back into authority or covariance.

### 9. Trilateration rewrites trust into statistical precision

`trilaterate_3d/2d` currently uses:

```text
weight = trust / sigma^2
```

which is equivalent to modifying measurement precision based on trust. That violates the stronger separation identified in #417.

Source trust/reputation should influence admission, policy, or hypothesis selection; the measurement covariance must remain a statement about physical/statistical uncertainty unless a calibrated model justifies a change.

The trilateration inputs also need finite/positive checks for anchors, sigmas, trust metadata, and solver outputs.

### 10. EKF covariance algebra needs qualification

The filter describes its update as Joseph form, but the implemented range update computes `(I-KH)P` and then adds `KRK^T`; it does not include the right-side `(I-KH)^T` factor required by the Joseph stabilized form.

The predict step also updates selected diagonal elements instead of calculating the full `F P F^T + Q`, so position/velocity cross-covariance is not propagated correctly.

Absolute position/velocity updates consume only covariance diagonal elements and ignore cross-axis covariance.

Before Position presents EKF covariance as qualified uncertainty, this math needs a focused correctness tranche with symmetry/PSD/property tests.

### 11. Ranging conversion APIs silently coerce invalid physics

Several ranging functions return an estimate unconditionally and use `.max(...)` to coerce negative or invalid values toward zero/minimum uncertainty.

For authority-bearing ingestion, invalid physical inputs should produce a typed error, not a plausible-looking range.

Examples requiring explicit contracts include:

- negative/non-finite time of flight;
- zero/non-finite bandwidth;
- invalid RSSI path-loss exponent;
- non-finite acoustic sound speed/timing uncertainty;
- invalid survey accuracy;
- invalid per-hop-distance assumptions.

### 12. Coverage grid can fail to terminate for an invalid step

`coverage_grid_2d` increments loop coordinates by a caller-provided `step` without validating that it is finite and strictly positive. A zero or negative step can prevent termination; NaN can produce misleading behavior.

Make grid generation fallible and bound maximum generated point counts/resources.

### 13. Space-navigation state lacks a precise frame/time contract

`SpaceNavigationEstimate` calls its state "ECI" but does not name which inertial frame. `epoch_jd` does not specify a time scale (UTC, TAI, TT, TDB, etc.), and `propagate(dt)` currently does not advance the epoch field. Altitude is calculated against a hard-coded spherical Earth radius even though the repository already has body models.

For serious space/interplanetary use, bind:

- observer/center body;
- exact reference frame;
- time scale and epoch;
- gravitational/body model/version;
- propagation model/version;
- uncertainty/covariance;
- state-transition provenance.

NASA SPICE is the appropriate interoperability reference for this layer.

## Recommended architecture after research

```text
External provider / local sensor
        │
        ▼
Observation adapter
  - source standard + version
  - raw source event identity
  - loss-aware mapping
        │
        ▼
Canonical Observation
  - typed subject
  - procedure/sensor profile
  - observed property
  - result
  - reference frame
  - observation time + uncertainty
  - measurement uncertainty
  - provenance
        │
        ▼
Admission / qualification
  - schema + finite/physical validity
  - auth/integrity
  - source status
  - freshness
  - failure-domain correlation
        │
        ├── rejected / conflicted / indeterminate
        │
        ▼
Estimator / Fusion
  - named algorithm profile
  - statistical assumptions
  - deterministic input commitments
        │
        ▼
Qualified Estimate / Trajectory
  - explicit uncertainty
  - reference system
  - derivation lineage
        │
        ▼
Spatial Relation/Event
  - entered / exited / crossed / near / along
        │
        ▼
Domain event
  Transport / Emergency / Robotics / Sol Atlas
        │
        ▼
Durable evidence checkpoint
  Holochain + optional external transparency receipt
```

## Priority after #415

1. Repair fusion/trust semantics (#417).
2. Qualify EKF/trilateration covariance and numerical contracts.
3. Implement explicit reference systems and versioned schemas (#416).
4. Introduce v2 epistemic evidence graph: claim, observation, derivation, attestation, qualification.
5. Introduce observation/receipt/authorship time model.
6. Introduce trajectories/spatial relations using Moving Features/GeoPose mappings.
7. Introduce Connected Systems/SensorThings adapters.
8. Add DCSA/ONE Record/EPCIS mappings in Transport, not Position.

Only after 1–5 should live multimodal transport ingestion be allowed to emit authority-bearing Position outputs.
