# Mycelix Position — Qualification Theorems v0.1

Status: non-authoritative qualification plan

Related: #413, #415, #416, #417

The purpose of this document is to keep implementation claims narrow. Passing one theorem does not imply the next.

## Q0 — Serialization safety

For every authority-facing v2 spatial type:

- parsing/deserialization is bounded;
- required identity/reference/provenance fields cannot be omitted;
- non-finite numeric values are rejected;
- enum/version handling fails closed for unknown authority-bearing semantics.

Does **not** establish physical plausibility, truth, or provenance authenticity.

## Q1 — Physical-domain validity

For every accepted observation:

- units are explicit;
- numeric values are within the declared physical/protocol domain;
- uncertainty is finite and non-negative/positive as appropriate;
- impossible protocol values are rejected rather than clamped into plausible values.

Does **not** establish sensor honesty or calibration.

## Q2 — Reference/time qualification

For every accepted spatial state:

- reference system/frame is explicit;
- local/body frames bind required parents/origins;
- time scale/clock domain is explicit where needed;
- observation time and authoring time are distinguishable;
- transformations preserve source/destination frame and profile lineage.

Does **not** establish transform accuracy beyond the qualified profile.

## Q3 — Observation provenance

For every qualified observation:

- subject identity is typed;
- observation source/procedure is identified;
- source event identity supports deduplication where available;
- source standard/version and mapping profile are retained;
- author/authenticator and observation source are not conflated;
- corrections/retractions preserve history.

Does **not** establish statistical independence between observations.

## Q4 — Estimator numerical validity

For every estimator/filter output claiming covariance:

- input covariance is valid for the algorithm profile;
- covariance updates preserve symmetry and positive semidefiniteness within explicit tolerance;
- invalid/ill-conditioned states return errors/indeterminate results rather than manufactured precision;
- deterministic replay with the same qualified input sequence produces the same output profile/commitment.

Does **not** establish calibration or correct model assumptions.

## Q5 — Fusion conservatism / correlation policy

For every fused estimate:

- the fusion profile states its assumptions about cross-correlation;
- unknown correlation is handled conservatively;
- source count is not equated with independence;
- source/reputation trust is not directly rewritten into physical covariance;
- contradictory evidence may yield a conflicted/indeterminate state instead of forced consensus.

Does **not** establish Byzantine resistance by itself.

## Q6 — Trajectory/event semantics

For every derived trajectory/spatial event:

- input states retain order and time/reference semantics;
- duplicate/out-of-order observations follow deterministic rules;
- geofence/boundary events include uncertainty and hysteresis/dwell policy;
- latest-authored, latest-observed, latest-qualified, and valid-at-time queries are distinct;
- predictions cannot satisfy observation-only event predicates.

Does **not** grant domain authority.

## Q7 — Disclosure safety

For every spatial read/export:

- disclosure policy is evaluated independently from possession of data;
- precision, delay, retention, purpose, recipient/capability, and expiry can be bounded;
- aggregate/region/proof modes cannot silently downgrade into exact trajectory disclosure;
- private people/cargo/vehicles are not public merely because public infrastructure is visible.

Does **not** establish anonymity against all auxiliary information.

## Q8 — Domain derivation

For every Transport/Emergency/Robotics/etc. event derived from Position:

- the domain event binds the spatial evidence/profile it consumed;
- the domain layer owns domain vocabulary and authority;
- Position does not import domain-specific semantics;
- spatial evidence alone cannot grant actuation, custody transfer, financial settlement, enforcement, or dispatch authority.

## Q9 — External interoperability

For each supported external profile:

- mapping is versioned;
- mandatory source fields are either represented or explicitly reported as lost/unmapped;
- round-trip tests exist where round-trip preservation is claimed;
- conformance tooling is used where the standards body provides it;
- an external mapping can never strengthen the internal epistemic class silently.

Candidate mappings include OGC OMS, OGC API Connected Systems, SensorThings, SensorML, GeoPose, Moving Features, GS1 EPCIS, DCSA T&T, IATA ONE Record, and future SCITT-compatible transparency exports.

## Rule

A PR must state the strongest theorem it actually establishes and explicitly list the stronger theorems it does not establish.
