# Mycelix Position / Transport — Qualification Research Refinements v0.2

Status: non-authoritative research and architecture note

Related issues: #416, #417, #418, #419, #433, #434, #435, #436, #437, #438

Related implementation stack: #423 -> #426 -> #427

## Evidence boundary

This document records research-backed design refinements. It does **not** upgrade the evidence status of any implementation PR.

At authoring time, the exact-head Position Qualification runs for #426 and #427 remain queued. No runtime PASS is claimed here.

The central design rule remains:

> Each theorem must be independently observable, versioned, and fail closed at the boundary where it becomes authority-bearing.

## 1. Refined theorem map

The original Q0-Q9 structure remains useful, but several categories need explicit sub-theorems so one numerical or systems property cannot stand in for another.

### Q2 — reference and time

Split Q2 into:

```text
Q2-F  reference/frame qualification
Q2-T  time-scale / clock-domain qualification
Q2-X  time-dependent transform provenance
```

A spatial tuple is incomplete without both a qualified reference and a qualified time basis where time matters.

### Q4 — estimator numerical validity

Split the remaining estimator work into:

```text
Q4-E.1 covariance PSD qualification             #426
Q4-F   full vector measurement covariance       #427
Q4-E.2 geometry observability / conditioning     #433
Q4-G   estimator + process-noise profile         #434
Q4-H   replay/profile commitment                 #434
```

These are deliberately separate from Q5 conservative cross-source fusion.

## 2. Covariance admissibility is not estimator conditioning

A symmetric covariance matrix with non-negative diagonal entries can still be indefinite. #426 closes that gap for the current EKF profile by qualifying the complete matrix.

But the reverse distinction also matters:

```text
valid PSD covariance
!=
well-conditioned estimator geometry
```

A trilateration normal/Fisher-information matrix may be invertible yet sufficiently ill-conditioned that the result should not be promoted to a strong qualified state.

For trilateration, expose a geometry qualification object rather than a boolean inversion result. It should carry at least:

- status (`WellConditioned`, `WeakGeometry`, `NearSingular`, `Singular`, `Indeterminate` or equivalent);
- effective rank;
- smallest/largest information eigenvalue or an equivalent factorization diagnostic;
- reciprocal condition number or an explicitly equivalent metric;
- geometry-only dilution/amplification metric where meaningful;
- physical residual RMS;
- normalized/weighted residual statistic;
- anchor count;
- profile identity.

Do not use raw determinant magnitude as the sole conditioning test. Determinants are strongly scale-dependent and do not provide a stable substitute for condition diagnostics.

## 3. Measurement covariance R and innovation covariance S are different contracts

For a vector Kalman update:

```text
S = H P H^T + R
```

The measurement-noise covariance `R` is a covariance and therefore may be positive semidefinite. It does not need to be strictly positive definite merely so `R` itself can be Cholesky-factored.

The ordinary update instead needs the innovation covariance `S` to be sufficiently positive definite / well-conditioned for the declared solve.

Therefore the stronger target after the initial #427 conservative implementation is:

1. qualify `R` independently as finite, symmetric, and PSD;
2. construct `S` from the qualified prior and `R`;
3. require/fail closed on the factorization/conditioning needed by the selected numerical profile;
4. preserve the distinction in diagnostics and errors.

A semidefinite valid `R` plus a solvable `S` should be admissible under a profile that claims this theorem.

## 4. Operational innovation gating is not statistical calibration

For a vector innovation `nu`, normalized innovation squared is:

```text
NIS = nu^T S^-1 nu
```

Under the declared Gaussian model, NIS is interpreted against a chi-square distribution whose degrees of freedom correspond to the measurement dimension.

Do not silently reinterpret a one-dimensional `sigma` threshold by multiplying by vector dimension and call that calibrated statistics.

Separate:

```text
online admission / outlier policy
from
model-consistency qualification
```

A versioned innovation policy should make its theorem explicit, for example:

```text
InnovationPolicy
  Disabled
  ScalarSigma { sigma }
  NisChiSquare { alpha, dof_policy }
  Named { profile_id }
```

Exact names are illustrative.

Offline qualification should support NIS and, when a truth/reference trajectory is available, NEES-style consistency tests over declared seeded scenarios.

Passing such a campaign qualifies the declared model/noise/scenario profile. It does not prove universal physical correctness.

## 5. Estimator identity must include the model, not only the algorithm family

`EKF` alone is not enough identity for an authority-bearing derivation.

A profile should commit at least:

```text
EstimatorProfile {
    profile_id,
    profile_version,
    state_model,
    process_noise_model,
    measurement_models,
    innovation_policy,
    covariance_realization,
    numerical_tolerances,
    parameter_commitment,
}
```

The current position/velocity process-noise fields should become part of a named, versioned physical model with explicit units and exact `Q(dt)` construction semantics.

Changing continuous-vs-discrete interpretation, process-noise units, `Q(dt)` construction, numerical tolerances, or innovation policy changes estimator semantics and therefore changes the profile identity.

## 6. Keep covariance/Joseph and square-root/factorized filters as separate profiles

The Joseph covariance-form implementation is a useful auditable baseline. Numerical filtering literature and JPL/NASA practice also motivate square-root or factorized realizations because finite-precision covariance arithmetic can lose positive-semidefiniteness.

Do not silently replace the qualified covariance/Joseph realization with a square-root implementation and call it the same replay profile.

Treat numerical realization as part of the estimator identity, for example:

```text
EkfCovarianceJosephV1
EkfSquareRootQrV1
UdFactorizedV1
SrifV1
```

Exact names are illustrative.

Equivalent mathematical intent does not imply identical finite-precision replay semantics.

## 7. Replay should bind evidence, model, numerical policy, frames, and time

A replay capsule should make hidden estimator context impossible:

```text
EstimatorReplayCapsule {
    estimator_profile_id,
    estimator_profile_version,
    initial_state_commitment,
    ordered_observation_commitments,
    reference_frame_commitment,
    qualified_time_profile,
    calibration_profile_commitments,
    numerical_policy_commitment,
    software_or_build_identity,
    output_commitment,
    diagnostics_commitment,
}
```

For any profile claiming deterministic replay:

- input ordering is canonical;
- duplicate handling is deterministic;
- no hidden wall-clock/randomness enters the computation;
- serialization/hashing is canonical;
- environment/floating-point assumptions are stated;
- changes in tolerances or numerical realization change profile identity.

If bitwise cross-platform determinism is not demonstrated, state the weaker replay theorem instead of implying it.

## 8. Reference frames and time are linked but not the same subsystem

Reference-frame identity belongs to #416. Qualified time belongs to #435.

A robust internal model should distinguish at least:

```text
observation time
!=
authoring/publication time
!=
receipt time
!=
validity interval
!=
local monotonic clock
!=
spacecraft/device clock
!=
UTC representation
!=
TAI / TT / GPS / TDB-style time scales
```

A numeric epoch is not a complete temporal fact without its basis.

External exchange profiles can use Unix/UTC-style representations when their profile declares that choice, but Position should not make one such representation the implicit universal internal clock.

## 9. Clock correlation is a derivation with provenance

A source device or spacecraft clock can require an explicit correlation profile before it can be compared with a standard time scale.

Represent that conversion as a derivation rather than erasing the source reading:

```text
ClockCorrelation {
    source_clock,
    destination_scale,
    correlation_profile,
    calibration_or_kernel_commitment,
    valid_interval,
    uncertainty_model,
    input_commitment,
    output_commitment,
}
```

Time-dependent transforms must bind the qualified transform time and the exact transform/kernel/profile lineage.

## 10. Terrestrial moving features and spaceflight state should share a kernel, not one overloaded trajectory profile

OGC Moving Features provides useful terrestrial/web trajectory semantics but is intentionally scoped to contexts where relativistic effects are not significant.

GeoPose provides stronger frame-graph concepts and supports astronomical reference systems.

For Mycelix, use a shared Position kernel for:

```text
reference identity
qualified time
uncertainty
provenance
transform lineage
evidence class
```

Then place profile-specific motion semantics above it:

```text
TerrestrialMovingFeatureProfile
NavigationOrSpaceflightStateProfile
```

Do not stretch one Earth/web trajectory contract until it implicitly claims orbital or deep-space navigation semantics.

## 11. Transport needs three linked graphs

Do not model physical movement, communication reachability, and evidence synchronization as one generic route graph.

Use at least:

```text
PhysicalRouteGraph
CommunicationContactGraph
EvidenceReconciliationGraph
```

### PhysicalRouteGraph

Edges represent movement of people, cargo, vehicles, or services through physical space.

### CommunicationContactGraph

Edges represent time-bounded opportunities for data transfer. A contact should carry a qualified time basis, directionality, expected latency, capacity, schedule confidence/class, source commitment, and optional security profile.

Distinguish at least:

```text
Scheduled
Predicted
Opportunistic
Observed
CancelledOrUnavailable
```

Scheduled-contact routing can interoperate with CCSDS schedule-aware/CGR-style concepts without reimplementing Bundle Protocol in Transport.

### EvidenceReconciliationGraph

Edges/receipts represent what causal/evidence frontier was synchronized across domains and what conflicts remain.

A successful data transfer does not automatically resolve application-level semantic conflicts.

## 12. Keep physical custody and DTN reliability semantics namespaced

Do not reuse a single unqualified `custody` concept for both logistics and delay-tolerant networking.

```text
physical cargo custody
!=
message/bundle forwarding or reliability state
```

Likewise:

```text
message delivered
!=
physical object delivered
```

Networking security can authenticate/protect transferred data without creating a spatial, custody, or proof-of-delivery truth claim.

## 13. OperationalDomain should own autonomy/reconciliation policy, not imply topology

A domain model should bind policy profiles rather than assume a particular network shape:

```text
OperationalDomain {
    domain_id,
    parent_domain?,
    authority_profile,
    reference_frame_profile,
    qualified_time_profile,
    evidence_policy_profile,
    sync_policy_profile,
}
```

Physical routes, communication contacts, and reconciliation events can all reference a domain while retaining their distinct meanings.

This is the right abstraction for intermittently connected terrestrial sites, vessels, aircraft, orbital assets, lunar sites, and later interplanetary domains.

## 14. Logistics interoperability must be loss-aware

External standards should be source profiles, not aliases for a universal internal `TransportEvent`.

Initial adapter families:

```text
Epcis20Adapter
DcsaTrackTraceAdapter { version }
IataOneRecordAdapter { version }
UnCefactMmtAdapter { release }
```

Every mapping should surface:

```text
InteropMappingResult<T> {
    value,
    source_profile,
    source_event_or_object_id,
    mapped_fields,
    unmapped_fields,
    normalized_fields,
    lossy,
    source_commitment,
}
```

Unknown or unrepresented source fields must be reported rather than silently dropped where the adapter claims semantic preservation.

## 15. Do not force object graphs, observations, and business events into one enum

A stronger internal Transport decomposition is likely:

```text
TransportObservation
TransportBusinessEvent
TransportPlan
TransportAssetState
TransportCustodyEvent
TransportDocumentClaim
TransportRelationship
TransportDerivation
TransportQualification
```

One external object may legitimately produce several linked internal evidence objects.

This aligns better with EPCIS business events, DCSA tracking events, ONE Record linked logistics objects, and UN/CEFACT multimodal reference semantics.

## 16. Preserve source time and epistemic meaning during interoperability mapping

In particular:

- EPCIS real-world event time is not the same as repository `recordTime`;
- a carrier/provider record remains a provider/business claim unless separately qualified;
- a port/facility identifier must not be manufactured into exact coordinates;
- movement/location does not by itself prove custody transfer;
- authenticated source data does not automatically become independent physical observation;
- newer adapters may read older source versions but must retain original profile/version provenance.

Only claim lossless or semantic-equivalent round trip where tests prove it.

Useful mapping classifications:

```text
LosslessRoundTrip
SemanticallyEquivalent
LossyButDeclared
Unsupported
```

## 17. Refined implementation order

Do not introduce the large Transport core while the numerical and evidence foundations are still ambiguous.

Recommended order:

```text
#423 Joseph covariance baseline
  -> #426 Q4-E.1 covariance PSD
  -> #427 Q4-F correlated vector measurement semantics
  -> #433 Q4-E.2 observability/conditioning
  -> #434 Q4-G/H model, consistency, replay
  -> #417 Q5 conservative fusion
  -> #416 Q2-F frames
  -> #435 Q2-T/X time + dynamic transform lineage
  -> #419 v2 epistemic evidence graph
  -> shared OperationalDomain / evidence foundations
  -> #436 transport/contact/reconciliation graph contracts
  -> #437 loss-aware logistics interoperability
  -> minimal mycelix-transport-core
```

The precise merge order can change when exact-head evidence exposes implementation defects. The dependency direction should not.

## 18. Earth -> Moon remains the best integration theorem

The first cross-domain scenario should prove that the architecture can represent, without semantic collapse:

```text
factory / warehouse
-> terrestrial road/rail/sea/air leg
-> launch-site custody transition
-> launch/orbit spatial state
-> scheduled/predicted communication contacts
-> cislunar/lunar communication
-> lunar operational-domain partition
-> lunar surface delivery
-> delayed evidence reconciliation
```

At every step, keep separately inspectable:

- physical position/motion;
- physical custody;
- communication reachability;
- evidence availability;
- evidence reconciliation;
- domain authority;
- time basis;
- reference frame;
- uncertainty;
- provenance.

## 19. External references reviewed

Primary standards and technical references informing this refinement include:

- OGC GeoPose 1.0;
- OGC API — Moving Features;
- NASA/JPL SPICE reference-frame, time, and spacecraft-clock documentation;
- JPL/NASA square-root and factorized Kalman-filter work;
- IETF RFC 9171 Bundle Protocol Version 7;
- IETF RFC 9172 Bundle Protocol Security;
- CCSDS Schedule-Aware Bundle Routing / contact-plan concepts;
- NASA LunaNet interoperability specification/framework;
- GS1 EPCIS and CBV 2.0;
- DCSA Track & Trace;
- IATA ONE Record;
- UN/CEFACT Multimodal Transport Reference Data Model.

External standards do not grant internal authority merely by being mapped. The adapter/profile must state exactly what is preserved, transformed, lost, and qualified.

## 20. Non-goals

This refinement does not claim:

- that the current queued PRs have executed successfully;
- universal EKF calibration;
- that square-root filtering is automatically superior for every profile;
- that one terrestrial trajectory standard covers relativistic/spaceflight navigation;
- that BPv7/BPSec/SABR should be reimplemented inside Mycelix;
- that network delivery proves logistics delivery;
- that a standards mapping upgrades source epistemic strength;
- that Mycelix replaces SPICE, GNSS, IERS, CCSDS, GS1, DCSA, IATA, or UN/CEFACT systems.

The goal is narrower and stronger: establish explicit composable contracts so Position can become a trustworthy spatial evidence substrate and Transport can consume it without erasing the distinction between physics, communications, provenance, and domain authority.
