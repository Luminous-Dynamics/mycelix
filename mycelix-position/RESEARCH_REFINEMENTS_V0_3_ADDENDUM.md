# Mycelix Position / Transport — Deep Qualification Audit Addendum v0.3

Status: non-authoritative research / code-audit addendum

Parent note: `RESEARCH_REFINEMENTS_V0_2.md`

Related implementation stack: #423 -> #426 -> #427

New/refined issues: #447, #448, #449, #450, #451, #453, #455, #456, #457, #458; refinement comment on #417.

## Evidence boundary

This addendum comes from direct inspection of the live #427 source plus external numerical, cartographic, navigation, and interoperability references.

It changes **no implementation head** and establishes **no runtime PASS**. At authoring time, the exact-head Position Qualification jobs for #426 and #427 remain queued.

The purpose is to make the next implementation tranches smaller and more truthful before additional code is stacked on unexecuted estimator heads.

## A. Q4 needs a solver theorem, not only a conditioning diagnostic

The current 3D trilateration path forms weighted normal equations:

```text
N = J^T W J
b = J^T W r
```

and directly solves/inverts the 3x3 normal matrix.

The deep audit found three semantics that must become explicit:

1. numerical convergence;
2. geometry rank/conditioning;
3. residual/model fit.

They are not interchangeable.

The current implementation can reach `MAX_ITERATIONS` and still accept the result whenever the physical residual RMS is <= 1 m. That makes an arbitrary domain-scale residual threshold stand in for numerical convergence.

The current implementation also skips an observation when the iterate lies within `1e-10` of an anchor, while retaining the original anchor count and residual denominator.

Refined theorem split:

```text
Q4-E1 covariance admissibility            #426
Q4-E2 geometry observability/conditioning #433
Q4-E3 nonlinear solver/convergence        #447
```

A first qualified solver profile should solve the whitened least-squares system directly with a rank-aware QR/orthogonal factorization rather than relying only on explicit normal-matrix inversion.

Expose at least:

```text
solver profile
active observation count
effective rank
conditioning result
convergence status
physical residual RMS
weighted cost/statistic
covariance approximation semantics
```

No universal 1 m convergence threshold.

## B. Q(dt) is estimator physics, not a pair of generic noise scalars

The EKF now correctly propagates:

```text
P' = F P F^T + Q
```

but the present compatibility profile still adds independent diagonal terms proportional to `dt` for position and velocity.

That is not yet a named stochastic-motion model.

Preserve it as an explicit legacy profile, then introduce physically interpretable process models with units and exact discretization semantics.

For continuous white acceleration in one axis:

```text
Q(dt) = q_a * [ dt^3/3  dt^2/2
                dt^2/2  dt     ]
```

For independent piecewise-constant acceleration over a step:

```text
G = [dt^2/2, dt]^T
Q(dt) = G sigma_a^2 G^T
```

These are different physical models with different parameter units and replay identity.

Future 3D profiles should allow anisotropic/correlated acceleration uncertainty in an explicit frame.

Tracked by #448 and #434.

## C. `sqrt(trace(P))` is a covariance summary, not a probability radius

The current trilateration output labels:

```text
sqrt(P_xx + P_yy + P_zz)
```

as a 1-sigma position uncertainty.

In multiple dimensions that is a useful RMS-like covariance summary, but it is not a one-dimensional sigma and is not automatically a 68% radial confidence region.

For an isotropic 3D covariance with per-axis sigma `s`:

```text
sqrt(trace(P)) = sqrt(3) * s
```

Keep the full covariance authoritative and name scalar derivatives by formula.

Distinguish:

```text
per-axis standard deviation
sqrt(trace(P))
principal-axis sigmas
covariance ellipsoid
Mahalanobis distance
probability-content confidence region
presentation accuracy radius
```

A Gaussian probability ellipsoid must bind dimension, confidence level, chi-square threshold/profile, and the estimator model under which Gaussian semantics are claimed.

Tracked by #449.

## D. Measurement covariance R PSD and innovation covariance S solvability stay separate

The correction from v0.2 remains important:

```text
R may be PSD
S = HPH^T + R must support the declared update solve
```

The initial #427 implementation conservatively Cholesky-factors `R` itself. A follow-up may admit valid semidefinite `R` if `S` is appropriately positive definite/well-conditioned.

Do not modify #427 until its exact-head qualification executes; record this as a later theorem/profile revision.

## E. Physical ingress needs measurement-domain profiles, not one global range cap

The pure measurement layer already includes deep-space ranging support, while the shared durable v1 validator rejects every range over 1,000 km.

That is a terrestrial deployment policy embedded as universal physics.

Split:

```text
numeric / serialization safety
measurement-profile physical validity
adapter / calibration profile validity
application admission policy
```

A UWB profile can retain tight physical limits while a cislunar radio-ranging profile represents legitimate Earth-Moon scale observations.

Do not fix this by merely raising one global constant.

The durable measurement profile must also avoid lagging behind the pure-library modality vocabulary.

Tracked by #451.

## F. Mathematical estimates must not be directly publishable authority

The current pure fusion layer lets a math-only `GaussianEstimate3D` implement a publication-shaped trait using synthetic metadata (`source_count = 1`, `timestamp = 0`).

The current v1 coordinator accepts a caller-supplied `PositionEstimateEntry`, overwrites only `computed_by`, and commits the remaining caller-supplied position/covariance/algorithm/time fields.

For v2, enforce construction states:

```text
SpatialClaim
SpatialObservation
MathematicalEstimate
SpatialDerivation
QualifiedSpatialEstimate
PublicationEnvelope
```

A stronger type must require stronger evidence-bound construction; it cannot be obtained by filling public fields or implementing a convenience trait.

A qualified estimate should bind exact input actions/commitments, ordered-input commitment, frame/transform path, qualified time, estimator/solver/process profile, numerical policy, full uncertainty, diagnostics, derivation, and qualification refs.

Tracked by #453 / #419.

## G. Q5 should begin with full-covariance two-source CI

The current `PeerFusion3D::fuse()` is a trust-weighted arithmetic average, not CI.

The existing `covariance_intersection_3d()` is diagonal-only and silently clamps the free weight.

A narrow first qualified profile should be full-covariance two-source CI:

```text
P_ci^-1 = omega P_a^-1 + (1-omega) P_b^-1
x_ci = P_ci [omega P_a^-1 x_a + (1-omega) P_b^-1 x_b]
```

with:

- one canonical full covariance per source;
- PSD/conditioning validation;
- explicit objective such as trace or log-determinant;
- deterministic bounded weight optimization;
- objective/optimizer/tolerances in replay identity;
- no reputation term in the statistical equations;
- explicit conflict/disagreement state;
- no Byzantine-resistance claim from CI alone.

Do not define N-source fusion as an unordered fold of pairwise CI. If sequential CI is used initially, bind ordering and state the approximation theorem.

Refined on #417.

## H. Frame transformation needs exact path identity

A source/destination frame pair is not enough when more than one transform path can exist.

Represent exact ordered transform edges and dependencies:

```text
FrameNode
TransformEdge
TransformPath
TransformPathQualification
```

Multiple valid paths may be consistent, policy-selected, or conflicting. Material disagreement must remain visible.

Separate transform capabilities:

```text
PointTransform3D
PoseTransformSE3
StateTransform6D
CovarianceTransform
```

A time-varying 6D state transform can alter velocity through frame-rate terms; a point transform must not silently stand in for it.

Propagate covariance with the declared transform Jacobian/profile and transform-model uncertainty.

Tracked by #450.

## I. Space navigation is currently a prototype profile, not a qualified cislunar estimator

The existing space-navigation module is useful but currently combines:

- generic `ECI` naming;
- bare Julian date;
- hard-coded Earth gravitational parameter/radius;
- two-body dynamics;
- first-order integration;
- unchecked propagation interval;
- arbitrary scalar uncertainty growth;
- no full state covariance.

Preserve it honestly as a legacy/demo profile.

A qualified orbital state needs:

```text
6D state vector
exact reference frame
qualified epoch/time scale
central-body/system identity
dynamics/force profile
integrator/propagator profile
6x6 covariance
measurement/ephemeris provenance
replay identity
```

A propagated orbital state is a prediction/derivation, not an observation.

Treat NASA LunaNet PNT, SPICE/NAIF frames/time/ephemerides, and mission flight-dynamics outputs as external interoperability/profile inputs rather than reimplementing them wholesale.

Tracked by #455.

## J. A body identity is not a body-shape/cartographic model

The current body abstraction is useful but concrete Earth/Moon/Mars implementations bundle identity with one fixed geometric approximation.

Split:

```text
CelestialBodyId
BodyShapeModel
BodyOrientationModel
CartographicReferenceFrame
Vertical / height reference
Surface / terrain realization
```

A spherical Moon profile remains useful for low-precision work, but it is an approximation profile, not synonymous with `Moon`.

Lunar reference-frame differences can be operationally large, so a lunar numeric latitude/longitude is incomplete without the exact cartographic/frame realization.

Support external IAU WGCCRE / NAIF-PCK-FK / mission model identities where appropriate.

Tracked by #456.

## K. Navigation health is a consumer profile, not universal fix-age logic

The current runtime maps last-fix age to:

```text
<=10s  Good
10..30s Degraded
>30s   Lost
```

and defaults failover to dead reckoning.

Those are prototype values, not universal navigation truth.

Separate:

```text
observation freshness
prediction horizon
uncertainty growth
geometry/observability
measurement availability
estimator consistency
time qualification
network reachability
navigation service availability
```

Health profiles can then define use-specific requirements and explicit failover strategies.

Network partition is not automatically navigation failure; network reconnection is not automatically navigation recovery.

Tracked by #458.

## L. Make Q0-Q9 executable as a theorem DAG

The existing theorem document already states that passing one theorem does not imply the next.

Turn that rule into a machine-readable qualification manifest rather than a scalar `quality` or global `qualified` bit.

Illustrative facet result:

```text
QualificationFacet {
    theorem_id,
    subject_commitment,
    status,
    verifier_or_profile,
    evidence_refs,
    dependency_commitments,
    evaluated_at,
    validity_context,
}
```

Statuses should distinguish at least:

```text
Established
NotEstablished
Failed
Indeterminate
NotApplicable
Expired
Superseded
```

Theorem prerequisites form a DAG. Consumer requirement profiles select the exact facets they need.

Examples:

```text
SolAtlasVisualizationV1
TransportPlanningV1
TransportCustodySpatialPredicateV1
RoboticsNavigationInputV1
ScientificEvidenceExportV1
```

These profiles consume Position evidence; they do not grant domain capabilities.

Tracked by #457.

## M. Refined dependency direction

The deeper audit suggests this target sequence:

```text
#421 bounded physical ingress / freshness compatibility
  -> #422 trust separated from physical precision
  -> #423 full EKF propagation + Joseph baseline
  -> #426 Q4-E1 covariance PSD
  -> #427 Q4-F full vector R baseline
  -> #433 Q4-E2 observability / conditioning
  -> #447 Q4-E3 solver / convergence
  -> #448 Q4-G stochastic process model
  -> #434 NIS/NEES + estimator profile + replay
  -> #449 uncertainty-summary / probability-region semantics
  -> #417 Q5 full-covariance conservative fusion

Q2 / evidence foundations in parallel after the numerical contract is stable:

#416 frame/reference identity
  -> #435 qualified time / clock correlation
  -> #450 exact transform-path / state / covariance transforms
  -> #456 celestial-body/cartographic model identity
  -> #451 measurement-domain profiles
  -> #419 epistemic evidence graph
  -> #453 evidence-bound qualified-estimate construction
  -> #457 machine-readable theorem DAG / consumer requirements
  -> #458 operational navigation-health profiles

Then:

shared OperationalDomain / evidence foundations
  -> #436 physical/contact/reconciliation graphs
  -> #437 loss-aware logistics interoperability
  -> minimal mycelix-transport-core
```

The exact merge order may adjust around dependency conflicts and hosted evidence, but these semantic directions should remain.

## N. Earth -> Moon acceptance theorem v0.3

The integration target is now more precise.

A successful Earth -> Moon cargo scenario should be able to prove, without semantic collapse:

```text
terrestrial observation / qualified state
-> terrestrial transport leg + physical custody evidence
-> launch-site transition
-> launch / Earth-orbit qualified or explicitly predicted state
-> cislunar propagation + qualified measurements
-> exact Earth/cislunar/lunar frame transitions
-> LunaNet / other external PNT evidence where available
-> lunar body-fixed/cartographic/local-site frame
-> lunar surface vehicle/logistics state
-> physical delivery/custody event
-> delayed evidence reconciliation across partitioned domains
```

And separately preserve:

- observation vs prediction;
- state vs covariance summary vs probability region;
- physical custody vs DTN/message forwarding;
- physical route vs communication contact vs reconciliation route;
- celestial-body identity vs cartographic realization;
- reference frame vs transform path;
- observation time vs clock correlation vs publication time;
- mathematical estimate vs evidence-bound derivation vs qualified estimate;
- theorem facets vs consumer policy vs domain authorization.

## O. Immediate implementation restraint

Do **not** mutate #426 or #427 merely because this research found later improvements while their exact-head jobs remain queued.

When hosted execution becomes available:

1. qualify/fix #426 exact head;
2. qualify/fix #427 exact head;
3. only then implement #433/#447 as the next estimator tranche;
4. keep each PR small enough that its strongest theorem is independently reviewable and executable.

That restraint is part of the evidence architecture, not delay for its own sake.
