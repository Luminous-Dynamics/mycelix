# Covariance Intersection Reference V1 — Separate Statistical Fusion from Trust Policy

**Status:** research-only diagonal-CI reference and regression qualification; no claim that the complete peer-fusion path is corrected  
**Owner issue:** [#417 — separate trust policy from mathematically conservative fusion](https://github.com/Luminous-Dynamics/mycelix/issues/417)  
**Base:** `Luminous-Dynamics/mycelix@dbc883566ea937ee8a273c1d33d153cc6262dd9b`  
**Reference profile:** `DiagonalCovarianceIntersectionReferenceV1`  
**Primary mirror:** `mycelix-position/lib/positioning`  
**Workspace mirror:** `mycelix-workspace/mycelix-position/lib/positioning`

## Paradigm shift: trust is not precision

An estimator's reputation or admission weight cannot be treated as inverse measurement variance. Source credentials, freshness, provenance, modality, fault-domain correlation and policy authorization answer whether an observation may participate. Covariance answers a different question: what uncertainty follows from a declared statistical model.

Those concepts must not be collapsed:

- higher reputation does not automatically imply smaller covariance;
- more peer reports do not establish independent information;
- cryptographic signature validity does not prove measurement accuracy;
- a fused mathematical point does not by itself become a publishable or evidence-qualified observation.

This reference freezes the pairwise numerical equation before changing production APIs or peer-fusion semantics.

## Scope: diagonal covariance only

The current estimate type represents a 3D mean and only the diagonal entries of covariance. The reference is therefore a **componentwise diagonal CI** contract, not a full 3×3 matrix implementation.

For each axis (i\), given two strictly positive finite variances (P_{a,i},P_{b,i}\) and a finite (omega\in[0,1]\):

[
J_{a,i}=1/P_{a,i},\qquad J_{b,i}=1/P_{b,i}
]

[
J_{f,i}=\omega J_{a,i}+(1-\omega)J_{b,i}
]

[
P_{f,i}=1/J_{f,i}
]

[
x_{f,i}=P_{f,i}\left(\omega J_{a,i}x_{a,i}+(1-\omega)J_{b,i}x_{b,i}\right)
]

Under the stated diagonal model, the fused variance lies between the two input variances on each axis. Equal input variance remains exactly equal to that variance for any valid (omega\). In particular, fusing two equal unit variances at (omega=0.5\) does **not** halve the variance.

This is a mathematical property of the declared CI equation, not evidence that the assumptions of any live positioning measurement hold.

## Existing source finding and exact scope of this PR

At the inspected base, `covariance_intersection_3d()` uses the diagonal information-form equation but silently clamps (omega\) and substitutes a fabricated (10^{15}\) precision for covariance values at or below the threshold. It does not reject non-finite input before computing the output.

The separate `PeerFusion3D::fuse()` path computes a trust-weighted arithmetic mean and weighted covariance diagonal. That implementation is **not** pairwise CI and does not become CI merely because the type/function is documented as Covariance Intersection.

The pre-existing pairwise-CI unit test also asserted that equal unit input variances produce a fused variance below 1.0. This PR corrects that regression expectation to exactly ([1,1,1]\). That is a test correction only; it does not change production fusion equations, introduce a fallible API, or harden the live peer-fusion path.

## Frozen cases and reference oracle

The corpus contains **22 cases: 12 valid and 10 invalid**, plus six semantic mutation controls.

Valid cases include:

- equal covariance with separated means;
- asymmetric means and unequal diagonal variances;
- (omega=0\) and (omega=1\) endpoints;
- unequal (omega\) values;
- input swap plus complementary weight;
- small and large—but finite—numeric scales;
- axis-specific variance differences;
- a deterministic covariance-bounds control.

Invalid inputs include negative or above-one (omega\), NaN weight, non-finite means/covariance, zero or negative covariance, reciprocal overflow, and overflowing information-weighted mean arithmetic. Invalid values are rejected rather than clamped or converted to a plausible-looking estimate.

Two identical test/fixture copies are stored beside both mirrored positioning crates. The exact-head workflow mechanically compares:
- production `fusion.rs` source;
- positioning crate manifests;
- workspace manifests and lockfiles;
- the reference test source and frozen corpus.

It runs the complete `positioning` package tests in both workspaces with pinned Rust. A successful pass would qualify the mathematical reference, the corrected equal-covariance regression, and the exact mirror state; it would not qualify the production peer-fusion design.

## Initial acceptance gates

These are proposed research gates, not completed results.

| Gate | Target |
|---|---:|
| Frozen reference corpus | 12/12 valid cases match expected values; 10/10 invalid cases reject |
| Equal covariance | 100% of equal-covariance cases preserve the original variance |
| Covariance interval | Every valid axis remains between its two declared input variances |
| Swap/complement symmetry | (CI(A,B,\omega)=CI(B,A,1-\omega)\) within the declared numerical tolerance |
| Property sweep | 10,000 deterministic generated pairs; zero finite/range/symmetry violations |
| Numeric failures | Zero non-finite means, covariances, reciprocals, weighted information or outputs accepted |
| Mirror consistency | Exact source, manifest, lockfile and reference-corpus parity between the two positioning paths |
| Trust-policy separation | Zero trust/reputation/confidence multipliers appear in the CI reference equation |
| Production peer fusion | **Not qualified by this PR** |
| Full covariance matrix CI | **Not established by this diagonal reference** |
| Multi-source CI optimization | **Not established by this pairwise reference** |

## Next production step after reference qualification

A separate implementation PR should:

1. Add a fallible covariance-structure/numeric-validation API with typed errors.
2. Reject non-finite means/covariance/weights, negative covariance and unqualified singular/zero-variance cases; avoid arbitrary precision surrogates.
3. Make `PeerFusion3D` use explicitly named statistical fusion or honestly rename it; do not label a trust-weighted average CI.
4. Put source admission/reputation policy before fusion, not inside the covariance equations.
5. Choose deterministic multi-source semantics and an objective for CI weights rather than folding an unordered peer vector.
6. Separate pure estimates from evidence-qualified publishable estimates, and resolve the duplicated `covariance_diag`/`covariance` authority through a versioned migration.
7. Test correlated sources, stale observations, disagreement/abstention and provenance in their own contracts. Passing CI alone does not prove Byzantine resistance or source independence.

No runtime peer-fusion policy, trust weight, covariance authority field, authorization rule or Holochain storage behavior is changed by this reference PR.

## Source

- [Issue #417: trust policy and conservative fusion](https://github.com/Luminous-Dynamics/mycelix/issues/417)
- [Parent spatial-truth contract, issue #413](https://github.com/Luminous-Dynamics/mycelix/issues/413)
- [Position non-finite ingress hardening, PR #415](https://github.com/Luminous-Dynamics/mycelix/issues/415)
