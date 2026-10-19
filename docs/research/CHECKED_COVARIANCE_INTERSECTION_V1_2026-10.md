# Checked Diagonal Covariance Intersection V1

**Status:** implementation hardening proposal; exact-head qualification pending  
**Issue:** [#417 — separate trust policy from mathematically conservative fusion](https://github.com/Luminous-Dynamics/mycelix/issues/417)  
**Base:** `Luminous-Dynamics/mycelix@dbc883566ea937ee8a273c1d33d153cc6262dd9b`  
**Implementations:** `mycelix-position/lib/positioning/src/fusion.rs` and the byte-identical workspace mirror.

## Why this is a new implementation slice

The independent diagonal-CI reference [PR #5227](https://github.com/Luminous-Dynamics/mycelix/pull/5227) passed at exact head `21a1f673955ef05ed58ef8fae0b79a65c4e1de48`. That qualified the equation, frozen corpus, 10,000-pair property sweep and mirrored crate state. It did **not** change the runtime equation.

The production function previously clamped invalid `omega` values into `[0,1]`, replaced covariance at or below (10^{-15}) with a fabricated precision of (10^{15}), and did not reject all non-finite inputs. Invalid inputs could therefore produce plausible-looking output rather than an explicit failure.

This proposal changes the pairwise public function to return `Result<GaussianEstimate3D, FusionError>`, updates the existing inconsistent equal-covariance test, and applies the same implementation to both mirrored positioning crates. The signature change is intentional: a non-fallible return type cannot represent rejected authority-bearing numeric input without choosing a fabricated output.

Repository search found no in-repository callers beyond the function's export and its own test. Downstream Rust consumers may still need to handle the new `Result`; review that compatibility impact before release.

## Contract

For each diagonal axis (i), input variance must be finite and strictly positive, means must be finite, and (omega) must be finite and in ([0,1]):

[
J_{a,i}=P_{a,i}^{-1}, quad
J_{b,i}=P_{b,i}^{-1}
]

[
J_{f,i}=omega J_{a,i}+(1-omega)J_{b,i}, quad
P_{f,i}=J_{f,i}^{-1}
]

The implementation normalizes the weighted information contributions before multiplying means:

[
x_{f,i}
=rac{omega J_{a,i}}{J_{f,i}}x_{a,i}
+rac{(1-omega)J_{b,i}}{J_{f,i}}x_{b,i}.
]

This is algebraically equivalent to the information-form CI equation and helps avoid unnecessary intermediate overflow. Any non-finite reciprocal/precision or output is rejected by a typed error. At (omega=0) or (1), the selected estimate is returned with the legacy covariance alias normalized to match the canonical `covariance_diag` field.

No arbitrary precision constants or silent weight clamping remain in this function.

## Trust is not precision

The CI equation receives a declared statistical weight and covariance. It does not use reputation or confidence as inverse covariance. Admission policy is upstream; statistical fusion is a different operation.

The production type's duplicate fields (`covariance_diag` and legacy `covariance`) remain a compatibility debt. This checked function treats `covariance_diag` as canonical and synchronizes the output alias. A separate migration should remove dual authority from the public serialized type.

## What is not fixed here

The current `PeerFusion3D::fuse()` remains a trust-weighted arithmetic aggregate. It is **not** converted into CI by this change, and its documentation now calls that out explicitly. The PR does not redesign multi-source CI optimization, source correlation, stale observation policy, innovation gating, measurement provenance or the publication boundary. It also does not establish full (3\times3) covariance CI, estimator accuracy or Byzantine resistance.

Those concerns remain tracked under #417 and require separate proof/baseline work. A pairwise CI primitive must not be presented as a qualification of the full peer-fusion path.

## Tests and acceptance gates

The paired positioning crate test suites add checks for:

- equal covariance remaining unchanged despite separated means;
- unequal covariance matching the independent reference values;
- (omega=0) and (omega=1) endpoints and canonicalized output alias;
- NaN/infinite/out-of-range weights, non-finite means, zero/negative covariance and reciprocal overflow rejecting explicitly;
- 10,000 deterministic generated pairs satisfying axis-wise covariance bounds and swap/complement symmetry;
- exact source, module-export, manifest and lockfile parity across both copies.

The pinned workflow also runs formatting, locked package metadata, and both positioning package suites with a clean postflight.

**Initial gates:**

| Measure | Target |
|---|---:|
| Invalid weights and numeric inputs silently accepted | 0 |
| Arbitrary singular-covariance precision substitutions | 0 |
| Equal-covariance cases gaining fake precision | 0 |
| Property-sweep bound/symmetry violations | 0 in 10,000 generated pairs |
| Source/manifest/lockfile mirror drift | 0 |
| Production peer-fusion equivalence to CI | Not claimed by this PR |
| Full covariance matrix, correlated-source and Byzantine guarantees | Not established |

No performance improvement is claimed. Once the safe API is qualified, benchmark it against the prior pairwise implementation on matched accepted inputs and invalid-input corpora, and keep the outputs and failures scoped to each algorithm profile.
