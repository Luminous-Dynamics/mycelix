# Correlated-covariance collision-probability reference campaign

**Status:** authored reference fixture and end-to-end Rust differential test; exact-head compilation and execution remain unverified until the associated GitHub Actions job completes.

## What is new

\`tests/pc_correlated_end_to_end.rs\` sends synthetic, explicitly framed states and full 6×6 covariances through the public \`ConjunctionAnalyzer::try_assess\` path. It checks four non-diagonal encounter-plane covariance cases against a separately computed bivariate-normal integral over a circular hard-body region. This extends—not replaces—the existing 13 scalar Orekit comparison cases, which use diagonal 2-D covariance inputs and directly exercise the local-density kernel.

This closes a specific coverage gap: the test checks that state-to-encounter-plane geometry, correlated covariance projection, unit conversion, and method identity all survive the full checked production path. The comparison deliberately keeps the name \`GaussianSmallDiskApprox2D\`; it does not call the method Foster, Alfano, or qualified operational Pc.

## Independent reference

The fixture is \`tests/fixtures/pc_correlated_gaussian_disk_v1.json\`. Its reference is the bivariate Gaussian mass over the circular region of radius \(R\):

\[
P_c = \frac{1}{2\pi\sqrt{\det(\Sigma)}}\int_0^R\int_0^{2\pi}
\exp\!\left[-\frac12(q(r,\theta)-\mu)^T\Sigma^{-1}(q(r,\theta)-\mu)\right]
r\,d\theta\,dr,
\]

where \(q(r,\theta)=(r\cos\theta,r\sin\theta)\), \(\mu\) is the encounter-plane relative miss vector, and \(\Sigma\) is the full 2×2 position covariance in that plane.

The frozen values were calculated separately from the Rust production code using tensor-product Gauss-Legendre quadrature in polar coordinates. Radial node orders 60, 100, 160, and 220 were compared; the angular node count was \`2 * radial_order + 24\`. The verification helper is \`tools/verify_pc_correlated_gaussian_reference.py\` and requires NumPy. Re-run it from the crate directory:

\`python3 tools/verify_pc_correlated_gaussian_reference.py\`

The Rust integration test allows at most **0.2% relative difference** between the production small-disk approximation and the numerical integral. It separately pins the expected value of the local-density formula tightly, so a basis or covariance-projection regression is distinguishable from the approximation's expected integration error.

## Frozen comparisons

| Case | Independent disk integral | Local-density approximation | Relative difference |
|---|---:|---:|---:|
| Correlated moderate miss | 3.7352878814555580e-4 | 3.7360462307441984e-4 | 0.0203% |
| Correlated anisotropic | 8.0790392351425748e-4 | 8.0826353938121670e-4 | 0.0445% |
| Correlated low-probability tail | 6.9433147936091366e-5 | 6.9422492738447484e-5 | 0.0153% |
| Correlated near-center | 4.5822890727077335e-4 | 4.5834924851410560e-4 | 0.0263% |

These are new synthetic regression cases, not upstream NASA or Orekit expected outputs. The 0.2% tolerance applies only to these frozen inputs; it is not a general error bound or calibration claim.

## Test construction and limitations

For this test only, the 2-D encounter plane is realized with relative velocity along +Z. Mycelix's projection basis then maps a synthetic relative position \`[-my, mx, 0]\` to the encounter-plane miss \`[mx, my]\`. Given the target encounter-plane covariance \`[[a,b],[b,c]]\`, each object's source-frame XY covariance is set to half of \`[[c,-b],[-b,a]]\`; the two independent object covariances therefore project to the specified combined 2-D covariance. This intentionally tests off-diagonal terms instead of silently reducing them to diagonal variances.

The states, 6×6 covariances, and epoch are synthetic, not orbit-determination products. The current covariance-source enum has no explicit synthetic test value, so the test uses a structurally accepted marker only; that metadata must not be interpreted as authentic provenance or covariance realism. The constructed encounter is suitable for checking the numerical software path, not for asserting a complete physical orbit or real conjunction.

To run the focused Rust test with the standalone crate:

\`cargo test --manifest-path mycelix-workspace/crates/orbital-mechanics/Cargo.toml --test pc_correlated_end_to_end\`

(Execute from the repository root.) The checked assessment requires positive-definite covariance and rejects geometries outside its empirical small-disk eligibility envelope; a rejection must remain inconclusive, not be translated into a low-risk finding.

## Qualification boundary

The test can establish only that the tested implementation reproduces a separately integrated Gaussian disk probability for a small synthetic case set. It does not establish covariance realism, TLE accuracy, cross-object independence in live data, calibration on historical conjunctions, behavior outside the small-disk envelope, flight qualification, or maneuver authority. Continue tracking those items under [Mycelix issue #4973](https://github.com/Luminous-Dynamics/mycelix/issues/4973).


### Second, structurally different numerical replay

The campaign now has a second reference calculation in `tools/verify_pc_correlated_gaussian_conditional_reference.py`. Instead of integrating area directly in polar coordinates, it conditions the bivariate normal on its x-coordinate: integrate the marginal density of X over `[-R, R]`, multiplied by the conditional probability that Y lies between `-sqrt(R²-x²)` and `+sqrt(R²-x²)`. This uses the conditional-normal CDF and one-dimensional Gauss–Legendre quadrature at orders 500 and 700, so the numerical path is structurally different from the tensor-product polar-area integral.

For the frozen four-case fixture, the second replay requires the order-to-order relative difference to be at most `5e-9` and the order-700 result to agree with the frozen integral reference within `5e-9` relative error. These are fixture-specific reproduction tolerances, not a general method error bound. Both independent numerical scripts must pass in CI before the numerical-reference portion of this campaign is considered reproduced; Rust end-to-end verification remains a separate gate.
