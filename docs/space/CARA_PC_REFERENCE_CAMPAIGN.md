# CARA Foster 2-D probability-of-collision reference campaign

**Status:** pinned reference fixture and independent Rust quadrature comparison test authored; exact-head Rust CI has not yet been observed passing.  
**Scope:** numerical research/verification for the Earth-orbit conjunction probability code. This is not flight qualification or authority to make mitigation decisions.

## Source of record

The machine-readable cases are in:

`mycelix-workspace/crates/orbital-mechanics/tests/fixtures/cara_pc2d_foster_reference_v1.json`

The fixture pins NASA's `nasa/CARA_Analysis_Tools` commit `ce0f00722c56808755ee0adbe47a4e3d56439484`, the `Pc2D_Foster.m` source blob, and the upstream test blob `dea6063ce5abdd7ad112f1ed55a15b4a24acc81b`. It records three source-asserted results for one encounter input: circular, square, and square-equivalent-area hard-body regions. The expected values are the assertions in that upstream test file, **not freshly executed results from this environment**.

NASA describes its publicly available CA software as containing source, test cases and expected results to verify implementations. The upstream implementation and test suite are authoritative for this pinned regression fixture only; they do not establish operational validation for Mycelix. See [NASA's public CARA software page](https://www.nasa.gov/cara/publicly-available-cara-software/) and [the pinned upstream unit test](https://github.com/nasa/CARA_Analysis_Tools/blob/ce0f00722c56808755ee0adbe47a4e3d56439484/DistributedMatlab/ProbabilityOfCollision/UnitTests/Pc2D_Foster_UnitTest.m).

## Independent numerical reproduction

Before adding the Rust test, a separate numerical integration of the pinned geometry/covariance and Foster Gaussian-area integral reproduced the upstream assertions within the upstream relative tolerance of 0.1%. A second composite-Simpson reproduction returned approximately:

| Region | Independent Simpson value |
|---|---:|
| Circle | `2.70598932657e-5` |
| Square | `3.44534649703e-5` |
| Square-equivalent area | `2.70601573490e-5` |

These are computed reproductions of the formula and transcribed fixture inputs, not outputs executed from the NASA MATLAB implementation. The circle result shows the expected small difference from the upstream adaptive quadrature and remains inside its stated tolerance. The Rust test uses the same method definition with fixed composite-Simpson quadrature, but its compile and exact-head execution remain unverified until GitHub Actions completes.

## How to use the fixture safely

1. Parse and validate the JSON before using any numbers. Record the fixture file SHA-256 in the eventual test receipt.
2. Preserve the source case's stated units and covariance layout. The input vectors are km and km/s; the 3×3 position covariances are in km². The upstream test calls the frame ECI but does not supply an epoch or a fully realized frame definition. Do not synthesize those missing fields or present this as a complete operational record.
3. Implement or adapt a Foster 2-D reference routine as a separately identified method. Do not relabel the existing `GaussianSmallDiskApprox2D` implementation or change its tolerances to make it pass this fixture.
4. Run each hard-body region separately. Require finite (P_c), (0 \le P_c \le 1), expected remediation status, and agreement with the pinned expected value at the upstream relative tolerance. Capture software revision, source blob, compiler/toolchain, command, complete output, and exit status.
5. Add independent cases for anisotropy, low/high miss distance, hard-body-radius variation, ill-conditioned projected covariance, and degenerate geometry. These do not receive invented expected values: obtain and independently validate outputs before promoting them to frozen regression cases.
6. Compare methods only where their assumptions and domains overlap. The Foster 2-D model and a 3-D probability method are not interchangeable; method disagreement is not automatically evidence that either output is wrong.

## Required result record

Each future comparison receipt must contain:

- fixture schema/version and SHA-256;
- upstream repository, commit, source path/blob, and test path/blob;
- selected method and hard-body-region shape;
- complete inputs, units, frame/epoch availability, and all conversions;
- reference output, Mycelix output, absolute/relative error and comparison tolerance;
- covariance validity and remediation indicators before and after any explicitly recorded projection;
- process/toolchain versions, command, exit status and immutable output digest;
- result state: `passed`, `failed`, `blocked`, or `not_run`.

A missing reference value or unavailable dependency is `blocked`/ `not_run`, never pass. A source-pinned result is numerical evidence only; it does not validate covariance realism, the source orbit determination, probability calibration on real events, or maneuver authority.

## External runtime dependencies

NASA's SDK is MATLAB source and uses Git LFS for some `.mat` data. Before putting it in CI, decide whether to run a licensed MATLAB environment, use an independent implementation such as Orekit for cases where its method assumptions match, or maintain a reviewed Rust implementation. The path chosen must declare method identity and dependencies. Do not silently add proprietary runtime dependencies to a generic Rust CI lane, and review NASA's applicable Open Source Software Agreements before adapting implementation code.

Orekit documents its Alfano 2005 method as assuming short-term linear relative motion, a spherical collision object, uncorrelated position covariance, Gaussian position uncertainty, and deterministic velocity; these are method-specific restrictions, not a generic replacement for any 3-D result. See [Orekit's method documentation](https://www.orekit.org/site-orekit-13.1.3/apidocs/org/orekit/ssa/collision/shorttermencounter/probability/twod/Alfano2005.html).

## Exit criteria

- [ ] Fixture parser and source identities checked automatically.
- [ ] A separately named reference implementation runs against all three frozen cases.
- [ ] Exact test outputs and a machine-readable receipt are persisted.
- [ ] Failure injection proves that missing reference data, wrong units, bad covariance, and a missing runtime cannot yield a passing state.
- [ ] Additional independent cases include provenance and validated expected outputs.
- [ ] Issue [Mycelix #4973](https://github.com/Luminous-Dynamics/mycelix/issues/4973) remains open until the campaign and method-use limits are independently reviewed.
