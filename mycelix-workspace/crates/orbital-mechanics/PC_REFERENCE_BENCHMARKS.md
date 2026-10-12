# Experimental 2D collision-probability reference benchmarks

**Status:** research/engineering guardrails only. This file does not qualify orbital risk or authorize maneuver decisions.

## Why this benchmark exists

The current `GaussianSmallDiskApprox2D` implementation evaluates the local Gaussian density at the projected miss vector and multiplies by the hard-body disk area. It is not the full Simpson-integrated Alfano 2005 algorithm, and it is not a Foster implementation.

Orekit's published `Alfano2005Test` provides scalar encounter-plane inputs and expected outputs for named Chan/CSM test cases. The local-density approximation is tested against 13 of those independent scalar reference cases so its observed difference is reproducible. The test is deliberately scoped to those inputs; it must not be represented as proof of accuracy over all encounter geometries.

Reference source:
- Orekit 13.1.5, `Alfano2005Test`: https://www.orekit.org/site-orekit-13.1.5/xref-test/org/orekit/ssa/collision/shorttermencounter/probability/twod/Alfano2005Test.html
- Orekit documents that this implementation uses Simpson integration for the Alfano 2005 method: https://www.orekit.org/site-orekit-13.1.3/apidocs/org/orekit/ssa/collision/shorttermencounter/probability/twod/Alfano2005.html
- NASA CARA provides a separate Foster/Elrod SDK with code and test cases: https://www.nasa.gov/cara/publicly-available-cara-software/

## Selected scalar reference cases

All scalar inputs are in meters. The expected Pc values below are those asserted by Orekit's test suite (some values are intentionally rounded there). Approximation values are calculated by the Mycelix local-density formula using the same scalar inputs.

| Reference case | Expected Pc | Mycelix approximation | Relative difference |
|---|---:|---:|---:|
| Chan 01: x=0, y=10, σx=25, σy=50, R=5 | 9.742e-3 | 9.80198673307e-3 | 0.616% |
| Chan 02: x=10, y=0, σx=25, σy=50, R=5 | 9.181e-3 | 9.23116346387e-3 | 0.546% |
| Chan 03: x=0, y=10, σx=25, σy=75, R=5 | 6.571e-3 | 6.60767000326e-3 | 0.558% |
| Chan 04: x=10, y=0, σx=25, σy=75, R=5 | 6.125e-3 | 6.15410897591e-3 | 0.475% |
| Chan 05: x=0, y=1000, σx=1000, σy=3000, R=10 | 1.577e-5 | 1.57659911484e-5 | 0.0254% |
| Chan 06: x=1000, y=0, σx=1000, σy=3000, R=10 | 1.011e-5 | 1.01088443285e-5 | 0.0114% |
| Chan 07: x=0, y=10000, σx=1000, σy=3000, R=10 | 6.443e-8 | 6.44320023245e-8 | 0.00311% |
| Chan 08: x=10000, y=0, σx=1000, σy=3000, R=10 | 3.219e-27 | 3.21458307994e-27 | 0.137% |
| Chan 09: x=0, y=10000, σx=1000, σy=10000, R=10 | 3.033e-6 | 3.03265329856e-6 | 0.0114% |
| Chan 10: x=10000, y=0, σx=1000, σy=10000, R=10 | 9.656e-28 | 9.64374923982e-28 | 0.127% |
| CSM 01: x=84.875546, y=60.583685, σx=57.918666, σy=152.8814468, R=10.3 | 1.9002e-3 | 1.89257357031e-3 | 0.401% |
| Chan 11: x=0, y=5000, σx=1000, σy=3000, R=50 | 1.039e-4 | 1.03896753657e-4 | 0.0031% |
| Chan 12: x=5000, y=0, σx=1000, σy=3000, R=50 | 1.564e-9 | 1.55277215503e-9 | 0.718% |

The unit test `selected_alfano_reference_cases_quantify_small_disk_approximation` re-computes the approximation and asserts that each selected case stays within 1% relative difference of its frozen reference value. The 13 cases cover Pc values from roughly `1e-3` to `1e-28`, but remain a small curated scalar set. This is a narrowly scoped regression threshold, not a statistical confidence bound.

## Explicit out-of-envelope case

Orekit's Alfano test case 3 includes x=-3.8872073 m, y=0.1591646 m, σx=1.4101830 m, σy=114.2585190 m, and R=15 m, with a reference Pc of 0.10038. The Mycelix local-density formula returns approximately 0.0156317535 for those values—about **84.4% relative error**. The radius is roughly 10.64 times the narrow standard deviation, so the local-density/small-disk assumption is inappropriate.

The checked path therefore uses an **empirical eligibility gate** before emitting Pc:
- normalized hard-body radius (R / sigma_{min} le 0.25);
- first-order log-density variation (R\|\Sigma^{-1}\mu\| le 0.5).

These cutoffs are early engineering filters anchored to the selected cases, not proven global approximation-error bounds. A failed gate yields `ApproximationOutsideReferenceEnvelope` and the caller must represent the event as **inconclusive**. It must not translate rejection into a negligible/low-risk band. The gate will need recalibration as a broader reference set is added.

## Required next qualification steps

1. Add the full NASA CARA Foster/Elrod test vectors and permitted data needed to reconstruct their inputs, with source, version, units, frame, and file hashes.
2. Build differential tests against an independent implementation for covariance anisotropy, cross-correlation, large hard-body radii, high normalized miss distances, and method-applicability limits.
3. Replace the local-density formula with an explicitly named validated method if broader operating coverage is required.
4. Replace `pc / 10` and `pc * 10` with validated uncertainty treatment, or keep those fields explicitly labelled as heuristic spread values and exclude them from claims of conservative bounds.
5. Keep PC, data quality, covariance realism, method applicability, and operational decision authority as separate fields in the mission evidence record.

## Safety boundary

NASA CARA notes that TLEs are not sufficient for collision-avoidance mitigation decisions and that Pc alone is not a sufficient risk-assessment parameter: https://www.nasa.gov/cara/research-and-development/. Passing these scalar regression tests does not establish a calibrated orbit covariance, qualified collision probability, operationally valid maneuver recommendation, or flight readiness.
