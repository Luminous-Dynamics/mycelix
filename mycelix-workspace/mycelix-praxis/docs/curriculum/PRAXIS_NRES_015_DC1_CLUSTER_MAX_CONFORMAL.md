# PRAXIS-NRES-015-DC1: Dependence-cluster maximum conformal calibration

**Status:** synthetic public-development baseline; not confirmatory or deployable.  
**Parent sequence:** NRES-015 protocol -> BT1 bounded bulk-tail reference -> DC1 dependence-cluster calibration.  
**Evaluator:** benchmarks/nres-capability-loop/src/cluster_calibration.rs.  
**CLI:** cargo run --manifest-path benchmarks/nres-capability-loop/Cargo.toml -- --calibrate-dependence-clusters.

## Research motivation

NRES-014 uses one source-family block as one split-conformal calibration unit. That is defensible only when the block scores are exchangeable at that unit. In real evidence systems, source families may share an incident, region, institution, supplier, season, reporting channel or common shock. Treating correlated families as independent calibration examples can overstate the effective sample count and understate the radius.

The 2026 literature makes calibration-law and dependence assumptions an explicit research concern. Ramos et al. study calibration-conditional coverage laws and extensions for clustered and stationary-mixing settings (PMLR 329, 2026). DC1 is not their method or a reproduction. It adds a transparent, conservative comparison point to the NRES stack: predeclare a cluster, take the maximum block nonconformity score inside it, and calibrate across cluster maxima.

## Method

Let a dependence cluster j contain a fixed, predeclared number r of source-family blocks. Let s_jk be the frozen NRES-014 W1 nonconformity score for block k in cluster j. Define the cluster score:

S_j = max over k in {1,...,r} of s_jk.

Given m calibration clusters, sort S_(1) <= ... <= S_(m). For target miscoverage alpha, use the split-conformal rank:

k = ceil((m + 1) * (1 - alpha)).

The calibrated threshold is S_(k) when k <= m. When k > m, DC1 emits a null radius and an explicit unbounded flag. It does not substitute the observed maximum and pretend it is a finite valid threshold.

If the future cluster maximum and calibration cluster maxima are exchangeable under the same predeclared cluster construction and fixed size, ordinary split-conformal rank reasoning gives marginal coverage at least 1-alpha for the new cluster maximum. Since every member block score is no larger than its cluster maximum, this implies simultaneous marginal coverage of all r blocks in that future cluster.

This trades sharpness for a clearer dependence unit. Taking a maximum may be conservative, particularly as cluster size grows. It does not create independent samples from dependent rows; it treats each cluster as one calibration unit.

## Required assumptions

- The dependence-cluster mapping is defined before calibration scores are inspected.
- Calibration and future clusters are exchangeable at the cluster level.
- The same cluster construction and fixed number of member blocks apply to calibration and deployment.
- Source-family IDs, block IDs and cluster IDs are unique across calibration and development splits.
- NRES-014 block score definition, metric, forecast and score computation are frozen before DC1 calibration.
- Cluster maxima are the only conformal observations. Member block counts do not increase the conformal sample size.

The method does not guarantee coverage when cluster maxima shift or are non-exchangeable, under arbitrary temporal dependence between clusters, for unseen cluster sizes, or after changing the cluster definition. It does not provide calibration-conditional coverage or a population-law correction within a block.

## Synthetic fixture arithmetic

The fixture has 10 calibration clusters with three member blocks each, and five independent public-development clusters of the same design. Calibration cluster maxima sort to:

[0.30, 0.40, 0.50, 0.55, 0.60, 0.65, 0.70, 0.80, 0.90, 1.00].

At alpha = 0.20, the rank is ceil(11 * 0.8) = 9 and the radius is 0.90. In the tiny synthetic development set, 3/5 cluster maxima are within the radius, and 13/15 individual block scores are within it. These are descriptive fixture diagnostics only, not a coverage estimate, calibration validation or confirmatory result.

## Conformance and tests

The module authors 13 Rust tests for:
- cluster-max rather than member-block calibration;
- one-based conformal rank on clusters;
- simultaneous coverage logic;
- unbounded radius when cluster count is insufficient;
- explicit unbounded-radius development behavior and a vacuity label when all clusters mechanically fall inside an infinite threshold;
- duplicate cluster IDs and source-family IDs;
- unequal cluster sizes;
- negative, null or missing scores;
- invalid miscoverage levels and active-curriculum fixtures.

The conformance corpus lists 13 invalid-input controls, five boundary cases and six round-trip requirements. Test source is authored, not confirmed passing. Rust compilation/execution must be recorded from an actual toolchain/CI run.

## Comparison and next steps

DC1 should sit beside, not replace, NRES-014:
- NRES-014: source-family block scores, useful when those blocks themselves are exchangeable.
- DC1: cluster maximum scores, a robust option when within-cluster dependence is unrestricted but clusters are exchangeable.
- Dependence-aware methods: future additions should evaluate block/moving-block bootstrap, mixing-aware conformal calibration and calibration-conditional coverage diagnostics under explicitly declared dependence models.
- Federated settings: report both per-client and pooled results; score sketches alone are not a privacy guarantee.

Benchmark the methods on a controlled dependence ladder: independent blocks; exchangeable clusters with strong within-cluster dependence; varying cluster sizes; common-mode shocks across clusters; temporal drift; and institution/client shift. Keep the number of independent clusters—not number of member blocks—as the primary sample-size axis. Report radius width, marginal cluster coverage, simultaneous member-block coverage, vacuity, decision regret and resource costs separately.

No single composite score is appropriate. The 2026 literature should be compared on a common data/split protocol, not by comparing our synthetic fixture numbers with paper headline metrics.
