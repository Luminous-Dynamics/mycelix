# Three-list capture–recapture benchmark protocol v0.1

**Status:** proposed benchmark protocol and synthetic known-answer cases; not a field-valid population estimate.

## 1. Statistical question

For three source lists A, B and C, seven capture histories are observed:

\[
100,\ 010,\ 001,\ 110,\ 101,\ 011,\ 111
\]

and 000 is unobserved. The three-way cross-product interaction is:

\[
\theta_{ABC} =
\frac{n_{111} n_{100} n_{010} n_{001}}
     {n_{110} n_{101} n_{011} n_{000}}.
\]

Let:

\[
R = \frac{n_{111} n_{100} n_{010} n_{001}}
         {n_{110} n_{101} n_{011}}.
\]

Then \(n_{000}=R/\theta_{ABC}\). A no-three-way-interaction log-linear baseline sets \(\theta_{ABC}=1\), but this is a model restriction, not an observed fact. Referral, common selection, time-varying capture and latent heterogeneity can create a non-zero three-way interaction.

The FSRE E0 reference instead accepts a declared interval \(\theta_{ABC}\in[\theta_{\min},\theta_{\max}]\) and returns sharp hidden-cell bounds within that interval, conditional on all seven observed cells and the declared model scope.

## 2. Comparators

Evaluate on identical frozen generated tables:

1. Two-list reference: #6246 exact theta-sensitivity bounds and conditional Lincoln–Petersen/Chapman baselines where permitted.
2. Three-list no-three-way log-linear baseline: set \(\theta_{ABC}=1\); do not treat it as ground truth.
3. FSRE three-list theta sensitivity: exact rational interval from the declared theta range.
4. Standard log-linear model selection: AIC and BIC over plausible hierarchical models with pairwise terms, with declared candidate model set.
5. Decomposable graphical log-linear models: preserve pairwise dependency structures and model-selection uncertainty.
6. Bayesian model averaging / model uncertainty: where a reproducible implementation and prior sensitivity are available.
7. Sample-coverage estimators: specifically evaluated under sparse and zero observed patterns.
8. Exact generated-truth oracle: the simulation generator knows the actual all-zero cell and all seven observed cells.

A 2023 simulation study reports that AIC-based log-linear selection performs well in several dependence scenarios, but its performance degrades for sparse/zero cells; decomposable-graph and sample-coverage methods can perform better in those settings:
https://pmc.ncbi.nlm.nih.gov/articles/PMC9826711/

Triple-system estimation commonly identifies the all-zero cell by imposing no three-way interaction. That is an assumption and should remain visible in every result:
https://academic.oup.com/jrsssa/article/182/4/1487/7068309
https://pmc.ncbi.nlm.nih.gov/articles/PMC4036210/

## 3. Synthetic data generator

Generate known-truth populations with explicit capture-history probabilities. Vary:

- each list's marginal capture probability;
- all three pairwise interactions;
- the three-way interaction across positive, null and negative regimes;
- latent strata and capture-probability heterogeneity;
- population size and rare subpopulation size;
- list-specific nonresponse and false-positive entries;
- referral/conditional capture (A causes B, B causes C);
- temporal population closure violations and migration;
- linkage false-positive and false-negative errors;
- common upstream source ancestry, list overlap and duplicated source lineage;
- strata missingness and incomplete capture-history reporting.

The generator must retain the true population denominator, true 000 count, all latent strata, all capture probabilities, the true interaction parameter and exact observed table before any linkage corruption.

Use independent train/tune/test seeds. Split tests by parameter regime, not only random rows, so the model-selection methods are challenged on unseen dependence structures.

## 4. Adversarial benchmark suites

### A. Interaction misspecification

- \(\theta_{ABC}=1\) (baseline correctly specified).
- \(\theta_{ABC}<1\) and \(\theta_{ABC}>1\) (both directions of misspecification).
- Extreme interactions producing wide intervals.
- A declared theta interval excluding the true theta.
- An interval that contains the truth but is deliberately too wide.

Report bias, absolute error, interval coverage and interval width separately.

### B. Sparse and zero-pattern cases

Construct a sweep where expected counts in one or more of the seven observed cells approach zero. Include every zero pattern individually and combinations.

The E0 reference currently abstains whenever any observed pattern is zero. This is intentionally conservative. Compare its abstention against decomposable-model and sample-coverage estimators, measuring both false precision and the usefulness of abstention.

Do not insert pseudocounts silently. Any smoothing or Bayesian prior must be explicit and included in the model receipt.

### C. Heterogeneity, referrals and common-source dependence

Create:
- shared upstream root with distinct terminal register names;
- unknown ancestry for one list;
- common selection into all three lists;
- pairwise referral mechanisms;
- three-way selection mechanism;
- latent high- and low-capture groups;
- changes to capture probabilities across time.

The source ancestry relation is not proof of statistical independence. Shared or unknown ancestry suppresses the no-three-way baseline in FSRE while retaining sensitivity bounds.

### D. Frame and denominator errors

Separate:
- missed population units;
- duplicate identities;
- records assigned to the wrong geography;
- content error (incorrect attributes);
- late arrivals and reference-time mismatch;
- incomplete population frame;
- incomplete seven-pattern universe.

A correct hidden-cell calculation cannot repair a false frame or mislinked population unit. Keep these error sources separately attributable.

## 5. Metrics

### Statistical quality

- hidden-cell bias and relative bias;
- absolute error and RMSE;
- nominal 90% and 95% interval coverage, where the method has sampling intervals;
- sensitivity-interval coverage of true hidden counts across known theta regimes;
- interval width and width-to-truth ratio;
- false point-identification rate;
- false finite-upper-bound rate;
- model-selection accuracy and uncertainty;
- sensitivity to candidate model set and priors;
- stratified versus pooled estimator discrepancy.

Do not compare a deterministic theta sensitivity interval with a sampling confidence interval as if they were the same object. Report the two separately.

### Correct abstention

- zero-pattern abstention correctness;
- incomplete-frame abstention correctness;
- incomplete-pattern-universe abstention correctness;
- shared/unknown-ancestry baseline suppression;
- no-three-way baseline suppression when assumption is absent or theta excludes one;
- false abstention rate on cases where assumptions are actually met.

### Compute and reproducibility

- p50/p95 runtime and peak memory;
- number of candidate log-linear models and convergence failures;
- arithmetic bit growth and exact verification time;
- input, output, generator and commit hashes;
- seed and full parameter configuration;
- exact deterministic rendering under capture-row permutation.

## 6. Targets (research targets, not achieved)

- 100% agreement with independently enumerated known-answer calculations for the E0 formula.
- 0 no-three-way point-baseline emission when the interaction assumption is undeclared, the theta interval excludes one, or any source ancestry is shared/unknown.
- 0 false finite upper bounds when the theta upper limit is unbounded.
- 0 numeric estimate emitted for a zero observed pattern under the E0 contract.
- 0 target-population total emitted from an incomplete population frame.
- At least 90% and 95% empirical coverage for methods claiming nominal 90% and 95% intervals, evaluated in separate regimes and with Monte Carlo uncertainty reported.
- Report the width/coverage tradeoff of theta sensitivity intervals; do not optimize for narrow intervals alone.
- No method wins on RMSE alone if it has a higher false-precision rate or worse interval coverage.

## 7. FSRE implementation map

- #6246: two-list dependence-aware capture–recapture E0.
- #6260 / #6262: three-list theta sensitivity E0.
- #6255: exact LP primal/dual/Farkas certificates and constrained distribution LP.
- #6200 / #6256: sparse LP scaling and HiGHS adapter.
- #6247 / #6249: decision episode and assurance graph.
- #6240 / #6207: integrated population-coverage and Decision Integrity Benchmark.

The paradigm shift is to treat the unseen all-zero cell as partially identified under explicit higher-order dependence assumptions, and to treat the model's permission to emit a point estimate as a separate qualification decision. A more sophisticated model is not automatically more credible if it hides the assumptions that identify the missing population.
