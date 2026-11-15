# FSRE linkage-impact benchmark protocol v0.1

**Status:** E0 benchmark protocol and synthetic known-answer evaluator. All performance targets below are proposed, not achieved.

## 1. The benchmark question

A record-linkage system is not evaluated only by the number of matching pairs it gets right. It must also be evaluated by the population, asset, service boundary or decision that a wrong match can distort.

The benchmark therefore separates four layers:

1. Pair/link quality.
2. Cluster/entity quality.
3. Population-frame and denominator impact.
4. Downstream decision and service impact.

A strong pairwise F1 score is necessary for many use cases, but it is not a sufficient proxy for a defensible population estimate or infrastructure decision.

## 2. Industry comparators: use each system for its actual purpose

### Splink 5.0.0 — probabilistic entity linkage

The official Splink 5.0.0 release page dated 28 September 2026 reports capacity of up to one billion records in under ten minutes. The same release states that the statistical model is unchanged from version 4. This is a maintainer-reported capacity claim, not an independently reproduced benchmark result in this repository:
https://moj-analytical-services.github.io/splink/blog/2026/09/28/splink-500-released.html

Splink is a serious comparator for blocking, candidate-pair generation, probabilistic scoring, clustering and throughput. FSRE should not claim to outperform it on generic record linkage without a matched reproduction using the same records, gold labels, blocking/candidate-pair counts, features, threshold, backend, hardware, memory and compute budget.

FSRE's intended differentiation is downstream assurance:
- effective-time and recorded-time identity replay;
- separation of person, asset, service boundary, owner, operator and account namespaces;
- false cluster-bridge impact on population denominators;
- fragmented-entity duplicate-counting risk;
- source ancestry and common upstream dependencies;
- coverage and partial-identification bounds;
- decision-specific false-safe and abstention outcomes.

### Statistics South Africa Census 2022 Post-enumeration Survey

Statistics South Africa describes PES as an independent sample survey used to evaluate census coverage and content errors. The 2022 PES selected 840 sub-enumeration areas across all nine provinces:
https://www.statssa.gov.za/?p=15645

The official report distinguishes coverage errors (missed or multiply counted persons/households) from content errors (inconsistent characteristics). It reports an estimated final true population of 61.4 million within the private-dwelling PES scope, compared with the 42.3 million Census 2022 count in scope, and a final net coverage error rate of 31.1%. It separately reports content inconsistency measures for tested characteristics:
https://www.statssa.gov.za/publications/P03015/P030152022.pdf

Do not collapse this final estimate and scope into other preliminary tables or broader de facto population totals. Preserve reference night, scope, collection design, estimation stage and interval with every comparison.

FSRE should benchmark its frame and denominator assurance against the dimensions PES measures. It must not present administrative linkage as a replacement for independent field enumeration or probability sampling.

### Office for National Statistics Census 2021

ONS documents separate coverage error (including nonresponse and overcoverage), measurement error, modelling uncertainty and sampling error. Its 2021 Census Coverage Survey process used logistic regression to estimate undercoverage and overcoverage propensity and derive weights:
https://www.ons.gov.uk/peoplepopulationandcommunity/populationandmigration/populationestimates/methodologies/coverageestimationforcensus2021inenglandandwales
https://www.ons.gov.uk/peoplepopulationandcommunity/populationandmigration/populationestimates/methodologies/qualityandmethodologyinformationqmiforcensus2021

This is a comparator for modelled response propensity, small-area quality, overcoverage treatment and uncertainty decomposition—not a generic entity-resolution benchmark.

## 3. Paradigm shift: evaluate identity errors by consequence

### 3.1 Count bias is not enough

Consider a benchmark with three true entities A, B and C. A candidate crosswalk merges A and B into one cluster, while splitting C into two clusters. The candidate system still emits three clusters for three true entities. Its unique-count error is zero, but both a false bridge and a fragmentation are present.

A single count-bias metric would say this example is perfect. Pairwise metrics improve the diagnosis, but do not measure how much service population is exposed to the merge or how much denominator mass can be double-counted by a split.

### 3.2 Non-netted harm metrics

Report separately:

- false bridge clusters and pairs of gold entities merged;
- gold population weight at risk inside bridge clusters (sum of entity weights less the largest entity weight in each bridge);
- fragmented gold entities and potential duplicate-counting weight;
- late-arrival identity leakage across recorded-time cutoffs;
- namespace collisions;
- pair precision/recall;
- cluster precision/recall;
- unique-entity count bias;
- weighted population-denominator bias.

Do not subtract fragmentation harm from bridge harm or compress rights, authority and safety into a weighted linkage score. Errors with different mechanisms must remain visible.

### 3.3 Decision-conditioned harm

For each false bridge or split, replay the downstream calculation using:
1. gold identities and gold frame;
2. candidate identities and candidate frame;
3. conservative partial-identification bounds where the crosswalk is unresolved.

Measure changes in:
- denominator and subgroup coverage;
- service-floor classification;
- robust-safe action set;
- intervention ranking and worst-case regret;
- abstention correctness;
- cost of additional clerical review or field inspection.

The evaluator must not infer that a false bridge necessarily changes the decision. It must measure the effect on each frozen episode.

## 4. E0 known-answer fixture

The public Rust reference is tools/fsre-linkage-impact.

Its synthetic fixture uses three gold entities and three candidate clusters:
- one candidate cluster merges A and B;
- C is split across two candidate clusters;
- a late-recorded row is assigned by the candidate after the knowledge cutoff;
- one future-effective row is excluded from the target population.

Expected results:
- gold entity count 3 and predicted cluster count 3: count bias +0;
- pair precision 1/3 and pair recall 1/4;
- one false bridge cluster, one gold-entity pair merged, 80 population-weight units at risk;
- one fragmented gold entity, 50 units of potential duplicate-counting risk;
- gold denominator 230 and predicted denominator 200: bias −30;
- one late-arrival leakage event, excluded from the as-of predicted clusters.

These values are synthetic known answers. They are not field estimates or a measured Splink comparison.

## 5. Matched comparator protocol

### 5.1 Freeze the experimental conditions

For every linkage method, record:
- immutable dataset digest and gold-adjudication digest;
- target namespace and population unit;
- effective-time and recorded-time cutoffs;
- source eligibility, rights and source-use conditions;
- record counts by source and subgroup;
- missingness and field-quality profiles;
- candidate-pair count after blocking;
- blocking rules, comparison features, term-frequency adjustments and thresholds;
- training sample, chunking, seed and approximation settings;
- software version, commit, solver/backend, hardware, thread count and memory;
- exact output crosswalk digest.

Do not compare runtime unless both methods were run on equivalent inputs, candidate-pair work and quality targets.

### 5.2 Split the gold corpus against leakage

Use separate development, threshold-selection and held-out evaluation partitions. Hold out by:
- time;
- geography and service boundary;
- operator or source lineage;
- entity family / cluster;
- source-system version.

Do not allow near-duplicate records or related entities to cross partitions where that would leak identity evidence. Publish the split manifest and digest.

### 5.3 Scale tiers

Use 15 million and 100 million records as primary scaling points where available. Treat one billion records as a stretch target. Splink 5's published capacity claim is not a mandatory FSRE target until the workload and backend are reproduced. Always report the post-blocking candidate-pair count: raw record count alone does not specify the amount of linkage work.

### 5.4 Statistical reporting

For pair and cluster metrics, publish confusion counts, precision/recall, PR curves, calibration where probabilities are output, confidence intervals and subgroup breakdowns. Use bootstrap resampling at the correct independent sampling unit; do not bootstrap individual pairs as if all pairs were independent.

For PES/DSE comparators, preserve survey design, weights, stratification, reference time, nonresponse adjustments and variance estimator. Report sampling uncertainty separately from linkage/model uncertainty.

## 6. Benchmark targets

Proposed targets, not achieved:
- 0 silent cross-namespace identity merges.
- 0 late-arrival assignments used in a historical decision cutoff.
- 0 gold/predicted weight inconsistencies accepted.
- 100% exact agreement with the finite E0 linkage-impact oracle on known-answer fixtures.
- 100% deterministic output under row permutation.
- Report pair precision and recall with confidence intervals; do not set universal thresholds before a use-case-specific harm budget exists.
- Target 0 false bridge errors in the high-consequence gold subset; report corpus size and confidence bounds rather than claiming universal safety.
- Report bridge and fragmentation harm independently, even where total count bias is zero.
- Report denominator-bias distribution and worst subgroup error, not only mean absolute error.
- For decision episodes, target 0 false-safe decisions on the frozen adversarial corpus and measure abstention correctness separately.
- For scale, report throughput and peak memory under matched quality and hardware; do not use unmatched headline comparisons.

A finite test corpus with zero failures is not proof of universal safety. Always disclose sample size, scenario diversity, confidence intervals and unresolved cases.

## 7. How FSRE can outperform the field responsibly

The fastest route is not to replace Splink or recreate a national statistics office inside one crate. Use strong specialist baselines:
- Splink for probabilistic linkage;
- PES / ONS methods for independent coverage and response estimation;
- exact finite-world enumeration for small mathematical oracles;
- constrained LPs for sharp partial-identification bounds;
- separate engineering solvers for intervention planning.

Then make the interfaces evidence-carrying. Each downstream decision consumes a crosswalk with namespace, cutoff, lineage, source dependence, scope, uncertainty and independent verifier disposition. Where these are not established, preserve a bound or abstain.

The performance claim worth proving is: given equivalent eligible evidence, FSRE reduces false-safe decisions and denominator corruption while preserving or improving service outcomes, and can show which evidence changed the decision.

That is a stronger and more meaningful comparison than a leaderboard based on pairwise F1 alone.
