# FSRE partial-identification, capture–recapture and identity-linkage benchmark v0.1

**Status:** benchmark protocol and proposed targets. No SOTA performance claim is made.

## 1. Industry baselines and what they actually measure

### South African Census 2022 Post-enumeration Survey

Statistics South Africa describes PES as an independent sample survey conducted after census enumeration to assess coverage and content error. PES 2022 selected 840 sub-enumeration areas across all nine provinces. Its publication reports undercount, overcount and content-inconsistency measures and uses adjustment factors derived from the PES to adjust the census count:
- official PES release: https://www.statssa.gov.za/?p=15645
- official PES 2022 report: https://www.statssa.gov.za/publications/P03015/P030152022.pdf

The report distinguishes scope and estimate stages. For example, it reports a preliminary national net undercount rate of 29.60% with a 95% interval of 28.00%–31.20% in one table, while its key findings separately report a final net coverage error rate of 31.1% for the final estimated true population within the private-dwelling PES scope. These should not be conflated: they refer to different estimation stages and population scopes.

**FSRE implication:** benchmark separate coverage error (missed, duplicate and mislocated people/households) from content/measurement error, and preserve the population scope, reference night, sampling frame, design weights, uncertainty and estimation stage as first-class metadata. Do not present a record-linkage result as a substitute for the independent field survey.

### UK Office for National Statistics Census 2021

ONS documents coverage error including non-response and overcoverage (duplicates and people counted in the wrong location), measurement error and modelling uncertainty. It describes combining census and Census Coverage Survey data with logistic-regression models to estimate response probabilities:
- quality and methodology information: https://www.ons.gov.uk/peoplepopulationandcommunity/populationandmigration/populationestimates/methodologies/qualityandmethodologyinformationqmiforcensus2021
- coverage estimation: https://www.ons.gov.uk/peoplepopulationandcommunity/populationandmigration/populationestimates/methodologies/coverageestimationforcensus2021inenglandandwales/pdf

**FSRE implication:** compare more than the final population point estimate. Report coverage, measurement, linkage, modelling and sampling uncertainty separately, then show how each affects a downstream decision. Stratification and small-area stability must be explicit.

### Splink 5.0.0

The official Splink 5.0.0 release (28 September 2026) states a maintainer capacity claim of up to 1 billion records in under 10 minutes and says the statistical model is unchanged from v4. Treat this as a capacity claim requiring matched reproduction, not as a verified FSRE comparison:
https://moj-analytical-services.github.io/splink/blog/2026/09/28/splink-500-released.html

Splink is a linkage engine and should be treated as a serious comparator for pair generation, scoring, clustering and throughput. FSRE's differentiator is downstream assurance: temporal lineage, source ancestry, population-frame completeness, duplicate/mislocation impacts, partial identification and decision integrity. Do not claim to beat Splink's linkage throughput or pair accuracy without an exact matched run.

## 2. Paradigm shift: from count correction to evidence-qualified population state

A single adjusted count hides too much. Model the population state as an explicit object with:

- target population and estimand;
- reference time and geography;
- unit of analysis (person, household, dwelling, asset, service connection);
- inclusion/exclusion rules and frame;
- source/issuer lineage and source ancestry;
- capture and linkage process;
- sampling and response mechanism;
- duplicate, missed, mislocated and content-error models;
- measurement uncertainty and model assumptions;
- rights, source-use and authority state;
- independent verifier disposition;
- action and service-floor consequences.

Every downstream aggregate must be traceable to the identity and frame decisions that produced its denominator.

Keep distinct namespaces for physical asset, person/household, service boundary, owner, operator, account and address. A shared address is not proof that two records describe the same person or asset; a shared upstream source is not independent corroboration.

## 3. Benchmark layers

### A. Linkage quality

Compare Splink 5 and FSRE crosswalks on a frozen, permissioned/synthetic gold-standard corpus. Include:
- pair precision/recall and PR curve;
- calibration and threshold sensitivity;
- cluster precision/recall;
- false cluster-bridge and missed-bridge rates;
- namespace-confusion rate;
- temporal identity errors;
- pair candidate counts after blocking;
- wall time, p50/p95, peak memory and storage.

A pair-level F1 score is insufficient: one false bridge can contaminate many entities and change a population denominator.

### B. Capture–recapture and dual-system estimation

Generate known-truth synthetic populations with controlled:
- equal and unequal capture probabilities;
- source dependence (positive and negative);
- heterogeneity and latent strata;
- population closure violations and migration;
- false-positive and false-negative linkage;
- duplicate and mislocated events;
- frame undercoverage;
- nonresponse and missingness;
- source ancestry / common upstream registers;
- temporal mismatch and late-arriving evidence.

Compare:
1. Lincoln–Petersen and Chapman (only under declared assumptions);
2. stratified dual-system estimators;
3. log-linear multi-list estimators;
4. logistic response models of the type documented by ONS;
5. dependence-aware sensitivity bounds;
6. FSRE exact finite-world oracle;
7. FSRE constrained distribution LP with exact primal/dual/Farkas certificates.

Do not use point-estimate RMSE alone. Evaluate interval coverage, interval width, bias, false precision, assumption sensitivity and decision stability.

### C. Partial-identification LP

The current E0 LP crate verifies exact rational primal/dual certificates and exact identical-column reductions. It also builds finite-world distribution LPs with linear aggregate/moment constraints.

For a finite set of candidate worlds \(w\), masses \(p_w\), query values \(q_w\), and known linear facts:
\[
\min/\max \sum_w p_w q_w
\]
subject to:
\[
\sum_w p_w = 1,\quad p_w\ge0,\quad
\ell_k\le \sum_w a_{kw}p_w\le u_k.
\]

Benchmark:
- bound agreement with exact enumeration on small finite instances;
- sharpness via explicit primal witnesses;
- dual certificate acceptance;
- Farkas infeasibility certificate acceptance;
- false point-identification rate;
- bound width and sensitivity to removing/relaxing each assumption;
- reduced/unreduced feasible-bound parity;
- world/column reduction ratio;
- rows, columns and nonzeros;
- exact arithmetic bit growth and verification time.

The current E0 test case has four query values (0, 10, 20, 30) and a known half-population group share. Exact primal/dual certificates prove bounds [10, 20], compared with [0, 30] without that aggregate constraint. A contradictory pair of aggregate statements (same group share at least 3/4 and at most 1/2) has a Farkas contradiction of -1/4. These are synthetic known-answer cases, not real-world estimates.

### D. Decision integrity

For each estimated/bounded population state, connect the output to decisions:
- false-safe action rate;
- robust-safe set precision/recall;
- regret and worst-case regret;
- service-floor violation probability/duration (when probabilistic assumptions are defensible);
- abstention correctness;
- time/cost to defensible decision;
- intervention harm caused by false identity joins or false denominators.

No solver status or record-linkage score may override source-use, rights, authority or service-floor gates.

## 4. Scale protocol

Use 15M, 100M and 1B record scale points where infrastructure and permissioned data permit. Freeze and publish:
- immutable dataset and gold-label digests;
- schema, number of fields and missingness profile;
- hardware, storage, backend, cluster size and thread count;
- Splink version/commit and solver version/commit;
- blocking rules, candidate-pair count, feature definitions and thresholds;
- sample/chunking choices and random seeds;
- all output hashes;
- pair/cluster quality, time, peak memory and storage;
- FSRE assurance and decision-quality metrics.

A billion-record headline without candidate-pair counts, hardware, recall and cluster quality is not a fair performance comparison.

## 5. Targets (research targets, not achieved)

- 100% exact LP bound agreement with #6197 on overlapping finite E0 cases.
- 0 false optimality claims: an exact optimum requires independently verified exact primal/dual equality.
- 0 false infeasibility claims: infeasibility requires a valid exact Farkas certificate.
- 0 false point-identification under intentionally incomplete evidence.
- 0 blocked/unknown rights, authority or safety states overridden by a high-value acquisition.
- 0 linkage/cluster errors silently changing denominator membership.
- 100% reproducibility from frozen inputs, options and commit hashes.
- Exact reduced/unreduced bound parity for every claimed lossless canonical-domain reduction.
- On small synthetic cases, 100% of known-truth interval endpoints covered; publish interval width and false-precision rate.
- For large cases, report performance only on matched hardware and matched quality targets; do not invent universal throughput goals before profiling.

## 6. Research frontier

### Query-implied causal orders

Rossetto and Antonucci (PMLR 343, 2026) show that counterfactual queries themselves imply partial variable orders that can parameterize LP bounds without a fully specified causal graph. FSRE should encode these orders as explicit assumptions and verify every resulting reduction:
https://proceedings.mlr.press/v343/rossetto26a.html

### Constraint-aware canonical-domain reduction

Choe et al. (UAI/PMLR 337, 2026) reduce canonical counterfactual state spaces by removing degrees of freedom irrelevant to the objective/constraints. FSRE's exact identical-column reduction is the narrowest special case. The next research target is objective-and-constraint-equivalent state aggregation with reversible provenance and exact bound parity:
https://proceedings.mlr.press/v337/choe26a.html

### Evidence acquisition as an optimization action

Select the next inspection based on which unresolved claim or decision could change, not generic information gain alone. Enforce rights, authority, safety, scope and physical feasibility as hard gates. Evaluate EVSI/ENBS only when outcome probabilities and benefits are calibrated; otherwise report robust regret and worst-case value.

## 7. Current public implementation map

- #6197: exact finite-world partial-identification oracle.
- #6200: LP backend and constraint-reduction research question.
- #6244/#6246: dependence-aware capture–recapture.
- #6247/#6249: typed decision episode and assurance graph.
- #6252/#6253: assurance-obligation acquisition planner.
- #6255: exact sparse LP primal/dual/Farkas certificate layer and world-distribution LP builder.
- #6256: planned HiGHS backend integration with exact certificate reconstruction.
- #6240: public water resilience and population coverage benchmark.

The central ambition is not merely to estimate population size faster. It is to make every population statement carry its frame, lineage, uncertainty and assumption boundary into decisions—and to produce a machine-checkable reason when the evidence does not justify a point estimate or action.
