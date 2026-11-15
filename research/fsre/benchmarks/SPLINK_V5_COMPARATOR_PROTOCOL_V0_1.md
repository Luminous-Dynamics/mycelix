# FSRE comparator protocol: Splink 5.0.0

**Baseline version:** Splink 5.0.0, release page dated 28 September 2026.

## Published capacity statements

The official Splink maintainer release page states that Splink is capable of linking 1 billion records in under 10 minutes. It also states that the statistical model is unchanged from version 4, while version 5 focuses on faster model training, easier/lighter installation, smaller dependency surface and cleaner incremental linkage:
https://moj-analytical-services.github.io/splink/blog/2026/09/28/splink-500-released.html

This is a maintainer-reported capacity claim, not an independently reproduced result in this repository. The release summary does not provide enough details about hardware, backend, candidate-pair count, blocking rules, feature distributions, storage, thresholds or required recall to establish a matched runtime comparison.

The older Ministry of Justice GOV.UK case describes production linkage work up to around 15 million records in under one hour. Preserve it as a historical use case, not as Splink's current scale ceiling:
https://www.gov.uk/government/publications/joined-up-data-in-government-the-future-of-data-linking-methods/splink-mojs-open-source-library-for-probabilistic-record-linkage-at-scale

## What the benchmark must freeze

At every scale point (15M, 100M, and 1B stretch where practical), record:
- exact Splink release and source commit;
- hardware, CPU/GPU, memory, storage, execution backend and cluster shape;
- raw record count in each source and total unique records;
- candidate-pair count after blocking and before scoring;
- blocking rules, comparison features, term-frequency adjustments, model training configuration and thresholds;
- number and type of fields, missingness, corruption and language distribution;
- random seeds, sampling fractions, chunking and any approximation used;
- p50/p95 runtime, wall time, peak memory and storage footprint;
- pair/edge precision and recall, cluster precision and recall, false-bridge rate and missed-match rate;
- model, edge and cluster evaluation outputs, with gold adjudication and confidence intervals.

Splink's published performance guidance notes that runtime is driven heavily by the number of pairwise comparisons after blocking and by comparison complexity. Sampling may trade accuracy for speed; chunking helps manage memory and execution at scale:
https://moj-analytical-services.github.io/splink/topic_guides/performance/drivers_of_performance.html
https://moj-analytical-services.github.io/splink/demos/tutorials/09_scaling_up_techniques.html

Splink's own evaluation guidance separates model evaluation, edge/link evaluation and cluster evaluation. FSRE should preserve the same layers:
https://moj-analytical-services.github.io/splink/topic_guides/evaluation/overview.html

## FSRE's differentiation

FSRE should not try to beat Splink's general probabilistic linkage engine on speed without reproducing a matched benchmark. Use Splink as a serious candidate generator / linkage baseline, then evaluate the downstream properties it does not itself claim to qualify as an integrated decision-assurance product:

1. Bitemporal identity replay and historical knowledge cutoffs.
2. Separation of physical asset, service boundary, owner and operator namespaces.
3. False cluster bridges and their effect on the population denominator.
4. Source ancestry, common upstream registers and dependence-sensitive coverage.
5. Frame and stratum completeness.
6. Decision-specific false-safe rate, abstention correctness and realized service outcomes.
7. Independently verifiable evidence and authority proof obligations.

A billion-record matcher can still produce an unjustified decision if a high-confidence cluster merges distinct assets, changes the historical denominator, or relies on duplicated upstream sources. Conversely, FSRE must not describe downstream assurance as a faster or more accurate matcher unless matched pair- and cluster-level results establish that.

## Benchmark result policy

The Splink 5.0 capacity statement is a target for independent reproduction, not an FSRE performance claim. The 1B scale should be a stretch data point. A failed attempt to reproduce the vendor/maintainer claim is still publishable if hardware, candidate-pair counts, runtime, memory and output quality are recorded.
