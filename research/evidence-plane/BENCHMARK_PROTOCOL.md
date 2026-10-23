# Evidence-aware benchmark protocol v0

## Before a run

Version and hash the protocol before inspecting test outcomes. Record research question/decision, primary endpoint/unit/minimum useful effect, hard constraints/falsification criteria, data versions/citations/rights/hashes, temporal/spatial coverage, conflicts and missingness, locked splits, candidate and baseline versions, resource/tuning/information budgets, timeout/retry/seed policy, statistical plan and independent trial count.

Split by units with dependent leakage: site, region, year/season, event, component lineage or operating episode. Report interpolation and extrapolation separately. Random rows from the same station/episode do not demonstrate generalization to a new region.

## Comparison arms

Include incumbent/current practice, a simple transparent baseline, classical optimization/statistical analysis, a strong contemporary version-pinned method with fair tuning, the candidate, targeted ablations and an adversarial negative control. Record omissions. Incumbent plus a simple baseline is the minimum in a decision benchmark.

Match input information, cutoff dates, compute/tuning budgets, precision where practicable, timeout/retry policy, preprocessing and human intervention. Disclose unavoidable differences. A large tuned search against an untuned default does not establish algorithmic superiority; include accessible/open baselines for affordable replication.

## Metrics and statistics

Never hide constraints in one score. Report the preregistered primary effect and uncertainty; meaningful quality/error/calibration metrics; tail/worst-case and distribution shift; wall time, compute, memory, energy, transfer and maintenance cost; direct impact outcomes; safety/service gates separately; reproducibility and independent verifier agreement. Keep raw per-trial results and failed runs.

For resilience use delivered critical service as denominator rather than nominal capacity or binary availability alone. For assessment tables preserve quantile semantics, units, risk treatment and assessment-unit scope. For spatial evidence preserve CRS, time/space support, aggregation, data kind and uncertainty.

Use uncertainty intervals and trial plans appropriate to data-generating process. Resample by independent clusters (site/year/episode) when rows are correlated. A p-value is not an effect size. Predeclare practical-effect thresholds and justify power. If data cannot support inference, label the result exploratory and collect independent data rather than repeatedly tuning on one holdout.

## Superiority and falsification

A superiority claim requires exact task/operating envelope, baseline versions, source digest and locked split, matched-budget account, primary effect/uncertainty, failures, hard-gate vector and independent replay. Reject or falsify the candidate if it violates a non-tradeable gate, fails material negative controls, uses leakage, misses the minimum useful effect or cannot be replayed. A gain in one score never offsets a serious service/safety failure.

## Antarctic resilience hypothesis (not a result)

After approved source ingestion and baselines are frozen, test the hypothesis of at least 10% lower fuel per delivered critical service on held-out episodes. Require no increase in critical-service shortfall or safety violations; report fuel/logistics burden, unserved critical-service hours, recovery time, renewable curtailment and uncertainty separately.

Do not test this as a field claim against the current synthetic four-period fixture. First obtain permitted data, calibrate against independent evidence and freeze holdouts. If prerequisites cannot be met, keep conclusions at S0/S1 and limit claims to software accounting.

"Better than SOTA" must name the strongest relevant competitor/version, match budgets, disclose repeatable evaluation and include external replication when stakes require it.
