# NRVCA benchmark protocol v0.1

## Why preregister

A beyond-GDP system can look superior by choosing favorable indicators, hiding incompatible sources, excluding bad cases, or changing weights after seeing results. NRVCA therefore uses a benchmark contract before source calculations begin. Each benchmark must name its exact target, unit of analysis, data vintage, incumbent, strongest feasible baseline, candidate method, primary metrics, acceptance rules, guardrails, and claim ceiling.

The companion design-only run is [NRVCA-SA-BRIDGE-001](../../../../fixtures/nrvca-sa-bridge-preregistration-v0.1.json); its contract is [nrvca-benchmark-preregistration-v0.1.schema.json](../../../../schemas/nrvca-benchmark-preregistration-v0.1.schema.json).

## Three independent benchmark tracks

### Track A — accounting and evidence integrity

Question: can we reconstruct official published values and source-native physical-account identities without unsupported mappings?

Baselines:
- direct reproduction of published official totals;
- independent source-native account reconstruction;
- NRVCA's source-bound bridge.

Primary metrics:
- published-value reproduction rate;
- account identity closure rate;
- source-reference resolution rate;
- unclassified residual count;
- incompatible time/unit/geography joins;
- source-coverage disclosure.

Hard gates: all qualified identities close within source precision; no unresolved residual is silently dropped; all source-bound claims resolve; no experimental account is promoted to official status; and unknowns cannot become zero.

### Track B — timeliness and forecasting

Do not start this track until data-vintage histories, release dates, revision policies and feature-availability dates are pinned. Compare against:
- persistence/seasonal naive;
- simple Bridge regression;
- Dynamic Factor Model;
- Lasso/Elastic Net;
- any proposed machine-learning or remote-sensing model.

Use real-time/vintage-aware expanding or rolling windows. For every origin, a feature is admissible only if it was actually available at that origin. Report MAE, RMSE, interval calibration, bias by cycle/regime, missingness, and incremental compute/data cost. Do not tune on the confirmatory holdout. Run separate country or period holdouts where sample size and task justify them.

The December 2025 IMF comparison of traditional econometric and ML GDP nowcasting uses pseudo-real-time rolling evaluation; it supports this benchmark discipline, not any assumption that one model always wins: [IMF Working Paper 2025/252](https://www.elibrary.imf.org/view/journals/001/2025/252/article-A001-en.xml).

### Track C — decision and real-world value

Question: does the additional accounting/capability information improve a real decision?

Compare the actual incumbent decision process, a simple evidence dashboard, and NRVCA-enabled analysis under equivalent source access, budgets, constraints and review time. Predefine one primary decision outcome (for example, fully loaded cost per successfully served unit) and retain guardrails for reliability, equity, safety, resilience and resource use. Use shadow operation before prospective interventions; seek a credible causal comparison and independent replication before any causal claim.

Prediction accuracy or a richer index alone is not an economic-impact result.

## General scoring rules

- No universal weighted “prosperity score” by default.
- Primary metrics and practical-effect thresholds are fixed before results.
- Report metrics separately when units or normative meanings differ.
- Preserve all missing, failed, non-converged, unqualified and unfavorable runs.
- Report sample sizes, coverage, confidence/uncertainty intervals, source digests and data vintage.
- Count fully loaded cost: data access, engineering, compute, integration, human review, maintenance, recovery and external-audit costs.
- Treat transfers separately from real-resource costs; avoid payer/receiver double counting.
- Validate by evidence tier: synthetic → source replay → held-out evaluation → shadow operation → prospective pilot → independent replication.

## What “better than SOTA” means

A model should beat the strongest fair baseline for a precisely defined task, not a weak straw baseline. A proposed candidate counts as superior only if it clears a preregistered practical-effect threshold on untouched data, its uncertainty interval supports improvement, its evidence requirements are met, and no hard guardrail regresses. For track A, correctness and traceability are hard gates rather than tradeable scores. For track C, an apparent gain is not credited if it merely shifts costs or harms to another group, geography, time period or ecological account.

The machine-readable schema validates the form of a preregistration; it does not prove that a study was preregistered on time or that results are trustworthy. The run manifest must bind the exact contract digest and timestamps.
