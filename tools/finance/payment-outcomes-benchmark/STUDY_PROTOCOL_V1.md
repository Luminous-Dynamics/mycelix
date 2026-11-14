# Payment Outcomes Benchmark — Study Protocol v1

**Status:** research protocol; no live provider study has been performed.
**Primary scope:** South African low-value domestic payments first. Cross-border corridors require separate corridor-specific cohorts and legal/operational profiles.
**Execution mode:** synthetic conformance first; authorised, effects-disabled shadow study next; controlled live measurement only under participant, provider and legal approval.

## 1. Research question and estimand

The benchmark asks:

> For the same eligible payment tasks, payer/payee segment, currency, amount band, merchant classes, time window and rail/service profile, which implementation produces the best risk-adjusted recipient outcome at the lowest all-in economic cost without weakening safety, privacy, accessibility or dispute rights?

The primary denominator is **eligible payment instructions**, not API calls, submitted messages, registrations, QR scans, accepted instructions, or system-wide transaction totals.

The primary outcome is **recipient funds available**, backed by evidence from the recipient side or the authoritative provider/rail event. These are separate lifecycle observations and must not be collapsed:

1. intent created;
2. payer authorised;
3. instruction dispatched;
4. provider accepted;
5. clearing/processing;
6. settlement;
7. recipient funds available;
8. settlement finality/legal discharge, where the applicable instrument and rulebook support those claims.

A timeout or unknown external result is retained as an unknown outcome. It is neither success nor a terminal failure until reconciled. Never remove unknown outcomes from the denominator or retry under a new operation identity.

## 2. Frozen cohort and comparability

All compared implementations must share one signed study manifest containing:

- study/corridor and jurisdiction;
- participant/product/service eligibility;
- currency, instrument and exact unit;
- amount distribution and task mix;
- payer and payee/merchant segments;
- merchant categories and geographic strata;
- test window, service-hours and timezone;
- exact provider/rail profile versions and source digests;
- task-corpus digest, parser/mapping versions and exclusion rules;
- fee schedule effective dates and FX reference methodology;
- sample-size/power calculation and stopping rules;
- event-time source, clock-quality bounds and missing-data handling.

Every provider must process the same task corpus or a pre-registered randomized allocation of equivalent tasks. For operational studies, use a randomized crossover or blocked assignment by merchant, payer segment, geography and time-of-day to control provider mix, network load and merchant selection. Record run order and incident windows. Do not compare a provider's best hour with another provider's full-day results.

The evaluator must reject inconsistent task/user/eligible-merchant denominators, currency changes, mismatched task-corpus or profile digests, and runs that omit the frozen cohort identity. The study manifest itself should be content-addressed and independently witnessed.

## 3. Outcome definitions and measurement

### Delivery and reliability

- recipient available: independent, source-backed evidence that the recipient can use the credited funds under that product's rules.
- terminal failures: known, terminally unsuccessful eligible instructions.
- unknown outcomes: unresolved/ambiguous result after timeout, disconnect or provider outage.
- returned/reversed: successor return/reversal events linked to a previously delivered payment; do not overwrite the original success.
- duplicate effects: more than one economic debit/credit for one logical operation identity.
- false finality promotions: an earlier lifecycle event presented as settlement/finality without the required source evidence.
- unexplained reconciliation deltas: per-account/per-instrument balances or event totals that do not reconcile to authoritative source records.

The synthetic conformance corpus uses zero-tolerance mutation controls for duplicate effects, false finality, unexplained deltas and authority/privacy violations. In live service operations, report incident counts/rates, confidence intervals, severity and remediation; do not imply that an observed zero proves future risk is zero.

### Latency

Record separate event timestamps for:
- customer confirmation/authorisation;
- provider acknowledgement;
- rail/provider completion;
- recipient funds availability;
- reconciliation closure.

Report p50, p95 and p99 for each lifecycle interval. Use monotonic clock measurements within each instrumented component and retain clock synchronisation/error bounds when correlating systems. A provider response time is not end-to-end recipient availability. Do not infer settlement from a low latency percentile.

### All-in cost

For each eligible instruction, preserve integer minor units and the currency. All-in cost must reconcile exactly to the sum of:

- payer fees;
- merchant/acquirer fees;
- FX spread/conversion cost using a declared reference rate and observation time;
- cash-out/withdrawal cost;
- return, reversal and dispute costs; and
- provider/network cost allocated under a pre-registered accounting rule.

Report both total cost and cost per recipient-available delivery. Do not report zero-cost to a payer while shifting the expense to merchants, agents, subsidies or public infrastructure. Keep operator minutes per 1,000 instructions separately; where labour cost is monetised, state the wage/overhead allocation and prevent double counting with provider/network costs.

### Inclusion and user outcomes

Measure:
- successful task completion and comprehension;
- mistaken-payee prevention and recovery;
- active merchants divided by a frozen eligible-merchant denominator;
- merchant time-to-usable-funds and reconciliation effort;
- accessible/non-smartphone fallback;
- connectivity/device constraints and geographic reach;
- support, dispute and correction response times.

Disaggregate by relevant participant, merchant type, amount, geography, device/connectivity and use case. Aggregate means must not hide a badly served subgroup.

## 4. Statistical analysis

- Pre-register hypotheses, primary/secondary outcomes, exclusions, model, sample-size calculation, minimum detectable effect, desired power and stopping rules before collecting live data.
- Derive required sample sizes from baseline rates and a stated effect size; do not choose an arbitrary fixed count.
- Report numerators, denominators, missingness and 95% confidence intervals. The reference evaluator uses Wilson intervals for binomial completion/task-success proportions.
- For p50/p95/p99 and cost distributions, a live evaluator should add bootstrap confidence intervals using resampling clustered by merchant/time block; do not treat transactions from one merchant or outage window as independent samples.
- Prefer paired or blocked comparisons on the same tasks. Report effect sizes and uncertainty, not p-values alone.
- Correct for multiple primary comparisons or pre-register one primary endpoint.
- Preserve all excluded records with machine-readable reason codes. No post-hoc dropping of timeouts, reversals, fraud cases, low-value payments or poorly performing participants.
- Publish sensitivity analyses for unknown outcomes, missing providers, outage windows, FX methodology and cost allocation.

## 5. Decision logic: safety gates then Pareto analysis

A candidate is disqualified from a conformance comparison if any seeded control produces:
- duplicate economic effect;
- false settlement-finality promotion;
- unexplained reconciliation delta;
- unauthorised secondary data use;
- essential-service denial solely because the user refused unrelated data-sharing consent; or
- mutation of the authorised payee/amount after user confirmation.

Do not combine these safety invariants into a score. A lower latency or lower fee cannot compensate for an unsafe effect or privacy violation.

After gate qualification, compare cost per delivered payment, completion, unknown/failure/return rates, p50/p95/p99, operator effort, user task success and merchant coverage using Pareto dominance. Report a trade-off where one metric improves while another worsens. A weighted composite may be provided only as a secondary sensitivity view with disclosed weights; it never overrides a failed hard gate.

## 6. Industry context versus matched comparison

Official UPI, Pix and PayShap statistics are retained as **market context**. Their windows, population definitions, participant coverage and product scopes differ; they do not constitute a direct leaderboard.

A fair comparison to an incumbent or industry reference requires authorised access to matched participant/product data and a common study protocol. Public aggregate data cannot establish provider-specific price, limits, end-to-end latency, reliability, merchant acceptance or error handling.

Relevant architecture precedents include:
- BIS/CPMI's Project Nexus blueprint for connecting domestic instant-payment systems through standardised interlinking rather than requiring every provider to join every national rail;
- BIS research on cross-border payment technologies, which highlights interoperability and multi-sided market/institutional frictions as constraints that cannot be solved by a technical connector alone;
- SARB's QR+ separation between standardised payment initiation data and execution by the selected underlying rail.

These precedents guide architecture and governance; they do not grant Mycelix provider access, legal status or system endorsement.

## 7. Evidence, privacy and publication

- Use synthetic data for conformance and mutation testing.
- Live data requires explicit authority, purpose limitation, minimisation, retention policy and independent review.
- Never publish account identifiers, raw credentials, personal transaction histories or joinable pseudonyms.
- Every result must include immutable input digests, toolchain/version, source profile, evaluator version, run timestamps, exclusions and reviewer identity.
- Store corrections, returns and disputes as append-only successors; never silently overwrite the record.
- Publish negative results, failed gates and limitations alongside positive metrics.
- Distinguish observed evidence, estimates, synthetic fixtures, provider claims and programme targets.

## 8. Claim ceiling and next gate

This protocol does not demonstrate a real service, a real cost reduction, a live integration or Mycelix superiority. Before a real comparison, Mycelix needs an authorised participant and provider, a signed study manifest, an independently reviewed event dictionary, approved privacy controls, and a source-backed reconciliation process.

The first credible win is not an invented percentage improvement. It is a reproducible, independently verified result on matched tasks that shows better all-in delivered cost or operational burden **without** worse completion, unknown outcomes, tail latency, access, privacy, disputes or reversals.
