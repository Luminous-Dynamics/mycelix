# Prism hierarchical retry and hedge budget reference model v0.1

**Status:** test-only reference model with 22 deterministic tests. It is not yet integrated with the production operation-semantics classifier or transport adapter and has no measured performance result.

## Why a second gate is necessary

A semantically safe retry can still overload a recovering service. Retry/hedge eligibility and retry-budget capacity are different decisions:

1. **Effect safety:** is this exact operation contract safe to repeat or hedge?
2. **Amplification safety:** is there global, per-workload and per-operation headroom to create another upstream attempt right now?

The budget must never make an ineligible retry eligible. The model accepts a local RetryEligibility class supplied by the semantics layer and then performs independent accounting.

## Budget hierarchy

The proposed production policy contains:
- **Global service budget:** concurrent extra attempts limited by a percentage of active logical operations, a minimum concurrency floor and an absolute in-flight ceiling.
- **Per-workload budget:** each workload/principal gets a separate share so one noisy tenant cannot consume the complete service recovery budget.
- **Per-operation ceiling:** maximum extra attempts over the lifetime of the logical operation. Completing a retry releases concurrency but does not restore this lifetime allowance.
- **Separate hedge ceiling:** speculative parallel copies are bounded independently from sequential retries.
- **Single-use permits:** reserve before creating the transport attempt; complete exactly once when the extra attempt reaches a terminal state.

A logical operation ID is unique for its lifetime and cannot be reopened after completion to reset the attempt counter. Every extra attempt still needs a fresh attempt ID, fresh one-use ExchangeCapability and current policy/network authorization.

## Comparison with Envoy v1.40

Envoy supports cluster retry budgets that limit concurrent retries as a proportion of active and pending requests. Its v1.40 circuit-breaker documentation gives a sample configuration using 25% with a minimum retry concurrency of 10; its retry-budget API documents a default budget of 20% and minimum retry concurrency of 3. The route architecture documentation says retry budgets are recommended to prevent retry storms.

Prism should match this mature overload control rather than claim to invent retry budgets. Its proposed extension is a hierarchical fairness model (global + workload + logical operation), a separate hedge cap, and a hard dependency on operation-effect eligibility before any budget is spent.

References:
- https://www.envoyproxy.io/docs/envoy/v1.40.0/configuration/upstream/cluster_manager/cluster_circuit_breakers
- https://www.envoyproxy.io/docs/envoy/v1.40.0/intro/arch_overview/http/http_routing

## Executable model

File: prism-egress/tests/retry_budget_model.rs

The 22 tests cover semantic eligibility, retry-versus-hedge qualification, global ratio and in-flight limits, per-workload ratio/fairness, per-operation lifetime attempt ceilings, separate hedge concurrency, one-use permits, logical-operation lifecycle, duplicate/unknown identities, checked ratio arithmetic and generated two-workload interleavings.

This is a source-level model, not a production mutex/atomic budget service. Production must use an atomic shared admission primitive across workers/processes or an explicitly partitioned, bounded budget whose safety survives restart and failover. An in-process HashMap cannot enforce cluster-wide concurrency.

## Production integration shape

The operation classifier should return a private, unforgeable RetryQualification value with one of: Denied, SequentialOnly, or SequentialAndHedge. The budget service consumes that value and returns a non-Clone ExtraAttemptPermit. Only the transport adapter may convert that permit into a new attempt.

The reservation and operation attempt count must be durable or fail safely across restart. A failed attempt still consumes the per-operation attempt allowance; releasing an in-flight permit only restores concurrency, not the lifetime retry count. If distributed budget state is unavailable, Prism should fail closed for extra attempts while allowing the initial authorized operation to proceed.

## PEAB measurements

Compare:
1. Envoy v1.40 retry_budget at 20%/min 3 and matched configurations (e.g. 25%/min 10);
2. one global Prism retry budget;
3. Prism global + per-workload ratio/concurrency budgets;
4. Prism global + workload + per-operation + hedge ceilings;
5. an unbudgeted retry/hedge control used only to quantify amplification.

Use identical active-operation patterns, fault schedules, request corpus, service capacity and semantic eligibility labels. Include workloads with balanced load, one noisy neighbor, one failing destination, all workloads failing, and recovery from partial outage.

Report extra attempts per logical operation, retry/hedge concurrency, total upstream amplification, original-operation completion rate, p50/p95/p99 latency, rejected extra attempts by reason, per-workload admitted share, starvation duration, budget state replication/recovery, and atomic reservation contention. Security gates: zero extra attempts without semantics eligibility; zero double-spent permits; zero reuse of a completed logical operation ID; and no per-workload bypass of the global limit.

Targets are zero ineligible attempts, zero budget double-spend, and bounded per-workload share within configured limits. Throughput and availability are optimized only inside those invariants. All benchmark values remain unmeasured until production integration and exact-head execution.
