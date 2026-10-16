# AeroCommons Benchmark Targets V1

**Status:** Proposed research targets, not measured results or operational acceptance criteria  
**Authority ceiling:** Offline synthetic replay and research qualification only  
**Date:** 2026-10-16  
**Parent:** [Mycelix AeroCommons PR #5024](https://github.com/Luminous-Dynamics/mycelix/pull/5024)

## Purpose

Use a scorecard that rewards correctness, falsifiability, and honest uncertainty before speed or throughput. Airspace coordination already has established strategic deconfliction and interoperability work: FAA UTM research includes intent exchange and strategic coordination, while ASTM F3548-21 specifies strategic UTM service roles and explicitly does not cover authorization or all tactical and non-participating-aircraft cases. This programme should complement that work, not claim it is novel or replace it.

Benchmarks below are **targets to aim for**, not achieved results. Report results by tier and scenario family; never compress hard-gate failures, model coverage, latency, access, and environmental impacts into a single scalar score.

## Tier 0 — Contract and fail-closed correctness

Target:

- 100% of the predeclared hard-invariant negative controls must return the specified reject, unresolved, degraded, or reassessment state; zero hard-gate violations may be accepted as eligible for planning.
- 100% of missing, unknown, or unsupported traffic/model pairs must be stopped before geometric work. Count and assert geometry checks remain zero on those paths.
- 100% of malformed, over-budget, incompatible frame/datum/time, stale/incomplete-view, and contradictory-history fixtures must match their declared fail-closed outcome.
- Every benchmark result must state its scope and authority ceiling; no fixture or score may imply flight authorization, real-world separation safety, or airworthiness.

Release rule: one missed critical negative control blocks promotion regardless of performance elsewhere.

## Tier 1 — Uncertainty composition

Scope: AIR-PB-015 and test-only scalar affine prototypes tracked by AIR-PB-017 through AIR-PB-019; AIR-PB-021 defines receipt-integrity checks.

Proposed targets:

- 100% agreement with hand-derived, committed vectors for the prototype's declared scalar domain.
- Zero interval under-approximations across exhaustive finite-domain tests on bounded small integers and a predeclared, reproducibly seeded generated corpus, including a distinct oracle that enumerates each shared latent source once where multiple contributions reference the same declared dependency ID; separately test two origins sharing one dependency, one origin with two independent dependency IDs, metadata mismatch, and stale/future contract validity.
- 100% agreement across 1,000 deterministic generated shared-source domains, plus 1,000 independent-source domains; include a deliberately narrowed interval mutant that the oracle must reject.
- 100% order invariance for canonicalization and final results across all permutations for small vectors, plus 100,000 deterministic permutations for larger generated cases.
- 100% rejection of absent uncertainty models, conflicting bounds or typed contracts for a shared dependency ID, unsupported units/frames/time bases, noncanonical identifiers, invalid/expired validity windows, unsupported provenance claims, arithmetic overflow, and configured work/domain violations.
- Retain source IDs, dependency IDs, and coefficient contributions so a reviewer can distinguish origin from dependence. A source ID alone must not justify cancellation; a shared dependency ID may group contributions only under the declared synthetic contract. Neither ID is cryptographic identity or proof that physical errors truly share a cause.

Important: these are targets for the scalar affine subset only. They do not validate nonlinear transformations, clocks, multi-axis geometry, stochastic assumptions, aircraft reachability, or physical sensor coverage.

**Separate deterministic set evidence from probabilistic calibration (AIR-PB-020).** A finite-set interval answers which values are admitted by the declared bounds; a probability estimate answers how likely outcomes are under a specified distribution. These are different evidence products. For the first synthetic calibration study, aim for expected calibration error <= 0.05 on a pinned holdout corpus (with predeclared binning and minimum sample counts), and report reliability diagrams, Brier score, uncertainty intervals and results stratified by encounter geometry, look-ahead horizon, uncertainty magnitude and source-quality class. This is a research-only target, not an operational aviation threshold. No probability estimate can establish a guaranteed outer bound, and no low risk score can waive a hard gate. A May 2026 study of state-based U-space conflict detection examines Gaussian position/velocity errors using Monte Carlo and analytical approximations, and reports sensitivity to uncertainty and encounter geometry; this reinforces the need for a separately calibrated probability track, not a blended safety score. [Study](https://doi.org/10.1016/j.ress.2025.112111).

## Tier 2 — Independent checker and mutation resistance

Scope: AIR-PB-002 and AIR-PB-016.

Proposed targets:

- 100% detection of the predeclared critical mutant set covering hard constraints, units, coordinate/time bases, stale inputs, applicability, uncertainty metadata, resource accounting, and authority scope.
- At least 95% detection of the broader eligible mutation set, with every surviving mutant classified and reviewed; the percentage must not hide any missed critical mutant.
- Zero crashes, timeouts, disagreements, unsupported schemas, or missing checker receipts may be converted into approval. Each remains a distinct outcome.
- Every checker records its source/build/dependency identities and shared-dependency graph. An independence claim must identify what is dissimilar and what remains shared.
- Include valid near-boundary controls and report false alarms as well as missed faults; an always-rejecting checker must not score as successful.

These are research acceptance thresholds, not universal aviation standards. The mutation operators, excluded mutants, denominators, and expected outcomes must be pinned before running the campaign.

## Tier 3 — Reproducibility and cross-implementation contracts

Proposed targets:

- 100% of receipts bind exact scenario/source bytes or manifest digests, model and rules versions, toolchain/build identity, checker identity, and output digest.
- 100% rejection across predeclared receipt-tamper controls when fixture bytes, contract bytes, outcome bytes, digest formatting, case count, or signature-status claims are mutated; explicitly distinguish unsigned integrity receipts from authenticated attestations.
- 100% of required metadata (units, frame/datum, time basis, validity, source provenance, uncertainty semantics and authority ceiling) survives serialization and each tested adapter boundary.
- Two separately implemented consumers must agree on every normalized outcome and required metadata field in the declared shared corpus; any disagreement yields an explicit differential report, not a majority-vote approval.
- Same inputs, declared versions and deterministic settings must produce byte-identical normalized reports, or explicitly document and test why byte identity is not part of the contract.

## Tier 4 — Capacity, resilience, and service performance

First establish the baseline; do not invent a latency target before profiling the current bounded workload.

Report median, p95 and p99 wall-clock latency, peak memory, input counts, candidate/pair checks, timeouts, and output sizes for:

- current declared caps and near-cap cases (including up to 300 reservation claims, up to 300 dynamic-constraint flights, and up to 1,000 reservation events where those caps apply);
- sparse, dense, and worst-case-conflict patterns, not just benign non-overlap cases;
- provider partitions, stale evidence, event reordering, duplicate delivery, forks, and recovery after capacity loss;
- cold and warm runs, with CPU/OS/toolchain details and pinned fixture digests.

Then evaluate scaling multipliers on the same workload families. Suggested research checkpoints are 10× and 100× the baseline input counts, but only after the data structures and compute budget are explicitly revised. Report algorithmic work as well as elapsed time; never obtain faster results by skipping hard checks, truncating candidate sets without declaration, or treating unknown as absent.

## Tier 5 — Planning quality and commons outcomes

Compare candidate policies on identical, pinned demand, constraints, disruptions, candidate-set generation and independent feasibility oracle.

Report separately:

- eligible and ineligible candidates before and after hard gates;
- number of feasible alternatives and eligible recovery options before/after each disruption;
- delay distributions, unserved demand, resource utilization and recovery time;
- source freshness, unknown-traffic/coverage gaps, unresolved counts, and compute cost;
- energy, noise, ground exposure and community/service-group outcomes only where their input data, units, uncertainty, provenance and aggregation rules are qualified.

A planning policy may improve a metric only among hard-eligible candidates. Do not allow throughput, low cost, reputation, or one group's access metric to compensate for a safety or model-applicability failure. Publish Pareto tradeoffs and subgroup denominators instead of one opaque winner score.

## Minimum experiment contract

Every result package must contain:

1. exact commit and clean/dirty workspace declaration;
2. fixture and source-data digest manifest;
3. model/applicability declaration and declared omissions;
4. hard-invariant outcomes before performance ranking;
5. checker/oracle identity and dependency lineage;
6. negative controls, fault mutations, counterexamples, and disagreements;
7. latency/work/memory results with reproducibility metadata;
8. explicit status: design-only, implemented-not-run, passed-on-exact-head, failed, or blocked;
9. a statement of what the result does not establish.

## Standards and research alignment

- [FAA UTM Concept of Operations v2.0](https://www.faa.gov/researchdevelopment/trafficmanagement/utm-concept-operations-version-20-utm-conops-v20) and [FAA UTM overview](https://www.faa.gov/uas/advanced_operations/traffic_management): distributed coordination and interoperability are existing work; this project must differentiate through explicit, composable evidence and reproducible tests rather than rebrand the concept.
- [FAA UTM Pilot Program Phase 2 Final Report](https://www.faa.gov/sites/faa.gov/files/uas/research_development/traffic_management/utm_pilot_program/FY20_UPP2_Final_Report.pdf): documents evaluations including strategic deconfliction and USS data-exchange performance. It is a useful baseline for reporting service-level metrics, not evidence that this prototype meets them.
- [ASTM F3548-21](https://store.astm.org/f3548-21.html): defines UTM service-provider interoperability and strategic roles; its stated scope does not establish flight authorization or comprehensive tactical/manned/non-participating aircraft conflict coverage.
- [NASA Software Assurance and IV&V (8 April 2026)](https://sma.nasa.gov/news/articles/newsitem/2026/04/08/reducing-risk-in-software-driven-hazards-across-nasa-programs-how-software-assurance-and-iv-v-strengthens-mission-safety): discusses common-cause failures, data validity, timing and targeted fault injection.

## Non-claims

Passing these research targets would establish only performance against the declared synthetic corpus and its oracle assumptions. They would not establish operational safety, airworthiness, regulatory compliance, permission to fly, a validated aircraft model, or absence of real-world collision risk.
