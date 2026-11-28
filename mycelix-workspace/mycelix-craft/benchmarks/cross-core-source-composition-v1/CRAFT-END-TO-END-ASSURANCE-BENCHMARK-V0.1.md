# Craft End-to-End Decision-Assurance Benchmark V0.1

**Status:** research specification; not a measured benchmark result  
**Scope:** offline reference benchmark; no production control or autonomous action  
**Purpose:** compare complete evidence-to-decision assurance workflows without collapsing semantic, safety, decision, and performance dimensions into one score.

## 1. Research thesis

The benchmark unit is a **decision episode**, not an event prediction, process model, graph query, or dashboard. An episode starts with a declared source universe and evidence-admission policy, and ends with a decision basis and independently verifiable outcome/effect receipts—or an explicit unresolved state.

A correct system must be able to say **unknown**. It must not silently turn missing sources into completeness, graph connectivity into independence, a producer's assertion into authority, or an execution request into a verified physical outcome.

## 2. Lifecycle under test

1. Source universe declaration and source/admission receipts.
2. Per-stream cuts and causal frontier selection.
3. OCEL temporal projection, including object lifecycle and relationship changes.
4. Policy/authority and safe-action envelope evaluation.
5. Decision Basis Capsule (DBC) construction.
6. Optional Evidence Acquisition Proposal (EAP), advisory only.
7. Evidence acquisition and source admission.
8. Reprojection/replay and a new DBC.
9. Execution request, observation and independently qualified outcome/effect receipts.

The prior DBC, EAP, and post-acquisition DBC are distinct immutable artifacts. An EAP does not mutate a DBC and is not itself authorization. A post-acquisition DBC is created only after the new evidence is admitted and replayed.

## 3. Benchmark tracks and metrics

### A. Semantic and temporal correctness
- OCEL 2.x import/export round-trip semantic parity.
- Event/object/relationship cardinality and attribute-change preservation.
- Valid-time/system-time query correctness where supported.
- Causal-frontier correctness under late arrivals, equal timestamps, forks, missing parents and per-stream cuts.
- Replay determinism and historical receipt immutability.

**Target:** 100% agreement with frozen semantic known-answer cases; zero accepted invalid frontier or replay mutation.

### B. Assurance and claim discipline
- False-safe rate.
- False-authority rate.
- False-completeness rate.
- False-independence rate.
- Claim-ceiling violations.
- Mandatory-evidence waiver rate.
- Unknown-outcome mutation-barrier violations.
- Unauthorized effects.

**Target:** zero on the frozen adversarial suite. Report numerator, denominator, confidence interval and every failing case; never hide failures in a weighted average.

### C. Evidence acquisition
- EVSI and EIG reported separately.
- Regret against exact finite-state oracle.
- Regret against budgeted hindsight optimum on synthetic episodes.
- Mandatory-evidence compliance.
- Cost breakdown: money, delay, burden and risk.
- Calibration and proper scoring rules only when ground-truth probabilities and calibration protocol exist.
- Coverage/abstention vs decision loss.

**Baselines:** mandatory-only, fixed schedule, random-safe, expert-priority, EIG greedy, EVSI greedy, and budgeted hindsight oracle. The hindsight oracle is an upper bound, not a deployable baseline.

### D. Adversarial robustness
Freeze cases for omission after commitment, duplicate and conflicting events, reordering, clock skew, late-arriving evidence, equal timestamps, forked streams, missing causal parents, source substitution, digest/schema substitution, stale policy, authority revocation, shared calibration/common cause, out-of-domain observations, unknown execution outcomes, and maliciously incomplete source universes.

### E. Efficiency and scale
Report throughput, p50/p95/p99 latency, peak RSS, CPU time, bytes read/written, index size, receipt/proof size, and recovery/replay cost. Separate cold and warm cache; streaming and batch; single-source and federated; ingestion and query; semantic parity must pass before performance is compared.

Scale ladder: 1K, 10K, 100K, 1M, 2.4M+, 7M+, 10M+ events. Add graph-only 50M-edge stress only where the baseline task is genuinely graph processing. Use multi-object fanout, high-cardinality relations, skewed streams, late arrivals and long retention.

### F. Interoperability and independent verification
- OCEL 2.x interchange.
- SCITT signed statement/receipt references where supported.
- Offline verification from exported artifacts and trust roots.
- Verifier portability across process boundaries.
- Explicit reporting of unsupported profile semantics.

## 4. Comparator policy

| Comparator | Fair task | Do not claim |
|---|---|---|
| PM4Py | Process discovery, conformance, OCEL handling | Decision authority or end-to-end assurance |
| Ocelescope | OCEL analysis and object-centric process queries | Equivalent DBC/EAP semantics unless implemented |
| OC-DCR | Object-centric discovery/conformance | Source-universe completeness or operational authority |
| XTDB | Bitemporal query semantics | Causal-frontier proof or decision assurance |
| TGB2 | Temporal graph representation/scale | Process conformance or safe decision quality |
| SCITT | Signed statements, transparency receipts | Truth/correctness of the underlying decision |
| Craft | End-to-end evidence-to-decision assurance | Any field/physical capability not independently qualified |

Record exact versions, configuration, hardware, data transformations, supported semantics, and exclusions. Do not compare headline numbers from different hardware or workloads. A baseline's unsupported task is marked NotComparable, not scored zero.

## 5. Frozen experiment protocol

- Freeze dataset hashes, generator version, seeds, ground truth, threat model, scoring code and environment manifest before benchmark execution.
- Include positive controls, negative controls and mutation tests for every proof obligation.
- Minimum five repetitions for performance; publish raw samples and summary statistics.
- Separate public real-world datasets, synthetic scale tests and adversarial fixtures.
- Publish per-case output, machine-readable results, exact commit SHA and artifact digests.
- Keep the evaluator independent from the system under test where possible.
- No composite score may compensate for a false-safe, false-authority, unauthorized-effect or mandatory-evidence failure.
- Classify each result as implemented, structural, local test, hosted exact-head pass, pending, or field/physical NotEvaluated.

## 6. Proposed v0.1 acceptance gates

1. Every lifecycle stage has a schema/receipt contract or an explicit NotEvaluated boundary.
2. Frozen semantic and adversarial KATs have deterministic expected outputs.
3. All zero-tolerance safety metrics are reported as raw counts and rates.
4. EVSI and EIG have counterexamples demonstrating their non-equivalence.
5. At least three task-separated baseline adapters or reproducible baseline protocols exist.
6. Performance protocol is reproducible and does not conflate unsupported semantics.
7. Offline verifier can validate exported receipts without access to the producer's live database.
8. Benchmark report records all unsupported/untested claims.

These are proposed gates, not achieved results.

## 7. Known limitations and non-goals

V0.1 does not establish field calibration, sensor accuracy, legal authority, human legitimacy, source truth, real independence, physical outcome, or production-scale performance. Cryptographic integrity detects certain changes; it does not make submitted facts true. A valid receipt is evidence of a statement and its registration, not proof that the statement is correct.

No autonomous dispatch, sampling, permit, advisory, service closure or other real-world mutation is in scope.

## 8. Immediate research sequence

1. Freeze scenario taxonomy and threat model.
2. Add machine-readable manifest and seed registry.
3. Build synthetic multi-source generator with deterministic ground truth.
4. Implement independent verifier and mutation harness.
5. Add OCEL/temporal and acquisition baseline adapters.
6. Run small-scale semantic parity first, then performance ladders.
7. Publish all failures and unsupported comparisons.


## 9. Additional paradigm shift: epistemic debt and observability budgets

A system should not only report what it knows. It should account for what it cannot know yet, why it cannot know it, what decisions that uncertainty blocks, and the cost/risk of leaving it unresolved. Define an **epistemic debt ledger** per decision episode:

- each unresolved claim or required evidence item;
- the missing source, time interval, object relation, authority, calibration or dependency;
- whether the gap is structural, unavailable, stale, disputed, out-of-domain or not yet evaluated;
- decisions and claim dimensions constrained by the gap;
- owner/remediation path, expiry and invalidation conditions;
- whether the gap is mandatory or optional;
- candidate observations and their EVSI/EIG/cost profile, if admissible.

Benchmark the ledger for completeness against a frozen ground-truth gap set. Measure unknown-detection recall, unsupported-certainty rate, time-to-detect, stale-gap persistence, duplicate-gap rate, and whether each gap correctly constrains the relevant claim ceiling. A system that admits uncertainty accurately should outperform one that emits confident but unsupported completeness.

Add **missing-not-at-random** and strategic-omission scenarios: sources disappear preferentially when they contain adverse outcomes; a source selectively reports successful events; evidence arrives after a decision frontier; a provider changes identifiers; and a party submits a validly signed but selectively incomplete feed. Cryptographic validity must not be scored as universe completeness.

Introduce a budgeted **observability allocation** task: given a fixed human-review, sensor, laboratory, latency and financial budget, choose the next source/observation to reduce decision loss and epistemic debt without relaxing mandatory evidence or safe-action constraints. Compare fixed, random-safe, expert, EIG, EVSI and constrained portfolio policies. Report fairness and burden distribution across affected populations where those data are explicitly available; do not infer missing values.

This is a benchmark dimension, not a new authority to waive required evidence. Mandatory gaps remain blocking regardless of acquisition cost or score.
