# Craft Industry Comparator Register and Matched Benchmark Protocol V1

**Tracking:** #6841  
**Temporal/evidence protocol:** #6825  
**DBC composition:** #6827 / #6833 / #6835  
**EVSI acquisition:** #6828 / #6829 / #6831

## 1. Competitive claim boundary

Craft is not one benchmark task. It spans (a) OCEL interchange and process analysis, (b) valid-time and knowledge-frontier reconstruction, (c) causal event-frontier validation, (d) source and evidence trust, (e) decision composition and authority, (f) action/outcome reconciliation and (g) decision-focused evidence acquisition.

No single comparison system covers all of those tasks. A benchmark result must therefore name the exact matched task and cannot transfer a win in import speed to trust, temporal correctness, process conformance or field outcomes.

The proposed competitive goal is:

1. **Parity or superiority on matched conventional tasks**, measured against the strongest semantically equivalent open baseline under identical data, format, hardware and protocol.
2. **Independent assurance advantages** in dimensions not covered by those baselines: exact accepted frontiers, no false absence without a qualified owner universe, immutable historical replay, non-widening claims, explicit authority and action-plan binding, and decision-relative evidence acquisition.
3. **Reproducibility**: any claimed result is tied to exact source commits, build/toolchain, data digests, environment, workload, repeated trials and raw output.

A broad "SOTA" claim is not permitted until all of those conditions are met for a clearly defined task.

## 2. Comparator registry

| Comparator | Task it is a fair baseline for | What it does not qualify |
|---|---|---|
| PM4Py | OCEL import/export, filtering, process discovery, event-log operations and supported conformance methods | Authenticated source completeness, causal distributed frontiers, legal authority or physical outcomes |
| Ocelescope | OCEL import/filter/export throughput and memory; useful current public matched-log reference | Evidence trust, decision justification and all process semantics |
| OC-DCR (2026) | Object-centric process discovery and alignment/conformance on compatible OCEL models | Evidence authentication, source-universe completeness or field truth |
| XTDB | Valid/system-time query semantics and history retrieval on a matched temporal query profile | Distributed source causal closure or evidence independence |
| TGB 2.0 | Temporal graph representation, event-edge scale and graph algorithm stress | OCEL interchange, process conformance, evidence trust or valid/knowledge-time replay |
| DJCC #6822 | Exact producer/consumer stage edges, profile-bound claims and non-widening structural composition | Cryptographic source verification, authority lawfulness or field qualification |
| SCITT / VDS / RFC 9162 | Signed-statement transparency, receipts and verifiable-data-structure proof interoperability | OCEL semantics, decision utility or physical outcome truth |
| Fixed schedule / domain-expert checklist | Safe evidence-acquisition policy baselines | Decision-theoretic optimization under equalized candidate budgets unless explicitly instrumented |
| Random-safe / EIG | Acquisition selection controls | EVSI decision utility or operational safety unless these are separate hard constraints |

### Public OCEL scale anchors

The Ocelescope public dataset table reports these corpus sizes:

| Public log | Events | Objects |
|---|---:|---:|
| P2P | 14,671 | 9,543 |
| Order Management | 21,008 | 10,840 |
| Container | 35,372 | 13,882 |
| AoE2 | 2,372,505 | 630,590 |
| Hinge | 38,528 | 23,771 |
| Angular | 27,847 | 28,317 |
| BPIC2017 | 1,202,267 | 31,509 |

These are corpus anchors from a public benchmark page, not results independently reproduced by Craft. Re-check file/version/licensing and hash the exact artifact before running.

### Public OCEL performance observations and caveats

Ocelescope's published library benchmark identifies its hardware as an i7-1260P, 32 GB RAM and Arch Linux, with five runs and median timing. A separate comparison against PM4Py uses Ocelescope 0.6.0, PM4Py 2.7.23.4 and Python 3.13.11, three runs, fastest-time reporting and a peak-RSS measure. Those are different experimental designs and must not be combined into one leaderboard.

On the AoE2 corpus, the published comparison reports PM4Py faster for JSON and XML import in that setup, while the streamed SQLite case favours Ocelescope. The appropriate lesson is not "one tool wins"; it is that results depend on the format and backend. Craft must rerun PM4Py and Craft on the same byte-identical source file, hardware, cache state, run count and semantic output checks before making a performance claim.

## 3. Benchmark workstreams

### A. OCEL 2.0 interchange and process analysis

**Baselines:** PM4Py and Ocelescope. OC-DCR is an additional process-model comparator, not a drop-in importer replacement.

Measure separately:
- parse/import;
- schema/profile validation;
- type and reference validation;
- normalization/canonicalization;
- object/event/relationship indexing;
- object-centric filtering;
- process discovery;
- conformance/alignment;
- export/round-trip.

The semantic parity gate must compare object/event identity and count, object types, event types, static and dynamic object attributes, event attributes, relationship qualifiers, event-to-object links, object-to-object links and declared value types. A successful import or equal record count alone is not semantic parity.

For process conformance, freeze model, log, cost function, split, noise injection and timeout. Report alignment cost, non-timeout coverage, timeouts, memory and runtime. Keep the OC-DCR evaluation and PM4Py's available conformance methods labelled separately unless their semantics are matched.

### B. Temporal reconstruction

**Baselines:** XTDB for matched valid-time/system-time historical queries; current Rust whole-document OCEL replay as the Craft reference; streaming/external-sort variants only after exact result parity.

Query families:
- valid-time cutoff with a fixed accepted knowledge frontier;
- later accepted correction that is valid in the past;
- future-valid but already-known relation;
- expiry at a half-open interval boundary;
- late-arriving event before/after accepted sequence;
- historical snapshot replay after appending later records;
- policy and authority history pinned at the same frontier.

Do not equate database system time with Craft's accepted sequence, source-recorded timestamp or authenticated causal event frontier. Report exact semantics in every comparison.

### C. Federated causal frontier

Use the independent Go reference and a future independent Rust implementation. This is a distinct owner/algorithm from #6804's progress antichain.

Workloads:
- single and multi-stream prefixes;
- balanced and highly skewed stream lengths;
- sparse and dense parent edges;
- equal timestamps with no causal relation;
- cross-stream parent closure;
- missing prefix, missing parent, duplicate event ID, fork, cycle, wrong epoch and malformed head digest;
- future malformed payload after an earlier selected frontier;
- input permutations.

Measure validation throughput, p50/p95/p99 latency, peak RSS, parent-edge count, frontier width, number of streams, receipt bytes and hashing cost. Report incomplete and invalid results separately. Causal ordering must never be inferred from wall-clock timestamps.

### D. Decision Justification Composition

**Baseline:** DJCC #6822's independently tested edge/profile contract; relevant SCITT/VDS receipts at exact message/profile boundaries.

Measure:
- canonical edge/capsule bytes;
- exact producer/consumer digest and schema binding;
- number of stages, edges and external receipts;
- profile validation and claim-ceiling propagation;
- mutation-suite false acceptance rate;
- byte-stable replay across canonical-equivalent ordering.

Do not count digest-shape validation as signature or trust verification. Report the six claim dimensions independently.

### E. Evidence independence

Use #6826 failure-domain lineage fixtures and threat-model profiles. Benchmark the graph algorithm separately from authenticated provenance. Include shared upstream data, calibration, key custody, operator, administrative control plane, transport and power domains.

A structural disjointness pass is not an independence pass. The positive independence claim remains NotEvaluated until an authenticated lineage attestation and declared threat-model completeness are available.

### F. Decision-focused acquisition

Compare mandatory-only, fixed schedule, expert checklist, random-safe, EIG greedy and EVSI greedy under identical safe candidate sets, budgets, delays, loss profiles and adjudication.

Report EIG and EVSI separately. EVSI is expected reduction in decision loss; EIG is expected uncertainty reduction. Include counterexamples where EIG is high but the safe action never changes, and where a rare high-consequence observation has low entropy reduction but high EVSI.

Mandatory sampling, protective action, privacy, access and worker safety remain hard constraints. A planner must abstain on unavailable, unqualified or out-of-domain likelihoods; it may not dispatch, issue permits, substitute tests or close a case.

## 4. Scale ladder and workload topology

Use the following planned ladder, subject to available memory and licensed data:

- 1K and 10K events: deterministic functional and mutation tests.
- 100K events: routine replay and multi-query stress.
- 1M and ~2.4M events: public-scale comparative workload (BPIC2017 / AoE2).
- 7M+ events: large-log workload where an appropriate public or synthetic corpus is available.
- 10M+ synthetic events: controlled high-scale stress with fixed seed and declared topology.
- up to 50M edges: optional temporal-graph-only stress inspired by TGB 2.0; not a process-mining or evidence-assurance comparison.

Always report object, event, relation and parent-edge counts. Equal event counts with radically different object degree, event-to-object fanout, relation density, dynamic-attribute cardinality or temporal correction rates are not the same workload.

## 5. Experimental protocol

1. Pin exact repo SHA, build digest, dependency lock, runtime and all dataset file digests.
2. Use identical hardware, kernel, storage, power profile, CPU affinity and memory limit for head-to-head comparisons.
3. Separate cold-cache and warm-cache trials; disclose page-cache management.
4. Run at least five timed repetitions per primary condition after disclosed warm-up. Report median, p95/p99 where sample size supports them, dispersion and confidence intervals. Do not compare fastest-of-three with median-of-five.
5. Measure independent stages rather than only end-to-end totals. Preserve raw traces and result manifests.
6. Validate semantic parity before including a timing in the comparative result.
7. Record timeouts, failures, out-of-memory, skipped tests and unsupported input separately. A skipped test is not a pass.
8. Do not weaken correctness gates or drop expensive fields to improve throughput.
9. Use the independent known-answer oracle and mutation tests as a release gate; performance results are not a substitute.
10. Publish enough commands, source pins and manifests for an independent party to reproduce the result.

## 6. Scorecard: report, do not collapse

| Dimension | Required output |
|---|---|
| Correctness | Exact KAT agreement; semantic parity; false acceptance; false rejection |
| Temporal integrity | Historical receipt mutation; valid/knowledge frontier confusion; correction-impact precision/recall |
| Completeness | False absence; false completeness; incomplete-owner handling |
| Trust / authority | Authentication and authority promotions; invalid scope/epoch/revocation handling |
| Independence | False-independence rate under shared cause and missing lineage |
| Outcome | False-effect rate; unknown-outcome barrier violations; reconciliation errors |
| Decision utility | Expected/realized EVSI; decision loss/regret; unnecessary holds; abstention quality |
| Performance | Wall/CPU time; throughput; p50/p95/p99; peak RSS; bytes; storage; timeout rate |
| Operational burden | Review time; recovery/requalification time; observation cost/delay; resident/worker burden |
| Fairness | Geographic/subgroup access, sampling burden and service delay |

## 7. Proposed acceptance targets (not achieved results)

- 100% agreement with every frozen positive and negative known-answer case.
- Zero accepted digest/schema/producer-ID substitutions.
- Zero false-safe outcomes in the frozen critical suite.
- Zero positive completeness when owner inventory authentication or completeness is absent.
- Zero positive independence without authenticated failure-domain lineage under the selected profile.
- Zero positive effect when execution/outcome reconciliation is unknown.
- Zero historical receipt mutation when the selected frontier is unchanged.
- No mandatory sample or protective action waived or delayed by EVSI selection.
- No aggregate accuracy score allowed to hide any of the above.

For performance, first achieve a reproducible matched baseline, then target 1.5x and 2x improvements on selected hot paths without semantic changes. These are engineering stretch targets to test, not SOTA claims; report memory and correctness alongside speed.

## 8. Evidence required before public claims

A public comparative claim needs:
- preregistered protocol and loss profile;
- exact commit and pinned toolchain;
- dataset name/version/license/hash and transformation manifest;
- matched hardware and repetition design;
- raw per-run results and summary statistics;
- independent oracle and semantic-parity outputs;
- all failed, timed-out and skipped conditions;
- clearly separated structural, authenticated, professional and field qualifications.

The current public baselines are task references, not proof of Craft superiority. This register should be updated only with reproducible results and source-dated methodology changes.
