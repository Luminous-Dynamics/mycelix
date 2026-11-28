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


## 10. Meta-audit: the inspection policy must itself be auditable

Selective monitoring changes which labels and evidence are observed. That creates a feedback loop: the policy selects cases, selected cases receive review, reviewer outcomes update the training set, and the next policy is trained on a selection-biased sample. A system can therefore appear to improve while systematically ignoring rare, previously unseen, or expensive-to-inspect risks.

Treat each inspection/acquisition policy as a versioned, auditable artifact. Bind:
- policy and model version, configuration and executable digest;
- training/validation data snapshot and extraction provenance;
- objective/reward definition, cost profile and constraints;
- uncertainty and calibration state;
- selection score, selected and rejected alternatives, budget state;
- selection propensity or declared deterministic selection rule;
- exploration/exploitation state and random seed;
- human reviewer assignment, outcome, override and rationale;
- policy updates, rollback triggers, incidents and post-incident corrections.

Benchmark policy behavior as well as the underlying model:

- selection-bias analysis and support/positivity checks;
- off-policy evaluation error against simulator ground truth;
- propensity calibration and weight variance/effective sample size;
- rare-event and previously uninspected-region coverage;
- residual risk among uninspected cases;
- inspection utility per unit cost and per human-hour;
- policy drift detection delay and false alarms;
- human override rate, override outcomes and time to rollback;
- ability to reconstruct why each case was selected or not selected.

Required counterexamples include: a policy that only inspects historical high-risk groups; a reward function that improves average utility by abandoning a rare high-severity class; reviewer outcomes missing not at random; deterministic selection with zero propensity for a critical region; a model update that changes selection without a new policy version; and an adversarial party that manipulates observable features to avoid inspection.

**Hard rule:** an off-policy estimate is NotEvaluated when the logged policy has no support for the action/region being estimated. Do not hide positivity failures with clipped weights and then report a confident estimate. Report supported and unsupported strata separately.

The meta-audit track is directly motivated by the 2026 survey's findings that selective labels can reinforce visible risks, that audit policies need off-policy evaluation and selection-bias analysis, and that policy versions, reviewer outcomes, governance constraints and human override need to be reconstructable. The benchmark must test the auditor's feedback loop, not just the predictions it consumes.

## 11. Research sources and standards context

- Becerril Villamil, H.R. et al., *Adaptive Process Mining and Selective Monitoring for Algorithmic Auditing: A Survey of Representations, Learning Policies, Decision Strategies, and Open Problems*, 10 August 2026, DOI: 10.3390/make8080234. Its synthesis argues that representation, learning, budgeted inspection and governance are often evaluated separately and calls for audit-ready benchmarks, explicit budgets, calibrated uncertainty, transfer and reproducible governance.
- Berti, A. et al., *OCEL (Object-Centric Event Log) 2.1 Specification*, version 2.1.0pre5, 1 October 2026. Use its object/event/relationship semantics and exchange-format requirements as the interchange baseline; do not reduce all events to a single case identifier.
- Birkholz, H. et al., IETF RFC 9943, *An Architecture for Trustworthy and Transparent Digital Supply Chains*, June 2026. Use SCITT-compatible signed statements and transparency receipts where relevant; keep authenticity/registration separate from truth and completeness.


## 12. OCEL 2.1 cross-format semantic parity

The benchmark now includes a static golden OCEL sample across JSON, single-file CSV and bundled CSV. The logical projection includes two event types, five object types, five E2O edges, two O2O edges, and six object attribute assignments (including a timed Pump state transition). The bundle includes the separate event/object/object-change/E2O/O2O tables and authoritative metadata map.

The current OCEL 2.1.0pre5 single-file CSV profile discriminates event, O2O, object declaration and object-attribute rows using `id`, `activity` and `timestamp`; the bundled profile uses typed tables and `ocel-meta.json`. Therefore, the benchmark compares the reconstructed semantic projection, not byte representation. Every import/export path must preserve endpoints, qualifiers, object types, primitive values and complete attribute histories. See the [OCEL 2.1 CSV profile](https://www.ocel-standard.org/specification/formats/csv/) and [bundled CSV/Parquet profile](https://www.ocel-standard.org/specification/formats/bundled/).

Static fixtures are present; the parser/importer/exporter and round-trip runner are not yet implemented. Bundled Parquet is declared as NotImplemented until a serializer and importer are available. No cross-format test pass is claimed.


## 13. Decision identifiability and industry-composite baseline extension — 28 November 2026

### 13.1 Decision Identifiability Envelope (CDIE)

The benchmark adds a decision-identifiability track, implemented separately in [CDIE v0.1 specification](https://github.com/Luminous-Dynamics/mycelix/blob/research/craft-decision-identifiability-v0.1-20261128/mycelix-workspace/mycelix-craft/benchmarks/cross-core-source-composition-v1/CRAFT-DECISION-IDENTIFIABILITY-ENVELOPE-V0.1.md), [issue #6865](https://github.com/Luminous-Dynamics/mycelix/issues/6865) and [draft oracle PR #6866](https://github.com/Luminous-Dynamics/mycelix/pull/6866).

For the finite set of worlds compatible with the admitted evidence, evaluate:

- **Robust-safe actions:** safe in every listed world.
- **Invariant optima:** optimal in every listed world, not merely optimal under one nominal estimate.
- **Worst-case loss and regret:** computed separately for robust-safe actions.
- **Minimax loss vs minimax regret:** reported as distinct compromise rules.
- **World-set coverage:** exhaustive frozen fixture, conditional declared assumption set, or NotEvaluated.

No probabilities are assigned to epistemic worlds. A minimax-regret action is not described as an invariantly justified action. A finite world list supplied by a caller does not establish completeness: global decision claims remain conditional or NotEvaluated unless world-set coverage is separately qualified. Hard-safety violations are non-compensatory and cannot be averaged away.

This track composes with GCPN #5668's ownership of outcome ascertainment and partial-identification estimators, and with Craft's sequential EVSI #6858 and robust model-ambiguity EVSI #6862. The first computes action invariance within an epistemic possibility set; the latter evaluate whether additional observations are worth acquiring under declared acquisition/loss models. Their interaction should be measured, not collapsed into one score.

### 13.2 Add a best-of-breed composite comparator

Task-separated specialist baselines remain necessary, but they are not sufficient for an end-to-end claim. Add a **best-of-breed composite baseline** assembled from the strongest interoperable components available for each stage:

1. OCEL import, object-centric discovery/conformance and analysis (PM4Py / Ocelescope / OC-DCR as appropriate).
2. Bitemporal history and valid-time/system-time queries (XTDB).
3. Policy decision evaluation using a native policy engine (Cedar).
4. Signed statement/transparency and pre-execution authorization artifacts (SCITT-compatible statement/receipt and Permit profile where implemented).
5. Action authority re-check, approval, execution and system-of-record confirmation artifacts (OAR profile where implemented).
6. Human reviewer using the same evidence packet, action set, time budget and escalation rules.

The composite must be given the same frozen source evidence, event/object semantics, causal-frontier requirements, action denominator, authority rules, world set, acquisition budget and ground truth as Craft. It must be allowed to use the tools as their maintainers intend, with adapters and transformations disclosed. Record every unsupported proof obligation as NotComparable or NotEvaluated; do not silently treat unsupported behavior as a failure or as a pass.

This is the strategically important end-to-end comparison: Craft must demonstrate that its composition layer adds measurable assurance or lowers total verification/reconciliation burden over a competent composition of existing tools, not merely over isolated components.

### 13.3 Updated comparator scope and public scale anchors

| Baseline | Matched benchmark responsibility | Explicit boundary |
|---|---|---|
| PM4Py | OCEL 2.0 import/export, filtering, discovery and conformance | Do not infer source completeness, decision authority or outcome truth |
| Ocelescope | Object-centric import, export, filtering, discovery and large-log memory/time behavior | Do not compare published headline numbers without matching their run protocol |
| OC-DCR | Declarative object-centric discovery and alignment-based conformance | Not a causal-frontier, authorization or source-trust proof |
| XTDB | Valid-time/system-time bitemporal queries and as-of historical state | Bitemporality is not a verifier-relative distributed causal frontier |
| TGB 2.0 | Temporal heterogeneous graph ingestion/representation and large-edge stress where applicable | Link prediction is not process conformance or decision assurance |
| Cedar | Policy evaluation, default-deny behavior, errors and determining-policy diagnostics | An Allow result is not proof of lawful authority, authentic inputs or actual effect |
| SCITT / Permit / OAR | Signed statements, transparency registration, action-specific authorization lineage, approval and outcome-record integration | Draft profiles and receipts do not establish truth or completeness by themselves |
| Best-of-breed composite | Full multi-tool evidence-to-decision workflow under a common episode and proof-obligation manifest | All adapters, versions, scope losses and human work must be disclosed |
| Craft | Causal frontier, evidence admission, claim ceilings, CDIE, DBC/EAP, replay and offline verification composition | No production or field claim without independent qualification |

Important current anchors:

- [Ocelescope's public benchmark](https://www.ocelescope.org/ocelescope/benchmark/) reports AoE2 at **2,372,505 events and 630,590 objects**. Its primary operation charts use five runs and medians, while its separate PM4Py library comparison reports three runs and fastest wall time (and peak RAM). These are different statistics; this benchmark will re-run all compared tools under one protocol rather than treating the published values as directly comparable.
- The [November 2026 OC-DCR paper](https://doi.org/10.1007/s44311-026-00061-9) formalizes object-centric DCR graphs and adds alignment-based conformance, with rediscovery, BPIC2017 and Order Management experiments. Use it as a current declarative-conformance baseline.
- [TGB 2.0](https://tgb.complexdatalab.com/) includes temporal heterogeneous graph datasets up to roughly 53.6 million edges. Use these for graph-scale stress only; its target task is future edge/relation prediction, not evidence-to-decision assurance.
- [XTDB's native bitemporal model](https://docs.xtdb.com/intro/what-is-xtdb.html) tracks system time and valid time, making it a suitable as-of history comparator. Craft must measure its distinct causal-frontier and evidence-completeness obligations separately.
- [Cedar 4.6](https://docs.cedarpolicy.com/) is a serious policy-language baseline. Its authorizer evaluates a principal/action/resource/context request against supplied entity data and policies and distinguishes Allow, Deny and evaluation errors. Craft should reuse and compare policy semantics, not relabel policy evaluation as proof of external authority.

### 13.4 SCITT interoperability boundary: decision, permit, action and outcome

The SCITT working-group document list now includes individual Internet-Drafts for [Decision Provenance](https://datatracker.ietf.org/doc/html/draft-lemonnier-scitt-decision-provenance-03) (17 November 2026), a [Pre-Execution AI Action Authorization Record / Permit](https://datatracker.ietf.org/doc/draft-munoz-scitt-permit-profile/02/) (20 November 2026), and [Open Action Record](https://datatracker.ietf.org/doc/draft-malhotra-scitt-oar/01/) (25 November 2026). These are work-in-progress Internet-Drafts, not final RFCs or IETF-endorsed standards.

The Permit profile records a policy-evaluated allow/deny/challenge for a proposed action and can bind the canonical request body. OAR's current draft addresses action-specific authority references and re-check at commit, human approval bound to the exact action and presented display, before/after state commitments, and confirmation by reading the system of record.

Craft should build adapters and proof-obligation crosswalks for these artifacts rather than minting a competing receipt format. The benchmark must still distinguish:
- permit/decision registration from permission actually being valid at commit;
- human authentication from approval of the exact action;
- execution acknowledgement from a state change;
- system-of-record confirmation from independent physical outcome;
- authenticated statements from complete source disclosure and factual truth.

Version-pin each Internet-Draft and test profile changes as explicit schema/version migrations. Do not treat an Internet-Draft as a stable final standard.

### 13.5 Publication-grade measurement discipline

Keep the existing five-repetition minimum for CI/nightly smoke and initial regression cells. For publication-grade performance claims, target **at least 30 timed trials per stable benchmark cell**, with predeclared warm-up, cold/warm cache, storage state and outlier policy. Publish raw observations and confidence intervals alongside p50/p95/p99, throughput, peak RSS, bytes read/written, receipt/proof size and requalification cost.

For each specialist and the composite baseline, record the exact tool release/commit, hardware, runtime, configuration, input format, semantic transformations and unsupported fields. Run semantic parity before timing. No result may be promoted from a different task, data distribution, cache state, number of runs or summary statistic.

### 13.6 Additional differentiating outcomes

The benchmark should now report the following without pooling them into one score:

- **Decision identification:** proportion of episodes with an invariantly optimal safe action, conditional identification, non-identification and coverage NotEvaluated.
- **Decision robustness:** worst-case loss/regret of safe actions; report minimax compromise separately from identified optimum.
- **Evidence value:** change in the identified-action set and worst-case regret after acquisition, alongside nominal/robust EVSI and EIG.
- **Assurance overhead:** marginal compute, storage, network, proof size, latency and human review introduced by evidence admission, composition and independent verification.
- **Correction locality:** impact-cone recall and precision, and incremental-vs-full requalification with exact semantic parity.
- **Independent auditability:** offline verification success, proof-obligation vector, reviewer disagreement and time to reproduce a historical decision.

Hard assurance gates remain non-compensatory. The target is the largest decision-justified coverage and smallest total assurance burden subject to zero high-consequence false-safe, false-authority, false-completeness, false-independence, unauthorized-effect and unknown-outcome-barrier violations on the frozen qualification corpus.


## 14. Decision-separating observation benchmark — 28 November 2026

The benchmark now includes a non-probabilistic observation-partition track, tracked by [issue #6870](https://github.com/Luminous-Dynamics/mycelix/issues/6870) and [draft PR #6871](https://github.com/Luminous-Dynamics/mycelix/pull/6871).

Given a finite declared world set and a candidate deterministic observation mapping, DSOP partitions worlds by outcome and checks each branch independently:
- robust-safe actions;
- actions optimal in every world within the branch;
- minimax loss and minimax regret;
- worst residual branch minimax regret relative to the pre-observation baseline;
- mapping completeness over every compatible world;
- failure-domain overlap, kept separate from decision discrimination.

An observation is decision-separating within the listed worlds only when every non-empty outcome branch contains at least one safe action optimal in every world in that branch. The action may differ between outcomes, enabling an adaptive action policy. A missing outcome mapping is an incomplete model and must not be filled with a default.

No probabilities are assigned to observation outcomes or inferred from the number of worlds in a branch. World-count shares are combinatorial fixture coverage statistics, not probabilities, propensities or expected utility. Acquisition cost, delay and burden are reported separately; DSOP is not EVSI.

### 14.1 Active feature acquisition and decision-focused research baselines

The [IJCAI 2026 survey on Active Feature Acquisition Strategies](https://doi.org/10.24963/ijcai.2026/860) frames acquisition as sequential decision-making / POMDP planning and groups methods into embedded cost-sensitive, model-based, model-free and hybrid families. It also highlights the need for realistic missingness/logging and stronger guarantees for non-myopic acquisition. This is important prior art: Craft should benchmark against those families, not claim the basic idea of sequential acquisition as novel.

The [UAI 2026 robust decision-focused learning paper](https://proceedings.mlr.press/v337/yamao26a.html) studies worst-case regret under measurement uncertainty and distribution shift using uncertainty sets. This is a relevant comparator for the model/loss robustness portion, but it does not by itself establish authenticated source admission, world-set completeness, authority or outcome provenance.

Task separation:
- **DSOP:** exact branch-wise decision identifiability without assigning outcome probabilities.
- **EIG:** expected uncertainty reduction under an admitted probabilistic model.
- **EVSI:** expected decision-loss improvement under an admitted loss and observation model.
- **Robust EVSI / robust decision-focused optimization:** worst-case decision value or regret over a declared model/uncertainty set.
- **Sequential EVSI / POMDP baselines:** adaptive acquisition policy under declared transition/observation probabilities and costs.

Do not compare these objectives through one blended score. Match the model assumptions, acquisition candidates, safety constraints, action set, cost budget and observation availability first. Where a baseline requires probabilities, DSOP's probability-free result is complementary rather than a numeric replacement.

### 14.2 Required DSOP controls

- An observation with one outcome for all worlds must not be called decision-separating when that branch retains competing optimal actions.
- A total observation mapping that yields different, branch-specific invariant actions is a successful adaptive separation.
- A genuine decision flip within any branch means universal separation fails.
- Missing mappings, unknown worlds or out-of-domain outcomes remain NotEvaluated or rejected; they are never coerced into a favorable result.
- Distinct outcomes do not prove independent sources. Shared material failure domains remain visible.
- A hard-unsafe action cannot enter a branch's robust-safe or identified set.
- Permuting world, action and observation order must preserve canonical output.

### 14.3 Metrics

Report universal branch-separation rate, identified-world count share, residual worst-branch minimax regret, branch count, incomplete-outcome-model rate, hard-safety exclusions, shared failure-domain overlaps, and acquisition cost/delay/burden independently. Keep zero-tolerance safety and authority metrics outside all aggregate performance scores.


## 15. Evidence-backed world exclusion and negative-claim assurance

Track [issue #6874 — Constraint Exclusion Certificate (CEC)](https://github.com/Luminous-Dynamics/mycelix/issues/6874) as the next proof-obligation layer above world-envelope CEGAR.

The benchmark must distinguish:
- a constraint that is syntactically well-formed;
- a constraint whose source artifact has integrity/authentication;
- a constraint whose source claim semantically supports the forbidden predicate;
- a constraint applicable to the exact object, population, time interval and accepted causal frontier;
- a constraint entitled to remove compatible worlds unconditionally;
- a constraint used only as a declared assumption;
- a constraint whose admission is Indeterminate, Rejected, NotEvaluated or UnsupportedByProfile.

A source receipt, digest, signature or inclusion proof does not on its own establish that the statement semantically excludes a world or that the source universe is complete. In particular, absence-based constraints require explicit authenticated source-universe and coverage evidence. Unadmitted constraints must not prune the world set.

CEC proposes the dispositions Established, Assumed, Indeterminate, Rejected and NotEvaluated/UnsupportedByProfile. Only Established constraints may support unconditional pruning within their qualified scope. Assumed constraints yield conditional world sets and conditional decision results. Indeterminate, Rejected, NotEvaluated and UnsupportedByProfile constraints retain matching worlds.

Adversarial measurements:
- false exclusion rate (zero on frozen hard negatives);
- retained-world recall against known truth;
- constraint-to-source-claim semantic support precision/recall;
- stale/revoked/out-of-scope constraint rejection;
- source-universe/absence-proof failures;
- historical envelope immutability under later corrections and revocations;
- additional corroboration cost to move from Assumed/Indeterminate to Established.

This is a non-compensatory assurance track. A single unsupported world exclusion can produce a false-safe decision and fails the qualification gate regardless of performance. CEC must compose existing source-trust, semantic claim-support, causal-frontier, identity, offline-verification and SCITT/VDS owners; it must not create a new signature or transparency format.
