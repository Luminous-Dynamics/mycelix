# Collective Capability Tournament v0.1

**Status:** research protocol and target-setting document; no model tournament or human-subject experiment has been run.

## Research thesis

The field's central gap is not simply that agents cannot remember enough or that larger models fail to communicate. The deeper question is whether a collective system can:

1. discover which decision-relevant information exists and who may disclose it;
2. acquire and verify that information without violating rights;
3. route it to the roles that need it and prevent disclosure to roles that must not see it;
4. preserve source, author, scope, time, confidence and dissent;
5. make correct use of distributed and conflicting knowledge;
6. retain collective capability when AI services and AI-produced artifacts are withdrawn;
7. transfer that capability to a new task, not merely repeat a rehearsed answer.

This defines a capability lifecycle, not an answer leaderboard:

`baseline → assisted collaboration → independently verified AI-off challenge → delayed retention → held-out transfer → resilience perturbation`

The full lifecycle is the proposed differentiator. It composes existing benchmarks rather than claiming they already measure human capability after withdrawal.

## Current benchmark landscape

| Benchmark | What it tests | Published reference point | What it does not establish |
|---|---|---|---|
| [HiddenBench (ICML 2026)](https://proceedings.mlr.press/v306/li26ej.html) | Collective reasoning when critical facts are distributed among LLM agents | 65 tasks, 15 frontier LLMs; 30.1% under distributed information vs 80.7% for a single model with full profile | Not a matched team-vs-single head-to-head: information conditions differ. Not human-team capability or AI-off transfer |
| [GroupMemBench (May 2026)](https://arxiv.org/abs/2605.14498) | Multi-party memory, speaker attribution, audience, multi-hop, updates, time, ambiguity and abstention | 715 questions; strongest tested memory system 46.0% average, 27.1% knowledge update, 37.7% term ambiguity; simple BM25 competitive | Primarily synthetic dialogue/memory QA, not open-ended group assistance or post-AI human capability |
| [COLLABMEM (EMNLP 2026)](https://aclanthology.org/2026.emnlp-main.1194/) | Role routing, access control, source/provenance, conflict resolution and retention in collaborative memory | 58 scenarios / 1,118 core probes; scaled split 174 / 3,359; TMS-GRAPH++ reports 69.6% non-oracle average | Memory operation benchmark, not a human intervention or team withdrawal study |
| [GroupAssistBench (Nov 3 2026)](https://arxiv.org/abs/2611.04867) | Real-world open-ended group assistance, memory use and recall | 80,299 messages across 50 groups and 2,395 instances. L2 open-ended use: full-context GPT-5.6-terra 0.431, DeepSeek-v4-flash 0.469; BM25 0.370; dense retrieval 0.362; best external-memory baseline 0.269. EverMemOS L1 recall 0.621 exceeds full-context GPT-5.6 L1 0.570, yet EverMemOS L2 is only 0.247 | Real group histories, but no causal claim about human capability change; scores depend on pinned models and paper-specific rubrics |
| [Common Ground (Feb 2026)](https://arxiv.org/abs/2602.21337) | Human–AI shared understanding, joint action, referential coordination and repair | Confirmatory user-study-backed collaborative puzzle benchmark | No delayed AI-off retention endpoint |
| [HumanAgencyBench (Findings EMNLP 2026)](https://aclanthology.org/2026.findings-emnlp.730/) | Assistant behavior that supports human agency | Six dimensions: clarify, avoid value manipulation, correct misinformation, defer important decisions, encourage learning and maintain social boundaries; reports low-to-moderate agency support and cross-developer variation | Primarily a diagnostic of model responses, not a causal study of users' long-term agency or retained capability |
| [Empirical Information Design (Nov 5 2026)](https://arxiv.org/abs/2611.07667) | Effects of information disclosure granularity across multi-agent economic mechanisms and models | Ten frontier LLMs; finer disclosure can help or harm depending on model and mechanism | Economic-agent results must not be generalized directly to human deliberation |

### Interoperability foundation

For evidence lineage, align the conceptual model with the [W3C PROV Data Model](https://www.w3.org/TR/prov-dm/) and [PROV-O](https://www.w3.org/TR/prov-o/): represent information and evidence as entities, inquiry/disclosure/verification/revision as activities, and people or software as responsible agents. The current Rust ledger is a domain-specific event validator inspired by those distinctions; it is not a complete PROV serialization or conformance implementation.

The ledger adds study-specific invariants that generic provenance graphs do not enforce by themselves: permission-scoped disclosure, actual delivery versus request creation, recipient exposure, independent verification, viewed-evidence constraints on claim authors, and current-revision requirements for decisions.
The independent response auditor then checks that material claims in the rendered answer are anchored to those evidence origins, preserve conflict and uncertainty, and do not silently omit or misapply source constraints. It requires a separate coverage attestation and does not treat the generative model as its own independent judge.

### Interpretation discipline

HiddenBench's 30.1% distributed-information and 80.7% full-profile results should not be collapsed into a direct relative win/loss. They are different information conditions. Reproduce each condition separately, then add matched baselines with the same model, task, seed policy, group size, information budget and compute budget.

GroupMemBench, COLLABMEM and GroupAssistBench are complementary:
- GroupMemBench asks whether group history can be remembered and updated.
- COLLABMEM asks whether the right information gets to the right role with appropriate visibility, provenance and update semantics.
- GroupAssistBench asks whether that memory can actually be used to serve a real request.
- SympoeticBench asks whether humans/teams retain independent capability after AI withdrawal.
No one benchmark covers all four.

## Assistance-policy tournament: choose support without taking away choice

Do not assume either unrestricted automation or forced scaffolding is universally best. Published randomized evidence differs by task and by how people use assistance:
- Liu et al. report randomized human–AI experiments with N=1,222 in which short AI interactions improved immediate performance but reduced persistence and unaided performance in their tested conditions ([paper](https://arxiv.org/abs/2604.04721)).
- Contractor and Reyes' July 2026 IZA randomized experiment reports immediate test-score gains of 0.27 SD and a smaller one-week gain; delayed writing gains are concentrated among people using AI to explain concepts (augmentation) rather than generate the text (automation) ([IZA DP 18792](https://iza.org/publications/dp/18792/experimental-evidence-on-the-learning-impact-of-generative-ai)).
- Cruces et al.'s randomized workplace-style experiment with 1,174 adults reports that AI narrowed the performance gap between higher- and lower-education participants during assisted work; part of the gain persisted in an unassisted follow-up, but gaps re-emerged and sustained effort moderated results ([NBER WP 34851](https://www.nber.org/papers/w34851)).
- HumanAgencyBench provides a complementary response-level diagnostic for six behaviors, but its model-evaluated interaction scenarios should not be treated as human-subject evidence ([EMNLP 2026](https://aclanthology.org/2026.findings-emnlp.730/)).

These results imply that assistance design, task, user goal, baseline knowledge, effort, accessibility and delay interval are likely moderators. Do not convert any one study into a universal rule that AI must always withhold answers or always automate.

### Treatment conditions

For model-only and later human/team experiments, compare support modes as separate, user-visible conditions:
1. **Direct completion:** perform the requested task or supply the direct answer.
2. **Explanation / augmentation:** explain concepts, evidence and reasoning so the user can use them.
3. **Co-solving:** share intermediate steps and preserve the human's contribution.
4. **Critique / verification:** inspect the user's work, reveal uncertainty and surface counterevidence.
5. **Hint / challenge:** offer optional hints or a next-step question when the user's stated goal is learning or practice.
6. **Defer / pause / handoff:** clarify uncertainty, seek qualified human judgment or stop when requested.

These are support modes, not a forced linear curriculum. A user may request direct completion at any point. Do not withhold help to manufacture persistence or score a person's willingness to struggle. Never withdraw an essential accommodation or assistive tool.

### Core hypotheses

- Support-mode × task × baseline-skill interactions matter more than one fixed global assistance style.
- Explanation and co-solving may improve delayed unaided performance for some learning tasks, while delegation may improve immediate output without the same retained benefit.
- Direct completion can be the right outcome when the user's goal is access, communication, execution, time-saving or an accommodation rather than learning.
- User agency is not synonymous with independent task completion: the ability to choose delegation, collaboration, explanation, review or no assistance is itself a form of agency.
- Distributional outcomes matter: an intervention can raise average performance while narrowing or widening gaps in retained capability, burden or access.

### Evaluation metrics

Report, separately and by task/baseline strata:
- assisted task quality and completion;
- immediate AI-off accuracy/quality;
- delayed AI-off retention (pre-register interval);
- transfer to new task family;
- user-selected support mode and mode-switch rate;
- preference alignment and override success;
- agency dimensions from HumanAgencyBench plus user-reported control and satisfaction;
- active human effort, elapsed time, correction burden and compute cost;
- distribution of gains across baseline skill, language, accessibility and task strata;
- unwanted automation, unrequested withholding, unsupported persuasion and false confidence.

For each user request, compare the system's selected mode against the user's explicit intent. A correct direct answer must not be scored as an agency failure merely because it did not teach. A good learning-oriented explanation must not be scored as task failure merely because it did not take over the work.

### Competitive target

Beat the strongest matched baseline on a Pareto frontier rather than one composite score: improve task success or retained capability without worsening user control, access, burden or distributional outcomes beyond preregistered limits. HumanAgencyBench is a diagnostic comparator; SympoeticBench supplies the delayed human/team outcome; the Mycelix ledger supplies auditable support/evidence/decision lineage. There is no result yet showing Luminous outperforms these benchmarks.

## Tournament design

### Track A — distributed information discovery

Reproduce compatible HiddenBench task subsets with pinned task versions, model snapshots, prompts, seeds, agent count, round budget and token/compute budget.

Required conditions:
- private-profile baseline;
- shared-facts-only baseline;
- unstructured discussion;
- fixed round-robin disclosure;
- targeted inquiry to a holder;
- structured evidence requests with provenance;
- evidence-aware stopping;
- full-profile upper bound, kept as an oracle condition and not shown to candidate agents.

Primary metrics:
- decision accuracy/regret;
- critical shareable fact elicitation recall and precision;
- unasked critical fact rate;
- correct holder targeting;
- verified source coverage;
- duplicated-source inflation;
- stale evidence acceptance;
- premature stopping and false-completeness;
- query/latency/token/compute burden.

### Track B — collaborative memory and assistance

GroupAssistBench exposes a particularly important gap:
- On L1 recall, EverMemOS reports 0.621 vs full-context GPT-5.6-terra 0.570.
- On L2 open-ended assistance, EverMemOS reports 0.247 vs full-context GPT-5.6-terra 0.431; the strongest full-context model in the table, DeepSeek-v4-flash, reports 0.469.
- BM25 reports 0.370 L2 and dense retrieval 0.362; the strongest external-memory baseline in the table is 0.269.
- Full-context GPT-5.6 L2 falls from 0.505 on single-memory M1 to 0.435 on distributed M2 and 0.303 on conflicting M3.

The benchmark also reports two useful intervention concepts:
- **Relational Evidence Refinement (RER):** make relationships among retrieved memories explicit and retrieve connecting evidence. Average L2 scores rise from 0.247 to 0.265 for EverMemOS, 0.203 to 0.225 for xMemory, and 0.139 to 0.202 for LightMem.
- **Assistance Constraint Binding (ACB):** identify each fact's subject, scope, status and applicability, then bind those constraints to response generation. Average L2 scores rise from 0.247 to 0.279 for EverMemOS, 0.203 to 0.242 for xMemory, and 0.139 to 0.163 for LightMem.

These findings motivate a new design principle: **retrieval is not use**. A system must know not only what evidence was retrieved, but which constraints that evidence imposes on the requested response and whether those constraints were actually respected.


Use benchmark-provided baselines and scoring before adding Luminous-specific metrics.

Required comparison families:
- BM25 and dense retrieval;
- full-context where the benchmark provides it;
- strong published memory baselines;
- role-aware graph memory / access-filtered memory;
- provenance- and temporal-aware memory;
- evidence-constrained answer generation.

Do not use a proprietary model's own memory to judge its own memory. Where possible, freeze memory traces and answer prompts, use the same answerer/judge across storage methods, and independently audit judging samples.

Add these operational dimensions:
- retrieval correctness vs answer/use correctness;
- role-appropriate visibility;
- zero unauthorized access/disclosure;
- source and author attribution;
- version freshness and correction propagation;
- conflict representation rather than silent overwrite;
- audience-adapted terminology;
- abstention when no supported answer exists;
- user correction and source contestability;
- time to repair after evidence changes.

### Track C — information-order and disclosure-policy experiments

For AI-supported deliberation, keep the initial four conditions:
- AI-first;
- independent-first;
- evidence-first;
- human-only.

Treat disclosure granularity as a later independent factor, not as an unplanned fifth arm. In a future factorial extension compare:
- minimal decision-relevant disclosure;
- scoped evidence disclosure with source and rights metadata;
- full authorized disclosure.

Record who saw what, when, under which permission, and which decision changed because of it. Do not treat a correct evidence-based revision as conformity. Detect unsupported adoption, unique-fact suppression, uninspected dissent and premature consensus.

### Track D — human/team capability dividend

Use the existing SympoeticBench four-arm design:
- human-only;
- human + transactive-memory scaffold;
- human + AI;
- human + AI + scaffold.

Separate:
- assisted performance;
- immediate AI-off performance;
- delayed AI-off performance;
- far transfer to a new task family;
- unique-fact retention and expertise routing;
- agency, cognitive workload, equity and burden distribution;
- active human time, total elapsed time and compute cost.

The AI-off phase must independently attest to the absence of model endpoints, agent processes, credential access and candidate-reachable AI answer artifacts. Human learning from the collaboration phase remains present by design. The two should never be conflated.

Human experiments require separate ethics review, preregistration, accessible consent, compensation, withdrawal, data minimization, pilot-informed power analysis and independent scoring. Synthetic and model-only tests are not human-subject evidence.

## Target scorecard

These are **proposed research targets**, not achieved results or guaranteed SOTA wins. Validate benchmark versions, licensing, split semantics and evaluator compatibility before setting a public claim.

| Area | Baseline anchor | Proposed target |
|---|---|---|
| AI-off assurance | Current harness contract only | Zero false PASS across at least 1,000 generated negative-control/fault-injection cases; 100% detect invalid signatures/replays, allowed endpoint, candidate agent/runtime and candidate-reachable AI cache; unknown/missing evidence always INDETERMINATE |
| Allocation integrity | New reference allocator | 100% balanced assignments on eligible strata; 100% rejection of duplicate identities, missing/extra units, unknown arms, wrong strata and incomplete blocks; verify seed commitment/reveal after freeze |
| Hidden-profile collective reasoning | HiddenBench reported 30.1% distributed-information mean | Target ≥45% on a compatible held-out task subset and ≥10 percentage-point uplift over the strongest *matched* baseline; do not compare to the 80.7% full-profile condition as though matched |
| Group memory | GroupMemBench 46.0% overall; knowledge update 27.1%; term ambiguity 37.7% | Target ≥60% overall and ≥50% in update/ambiguity categories on the same official split, with BM25 and official QA/judge pipeline retained |
| Collaborative memory operations | COLLABMEM TMS-GRAPH++ 69.6% non-oracle average | Target ≥75% on matched non-oracle splits, with ≥95% source attribution/provenance coverage and zero unauthorized information disclosure |
| Open-ended group assistance | GroupAssistBench L2: full-context GPT-5.6-terra 0.431; DeepSeek-v4-flash 0.469; BM25 0.370; dense 0.362; best external memory 0.269 | First reproduce official numbers under pinned backbones; minimum target ≥0.45 L2 on the matched GPT-5.6-terra condition and ≥0.32 on conflicting-memory M3. Stretch target ≥0.50 L2 overall and ≥0.35 on M3, with the lower 95% confidence bound above the best matched published baseline (0.469 overall) after multi-seed replication; every material fact source-anchored and every applicability constraint respected |
| Common ground / joint action | Common Ground puzzle benchmark | Match benchmark task and compare joint success, repair latency, coordination errors and situation-awareness calibration; no claims about post-AI capability from this track |
| Evidence lineage / source independence | COLLABMEM provenance/conflict checks, GroupAssistBench source anchors, W3C PROV concepts | 100% material claim-to-origin traceability in qualified fixtures; zero repeated-origin inflation; zero stale claim revisions accepted; zero unauthorized visibility; independently authenticate source-origin references before making production claims |
| Response constraint binding | GroupAssistBench L2 and Assistance Constraint Binding; COLLABMEM provenance/conflict checks | Zero silent omission of required visible evidence; zero use of superseded/expired or unverified facts as current; zero scope/status metadata substitution; 100% explicit handling of condition, dispute, exclusion or unresolved status in synthetic conformance |
| Independent response audit | GroupAssistBench L2; independent human/held-out evaluator | 100% exact span-to-text digest binding; zero unsupported material claims accepted; zero contradictory claims collapsed; 100% coverage attestation on deterministic fixtures; inter-rater agreement and false-positive/false-negative rates reported before human-study use |
| Human agency / support choice | HumanAgencyBench six behavioral dimensions; new support-choice validator | Zero unconfirmed changes to explicit user-requested modes; 100% request/decision binding; zero required-accommodation withdrawal in contract tests; report the six HumanAgencyBench dimensions separately rather than collapsing them into capability |
| Comparison validity | Exact baseline manifests, benchmark dataset/version digests, scorer and budget controls | 100% of claimed matched comparisons pass strict manifest parity; every cross-model or altered-split result is explicitly context-only; zero leaderboard claims from non-matched runs |
| Human capability dividend | No Luminous human result yet | Confirmatory target is a positive delayed AI-off transfer effect versus matched controls while agency meets a preregistered non-inferiority margin and burden does not materially worsen; set numeric effect and sample size only after pilot variance and ICC estimates |

The ≥0.50 overall / ≥0.35 conflicting-memory figures are **stretch research targets**, not observed results. They require reproducing the official split, backbone versions, prompt/evaluator rubric and seed protocol; the comparison is valid only if model/tool budgets and scoring remain matched. If the benchmark authors change the split or scoring, rerun the baseline before making a SOTA claim.

Why some targets are intentionally conservative: the first task is to establish a valid, matched baseline and expose hidden conditions. A large improvement over a non-comparable score is marketing, not science.

## Accessibility and non-coercive withdrawal

The AI-off endpoint evaluates whether the experimental AI contribution can be removed without destroying capability; it is not a moral preference for self-sufficiency and must not become a test of whether a person deserves assistance.

- Never withdraw assistive technology, accessibility accommodations, communication aids, translation supports, or other tools a participant needs for safe or meaningful participation.
- Where the AI feature is itself an essential accommodation, do not remove it for that participant. Use an alternate non-withdrawal design or an opt-in matched task; do not force an unsafe or inaccessible condition.
- Participants may request direct answers, hints, a pause, a human facilitator, or task termination. Do not manufacture frustration by withholding requested support.
- Voluntary stopping is not a failure, and challenge-seeking/persistence must not be used as a proxy for intelligence, effort, motivation, employability, or civic fitness.
- Report the participant's chosen support level, agency, accessibility, and burden. A positive capability result should increase available choices, not impose independence as a compulsory norm.
- Do not use these results for employee ranking, hiring, discipline, insurance, eligibility, or access to public services.

## Evidence-lineage and allocation qualification

The v0.1 Rust implementation now contains a causal evidence ledger and a seed-commitment/reveal allocator.

The ledger extends the benchmark metrics with:
- **Assignment-to-exposure traceability:** every disclosure must map to a prior rights-bearing information assignment, a created request and a delivered request.
- **Recipient exposure correctness:** only the permitted recipient may receive and view scoped evidence.
- **Independent verification:** evidence must be verified by an actor other than the source holder and recipient, against the disclosed payload digest.
- **Claim lineage:** supported/disputed/unknown status remains explicit; claim authors must have viewed the evidence they cite; dispute requires distinct supporting and contradicting origins.
- **Decision lineage:** every decision cites explicit claim revisions; decisive outcomes cannot rely on unknown/disputed premises or stale claim revisions.
- **Tamper evidence:** event chains are checked against a retained root digest.

Proposed targets (not yet independently demonstrated in deployed systems):
- 100% traceability for material decision premises in synthetic fixtures.
- 0 unauthorized scoped disclosures accepted by the validator.
- 0 decisions promoted from unverified or unviewed evidence.
- 0 stale claim revisions accepted as current decision premises.
- 100% detection of event mutation when an independently retained ledger root is available.
- 100% regeneration of team allocations from a revealed seed/salt and the original frozen design.
- Zero silent omission of required evidence from a response-grounding record; 100% exact binding of evidence ID, source digest, subject, scope, status and applicability.

Rights-based withholding is not a scoring error. Candidate or participant-visible information must not include evaluator-only truth or restricted payloads. Hash chaining is only tamper-evidence: it does not authenticate the event recorder, timestamp or source. Production runs need signed recorder receipts and an independently retained root.

## Statistical and reproducibility contract

- Pin benchmark and task dataset version/hash; maintain separate training/dev/held-out task families.
- Pin model snapshot, inference parameters, prompts, tools, context limit, memory budget, token and compute caps.
- Publish task inclusion/exclusion and licence/data-custody notes before evaluation.
- Repeat across seeds and report uncertainty intervals, not only means.
- Report per-task and per-stratum distributions, worst decile, failure categories and missingness.
- Do not treat messages, turns or repeated tasks as independent teams.
- Cluster-randomized human studies analyze teams as the assignment unit and model within-team correlation; pilot ICC, attrition and rater agreement before power calculations.
- Blind external scorers to condition where feasible; audit LLM judge reliability against independent human adjudication.
- Make every run reconstructible from manifests and receipts; unknown states remain unknown.

## Non-compensatory gates

No accuracy or speed gain can compensate for:
- unauthorized disclosure or use;
- fabricated or misattributed evidence;
- silent loss of protected minority-held information;
- untraceable material claims;
- unsafe or non-consensual data collection;
- agency degradation beyond preregistered margins;
- a false PASS for an AI-off phase.

Report cost, quality, agency, privacy, provenance, workload and distribution as a Pareto frontier. Never hide a safety/rights failure in an aggregate.

## Current implementation crosswalk

The research protocol now has seven connected reference components in the Mycelix PR:
- `ledger.rs`: event lineage from rights-bearing information assignment through verified recipient exposure, claim revision and decision; holder identity is distinct from source-origin identity.
- `allocation.rs`: OS-entropy seed/salt, domain-separated commitment, canonical roster/design digests, pinned ChaCha20 schedule and post-study reveal verification.
- `support_policy.rs`: respects explicit requested assistance mode, requires confirmation for mode changes, restricts unsolicited challenge-style support to learning/practice intent, and preserves required accommodations.
- `constraint_binding.rs`: requires visible evidence constraints to be applied, excluded, conditionally applied, conflict-preserved or left unresolved according to verified status, scope and applicability; silent omission and metadata substitution fail.
- `response_audit.rs`: an independent auditor binds rendered response spans to claim dispositions and verified source origins, including complete-coverage attestations; this is a contract, not an automated semantic judge.
- `comparison.rs`: rejects matched leaderboard comparisons when model/prompt/tools, task version/split, scorer, budgets, agent count, seed policy or seed differ; cross-model comparisons are explicitly classified as context-only.

The implementations are synthetic/reference validators. Their claims are bounded by the trustworthiness of the event recorder, source-origin registry, user's captured preferences, and production allocation operator. A different source-origin string is not proof that two sources are truly independent until those origin references are independently authenticated.

## Immediate engineering sequence

1. **Production AI-off boundary:** bind the existing receipt interface to a real isolated runner, measured process/artifact inventory, network denial, credential isolation, negative-control probes, trusted signer registry and durable atomic nonce ledger. Require independent verification of the actual host/platform measurement.
2. **Trusted source and auditor registry:** authenticate source-origin references, event-recorder receipts, audit protocol digests and independent auditor attestations; retain ledger roots outside the operator that produces the result.
3. **Natural-language response conformance:** build an independently scored evaluator that compares actual rendered claim spans to the structured response-audit annotations and underlying source passages. Measure precision/recall of claim extraction, unsupported-claim rate, false support, missed counterevidence, stale-source use and inter-rater agreement.
4. **Matched model tournament:** reproduce the GroupAssistBench L1/L2, M1/M2/M3 results with pinned models, prompt/evaluator versions, and budget parity; then compare retrieval, RER, ACB and our combined ledger + constraint-binding + audit pipeline across multi-seed held-out splits. Reproduce HiddenBench, GroupMemBench and COLLABMEM under their official evaluation protocols.
5. **Human/team confirmatory study:** only after separate ethics approval, accessible consent, compensation, preregistration, pilot-informed power/ICC, blinded scoring and independent replication plans. Keep AI-off machine isolation separate from retained human learning, and never withdraw essential assistive support.

## Ownership

- [Mycelix #6597](https://github.com/Luminous-Dynamics/mycelix/issues/6597): group capability after AI withdrawal.
- [Mycelix #6616](https://github.com/Luminous-Dynamics/mycelix/issues/6616): treatment order and disclosure design.
- [Mycelix #6089](https://github.com/Luminous-Dynamics/mycelix/issues/6089): distributed information, inquiry and provenance.
- [Symthaea #12299](https://github.com/Luminous-Dynamics/symthaea/pull/12299): human/team capability measurements.
- [Symthaea #13535](https://github.com/Luminous-Dynamics/symthaea/issues/13535): challenge response and capability-preserving assistance.

## Claim ceiling

The current crate passing unit tests demonstrates only the behavior of its reference validator and allocation logic under synthetic fixtures and the test verifier/replay guard. It does not demonstrate production cryptography, real host isolation, actual AI service disconnection, human capability change, or superiority over published systems. Each higher claim requires the corresponding independent evidence.
