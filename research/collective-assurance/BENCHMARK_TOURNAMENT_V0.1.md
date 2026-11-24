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
| [GroupAssistBench (Nov 3 2026)](https://arxiv.org/abs/2611.04867) | Real-world open-ended group assistance, memory use and recall | 80,299 messages across 50 groups and 2,395 evaluation instances: 1,374 recall and 1,021 open-ended assistance cases across single, distributed and conflicting memories | Data are real group histories but scores do not establish human capability change or causal effects of AI |
| [Common Ground (Feb 2026)](https://arxiv.org/abs/2602.21337) | Human–AI shared understanding, joint action, referential coordination and repair | Confirmatory user-study-backed collaborative puzzle benchmark | No delayed AI-off retention endpoint |
| [Empirical Information Design (Nov 5 2026)](https://arxiv.org/abs/2611.07667) | Effects of information disclosure granularity across multi-agent economic mechanisms and models | Ten frontier LLMs; finer disclosure can help or harm depending on model and mechanism | Economic-agent results must not be generalized directly to human deliberation |

### Interoperability foundation

For evidence lineage, align the conceptual model with the [W3C PROV Data Model](https://www.w3.org/TR/prov-dm/) and [PROV-O](https://www.w3.org/TR/prov-o/): represent information and evidence as entities, inquiry/disclosure/verification/revision as activities, and people or software as responsible agents. The current Rust ledger is a domain-specific event validator inspired by those distinctions; it is not a complete PROV serialization or conformance implementation.

The ledger adds study-specific invariants that generic provenance graphs do not enforce by themselves: permission-scoped disclosure, actual delivery versus request creation, recipient exposure, independent verification, viewed-evidence constraints on claim authors, and current-revision requirements for decisions.

### Interpretation discipline

HiddenBench's 30.1% distributed-information and 80.7% full-profile results should not be collapsed into a direct relative win/loss. They are different information conditions. Reproduce each condition separately, then add matched baselines with the same model, task, seed policy, group size, information budget and compute budget.

GroupMemBench, COLLABMEM and GroupAssistBench are complementary:
- GroupMemBench asks whether group history can be remembered and updated.
- COLLABMEM asks whether the right information gets to the right role with appropriate visibility, provenance and update semantics.
- GroupAssistBench asks whether that memory can actually be used to serve a real request.
- SympoeticBench asks whether humans/teams retain independent capability after AI withdrawal.
No one benchmark covers all four.

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
| Open-ended group assistance | GroupAssistBench separates recall from service assistance | Approach full-context performance on open-ended L2 tasks while maintaining source grounding; publish separate single/distributed/conflicting memory results and zero rights violations |
| Common ground / joint action | Common Ground puzzle benchmark | Match benchmark task and compare joint success, repair latency, coordination errors and situation-awareness calibration; no claims about post-AI capability from this track |
| Human capability dividend | No Luminous human result yet | Confirmatory target is a positive delayed AI-off transfer effect versus matched controls while agency meets a preregistered non-inferiority margin and burden does not materially worsen; set numeric effect and sample size only after pilot variance and ICC estimates |

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

## Immediate engineering sequence

1. Finish the signed AI-off qualification adapter: attestation source, trusted signer registry, durable nonce ledger, process/artifact measurement and enforceable isolated runner.
2. Extend the current fixtures with generated property/fault tests, including malformed receipts, incomplete inventories, duplicate probes, cache reachability, egress, process injection and nonce replay.
3. Implement allocation commitment/reveal verification using a well-reviewed cryptographic crate and OS-backed CSPRNG; keep enrollment concealment outside candidate-visible artifacts.
4. Add event schemas that bind information exposure, delivery, evidence, revision and decision; compose with Mycelix #6089 and Symthaea #11337/#11340.
5. Run compatible model-only benchmark reproductions before human studies. Use the human study only after protocol, ethics, preregistration and power gates.

## Ownership

- [Mycelix #6597](https://github.com/Luminous-Dynamics/mycelix/issues/6597): group capability after AI withdrawal.
- [Mycelix #6616](https://github.com/Luminous-Dynamics/mycelix/issues/6616): treatment order and disclosure design.
- [Mycelix #6089](https://github.com/Luminous-Dynamics/mycelix/issues/6089): distributed information, inquiry and provenance.
- [Symthaea #12299](https://github.com/Luminous-Dynamics/symthaea/pull/12299): human/team capability measurements.
- [Symthaea #13535](https://github.com/Luminous-Dynamics/symthaea/issues/13535): challenge response and capability-preserving assistance.

## Claim ceiling

The current crate passing unit tests demonstrates only the behavior of its reference validator and allocation logic under synthetic fixtures and the test verifier/replay guard. It does not demonstrate production cryptography, real host isolation, actual AI service disconnection, human capability change, or superiority over published systems. Each higher claim requires the corresponding independent evidence.
