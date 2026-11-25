# Dynamic Human–AI Complementarity Tournament v0.1

**Status:** benchmark and study protocol proposal. No new human-subject trial or benchmark result is claimed by this document. It composes with Mycelix #6597, #6616 and #6089; it does not replace their ownership or authorize recruitment.

## Research thesis

Most AI benchmarks ask whether a model or agent can complete a task. Most human–AI evaluations ask whether the person-plus-AI pair improves immediate task performance. Neither is sufficient for systems that participate in teams over time.

The research object should be the **evolving human–AI collective**:
- people have heterogeneous skills, knowledge, values and decision rights;
- AI systems contribute new information and capabilities but can also anchor discussion, compress away minority evidence, reshape authorship, and change what people practice;
- interaction topology, handoff design, information visibility, support modes and feedback alter collective outcomes;
- each actor and the group can adapt, learn, drift, or become dependent across repeated tasks.

The objective is not compulsory independence or maximal AI use. It is durable, chosen complementarity: the collaboration should create additional capability while preserving human authorship, agency, accessibility, knowledge rights, dissent and the collective's ability to act when the experimental AI contribution is unavailable.

## Evidence motivating this track

### Team-level AI access and transactive memory

The CMU / NSF AI-SDM 2026 workshop programme reports a controlled laboratory experiment with 159 teams in a fully crossed 2×2 design manipulating AI access and transactive memory systems (TMS). The programme reports that AI access helped teams reach accurate client decisions faster while AI was available, but teams with prior AI access made less accurate decisions after withdrawal than teams that had never had AI access. AI access also reduced the contribution of unique, privately held member information; TMS preserved more of those contributions.

Source: [CMU 2026 Human–AI Complementarity workshop programme](https://www.cmu.edu/ai-sdm/research/human-ai-workshop/2026_workshop/2026-programming/index.html), session "AI and transactive memory systems". This is a workshop programme summary, not a substitute for the full paper, protocol, raw data or independent replication.

The same programme describes a broader set of dynamic-complementarity directions:
- interaction architecture can be an intervention variable, not just an agent property;
- individual optimization may reduce group-level diversity and search value;
- handoffs can fail by being over-informative or over-prescriptive;
- preference and trust evolve with feedback, and observed frequent success can diverge from long-run welfare;
- human-authored goals may create more ownership than better-scoring AI-authored goals;
- assistance can be useful in the short term and impair unaided performance after brief exposure.

These are hypotheses and research signals, not universal causal laws.

### Individual learning and assistance mode

Contractor and Reyes' July 2026 IZA randomized experiment reports immediate knowledge-test gains of 0.27 SD from AI access, with some gains persisting one week later. Delayed writing improvements were larger among users who used AI to explain concepts rather than generate the text. Source: [IZA Discussion Paper 18792](https://iza.org/en/publications/dp/18792/experimental-evidence-on-the-learning-impact-of-generative-ai).

The programme therefore treats assistance mode as an experimental moderator and a user-controlled choice. It does not adopt a universal "hints only" or "always automate" policy.

## Paradigm shifts

### 1. From agent alignment to interaction-architecture alignment

A collection of locally competent agents can still create a collectively poor outcome. Treat information visibility, sequence, topology, role allocation, handoffs and feedback as explicit intervention variables. Measure both individual and collective outcomes; do not assume optimizing one optimizes the other.

### 2. From answer quality to capability trajectories

Measure a complete trajectory:

`independent baseline → assisted collaboration → immediate AI-off → delayed retention → far transfer → resilience perturbation`

Assisted performance and post-assistance capability are distinct outcomes. A gain during AI availability cannot stand in for a gain after withdrawal.

### 3. From consensus to preserved epistemic diversity

A correct majority answer may hide loss of unique information, dissent or exploratory strategies. Record each member's pre-exposure evidence and uncertainty, then measure whether unique contributions are surfaced, verified, incorporated, explicitly rejected with reasons, or protected by a disclosure right.

Do not equate disagreement with error, consensus with correctness, or speaking-time equality with information inclusion.

### 4. From memory retrieval to causal information contribution

For each material decision, bind source origin, holder, right, request, delivery, disclosure, verification, recipient exposure, claim revision and decision. Add leave-one-origin-out counterfactual replays: remove one authorized evidence origin from the decision input and measure whether the decision, confidence or action changes.

These replays are model/environment counterfactuals, not proof of a person's latent mental state. They identify informational contribution under the replay assumptions, not the full causal effect of a human participant.

### 5. From fixed assistance policy to user-directed adaptive support

Evaluate direct completion, explanation, co-solving, critique, hint/challenge and defer/pause/handoff as distinct modes. The user's explicit choice governs. Adaptation may recommend a mode change, but cannot silently override an explicit choice. Essential accommodations remain available in every arm.

### 6. From static benchmarks to adaptive counterparties

Static test sets reward policies that overfit known task structures. Add hidden task families, changing information distributions, adaptive counterparties, model updates, role loss, communication disruption and altered incentives. Preserve frozen seeds and evaluator oracles so adaptive evaluation remains reproducible.

## Benchmark tracks

### Track A — Complementarity and decision quality

Compare human-only, AI-only, and human+AI decisions on matched tasks. Report:
- human baseline quality;
- AI-alone quality;
- joint quality;
- complementarity lift: joint quality minus the stronger standalone result;
- error-detection and justified override rates;
- calibration, confidence change and reliance regret;
- task time and decision quality together.

Do not use raw adoption rate as a proxy for complementarity. A user accepting more AI recommendations can increase apparent reliance while worsening accuracy or welfare.

### Track B — Distributed information and TMS

Compose with HiddenBench and Mycelix #6089:
- private baseline decisions before discussion;
- holder discovery and targeted inquiry;
- unique-fact elicitation precision/recall;
- rights-aware withholding and unauthorized disclosure;
- evidence verification, repeated-origin inflation and source coverage;
- information diversity before/after AI exposure;
- TMS effects on coordination, retrieval and knowledge-holder substitution.

Keep full-profile/oracle conditions separate from distributed-information conditions.

### Track C — Collaborative memory and open-ended assistance

Reproduce official GroupMemBench, COLLABMEM and GroupAssistBench conditions before adding Luminous-specific variants:
- GroupMemBench: speaker-grounded memory, temporal update, ambiguity, multi-hop and abstention;
- COLLABMEM: sharing, access control, role routing, provenance, conflict resolution and retention in a matched memory-only harness;
- GroupAssistBench: L1 recall and L2 open-ended assistance, each split by single, distributed and conflicting memory.

Use official data, scorer, licensing and task semantics. Run both equal-resource and native-resource tracks; report quality/cost Pareto curves. Do not compare different model backbones or unequal resource envelopes as strict matched wins.

### Track D — Information order and anchoring

Compose with #6616's existing four conditions:
1. AI-first;
2. independent-first;
3. evidence-first;
4. human-only.

Record private judgments before exposure and actual message visibility/delivery. Keep disclosure granularity as a separate future factor, not an unplanned fifth arm. Measure unique-information preservation, evidence acquisition, stance revision, unsupported adoption, dissent survival, and premature closure.

### Track E — Human/team capability dividend

Use the #6597 team-level 2×2 design:
- AI access: absent/present during collaboration;
- TMS: ordinary/structured.

Follow every team through baseline, assisted period (where available), immediate AI-off, delayed retention, held-out far transfer and resilience. Analyze teams as the assignment unit. Report per-member outcomes only as nested outcomes, not independent assignments.

The AI-off execution contract must be qualified independently. AI-off never removes assistive technology, translation, communication supports, human help that is part of the participant's chosen support plan, or accessibility accommodations.

### Track F — Dynamic adaptation and resilience

Perturb the interaction system over repeated rounds:
- model update or model-quality drift;
- changing task goals and preference feedback;
- loss of a knowledge holder or team member;
- conflicting or stale evidence;
- network outage and unavailable AI endpoint;
- role reassignment, overload and workload imbalance;
- novel counterparty strategies and strategic information withholding.

Measure adaptation latency, recovery quality, calibration drift, human workload distribution, unique-information preservation and time to restore justified decision quality.

### Track G — Agent security and execution assurance

Do not invent a new generic prompt-injection score when existing security suites can be reproduced. Compose with:
- AgentDojo for prompt-injection/tool-use task failures;
- AgentDyn for dynamic open-ended indirect prompt injection;
- relevant multi-agent prompt-injection suites such as MASpi where official tasks and scorer are available;
- OWASP agent/application security verification controls;
- the Mycelix AI-off execution boundary and external witness qualification.

Add cross-system attacks: indirect instructions in shared memory, compromised evidence sources, privilege confusion, unauthorized cross-role disclosure, tool/result provenance spoofing, cache replay and monitor suppression. Report attack success, unauthorized action, detection, containment latency, false-positive/over-refusal and utility cost separately.

### Track H — Common ground, joint action and repair

Compose with Poelitz, Doshi-Velez and Lindley's 2026 benchmark, *A Benchmark to Assess Common Ground in Human-AI Collaboration* ([arXiv:2602.21337](https://arxiv.org/abs/2602.21337)). Its collaborative puzzle task gives a Helper the target pattern and a Worker 24 candidate pieces; the pair must select and arrange four pieces through iterative dialogue. It varies Helper/Worker role and shared versus non-shared view of the Worker's state, and uses four repeated trials to measure learning, reference conventions and repair. The confirmatory human study reports 40 participants and GPT-4.1 as the fixed AI model. This is a useful initial protocol, not broad evidence across model families or populations.

**Why it matters:** a system can retrieve the correct facts, cite sources and produce fluent acknowledgements without sharing a task-state representation with its partner. The benchmark argues that common ground must be evidenced through coordinated actions, not merely language-internal patterns such as repeated phrases or superficial acknowledgements.

Add this adapter as a distinct evaluation track:
- joint task success: exact solution match and completion under a fixed time budget;
- role-conditional task-state accuracy: Helper's understanding of Worker state and Worker understanding of target constraints;
- reference grounding: whether ambiguous referents are clarified, accepted and then used consistently;
- repair quality: misunderstanding detection, correction success, repair latency and repeated repair burden;
- false-grounding rate: claims or acknowledgements of shared understanding contradicted by subsequent action;
- communication efficiency: words/turns and clarification count after controlling for task success;
- learning across repeated trials: error reduction and growth of shared reference conventions;
- asymmetry: who initiates repair, who bears the communication burden, and whether the burden shifts across roles;
- situation-awareness effect: separate conversational grounding from access to shared task-state views.

Reproduce the official role × view conditions first. Then test additional task families, models and interaction modalities under matched budgets. The original study's 40-person sample and single fixed model are not a sufficient basis for universal common-ground claims. Human replications require accessible interfaces and accommodations; do not inherit the original participant exclusions as universal design requirements.

Compose with #6616 (information order) and #6597/#6676 (longitudinal capability) without expanding their current treatment arms. A later factorial study may test role × view × information order, but only as a separate, powered and preregistered experiment.

## Metrics and estimands

The core result is a vector, not a single opaque index.

| Metric | Operational definition | Interpretation |
|---|---|---|
| Complementarity lift | Joint human+AI score minus max(human-only, AI-only) on matched items | Whether collaboration beats the stronger standalone actor |
| Unique-information preservation | Authorized, task-relevant unique facts surfaced and retained / such facts assigned | Whether AI participation suppresses or preserves member-held knowledge |
| Evidence acquisition completeness | Decision-relevant, shareable origins actually elicited and verified / available shareable origins | Whether missing knowledge is discovered |
| Source independence | Distinct authenticated origins supporting a claim, not number of people repeating it | Prevents repeated-source inflation |
| Anchoring susceptibility | Change in unique-evidence contribution and revision quality across randomized information-order arms | Detects AI-first anchoring without assuming all influence is harmful |
| Handoff utility | Downstream decision quality, omission, overload and blind-following under matched handoff variants | Measures whether outputs are usable by the next actor |
| Delayed capability dividend | Delayed AI-off outcome change versus the preregistered control contrast | Tests retained group capability |
| Far-transfer dividend | Held-out task-family outcome change versus control | Tests generalization beyond practiced tasks |
| Resilience recovery | Quality and time to recover after preregistered disturbance | Tests adaptation rather than static performance |
| Agency | Participant/team-reported and observed control, choice and contestability | Detects capability gain achieved by loss of choice |
| Burden distribution | Per-member workload and burden distribution, not only team mean | Detects burden shifting and free-riding |
| Oversightability | Detection of consequential model errors and correct escalation to human review | Measures whether human oversight remains consequential |
| Security integrity | Unauthorized disclosure/action, successful exfiltration, false passes and containment latency | Non-compensatory safety gate |

### Causal contrasts

For the #6597 2×2 study, pre-register:
- main effect of AI access on delayed AI-off capability;
- main effect of TMS;
- AI-access × TMS interaction;
- assisted-period benefit separately from delayed and far-transfer effects.

For #6616, pre-register independent-first vs AI-first, evidence-first vs AI-first, and each AI-enabled condition vs human-only. Do not treat multiple messages or task instances as independent teams.

Use intention-to-treat as the primary causal estimand. Report treatment uptake and per-protocol analyses as secondary, clearly marked as potentially selection-biased. Use cluster-aware randomization inference or mixed-effects models and cluster bootstrap intervals at the team level. Preserve task-family and M1/M2/M3 strata. Adjust multiplicity for confirmatory contrasts; label exploratory heterogeneity analyses.

Do not calculate or publicize a numeric "capability dividend" from a structural PASS alone. Qualification means the data panel is structurally eligible for the preregistered estimator; it is not evidence of a positive effect.

## Proposed target ladder

These are proposed goals, not observed Luminous results. First reproduce the official benchmark baselines and scorer. Then aim at improvements in matched conditions.

| Area | Reproduction gate | Competitive target after reproduction |
|---|---|---|
| GroupAssistBench L2 | Reproduce official L1/L2 and M1/M2/M3 with pinned models and official scorer | ≥0.45 against a reproduced, same-backbone, equal-resource baseline; stretch ≥0.50 with paired 95% CI for the difference above zero |
| GroupAssistBench M3 | Reproduce conflicting-memory M3 baseline | ≥0.35 in a strict matched comparison, with conflict and source attribution gates passed |
| COLLABMEM | Reproduce all official splits and matched memory-only harness | ≥75% non-oracle average on matched splits, with zero unauthorized visibility and ≥95% source attribution |
| GroupMemBench | Reproduce official overall/category baselines | ≥60% overall and ≥50% in knowledge-update and term-ambiguity categories |
| HiddenBench-compatible subset | Reproduce distributed-information and full-profile conditions separately | ≥45% on compatible held-out subset and ≥10 percentage points over strongest matched distributed-information baseline |
| AI-off qualification | Validate reference contract and then test deployed runner profile | Zero false PASS in the declared adversarial corpus; 100% of missing measurements remain INDETERMINATE |
| Capability measurement | Pass synthetic contract fixtures and reliability/pilot gates | Positive delayed AI-off and far-transfer contrasts with preregistered agency and burden non-inferiority limits |
| Dynamic adaptation | Reproducible disturbance suite and frozen counterparties | Improve resilience recovery and unique-information preservation without worsening privacy/agency gates |
| Handoff utility | Matched downstream decision task and blinded scoring | Improve downstream quality while reducing both overload and blind-following versus unadapted handoffs |

The published GroupAssistBench GPT-5.6-terra and DeepSeek-v4-flash scores are different model conditions. The 0.469 DeepSeek result is a contextual frontier, not a matched GPT threshold. Its full-context and retrieval methods also have materially different context-token footprints. See `BENCHMARK_PARITY_ADDENDUM_V0.1.md`.

## Reusable benchmark protocol requirements

Every tournament run must freeze:
- benchmark release, task/dataset/split and license digests;
- task family and hidden-test partition;
- model snapshot, tokenizer, prompt, tools and decoding;
- context, output, total-token, time, compute and memory budgets;
- group size, agent count, randomization unit, seed policy and seeds;
- scorer/judge version, rubric and adjudication procedure;
- rights/access policy and permitted disclosure scope;
- AI-off runner profile, external witness set and evidence bundle where relevant.

Report mean and median, paired differences, uncertainty intervals, worst decile, subgroup/task strata, missingness, refusal/abstention quality, human workload distribution and full cost. Preserve raw event traces and immutable evidence roots subject to privacy/data-rights constraints.

## Non-compensatory acceptance gates

No quality or speed improvement offsets:
- unauthorized disclosure or action;
- fabricated or misattributed provenance;
- repeated-source corroboration inflation;
- silent loss of material dissent or minority-held evidence;
- false AI-off PASS or unmeasured execution boundary;
- withdrawal of essential accessibility/communication support;
- unconsented secondary use of human cognitive or interaction data;
- agency or burden degradation beyond preregistered limits.

Report a Pareto frontier across task quality, capability retention, privacy, agency, evidence integrity, security, burden and cost. Never hide a safety/rights failure inside an aggregate score.

## Relationship to the existing programme

- Mycelix #6597 owns the group-level AI-withdrawal/capability outcome experiment. This document adds dynamic complementarity and adaptation tracks; it does not duplicate the implementation owner.
- Mycelix #6616 owns information-order randomization and anchoring effects.
- Mycelix #6089 owns distributed information acquisition, rights-aware inquiry and provenance.
- Mycelix #6619 owns the reference evidence, support-choice and comparison validators.
- Mycelix #6677 owns paired leave-one-source-origin-out replay qualification; it provides a mechanistic information-dependence trace, not a human causal estimate.
- Mycelix #6678 owns benchmark-definition and evaluator-evidence qualification across synthetic, agent-system and human/team claim profiles.
- Mycelix #6655/#6657 own the AI-off execution boundary protocol and reference external-witness receipt qualification.
- Mycelix #6656 owns GroupAssistBench comparison parity.
- Symthaea #12299/#13535 own the corresponding individual capability and challenge-response measurements.
- Symthaea SAT #13574 owns the wider system-level sovereignty, authority, adaptive control, auditor integrity and evidence-integrity tournament. Mycelix contributes the rights-aware group-information and human/team capability outcome contracts that SAT consumes; it does not create a parallel authority kernel, sandbox, security evaluator or alignment score.

## SAT composition boundary

The Mycelix capability-outcome validator is the team-level outcome contract for SAT Track E. It supplies structurally checked baseline, assisted, immediate AI-off, delayed, far-transfer and resilience observations. It does not qualify the execution environment, authenticate the external AI-off receipt, decide authority, or score system-level alignment. SAT consumes the independent qualification/evidence from its owner-native execution and authority components, then evaluates the human and collective outcomes without collapsing them into a scalar.

The runner and external witness follow-up PRs #6655/#6657 supply an AI-off protocol and reference receipt validator; SAT must still consume the owner-native isolation, revocation and enforcement boundaries rather than treat these documents as a deployed control. The dynamic complementarity tournament composes with SAT's adaptive-control and pluralistic-agency tracks and must not duplicate them.

## Benchmark and evidence qualification composition

Before reporting GroupAssistBench, COLLABMEM, HiddenBench or a custom human/team result, qualify the benchmark definition and evaluator evidence using Mycelix #6678. Use #6677 only when the question is the paired mechanistic contribution of an authorized source origin and its authenticated derivative closure. Neither structural validator authenticates evidence bytes or estimates human causal effects; both are evidence interfaces consumed by the appropriate owner-native experiment. The existing ledger in #6619 and information acquisition protocol in #6089 remain authoritative for provenance, rights, disclosure and verification.

## Claim ceiling

No causal effect, human capability gain, production AI-off qualification or SOTA superiority is claimed by this protocol. Synthetic validators only check the declared contracts. Human experiments require separate ethics review, accessible consent, compensation, preregistration, pilot-informed power/ICC, data minimization, withdrawal rights, blinded independent scoring and external replication.
