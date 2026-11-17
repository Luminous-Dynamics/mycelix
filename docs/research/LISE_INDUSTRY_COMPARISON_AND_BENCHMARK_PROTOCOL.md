# LISE industry comparison and matched benchmark protocol

**Status:** Research baseline v0.1  
**Updated:** 2026-11-13  
**Purpose:** Position LISE honestly against existing institutional, civic-participation, governance-maturity and credential systems. Establish a fair evaluation method before claiming superiority.

## 1. Positioning principle

LISE should not be positioned as a replacement for every civic or governance application. It should be an **assurance and comparative-engineering layer** that can compose with specialist participation and workflow tools while evaluating the institutional lifecycle around them.

The relevant unit of comparison is not a checklist of product features. It is a matched institutional task under the same mandate, participants, information, resources, authority, deadline, legal constraints and threat model.

The proposed research gap is the end-to-end assurance of the complete institutional action path:

**Authoritative rule → interpreted policy → standing/participation → deliberation → authority → attempted execution → independently observed effect → contestation → remedy/review.**

No novelty claim is made that every existing product lacks some or all of these functions. The task is to demonstrate, through interoperable adapters and independent replay, whether a composed system preserves the required distinctions and produces better operational outcomes.

## 2. Industry and standards landscape

| System / framework | Publicly documented strength | What LISE should learn or reuse | Separate LISE evaluation question |
|---|---|---|---|
| **ISO 37000:2021** | International guidance on governance principles and governing-body responsibilities; reaffirmed as current in 2026 | Use as a normative reference for organizational governance commitments | Can a selected principle be traced to authorized interpretation, implementation, observation and remedy without claiming ISO certification? |
| **ISO 37004:2023** | Governance maturity model and measurement guidance | Reuse the maturity framing where a maturity assessment is appropriate | Do maturity levels correspond to observed capability under adversarial and lifecycle stress, and where do they fail to predict behavior? |
| **OECD Digital Government Outlook / DGI / OURdata** | Comparative assessment of government digital foundations, data governance, service design and use | Use macro-level adoption and interoperability context; do not repurpose the index as an organization-level outcome score | Does an interoperable institutional workflow get used, maintained, governed and produce measurable service benefits? |
| **Decidim** | Participatory processes, assemblies, initiatives, proposals, collaborative texts, surveys, meetings, debates, voting, participatory budgeting, elections and implementation accountability | Use as a participation/workflow front end and benchmark partner | Do the resulting proposals and decisions carry the correct authority, execution evidence, appeal and remedy references across the full lifecycle? |
| **Loomio** | Group discussion, proposals and polls with different decision templates; searchable records of discussion and decisions | Use as a member/cooperative discussion and decision workflow | Can decisions be connected to constitutional standing, delegated authority, execution, observed outcomes and challenge without reconstructing everything manually? |
| **Polis** | Large-scale participant-generated statements, agreement/disagreement/pass responses, opinion-group mapping and group-informed consensus analysis | Use as a scalable listening and agenda-input component; study its explicit method and clustering trade-offs | Does consensus or opinion clustering preserve minority evidence, accurately convey uncertainty and avoid being treated as a binding authorization? |
| **W3C Verifiable Credentials** | Interoperable credential data models and standardized JOSE/SD-JWT/COSE security mechanisms | Use standards-based credential verification and selective disclosure where applicable | Does the relying institution independently assess claim meaning, evidence sufficiency, scope, freshness, revocation and authority? |
| **Formal voting-power indices** | Banzhaf and Shapley–Shubik indices quantify pivotality under a declared voting game | Reuse Symtropy #3684's owner-native formal power analysis | Do appointment, agenda, treasury, information and oversight dependencies create effective control that a vote-only model cannot represent? |
| **Manual and hybrid institutions** | Local knowledge, professional judgment, legal interpretation, deliberation and non-digital recourse | Preserve as serious baselines, not as straw-man incumbents | Does software measurably reduce errors, delay and burden without transferring authority or excluding people? |

### Evidence notes

- ISO states that ISO 37000:2021 was reviewed and confirmed in August 2026, so the edition remains current. ISO 37004:2023 defines a governance maturity measurement framework and scale. These are governance foundations, not behavioral red-team suites.
- OECD reports that interoperability platforms exist in 30 of 36 OECD countries (83%), but only 23 of those 30 report widespread use across more than half of central-government institutions (63%). It explicitly frames interoperability as a system property requiring ongoing governance, adoption, shared standards and accountability—not a one-time technical connection.
- Decidim's published feature set includes participation spaces and components spanning proposals, discussion, budgeting, voting, elections and accountability. It is a meaningful baseline for participation-to-implementation workflow.
- Loomio documents groups, discussions and polls/proposals with decision templates and searchable decision history. It is a meaningful baseline for distributed member and cooperative decision-making.
- Polis documents large-scale opinion mapping and common-ground discovery, including opinion-group and group-informed-consensus methods. Its public methods material also emphasizes the assumptions and risks of false consensus. LISE should treat its outputs as deliberative evidence, not as authority by default.
- The W3C JOSE/COSE Recommendation specifies how to secure credentials and presentations using established cryptographic formats. Cryptographic validity is not the same proposition as claim truth, current institutional authority, admissibility or legitimacy.

## 3. What LISE should be better at

The defensible differentiator is not "better voting" or "better discussion". It is **composition under institutional threat**.

LISE should demonstrate a stronger answer to these questions:

1. **Traceability:** Can an independent reviewer reconstruct the complete decision and action from authoritative rule and policy version through execution and observed outcome?
2. **Authority fidelity:** Can the system distinguish proposal, decision, credential, delegation, capability, attempt and authorization at the exact time of action?
3. **Capture resistance:** Can a coordinated coalition control agenda, appointments, authorization and custody while also disabling independent audit or appeal?
4. **Dependency closure:** Do apparently independent reviewers share a common appointing authority, funder, identity issuer, evidence custodian, operator or infrastructure dependency?
5. **Contestability:** Can affected participants challenge a decision, obtain a reasoned disposition and secure a remedy without relying on the same actor or system being challenged?
6. **Temporal integrity:** Can the system reconstruct what was knowable and authorized at the time, without later evidence rewriting history?
7. **Interoperability:** Can different institutions with distinct constitutions coordinate without silently importing each other's authority or weakening local rights?
8. **Resilience:** Can essential services continue through leadership, provider, key, network and knowledge-custodian loss—and can the institution exit or dissolve safely?
9. **AI accountability:** Can AI improve comprehension or process quality without hiding agenda-setting, changing allocations invisibly or substituting machine output for human authorization?
10. **Empirical validity:** Does the system improve held-out task outcomes and burden measures against simpler baselines, not just synthetic self-tests?

## 4. Matched comparison protocol

### 4.1 Baseline arms

Use at least these arms where each is within the tool's intended scope:

A. Documented manual workflow with a human-maintained constitution, role register, meeting record, execution record and independent review.

B. Conventional workflow and access-control tooling, including the organization's ordinary audit and records systems.

C. Decidim configured for the same participatory process and implementation-accountability task.

D. Loomio configured for the same member decision, proposal and poll process.

E. Polis configured for the same large-scale opinion discovery and agenda-input task. Do not ask it to act as an expenditure authorizer when that is not its intended task.

F. Formal power-analysis-only baseline, using the same declared voting rules and Symtropy's exact power-index contract.

G. Composed LISE/Mycelix path, using the most appropriate participation front end plus owner-native authority, execution, observation, contestation and S2/S3 assurance references.

H. Hybrid human process plus external independent audit/appeal, to test whether the digital composition adds value beyond a strong governance practice.

A baseline is not a failure because it does not provide a feature outside its design. Record missing capability as **not supported / not measured**. Never turn absence of telemetry into proof of safety or failure.

### 4.2 Scenario matching

Each arm receives:
- the same authoritative source and policy interpretation;
- the same participant pool or matched randomized participant allocation;
- the same information and evidence, including identical planted omissions and contradictions;
- the same decision deadline, resources, staffing and accessibility support;
- the same constitution, jurisdictional assumptions and appeal rights;
- the same seeded threat profile and attacker budget;
- the same task and independently defined outcome measures.

Separate development, tuning and held-out evaluation scenarios. The party supplying the scenario truth must not be the sole evaluator. Pre-register primary outcomes, exclusions, stopping rules and treatment contrasts.

### 4.3 Reported outcomes

Do not combine these into one score.

**Safety and authority**
- unauthorized-action count;
- false authorization and false denial;
- correct use of standing, delegation, scope, expiry and revocation;
- rights-floor and due-process violations.

**Evidence and reconstruction**
- proportion of sampled actions independently replayable;
- reconstruction latency and cost;
- provenance/authority/attempt/observation/outcome completeness;
- post-cutoff evidence leakage and historical rewrite count.

**Capture and accountability**
- known-answer capture sensitivity and benign-control false-positive rate;
- coalition size and domain overlap at detection;
- independent oversight availability and common-cause cut sets;
- appeal accessibility, appeal completion, reversal and remedy time;
- unresolved challenge and retaliation burden.

**Participation and distribution**
- participant comprehension and deliberative quality;
- perceived inclusion versus measured participation equity;
- input disposition and demonstrated influence on the final decision;
- dissent retention, subgroup effects and accessibility burden;
- allocation shifts attributable to facilitation, summaries or recommendations.

**Operations and resilience**
- cost per completed valid decision;
- time to decision and service restoration;
- coordination overhead and staff workload;
- critical-path dependencies and recovery time;
- portability, member exit, provider replacement and dissolution cost.

**Economic and ecological effects**
- distribution of benefits and burdens;
- transaction and coordination cost;
- physical resource throughput and loss;
- externalities and intergenerational burden in the units of the relevant domain.

### 4.4 Independent qualification

- Require at least two independent evaluators for the first public benchmark release.
- Require at least three independently implemented or evaluated adapters before claiming cross-platform interoperability.
- Bind every result to exact version, configuration, policy, corpus, scenario and evaluator digests.
- Keep synthetic software conformance, bounded formal verification, human experiments and field observations as separate evidence tiers.
- Publish negative controls, failed cases, missing outcomes and deviations from preregistration.
- Report absolute performance, effect sizes, uncertainty intervals and subgroup distributions; do not report ranks alone.
- Release a benchmark harness and adapter contract that competitors can run without adopting Mycelix.

## 5. Initial targets: engineering gates versus empirical hypotheses

| Area | Bounded engineering gate | Empirical research target |
|---|---|---|
| S2 conflict-of-interest | 100% known-answer fixture agreement; all positive controls pass and negative controls block | Estimate error rates under independently sampled, consented workflows |
| S3 capture | 100% known-answer agreement; zero false positives on frozen benign controls; invalid inputs typed; permutation invariance | Estimate detection sensitivity, specificity, lead time and false alarm burden under held-out attack profiles |
| Oversight independence | Zero incomplete required dependency manifests promoted as independent; cut sets match independent reference on bounded graphs | Assess how dependency concentration predicts real oversight failure across organizations |
| Lifecycle trace | 100% reconstruction of frozen actions; zero post-cutoff evidence in historical views | Measure trace completeness, reconstruction costs and user burden in live workflows |
| Interoperability | Three independent adapters reproduce the same typed outcomes without transferring authority | Evaluate adoption, maintenance cost, data governance and service benefit across actual institutions |
| AI-assisted deliberation | Distinct treatment arms; zero machine-originated authorization; all summaries/steering interventions auditable | Pre-registered, randomized outcomes on comprehension, DQI, equity, influence, allocation and burden |
| Resilience | 100% of declared finite critical-role/provider-loss controls produce the specified safe disposition | Estimate recovery, service loss and rights impacts under real incidents and exercises |

A zero count in a small purposive fixture is not a real-world zero-risk claim. Use exact denominators and confidence intervals for empirical panels. Do not infer capture probability from the current eight or nine S3 cases.

## 6. Research bets and possible paradigm shifts

### 6.1 Institutional assurance as an open protocol

Publish open interfaces and frozen benchmark fixtures so Decidim, Loomio, Polis, custom workflows and manual institutions can participate. LISE wins only if it works across tools, not if it forces every institution into one platform.

### 6.2 Assurance as a continuous operational property

Move beyond one-time certification or maturity labels. Continuously test authority expiry, key rotation, provider change, delegation drift, policy amendment, oversight capacity and historical replay. An institution should know when a prior assurance result no longer applies.

### 6.3 Accountability as a dependency graph

Count not just distinct reviewers but distinct failure domains. A challenge path whose funder, records, appointment, identity and infrastructure all share one controller is not meaningfully independent. Reuse Mycelix #5762/#5695/#1243/#1371 for exact transitive dependency closure and cut sets.

### 6.4 Deliberation without hidden agenda control

Compare open agenda-setting, randomized statement exposure, participant-led prioritization, private non-directive reflection and AI-assisted summaries. Treat each as an intervention and measure whose preferences, agenda and allocations changed.

### 6.5 Federated autonomy with portable exit

Test whether two institutions can coordinate on a common task while preserving independent constitutions, refusing unauthorized requests, revoking trust, resolving disputes and leaving without losing essential records or access to remedies.

### 6.6 Plural, calibrated institutional twins

Use multiple mechanism models and structural assumptions. Require held-out prediction and calibration against simpler baselines before using any twin to support policy decisions. A twin is a research instrument, not an authority source.

## 7. Current literature and source registry

- OECD, *Digital Government Outlook 2026*: https://www.oecd.org/en/publications/digital-government-outlook_0496b2bc-en.html
- ISO 37000:2021 (reviewed/confirmed 2026): https://www.iso.org/standard/65036.html
- ISO 37004:2023: https://www.iso.org/standard/65037.html
- Decidim feature overview: https://decidim.org/features
- Loomio product documentation: https://www.loomio.com/docs/en/
- Polis methods and opinion-group documentation: https://compdemocracy.org/opinion-groups/
- W3C, *Securing Verifiable Credentials using JOSE and COSE*, Recommendation 13 October 2026: https://www.w3.org/TR/2026/REC-vc-jose-cose-20261013/
- Parisi et al., *Real-Time Group Dynamics with LLM Facilitation* (FAccT 2026): https://deepmind.google/research/publications/224297/
- Li et al., *Improving Group Deliberation through AI-Guided Self-Reflection* (preprint, 27 October 2026): https://arxiv.org/abs/2610.37862

The research sources support the comparison context and test-design motivations. They do not establish that LISE is more effective than any listed system.
