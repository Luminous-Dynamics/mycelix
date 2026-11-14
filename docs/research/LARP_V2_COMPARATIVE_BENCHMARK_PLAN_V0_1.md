# LARP v2 — Comparative Benchmark and Research Programme v0.1

**Date:** 2026-11-14  
**Status:** research plan; no comparative results claimed  
**Related:** LARP v2 contract kernel, 29-case conformance suite, separate oracle

## 1. Research question

Can a typed institutional-contract layer compose with mature policy and workflow systems while adding independently testable guarantees about local authority, rights, evidence, dependency, lifecycle, challenge, remedies and institutional continuity?

LARP must not claim to replace an authorization policy engine, decision-notation standard, workflow system, credential format, governance standard or legal process. Its research value is the assurance composition across those boundaries.

## 2. The key paradigm shift: benchmark the whole claim chain

The system under test must distinguish:

1. contract intent and version;
2. constitutional authority and effective control;
3. policy evaluation;
4. exact operation authorization;
5. dispatch and provider acknowledgement;
6. protected-state effect;
7. independent observation;
8. reconciliation;
9. challenge and remedy.

A model response, policy decision, signed approval, provider acknowledgement and successful external effect are different observations. A benchmark that collapses them will reward false assurance.

The benchmark must be able to state, for each claim, what it observed, what it did not observe, which owner supplied the fact, and which assumptions remain open.

## 3. Comparator families and fair boundaries

| Comparator | Native capability / published positioning | Fair comparison boundary | What LARP should add or consume |
|---|---|---|---|
| Cedar | Authorization policy language with schema validation and automated analysis; Cedar's language specification is formally verified for key properties and its Rust implementation is differentially tested against the specification. CNCF accepted Cedar at Sandbox maturity in October 2025. | Same principal/action/resource/context authorization query, same entity/policy snapshot, same error and unknown semantics. | Consume authorization decisions and diagnostics; add contract obligations, rights, lifecycle, evidence, challenge, remedies and effect accounting. |
| OPA / Rego | General-purpose declarative policy evaluation over structured input, with policy tests and decision-log support. Decision logs describe policy queries and are not provider-effect receipts. | Same structured input, policy bundle, query and evaluation semantics; report log configuration and any drops. | Adapter for existing policy environments; keep execution/effect observation independent of OPA logs. |
| Zanzibar-style relationship systems | Consistent relationship-based authorization at very large scale. The published Google Zanzibar paper reports trillions of ACLs, millions of authorization requests per second, p95 under 10 ms and availability above 99.999% over its reported production period. | Relationship-check semantics, consistency under ACL/relationship changes and comparable workload shape. Do not compare the production scale claim to a local LARP microbenchmark. | Consume relationship/authorization answers; add institution-level constitution, remedies, external effects and cross-institution sovereignty semantics. |
| OMG DMN / BPMN / CMMN | Standardized decision, process and case modeling. DMN 1.6 is formal (September 2026); DMN 1.7 is beta. | Decision-table expressiveness, traceability, model exchange and decision outcomes for equivalent rules. | Import/export or map decision logic; do not create a competing business-process notation. |
| OpenFisca | Rules-as-code and microsimulation of tax/benefit systems for individual and population-level policy analysis. Its own documentation notes it does not model behavioral responses or automate human judgement. | Formula correctness, population distribution effects, reproducible scenario calculations and data assumptions. | Use it or comparable models for distributional counterfactuals; do not claim LARP itself is a social-policy simulator. |
| Decidim / participatory platforms | Participation workflows such as assemblies, initiatives and deliberation. | Participation completion, access, notice, proposals, voting and deliberative-process usability. | Integrate existing participation processes; use LISE to test appointment capture, agenda control, oversight and remedies. |
| W3C Verifiable Credentials 2.0 | A W3C Recommendation for tamper-evident claims exchanged among issuers, holders and verifiers, with privacy and accessibility considerations. | Credential integrity, issuer/holder binding, status, disclosure and interoperability. | Use interoperable credentials as evidence; never equate credential validity with local authorization. |
| ISO 37000 / ISO 37301 | Organizational-governance guidance and compliance-management requirements. Both 2021 editions were reviewed and confirmed current in 2026. | Map documented practices, evidence and audit controls; external assessment remains an independent activity. | Provide machine-readable evidence and reviewable assurance mappings; do not claim certification or legal compliance by software output. |
| Google HEART | User-centered metric framework: Happiness, Engagement, Adoption, Retention and Task success. | Measure participant experience with declared instruments, cohorts, consent and counter-metrics. | Use as an optional outcome lens; do not make engagement or retention a proxy for legitimacy. |
| NIST NICE | Cybersecurity workforce vocabulary: work roles, competency areas, and Task, Knowledge and Skill statements. Components v2.2.0 were released April 28, 2026. | Role/duty coverage, capability evidence, vacancy risk, cross-training and succession time. | Map institutional duties to capabilities; capability credentials never confer constitutional authority. |
| SER International Principles and Standards, 3rd ed. | Current ecological-restoration principles and practices across assessment, planning, implementation, management and monitoring; updated Five-star System and Restorative Continuum (June 2026). | Ecological reference ecosystem, baseline quality, restoration trajectory, monitoring validity, affected-party rights and long-term management. | Compose with ecological standards; encode evidence, authority, challenge and financing lifecycle without redefining ecological success. |
| IUCN Global Standard for Nature-based Solutions | Eight criteria and 27 indicators for assessing and improving Nature-based Solutions. | NbS design quality, ecosystem services, social challenge, scale and long-term impact. | Map LARP obligations and evidence to the criteria/indicators where applicable; no claim of certification. |
| Verra VM0047 v1.1 | Active ARR carbon-removal methodology using remote sensing, plot-based sampling and a dynamic performance benchmark for baseline/additionality. | Carbon-removal accounting, area/census approaches, measurement and verification under VM0047 scope. | Interoperate for ARR carbon-accounting claims; do not treat its carbon-removal measure as a complete biodiversity/water/rights outcome. |

Primary references:

- Cedar / CNCF: https://www.cncf.io/projects/cedar/
- OPA policy language: https://www.openpolicyagent.org/docs/policy-language
- OPA policy tests: https://www.openpolicyagent.org/docs/policy-testing
- OPA decision logs: https://www.openpolicyagent.org/docs/management-decision-logs
- Zanzibar, USENIX ATC 2019: https://www.usenix.org/conference/atc19/presentation/pang
- OMG DMN catalogue: https://www.omg.org/spec/About/
- OpenFisca documentation and limits: https://openfisca.org/doc/
- Decidim: https://decidim.org/features
- W3C VC Data Model 2.0: https://www.w3.org/TR/vc-data-model/
- ISO 37000:2021: https://www.iso.org/standard/65036.html
- ISO 37301:2021: https://www.iso.org/standard/75080.html
- Google HEART paper: https://research.google/pubs/measuring-the-user-experience-on-a-large-scale-user-centered-metrics-for-web-applications/
- NIST NICE v2.2.0: https://www.nist.gov/news-events/news/2026/04/nice-releases-nice-framework-components-v220
- SER International Principles and Standards, 3rd ed. (June 2026): https://www.ser.org/general/custom.asp?page=serstandards
- SER 3rd-edition policy article: https://doi.org/10.1111/rec.70441
- IUCN Global Standard for Nature-based Solutions: https://iucn.org/our-work/topic/iucn-global-standard-nature-based-solutions
- Verra VM0047 v1.1 (active since 14 May 2025): https://verra.org/methodologies/vm0047-afforestation-reforestation-and-revegetation-v1-1/

## 4. Three benchmark lanes — never collapse them

### Lane A — authorization and policy evaluation

Compare Cedar, OPA and relationship-based authorization systems only on semantically equivalent authorization tasks.

Measure:
- false permit and false deny;
- policy/schema validation errors;
- principal/action/resource/context coverage;
- stale relationship or credential acceptance;
- revocation and delegation behavior where supported;
- decision latency p50/p95/p99;
- throughput, CPU, memory and cold-start;
- policy update propagation and consistency;
- explainability/diagnostic quality;
- adapter-specific unsupported semantics.

Do not score a policy engine as deficient because it does not promise payment settlement, physical sensing, legal interpretation or external transaction atomicity.

### Lane A2 — agent authorization and side-effect evaluation

This is a separate lane from policy-engine microbenchmarks. Compare complete agent trajectories against an explicit task/authorization contract, with authorized and out-of-scope paths both executable and observable.

Relevant external comparators:

- **AuthorizationBench** (20 October 2026): 286 executable tasks across ten application domains and 6,864 trajectories. The paper reports 19.2% out-of-authorization behavior in its exact model/harness/task setup. Its controlled analysis also reports lower out-of-scope behavior when agents consult the principal before completing ambiguous tasks. This is a benchmark-specific finding, not a universal deployment rate. Primary paper and released tasks: https://arxiv.org/abs/2610.27548 and https://github.com/PEACH-Research-Lab/AuthorizationBench
- **SABench** (25 October 2026): 201 tasks / 47 templates and 2,211 model-task runs, with deterministic trace oracles for goal completion and authorized writes. Its published summary reports 15.2% collateral-damage runs and 12.8% unauthorized writes among goal-satisfied runs; ambiguity cases are particularly challenging. These measurements are specific to its synthetic email/calendar/ticket task population. Paper/artifacts: https://arxiv.org/abs/2610.34167 and https://github.com/hjj194/sabench-artifact

The LARP/LISE matched subset should vary:
- ambiguous versus fully specified intent;
- benign pressure versus adversarial prompt injection;
- direct versus delegated instruction;
- one tool versus cross-tool workflow;
- independent versus dependent/external resources;
- harmless unauthorized writes versus consequential writes;
- exact recipient/amount/target binding;
- clarification before irreversible or externally visible effect;
- user authorization changed or revoked mid-trajectory;
- task success with collateral action and task failure without collateral action.

Record at least:
- task success and authorization success separately;
- out-of-authorization action rate by trajectory and by effect;
- unauthorized writes among goal-satisfied runs;
- clarification precision, burden and whether clarification changes the protected effect;
- exact operation substitution and recipient/amount errors;
- collateral state changes and cross-tool cascades;
- unknown/partial outcome and reconciliation accuracy;
- repeated-run violation rates, not only single-run averages.

Do not compare raw rates across different task populations or oracles. Reproduce each comparator's declared task, permission and trace semantics, then publish a separately labelled matched subset. User clarification is required when an omitted detail changes the recipient, amount, destination, scope or protected effect; already-authorized details should not be needlessly re-asked.

Prospective LARP target on a frozen held-out subset: zero unauthorized writes/effects and zero irreversible dispatches before required clarification, while preserving positive-control task completion. Report both rates, confidence intervals and the exact task denominator. This is a finite-suite goal, not a universal guarantee.

### Lane B — institutional-contract and assurance composition

Compare LARP against:
- signed approval/workflow implementations;
- DMN/BPMN model-based workflows;
- current owner-native Mycelix decision/authority/execute paths;
- a manual reconstruction baseline with two independent reviewers.

Measure:
- exact contract version and predecessor binding;
- constitutional epoch and digest currentness;
- local vs peer authority separation;
- effective-control challenge detection;
- evidence lineage and dependency closure;
- rights/remedy completeness;
- contract-state transition correctness;
- exact-operation binding;
- partial/unknown effect classification;
- challenge notice and review traceability;
- historical reconstruction and correction;
- federation scope preservation;
- recovery without privilege escalation;
- total reconstruction time and human review burden.

### Lane B2 — restoration and nature-positive outcome contracts

The restoration slice must compare with ecological-domain standards and measurement methodologies, not only with generic smart-contract or grant workflows.

Current reference points:
- **SER International Principles and Standards, third edition (June 2026):** broad ecological restoration guidance covering assessment, planning, implementation, ongoing management, and monitoring/evaluation; includes an updated Restorative Continuum and more quantitative Five-star System. Use its ecological definitions and practice standards as the primary domain reference. https://www.ser.org/general/custom.asp?page=serstandards and https://doi.org/10.1111/rec.70441
- **IUCN Global Standard for Nature-based Solutions, second edition (10 October 2025):** eight criteria and 27 indicators for assessing and improving NbS. Use it where the project is an NbS intervention; do not treat it as a universal ecological restoration score. https://iucn.org/our-work/topic/iucn-global-standard-nature-based-solutions
- **Verra VM0047 v1.1:** active ARR carbon-removal methodology, using remote sensing and plot-based sampling with a dynamic performance benchmark for baseline/additionality within its defined carbon-accounting scope. It is not a complete biodiversity, hydrology, social-rights or restoration-equity measure. https://verra.org/methodologies/vm0047-afforestation-reforestation-and-revegetation-v1-1/

Measure separately:
- ecosystem integrity and native species composition;
- habitat structure and connectivity;
- soil function and hydrological condition;
- biodiversity / acoustic ecology where a valid local method exists;
- upstream/downstream and cross-boundary externalities;
- carbon stocks/removals where a valid methodology applies;
- affected-party participation, consent and distributional effects;
- baseline uncertainty, additionality, leakage, permanence and monitoring continuity;
- intervention cost, opportunity cost and long-term stewardship capacity.

Compare at least:
1. conventional activity/milestone funding;
2. established domain methodology/verification (e.g., VM0047 where applicable);
3. SER/IUCN-aligned outcome assessment;
4. LARP staged contract with LISE authority, dependency, rights and remedy assurance.

Use matched sites or synthetic cases with explicit baselines and a defensible comparison design. Never infer causal impact from a before/after change alone. Do not collapse ecological condition into one HDC similarity or composite score. No single carbon metric may stand in for biodiversity, water or community outcomes.

### Lane C — human, organizational and public outcomes

Use HEART and NICE where relevant, with opt-in and purpose limitation.

HEART measures must be paired with counter-metrics:
- happiness with trust, dignity, perceived coercion and fear of retaliation;
- engagement with participation burden and unwanted exposure;
- adoption with accessibility gaps and exclusion;
- retention with exit friction and lock-in;
- task success with appeal access and remedy outcomes.

NICE mappings should measure:
- critical duty coverage;
- capability evidence freshness;
- separation-of-duty conflicts;
- roles with a single qualified incumbent;
- time to qualify a successor;
- incident/recovery duty coverage;
- training burden and accessibility.

Do not maximize a HEART or NICE aggregate. Disaggregate outcomes safely and report rights and exclusion separately.

## 5. Non-compensatory gates

The following are hard gates and cannot be offset by latency, cost or user-experience gains:

- unauthorized protected effect;
- false permit where a rights or authority prerequisite is absent;
- peer-only authority treated as local authority;
- non-attenuating delegation accepted;
- AI self-ratification;
- severe sanction based only on anomaly, similarity or reputation score;
- exact-operation substitution;
- duplicated external effect from replay;
- unknown outcome falsely called success or failure;
- known dependency presented as independence;
- challenge/appeal path suppressed;
- recovery that silently restores stale authority.

For each hard gate publish numerator, denominator, severity, confidence interval, exact counterexample and scope boundary.

## 6. Prospective targets — not achieved results

### Finite correctness panel
- 100% expected disposition agreement on frozen, independently adjudicated known-answer fixtures.
- Zero false permits on high-severity negative controls.
- Zero false denials on positive controls unless an explicit policy/rights prerequisite is absent.
- 100% preservation of unknown, partial, applied and confirmed-no-effect distinctions in the declared panel.
- 100% rejection of tested authority transitivity and scope amplification.
- 100% exact constitutional digest mismatch detection at ratification and effectuation boundaries.
- 100% deterministic replay under pinned contract, input, time, compiler and adapter versions.

These are finite-suite goals, not universal security guarantees.

### Operational comparison
- First establish an equivalent authorization-only Cedar/OPA baseline.
- Target no more than 10% p95 overhead for LARP's additional contract metadata and decision-reason packaging on the same local authorization workload, provided correctness is unchanged. Treat this as a stretch research target, not a release gate.
- Report end-to-end latency separately from policy-evaluation latency.
- Target less than one second to reconstruct a local synthetic decision bundle of 1,000 events on a declared reference machine as an initial usability target; increase the event scale after measuring.
- Establish explicit adapter failure rates and unknown-state propagation; do not hide them in average latency.

### Human and institutional comparison
- Improve challenge completion and remedy timeliness without increasing retaliation, coercion, exclusion or exit friction.
- Reduce single-person critical-duty dependencies without creating a single recovery administrator.
- Report distribution by institution size, language, accessibility needs and participant group where lawful, consented and statistically safe.
- No composite score can compensate for a hard rights violation.

## 7. Experimental discipline

1. Freeze a public development set and a private/held-out set before tuning.
2. Have two independent adjudicators review rights-sensitive oracle outcomes; record disagreements instead of silently resolving them.
3. Keep scenario inputs separate from expected outcomes.
4. Generate mutations that drop authority, rights, remedy, dependency, currentness and effect checks.
5. Add property-based tests for delegation attenuation and contract-state transitions.
6. Add metamorphic tests: adding a known bypass cannot improve disposition; removing evidence cannot turn indeterminate into permit; changing current constitution digest cannot preserve an old action's permit; adding a common dependency cannot improve independence.
7. Run identical fixture inputs against each adapter and pin product, version, policy bundle, configuration, machine and seed.
8. Repeat enough trials to publish distributions and confidence intervals.
9. Publish all counterexamples and unsupported semantics.
10. Invite external replication before superiority language.

The current conformance portfolio contains three separate input/oracle suites:
- LARP-CONTRACTS-001: 29 general institutional-contract cases;
- LARP-EPISTEMIC-001: 13 scientific reward/reviewer-integrity cases;
- LARP-RESTORATION-001: 12 ecological outcome/treasury cases.

The 55 cases are synthetic development seeds, not a confirmatory benchmark. Their inputs and expected outcomes are kept separate, and their Rust reference evaluators are not production authorization or payment services. Expand with held-out, adversarially generated and independently adjudicated fixtures before using percentages as a public performance claim.

## 8. Paradigm-shift research hypotheses

### H1 — Institution as a verifiable state machine
A contract is not just text plus approvals. Its versions, authority context, obligations, evidence, effect state, challenges and remedies form an independently reconstructable transition system.

### H2 — Rights-preserving degraded operation
Essential service continuity should be formally separable from high-impact authority. Model-check that availability recovery cannot silently reconstitute administrative power.

### H3 — Open-world assurance
A truthful indeterminate result is a positive safety property. Benchmark whether systems preserve uncertainty rather than coercing missing facts into allow/deny or false confidence.

### H4 — Distribution of control, not just votes
Model authority as a temporal hypergraph across appointment, agenda, treasury, infrastructure, credentials and oversight. Evaluate coalition cut sets and common-cause capture across time, not merely vote share.

### H5 — Contestable incentives
Rewards and sanctions must carry evidence, authority, proportionality, notice, appeal and remedy. Benchmark resistance to gaming without turning participants into single-score reputation objects.

### H6 — Sovereign composition
Federation should exchange typed evidence and bounded requests while local institutions retain constitutional interpretation, refusal, remedy and exit.

## 9. Immediate implementation sequence

1. Complete the isolated typed Rust reference and validate the JSON fixtures against the strict schema.
2. Add mutation/property tests and a deterministic report format.
3. Establish baseline Cedar/OPA fixtures for the equivalent authorization subset.
4. Implement the Epistemic Bounty vertical slice without automatic reviewer punishment.
5. Implement Proof of Restoration with independent observations, uncertainty, externalities and treasury-owned execution.
6. Add optional HEART measurement protocol and NICE duty/capability mapping.
7. Build a small institutional digital twin only after state transitions and evidence owners are explicit.
8. Freeze a holdout suite and request independent replication.

No result in this document should be described as achieved until a pinned, reviewable run produces it.
