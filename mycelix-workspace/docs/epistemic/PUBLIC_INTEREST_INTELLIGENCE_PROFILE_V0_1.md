# Public-Interest Intelligence Network — Mission and Evaluation Profile v0.1

**Status:** proposal / institutional and evaluation profile only  
**Date:** 2026-10-09  
**Related program maps:** Mycelix #2625 (EPI-OSINT-000), Mycelix #2938 (OPS-INTEL-000), Mycelix #2945 (comparative capability matrix)  
**Purpose:** define how the existing Mycelix/Symthaea evidence and reasoning architecture could grow into a distributed public-interest intelligence institution without creating a duplicate ontology, a centralized truth authority, or an unaccountable surveillance system.

This document proposes a mission, operating model, staging plan, and evaluation profile. It changes no runtime behavior and grants no collection, disclosure, or action authority.

## 1. Executive thesis

Build an **Intelligence Commons**: a federation of independent people, research groups, civic institutions, domain specialists, and software nodes that can investigate public-interest questions together while preserving the origin, limitations, uncertainty, and contestability of every important conclusion.

The ambition is not to copy the CIA, Five Eyes, or a commercial intelligence platform. It is to demonstrate measurable superiority on carefully selected tasks where a decentralized network has structural advantages: public-evidence reproducibility, independent challenge, local and multilingual participation, resilient operation across organizational boundaries, transparent correction, and privacy-preserving exchange.

A broad claim such as “better than the CIA” is not an evaluable engineering target. Classified collection, clandestine sources, state resources, policy mandates, and operational capabilities are not publicly observable on comparable terms. We must make **versioned, task-specific claims** and publish enough evidence for others to reproduce or dispute them.

The initial product should be described as a **public-interest intelligence network**, not a government-agency replacement. Its first remit is bounded public-source analysis and defensive resilience. It does not include clandestine collection, unauthorized access, covert action, generalized surveillance, or intelligence-led targeting of people.

## 2. Mission and constitutional invariants

The mission is to help people and institutions understand consequential events earlier and more accurately, recognize what remains unknown, and coordinate lawful, proportionate responses.

The following invariants are constitutional constraints, not soft product preferences:

1. **Evidence before assertion.** A report must link important claims to exact evidence objects and selectors, explain derivations, preserve counter-evidence, and identify what would falsify its conclusions.
2. **Uncertainty is data.** “Unknown,” “insufficient evidence,” “conflicting,” “stale,” “not independently corroborated,” and “indeterminate” are valid outputs, not failures to be concealed.
3. **No central truth oracle.** No model, operator, institution, reputation score, vote, or majority can turn a claim into truth merely by asserting it.
4. **Independence is demonstrated, not counted.** Ten articles derived from one wire report are not ten independent observations. Dependency is a scoped evidence-graph property.
5. **Reasoning does not grant authority.** Symthaea can propose hypotheses, forecasts, investigations, and explanations. Those outputs do not themselves authorize collection, publication, disclosure, intervention, or execution.
6. **Cryptographic integrity is not factual truth.** A valid signature or receipt can establish a bounded integrity/authorship statement; it cannot establish that the signed proposition is true.
7. **Privacy follows the data.** Public availability is not unlimited aggregation authority. Sensitive information retains purpose, minimization, access, retention, and redaction constraints through derived outputs.
8. **Correction preserves history.** New evidence can revise the present assessment but must not rewrite what was knowable at an earlier evidence frontier.
9. **Dissent remains inspectable.** Material disagreement is preserved with its basis. A governance vote may settle a policy question but cannot rewrite an empirical evidence record.
10. **Public benefit is the default.** The network should not optimize for engagement, political persuasion, organizational prestige, or the volume of published alerts.

The canonical distinctions already set out by EPI-OSINT-000 and EPI-000 remain authoritative for evidence semantics. This document does not create a second set of evidence types.

## 3. What decentralization should make possible

These are hypotheses to measure—not claims already demonstrated.

### 3.1 Strong candidate advantages

- **Independent reproduction:** different organizations can verify a public report from its cited evidence without trusting the original publisher's private database.
- **Plural local context:** local researchers, language communities, domain experts, and affected communities can contribute contextual knowledge without becoming subordinate to one central analysis desk.
- **Federated continuity:** a regional or organizational node can continue operating during a network partition, then reconcile evidence and disagreement without silently erasing local history.
- **Visible provenance:** readers can trace a conclusion from summary to claim, exact artifact, relevant passage or selector, acquisition context, and transformation lineage.
- **Fast correction:** an affected source, analyst, or member can submit a counterexample and see which claims, reports, and downstream projections depend on it.
- **No single institutional veto over public evidence:** other nodes can preserve an independently supported assessment, while clearly marking conflict, access limitations, and confidence.
- **Privacy-preserving collaboration:** a node can publish an authorized derived conclusion or opaque commitment without exporting all underlying data.
- **Low-cost replication:** small groups can reuse published protocols, fixtures, adapters, and evaluation corpora rather than rebuilding an entire intelligence stack.

A centralized agency could adopt some of these practices. The distinctive bet is that decentralization makes independent participation, verifiability, and exit properties of the system rather than discretionary exceptions.

### 3.2 Capabilities we must not pretend to have

Decentralization does not automatically supply classified sources, human-source recruitment, privileged access, language expertise, reliable sensors, subject-matter competence, or high-quality judgment. It can also increase noise, coordination cost, Sybil manipulation, and adversarial exposure.

No “global superiority” claim is justified until real results establish it under a disclosed workload and evaluation protocol.

## 4. Institutional structure: a federation of bounded cells

Start with a small number of interoperable research cells, not a single command hierarchy and not an unmoderated public swarm.

### 4.1 Cell roles

- **Local/source stewards** preserve the context and permitted use of source artifacts. They may withhold sensitive payloads while publishing an explicit omission state or authorized derivative.
- **Domain analysis cells** investigate a bounded question—such as a disaster warning, public-health infrastructure issue, supply disruption, or defensive cyber advisory—using declared procedures and source scopes.
- **Independent challenge cells** attempt to falsify important conclusions, search for alternative explanations, inspect source dependencies, and audit uncertainty. They must not simply reuse the same model, prompt, source list, or institutional assumptions and call the result independent.
- **Evidence custodians** maintain canonical Mycelix evidence identities, provenance, currentness, and access/handling semantics. They do not certify substantive truth merely by accepting a record.
- **Publications and response stewards** decide whether a report is fit to publish under a declared policy. A report's publication status remains distinct from its epistemic status.
- **Rights and process oversight** handles privacy complaints, conflicts of interest, correction/appeal, abuse reports, and review of consequential person-related claims.

One organization may initially perform several roles to bootstrap, but the role combinations and resulting independence limitations must be declared. Later qualification must not pretend those same-role checks were independent.

### 4.2 Governance design

Governance should be polycentric and contestable:

- A public charter defines scope, prohibited uses, contributor rights, appeals, retention and publication rules.
- Regional and domain cells retain control over their own lawful source relationships and protected data.
- Cross-cell policy changes require transparent proposals, versioned decisions, and explicit effective times.
- Independent reviewers can challenge source handling, methods, or conclusions without having to win a popularity contest.
- Funding, major conflicts of interest, paid-source relationships, and sponsor restrictions are disclosed.
- Analysts can publish minority assessments under the same evidence rules; governance cannot force a scientific consensus.
- Emergency publication or disclosure exceptions are narrow, temporary, reason-coded, and retrospectively reviewable.
- Members can leave or fork the public software and protocols. Exit must not require surrendering locally held data or accepting a single global authority.

Trust credentials and reputation may help assess a contributor's past behavior under a specified domain and time window. They must not act as universal truth scores or give senior members permanent epistemic veto power.

## 5. Technical responsibility boundaries

Compose the existing projects; do not create another all-purpose intelligence ontology.

### Mycelix — semantic evidence and policy boundary

Owns canonical evidence/claim semantics, artifact and assertion identity, provenance and derivation, source-dependency assessment, time/currentness, handling status, policy references, and the status of evidence admission. Native domain owners retain their own authoritative state.

### Symthaea — bounded reasoning and investigation

Consumes explicitly admitted or candidate evidence under a declared information frontier. It may generate candidate assertions, competing hypotheses, contradictions, disconfirmation searches, temporal models, forecasts, decision sensitivities, and next-information proposals. It must disclose assumptions and uncertainty and remain able to abstain.

Symthaea's consciousness-related indicators, Φ values, coherence values, reputation scores, or internal confidence must **not** be used as a trust/authority multiplier unless a separate, independently validated protocol shows that the specific quantity is predictive and calibrated for the stated task. Fluent explanation, internal integration measures, and model confidence do not establish factual reliability.

External content must remain untrusted data, not instructions. Research output must not silently enter persistent grammar, curriculum, privileged memory, or action paths. This direction composes the RES-EPI and RES-SEC work in the public Symthaea repository.

### Independent verification — not merely re-running the same model

The verifier should be a separately invoked component with a frozen procedure, exact input and output identities, and a declared independence class. For high-impact assessments, independence should include genuinely different evidence acquisition, implementation, organizational ownership, or reasoning methods—not just a different model name.

The verifier evaluates the declared protocol. It cannot certify the external world beyond what that protocol and evidence support.

### Sol-Atlas and user-facing surfaces — explanation, not truth authority

Visualizations should let readers traverse report → claim → evidence relation → exact artifact selector → derivation and dependency history. Maps, timelines, colors, ranking, and graph layout must not silently strengthen the stored epistemic status.

### Capability and execution systems

Collection, protected-data disclosure, external publication, or intervention requires a separate, current, purpose-bound authorization path. A research result or recommendation is never itself a permit.

## 6. Bootstrap-to-global roadmap

Progress is earned through gates, not calendar promises or feature counts.

### Stage 0 — freeze the semantic and safety foundation

Compose the existing Mycelix EPI-000 / EPI-OSINT program and its acquisition-boundary work with Symthaea's external-content and claim-lineage repair work. Reuse the cross-domain operational fixture and noninterference program rather than inventing another shared truth model.

Required properties:

- exact artifacts and selectors are representable;
- assertion, claim, hypothesis, evidence relation, source reliability, calibration, and action authority remain distinct;
- stale, missing, contradictory, protected, superseded, and indeterminate states survive every adapter;
- hidden evaluator data and future evidence cannot influence an earlier assessment;
- external bytes cannot become trusted instructions or persistent learning authority;
- the design passes deterministic offline negative controls before broad network collection is added.

A passing document test is not runtime qualification. A queued workflow is not a pass. Only exact-head executed evidence qualifies the tested subject.

### Stage 1 — one fully reproducible case

Pick **one bounded, public-interest event** with public source material and a testable outcome. A disaster/early-warning case is a good initial candidate because time-to-warning, missed events, and false alerts are measurable without creating a people-tracking product.

Freeze the question, time cutoff, permitted sources, evidence snapshots, expected scoring procedure, and challenge policy before viewing the held-out outcome.

Produce one complete investigation capsule:

1. decision question and information frontier;
2. exact source artifacts, capture context, timestamps, and selectors;
3. source dependencies and known coverage gaps;
4. source-bound assertion candidates;
5. explicit supporting and contradicting relations;
6. competing hypotheses and their differentiating evidence;
7. calibrated or explicitly uncalibrated uncertainty;
8. falsifiers, missing information, and decision sensitivities;
9. independent verification trace and dissent;
10. a publication decision and a later outcome/reconciliation record.

First demonstrate replay and noninterference against synthetic/frozen fixtures; then add a limited public-source case under declared legal and privacy policies.

### Stage 2 — a small multi-organization federation

Invite a few genuinely independent nodes with different source access, language/context, or domain competence. Begin with read-only public evidence exchange. Preserve each node's acquisition context and local limitations. Do not force all organizations to disclose raw data or merge their subject databases.

Exercise partition, delayed synchronization, conflicting assessments, member exit, source correction, and recovery. A node coming back online must not silently overwrite another node's independent history.

### Stage 3 — a live, narrow public service

Operate one clearly scoped service—such as public disaster claim verification or defensive vulnerability/advisory reconciliation—with:

- a visible scope and prohibited-use policy;
- explicit alert thresholds and expiry/currentness behavior;
- a correction channel and incident response process;
- separate analysis and publication authority;
- published performance, failure, and abstention statistics;
- no autonomous intervention.

Real-time operation begins only after historical replay and adversarial tests establish that the pipeline preserves evidence semantics and does not leak protected or future data.

### Stage 4 — regional and domain federations

Add language-specific, local, and subject-matter cells through versioned adapters. Let them retain context and disagree. Create public schemas, reusable training/evaluation fixtures, secure contribution workflows, and funding that does not allow one sponsor to set the truth model.

Scale only where measured utility rises without unbounded growth in false alerts, privacy leakage, coordination costs, or source dependence.

### Stage 5 — a global intelligence commons

A global network is a federation of accountable regional and domain institutions, not a single planetary database. Each node can verify shared reports, preserve its own history, choose lawful local participation, and withdraw from a relationship. Cross-network products show the source frontier and known blind spots of each participating node.

Institutional maturity would require durable funding, succession and key-recovery plans, operational security, distributed maintenance, independent complaint handling, external security/privacy review, incident disclosure, and transparent governance. Open source alone is not enough.

## 7. Comparative evaluation: define “better” before measuring

Never use one platform-wide number. Record a dated, versioned comparison per task and dimension, following the evidence-class discipline in OPS-INTEL-013 / #2945.

### 7.1 Candidate dimensions

| Dimension | Required observable |
|---|---|
| Factual support | Which published claims are supported, contradicted, unresolved, or unsupported under the frozen evidence protocol? |
| Provenance completeness | Fraction of material claims with recoverable artifact, selector, acquisition context, and derivation lineage; list all exclusions. |
| Source independence | Correct identification of shared upstream causes under hidden dependency fixtures; never treat source count as independence. |
| Probabilistic calibration | Brier/log score and calibration diagnostics on forecasts with resolved outcomes; publish sample size and uncertainty. |
| Early warning | Lead time at a predeclared false-alert rate, alongside missed-event rate. |
| Contradiction handling | Whether conflicts and serious alternatives survive summarization and ranking. |
| Correction latency | Time from valid counter-evidence to affected report correction, with old reports remaining historically reconstructable. |
| Information-flow safety | No change in deterministic earlier-frontier output when only future/evaluator/protected payload changes. |
| Adversarial resilience | Results under source poisoning, coordinated duplication, misleading timestamps, broken provenance, prompt injection, and Sybil-style contribution patterns. |
| Privacy | Protected-field leakage, unnecessary retention, unauthorized disclosure attempts, and quality of redaction/omission handling. |
| Federation | Correct behavior under partitions, stale nodes, conflict, member exit, and delayed reconciliation. |
| Operational value | Latency, resource cost, uptime under a declared profile, analyst effort, and cost per useful verified alert. |

Metrics remain separate. A serious failure in privacy or hard safety constraints cannot be compensated for by a high accuracy average.

### 7.2 Baselines and comparison language

For every benchmark, compare at least the following where relevant and measurable:

- a deterministic rule-based baseline;
- Symthaea without the federated evidence/verification workflow;
- the complete Mycelix/Symthaea workflow;
- available public reports or established open-source workflows at the same information cutoff;
- a documented human/organizational baseline only where its inputs, scope, and evaluation method can be stated fairly.

A task can be evaluated against a known-answer oracle when the outcome can be frozen without exposing the oracle to the solver. For probabilistic forecasting, register the question, resolution criteria, deadline, probabilities, and scoring rule before the outcome is available. Use hidden/future/protected information noninterference tests as well as lineage checks.

No test should award credit for a constant output, unjustified abstention, shortcut leakage, hindsight, or an attractive narrative. Preserve fold/task-level results so aggregate performance cannot hide critical regressions. External expert review is valuable for public legitimacy and scientific interpretation, but it must complement—not replace—deterministic fixture, replay, and adversarial gates.

### 7.3 What would justify a superiority claim?

A valid statement must have the form:

> Under task profile **T**, benchmark generation **B**, information cutoff **F**, and evaluation protocol **P**, exact subject **S** achieved result **R** relative to baseline **C**, with limitations **L**.

The claim must name the tested version/commit and evaluation artifact. It must be retractable if the benchmark is invalidated or a material bug is found.

We cannot make a fair blanket claim that a public-source system is better than the classified CIA/Five Eyes portfolio. We can aim to demonstrate superior reproducibility, public auditability, correction, or performance on specific open-evidence tasks against explicit baselines.

## 8. Threat model and prohibited transformations

Design against at least:

- coordinated misinformation and circular sourcing;
- Sybil identities and manufactured consensus;
- source laundering through summaries or mirrors;
- stale evidence presented as current;
- hidden evaluator/future data leakage;
- prompt injection and hostile instruction-like documents;
- model or verifier shared failure modes;
- organizational capture, undisclosed funding influence, and reputation coercion;
- unauthorized disclosure through prompts, telemetry, graphs, and derived reports;
- false identity resolution and the conversion of association into guilt;
- censorship, network partition, key loss, and malicious node operators.

The network must not provide a generalized doxxing or people-tracking substrate. It must not facilitate unauthorized access, covert surveillance, stalking, personal targeting, or collection that bypasses access controls. Correlation, graph proximity, a public post, or an identity hypothesis is not sufficient grounds for a consequential attribution. Person-related work, if ever justified under a lawful public-interest mandate, requires a separate policy, data-minimization profile, accountable review, correction path, and explicit purpose-bound authority; it is outside the bootstrap scope.

For external web acquisition, respect applicable law, site access policies, rate and resource limits, and a declared public-target safety profile. No broad crawler is needed to establish the initial architecture.

## 9. Go/no-go gates

Do not expand collection or autonomy until each preceding gate has exact executed evidence.

- **G0 — semantics:** canonical evidence distinctions, source/claim lineage, and uncertainty/non-evidence states survive known-answer tests.
- **G1 — security:** adversarial external content cannot become instructions, persistent trusted learning, protected disclosure, or action authority.
- **G2 — temporal integrity:** future/evaluator data do not influence earlier-frontier outputs; historical assessments remain immutable.
- **G3 — verification:** independent checks bind an exact procedure and inputs; a verifier result proves only its declared claim.
- **G4 — usefulness:** the system beats or complements a frozen baseline on at least one declared task without violating predeclared safety, privacy, and false-alert constraints.
- **G5 — federation:** multi-node conflict, partition, correction, and withdrawal work without a central overwrite authority.
- **G6 — operational maturity:** observability, incident response, succession, access administration, maintenance, and oversight exist before the network claims global readiness.

Every gate has an explicit owner, exact subject, required artifacts, negative controls, forbidden-path checks, and a non-claim statement. If the evidence is incomplete or the benchmark cannot discriminate success from a trivial baseline, the result is **inconclusive**, not pass.

## 10. Research basis and alignment

This profile composes—rather than replaces—the repository programs:

- [Mycelix EPI-OSINT-000 / #2625](https://github.com/Luminous-Dynamics/mycelix/issues/2625) — evidence-bearing OSINT program and project ownership boundaries.
- [Mycelix OPS-INTEL-000 / #2938](https://github.com/Luminous-Dynamics/mycelix/issues/2938) — evidence-native operational intelligence architecture.
- [Mycelix OPS-INTEL-013 / #2945](https://github.com/Luminous-Dynamics/mycelix/issues/2945) — dated, dimension-specific comparison and benchmark discipline.
- [Mycelix OPS-INTEL-TEST-001 / #2942](https://github.com/Luminous-Dynamics/mycelix/issues/2942) — shared synthetic operational fixture.
- [Symthaea RES-SEC-001 / #5372](https://github.com/Luminous-Dynamics/symthaea/issues/5372) and [RES-EPI-001 / #5370](https://github.com/Luminous-Dynamics/symthaea/issues/5370) — external-content and claim-source-lineage hardening.

External methodological anchors:

- UK Government, [Professional Development Framework for all-source intelligence assessment](https://www.gov.uk/government/publications/intelligence-analysis-professional-development-framework/the-professional-development-framework-for-all-source-intelligence-assessment) — source evaluation, analytical audit trails, hypothesis/scenario work, and explicit probability/confidence.
- UK Government, [PHIA Common Analytical Standards](https://www.gov.uk/government/publications/phia-common-analytical-standards) — consistent standards for rigor and integrity.
- CIA, [A Tradecraft Primer: Structured Analytic Techniques for Improving Intelligence Analysis](https://www.cia.gov/resources/csi/books-monographs/a-tradecraft-primer) — structured methods for uncertainty, ambiguity, and cognitive-bias mitigation.
- NIST, [AI Risk Management Framework](https://www.nist.gov/itl/ai-risk-management-framework) — lifecycle risk governance.
- NIST SP 800-150, [Guide to Cyber Threat Information Sharing](https://csrc.nist.gov/pubs/sp/800/150/final) — trust, source, handling, and sharing controls in defensive cyber information exchange.
- Global Legal Action Network and Bellingcat, [Open-source investigations methodology](https://www.j-and-a.bellingcat.com/methodology) — careful acquisition and handling of public-source evidence for accountability processes.

These documents inform the design; they do not certify this network or establish that any implementation conforms.

## 11. Explicit non-claims

This proposal does not establish that Symthaea is generally intelligent, that its consciousness-related metrics indicate factual reliability, that Mycelix's current research components are production-ready, or that any component has been independently audited.

It does not establish CIA/Five Eyes superiority, OSINT completeness, source truthfulness, prediction accuracy, legal admissibility, safe autonomous investigation, or production readiness. Those remain open questions requiring exact, reproducible evidence.
