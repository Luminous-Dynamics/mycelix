# Cross-Scale Wisdom Evaluation: a pluralistic, consent-bounded research contract

**Status:** Proposal for review; not an implementation claim.  
**Owner:** Mycelix civic / commons boundary, with optional adapters to Symthaea, Symtropy, and Sol-Atlas.  
**Design rule:** Improve the conditions for reflection and cooperation without ranking souls, prescribing a worldview, or exposing private inner life.

## 1. Why this layer is needed

Symthaea already tracks evidence-bounded practical-wisdom work and adversarial/longitudinal evaluation. Mycelix already has typed deliberation and explicit representation of contested evidence. This proposal does not replace those systems or create a second wisdom engine.

The missing research seam is cross-scale evaluation:

**voluntary practice → private reflection → chosen action → consented community deliberation → observable outcome → reflection and revision**

The objective is to test whether particular, well-described practices help people reason more carefully and cooperate more constructively in specific settings. We must not infer that a person is spiritually advanced from a self-report, infer that a community is “more conscious” from an aggregate score, or infer causality from a before/after correlation.

A successful project may find that a practice helps, has no detectable effect, helps some participants but not others, or causes harm. All are valid research outcomes.

## 2. Normative and epistemic invariants

1. **No universal wisdom, enlightenment, virtue, or consciousness score.** Report component-level observations and their uncertainty. Do not use a composite score to allocate rights, reputation, services, leadership, or voting power.
2. **Pluralism without false equivalence.** Preserve disagreement between traditions, including incompatible claims. Explain their sources and contexts; do not flatten them into an invented common doctrine.
3. **Evidence categories stay distinct.** Historical/textual evidence, philosophical interpretation, normative recommendation, reported experience, empirical outcome, and causal evidence are different kinds of statements.
4. **Correlation is not causation.** Self-report, attendance, retention, eloquence, popularity, network centrality, and community approval are not efficacy evidence.
5. **No metaphysical inference from system telemetry.** An internal Symthaea metric, Phi estimate, HDC similarity, model confidence, or reported mystical state cannot establish that a person is enlightened or that a system has proved consciousness.
6. **Agency over engagement.** No streaks, shame loops, simulated spiritual authority, artificial scarcity, dependency-maximizing design, or nudges that exploit grief or vulnerability.
7. **No coercive disclosure.** Reflections and raw practice journals stay local/private by default. Participating in a community must never require disclosure of an inner experience.
8. **Dissent remains durable.** A closed decision does not erase unresolved objections, affected-party concerns, contradictory evidence, or minority interpretations.
9. **Human support remains available.** The system must not impersonate clergy, a spiritual director, a therapist, or emergency services. Where a practice appears to worsen distress, allow a clear stop/grounding path and provide appropriate human-support guidance.
10. **Exit is a first-class flow.** A participant can decline, pause, withdraw permission for future uses, and leave a community without an adverse reputation mark.

## 3. Proposed practice record

Each catalogued practice should have a versioned, reviewable record. Field names below are conceptual; they are not yet a stable API.

- **Identity and provenance:** stable practice ID, version, source references, language, originating tradition/community, contributor/steward, date, and culturally appropriate permissions.
- **Interpretation boundary:** what the tradition or author says; what a modern interpreter infers; what empirical research suggests; what is unknown or disputed.
- **Intended outcome:** a narrowly defined capability or experience (for example, noticing a thought before reacting), never “increase consciousness” as an unqualified success criterion.
- **Hypothesis and mechanism:** the predicted change, plausible alternative explanations, and a falsifiable outcome.
- **Applicability and adaptation:** audience, context, accessibility, optional participation, contraindications or known cautions, and alternative/grounding practices.
- **Risk and stop conditions:** foreseeable adverse reactions, check-in cadence appropriate to intensity, escalation/hand-off rules, and how adverse events are recorded.
- **Consent and data plan:** what is collected, purpose, retention, local vs shared data, who can see each field, revocation/withdrawal behavior, and the limits of deletion in a replicated network.
- **Evaluation plan:** primary and secondary outcomes, comparison condition, assessment timing, exclusion rules, missing-data handling, subgroup checks, and preregistration reference when applicable.
- **Evidence status:** traditional teaching, qualitative report, preliminary association, controlled evidence, replicated evidence, null finding, or adverse-event finding. These labels must not be promoted automatically by popularity.
- **Revision lineage:** who changed the record, what evidence changed, and which prior versions were used in a study.

The catalog should permit a community to curate a tradition-specific description without allowing it to silently edit another community's record. Community attestation authenticates attribution; it does not prove metaphysical truth.

## 4. Data boundary and event lifecycle

Prefer the following lifecycle:

1. **Offer:** display the practice, source, expected duration, uncertainties, alternatives, and risks.
2. **Consent:** record purpose-specific permission. No consent means no collection or sharing.
3. **Perform or decline:** the participant may stop at any time. A completed session is not assumed just because it was offered or opened.
4. **Private reflection:** store raw journal content locally by default. The participant chooses whether to export a small summary, and previews it before sharing.
5. **Optional contribution:** only the expressly permitted fields leave the private boundary. Pseudonymization is not a guarantee against re-identification.
6. **Deliberation:** the community receives a bounded evidence packet with its provenance, limitations, and uncertainty. Do not include raw intimate narratives by default.
7. **Outcome observation:** record the predeclared outcome, observation method, missingness, and plausible confounders.
8. **Revision:** preserve the version and information available at decision time; do not let hindsight rewrite the original prediction.
9. **Withdrawal and revocation:** prevent future processing or access where the system controls it, emit an auditable revocation result, and disclose any replicated or previously exported copies that cannot technically be recalled.

### Replicated-data warning

Holochain/DHT and other replicated systems require careful communication about deletion. Revoking access or stopping future processing is not equivalent to erasing every copy already replicated, exported, cached, or independently retained. Raw spiritual journals, prayer content, mental-health disclosures, and private reflections should therefore not be written to a public or broadly replicated commons as the default.

Any implementation must state its exact guarantees and limits, including the threat model and what happens to already disclosed data.

## 5. Evaluation ladder

The following are **candidate engineering gates and research targets**, not validated clinical cutoffs or claims about the present system. Final thresholds must be precommitted with researchers, community representatives, and an independent safety reviewer before confirmatory testing.

### E0 — Semantics and provenance

Build deterministic fixtures that contain conflicting worldviews, empirical claims, traditions, reported experiences, normative judgments, and fabricated citations.

Measure:
- 100% round-trip preservation of identifiers, versions, provenance, evidence type, and unresolved status in the fixture suite.
- 100% of high-impact statements in evaluated outputs have a traceable source or are explicitly labeled as an unsourced interpretation.
- Zero silent conversion of popularity, seniority, credential, trust/reputation, centrality, or confidence into truth or decision authority.
- Zero cases in the frozen adversarial set where incompatible traditions are silently collapsed into one canonical doctrine.

A pass establishes conformance to the fixture suite only; it does not establish cultural completeness.

### E1 — Consent, privacy, and autonomy

Use synthetic records and adversarial traces before handling real participant data.

Measure:
- 100% of missing, expired, mismatched-purpose, or revoked consent cases fail closed for the prohibited operation.
- 100% of tested revocations block subsequent controlled reads/writes within the system's declared consistency contract and produce an inspectable result.
- Zero raw journal text leaves local storage in tests unless that exact export is explicitly approved by the participant.
- Zero known routes by which refusal, withdrawal, or leaving a practice reduces a user's reputation, access to unrelated services, or governance rights.
- Explicit tests for small-cohort re-identification, metadata leakage, backup/export copies, administrator misuse, and malicious community stewards.

Do not describe these results as “privacy proved” or “deletion guaranteed.” Report the tested threat model and limitations.

### E2 — Individual capability pilot

Start with a 20–40 participant feasibility/usability and safety pilot only if ethical review and appropriate consent are in place. This sample is for workflow and instrument testing, not for establishing efficacy.

Potential outcomes include meta-awareness, decentering, non-reactivity, perspective-taking, and alignment between a participant's own stated values and chosen actions. Select validated instruments and behavioral tasks with researchers rather than inventing a consciousness scale. Respect licensing and cultural/language validation requirements for any instrument.

For a later powered study:
- Preregister one primary outcome and a smallest effect size of interest with independent input.
- Include baseline, post-program, and follow-up (for example, three months), not engagement metrics alone.
- Report effect estimates, confidence intervals, missingness, adverse events, and subgroup differences.
- Treat an effect such as standardized mean difference 0.20 as a *candidate planning threshold only*, to be justified for the chosen measure and context before data collection.
- Do not interpret increased self-transcendence reports as proof of greater wisdom, moral behavior, metaphysical truth, or clinical benefit.

Use an active comparison condition where feasible. A pleasant experience, high satisfaction score, or high retention rate is not an efficacy result.

### E3 — Community deliberation and cooperation

Use structured scenarios first and real community projects only with voluntary participation and appropriate oversight. Compare, where feasible, a normal deliberation condition against a condition using the practice and evidence packet.

Primary measurements should be defined for each task, not merged into one group score:
- Decision quality against a blinded, task-specific rubric.
- Whether alternatives, uncertainty, affected parties, and counterarguments are represented.
- Whether minority objections and unresolved issues survive serialization and decision closure.
- Ability to revise a decision when valid new evidence appears.
- Perceived procedural fairness and safety, assessed across participant groups.
- Follow-through on the decision's observable commitments.

Candidate gates:
- 100% preservation of objections and uncertainty records in deterministic lifecycle tests.
- No material regression in predefined fairness or safety indicators for any adequately sampled subgroup.
- A practically meaningful benefit over an active comparison condition must be defined before confirmatory testing; a standardized effect near 0.20 may be explored in planning, not adopted as a universal pass threshold.
- Replicate across more than one community and task before making general claims.

Do not give individuals or communities a “consciousness rank.” Report uncertainty and divergent effects.

### E4 — Cross-scale simulation and environmental stewardship

Symtropy can explore agent-based hypotheses about how attention, trust, cooperation, scarcity, incentives, social influence, and governance rules interact. Simulated agents are not human participants, and simulation outputs are not human efficacy evidence.

Sol-Atlas can supply grounded system contexts (for example, water, energy, biodiversity, or neighborhood infrastructure) when the source data has appropriate provenance and limitations.

For every simulation:
- Publish the model assumptions, parameters, seed, version, and sensitivity analyses.
- Compare against trivial baselines and adversarial worlds (free riding, noisy reports, coercive leaders, resource shocks, false consensus, and strategically manipulated evidence).
- Test whether aggregate benefits hide subgroup harms or concentration of power.
- Record which outcomes are synthetic, observed, or inferred.
- Validate predictions against measured real-world data before making operational recommendations.

The success target is improved, explainable performance on a defined stewardship task—not a claim that society has achieved a higher level of consciousness.

### E5 — Longitudinal safety and independent replication

Only proceed to longer pilots after E0–E3 behavior is qualified and human oversight is established.

- Predefine pause/stop conditions and an adverse-event response path.
- Monitor worsening distress, disorientation, unwanted dependency, coercive disclosure, privacy incidents, and social exclusion.
- Publish negative and null outcomes as well as positive results.
- Maintain separate evidence lanes for deterministic software tests, adversarial red-team tests, feasibility pilots, controlled human studies, and independent replication.
- A queued CI workflow or an unexecuted test plan is not a pass; every status must name the exact commit, test version, corpus version, execution receipt, and result.

No fixed “zero harm in the field” claim is supportable. The quality target is timely detection, transparent reporting, response, and prevention of repeatable failure.

## 6. How the projects connect without merging responsibilities

| Project | Responsibility | Not its responsibility |
|---|---|---|
| Symthaea | Individual reflection workflows; evidence-aware dialogue; adversarial and longitudinal evaluation of the reasoning model. | Certifying enlightenment or deciding which worldview is true. |
| Mycelix | Consent-aware community membership and sharing; source/provenance records; deliberation with preserved dissent; auditable decision lineage. | Storing raw intimate reflections by default or turning reputation into spiritual authority. |
| Symtropy | Reproducible simulation of interaction, incentives, resource constraints, and cross-scale hypotheses. | Treating simulated agents as empirical evidence about actual people. |
| Sol-Atlas | Grounded spatial/ecological/system context and inspectable datasets for stewardship tasks. | Converting planetary visualization or awe into proof of a metaphysical thesis. |

Treat integrations as optional adapters with versioned contracts. Each project remains independently useful. Begin with a shared research vocabulary and fixture format; do not begin with a mandatory distributed runtime or a shared universal identity.

## 7. Minimal execution plan

1. **Freeze the evaluation vocabulary and claim ceilings.** Agree on the difference between report, interpretation, outcome, causal evidence, and normative judgment.
2. **Create an initial 30–50 case synthetic corpus.** Cover incompatible religious/nonreligious positions, minority objections, uncertain evidence, adversarial authority claims, fabricated sources, consent revocation, distress, and requests for spiritual ranking. Publish the corpus version and ground-truth rationales.
3. **Build deterministic conformance checks.** Test type/provenance preservation, consent boundaries, withdrawal behavior, no scalar ranking, and no authority escalation from confidence or reputation.
4. **Run a tabletop red team with affected users and cultural/religious stewards.** Include people who prefer nonreligious practices and people who do not want to disclose spiritual beliefs. Participation must be compensated where possible and entirely voluntary.
5. **Only then design the human pilot.** Obtain independent ethics review, select validated measures, preregister hypotheses and analysis, power the study appropriately, and publish the protocol before collecting outcome data.

### Exit criteria for this proposal

The proposal is ready to move from design into implementation when:
- an accountable maintainer accepts ownership of the semantics and interfaces;
- overlapping Symthaea/Mycelix issues and data models have been reconciled;
- reviewers approve the consent, replication, privacy, and adverse-event boundaries;
- a frozen corpus and deterministic checker have named versions;
- proposed thresholds are distinguished from validated thresholds;
- the human-evaluation plan has independent oversight and preregistration requirements.

## 8. Research basis and limitations

This proposal is motivated by, but does not claim more than, existing research:

- De Oliveira et al., “Cultivating Self-Transcendence Through Meditation Practice,” *Psychological Reports* (online first 2024): a two-stage study associated meta-awareness and dis-identification processes with different aspects of self-transcendent experience. The results motivate specific hypotheses; they do not establish a universal developmental sequence. https://pubmed.ncbi.nlm.nih.gov/38669443/
- Jeste et al./wisdom-measurement literature, “Measuring Wisdom: Existing Approaches, Continuing Challenges, and New Developments”: reviews multiple approaches and persistent construct-validity and ecological-validity challenges. https://academic.oup.com/psychsocgerontology/article/73/8/1393/4769351
- Farias et al., “Adverse events in meditation practices and meditation-based therapies: a systematic review” (2020): reports a wide range of adverse experiences and substantial heterogeneity across study types. The estimate should not be treated as a universal rate for any specific practice. https://pubmed.ncbi.nlm.nih.gov/32820538/
- Tretter and Puzio, “Red lines for religious AI,” *Nature* (14 October 2025): identifies concerns including manipulation, spiritual abuse, and dependency. https://www.nature.com/articles/d41586-025-03359-z
- Rask and Shin, “Integrating Paths: Enhancing Deliberative Democracy Through Collective Intelligence Insights,” *Societies* (18 December 2024): reviews possible connections between collective intelligence, deliberation, and participatory governance; it does not guarantee that any specific tool improves decisions. https://www.mdpi.com/2075-4698/14/12/270

The engineering contribution proposed here is the cross-scale contract: explicit evidence categories, participant sovereignty, reproducible tests, pluralistic deliberation, and a disciplined bridge from simulation to human evidence. It is a research proposal until the implementation and evaluations exist.

## 9. Relationship to existing work

This work is intended to complement, not supersede:

- Symthaea [#6451 — WISDOM-001](https://github.com/Luminous-Dynamics/symthaea/issues/6451): evidence-bounded philosophy and practical-wisdom projection.
- Symthaea [#2131 — WCARE-09](https://github.com/Luminous-Dynamics/symthaea/issues/2131): adversarial and longitudinal wisdom-care evaluation.
- Symthaea [#2669 — RQ-006A](https://github.com/Luminous-Dynamics/symthaea/issues/2669): quarantine heuristic meta-confidence from capability authority.
- Mycelix [#3116 — MYC-GOV-003A](https://github.com/Luminous-Dynamics/mycelix/issues/3116): typed issue/claim/objection deliberation graph.
- Mycelix [#3126 — MYC-INT-012A](https://github.com/Luminous-Dynamics/mycelix/issues/3126): preserve open-world uncertainty and contested evidence.

No existing implementation or benchmark is assumed to have passed by virtue of these planning issues.
