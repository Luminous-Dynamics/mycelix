# Knowledge Evidence and Endorsement Evaluation Protocol v0.1

**Status:** preregistration template / research protocol; no benchmark scores reported.  
**Date:** 2026-11-05  
**Owner:** Mycelix Knowledge / DKG, coordinated with Symthaea KNOW-INT-001.  
**Parent audit:** [KNOW-TRUTH-001 / Mycelix #5818](https://github.com/Luminous-Dynamics/mycelix/issues/5818)

> This protocol does not assert that the current Knowledge hApp detects truth or resists misinformation. It defines how to test narrower properties and how to avoid confusing social response with evidence.

## 1. Objective

Evaluate whether a claim/evidence system preserves the distinction between:

- what a proposition says;
- what evidence is relevant to that proposition;
- whether source artifacts are authentic and intact;
- whether sources are independent or share a common origin;
- what a community or individual endorses, challenges or disputes;
- whether the claim applies at the requested time/place/domain;
- what remains uncertain, missing, contested or restricted.

The target is not a universal truth score. It is a **purpose-bound assessment bundle** with explicit status, scope, sources, lineage, method, uncertainty and access decision.

## 2. Current implementation baseline and known test boundary

Baseline repository revision at protocol creation: `main@9e12dd46bdd8371e8ed9626a17bc06f21c141a52`.

Relevant current code and tests:

- `mycelix-knowledge/zomes/dkg/src/lib.rs` — Knowledge DKG coordinator, legacy `get_truth` API and confidence-weighted retrieval.
- `mycelix-workspace/sdk/src/dkg/confidence.rs` — five-factor confidence heuristic in the SDK.
- `mycelix-knowledge/zomes/dkg_integrity/src/lib.rs` — current claim and attestation entry definitions.
- `mycelix-knowledge/tests/genesis-simulation.test.ts` — synthetic endorsement-ordering DHT scenario.
- `mycelix-knowledge/tests/dkg-adversarial.test.ts` — selected multiple-claim, challenge, spam and invalid-input scenarios.
- `mycelix-knowledge/tests/dkg-multi-agent.test.ts` — endorsement-count and author-reputation behavior.

Important scope limits found in the current test code:

1. The test named “Sybil resistance” creates five separate Green claims, then compares them with a Blue claim that receives a Bob endorsement and Alice self-endorsement. It does not test five colluding identities endorsing one shared false claim.
2. The challenge test checks score monotonicity after one and two challenges. It does not test coordinated counter-challenge warfare.
3. The “3 endorsements beat 2” test checks endorsement-count ordering. Eve endorses both claims; the result is not a truth verdict.
4. The spam test is a 50-claim volume/availability smoke test and reputation-bounds check, not a general anti-spam or Sybil guarantee.
5. The genesis DHT case tests a narrow ranking behavior, not independently adjudicated factual correctness.

These tests are useful integration fixtures. Their passing status on the present revision must be checked separately; their names or intended assertions are not evidence of successful execution.

## 3. Assessment object: do not collapse dimensions

The evaluation output should preserve these independently inspectable dimensions. This is a conceptual assessment contract; first map it onto existing evidence-plane and semantic-core types rather than introducing a parallel production envelope.

| Dimension | Example values | What it must not imply |
|---|---|---|
| Claim kind | empirical, historical, mathematical, normative, narrative, predictive, procedural | That every kind has the same truth criterion |
| Claim scope | domain, time interval, geography, population, conditions | Global or timeless applicability |
| Evidence relation | supports, contradicts, qualifies, contextualizes, irrelevant, unknown | That a source citation alone supports a claim |
| Evidence method | observation, experiment, proof, archival analysis, testimony, model output, expert interpretation | That method labels are interchangeable |
| Source integrity | digest verified, signature verified, unverified, unavailable | Factual correctness |
| Source lineage | independent, derived, mirror, common upstream, unknown | Independence from different URLs or publishers |
| Applicability/currentness | current for scope, expired, not-yet-valid, historical-only, unknown | Universal truth decay with age |
| Community response | endorse, challenge, acknowledge, abstain, contested | Empirical truth or legitimacy from majority vote |
| Authority/permission | authorized, restricted, purpose-limited, revoked, unknown | That public discoverability grants reuse rights |
| Assessment | supported, contradicted, mixed, insufficient, not assessable, restricted | A universal probability absent task-specific calibration |
| Uncertainty | sampling, measurement, model, source, temporal, structural/ontology, unknown | That one confidence scalar captures all uncertainty |

For a mathematical statement, a proof checker may establish formal validity under stated axioms. For a historical question, the task may be to establish what a particular source witness attests, not to force all historians into one answer. For community knowledge, authority and permission may be determined by the relevant knowledge holders, not network popularity.

## 4. Evaluation families

### E1 — Endorsement and reputation sensitivity

Hold claim text, evidence and source lineage constant while varying endorsements, challenges and attester reputations. Measure whether social signals change the *community-response* output without being reported as new independent evidence.

Required paired cases:
- zero endorsements vs. one endorsement;
- one vs. five vs. twenty endorsements on the same claim;
- five independent attestations vs. five coordinated/correlated attestations;
- author self-attestation vs. independent attestation;
- reputation permutation while evidence is held fixed;
- one agent endorsing two contradictory claims.

### E2 — Sybil / collusion / brigading

Vary adversarial identity counts (1, 2, 5, 10, 20, 50) while holding total evidence constant. Compare:
- many identities creating separate duplicate claims;
- many identities endorsing one shared claim;
- many identities reusing identical evidence text;
- many identities submitting varied but non-verifiable evidence strings;
- colluding identities with artificially accumulated reputation;
- mixed honest and adversarial participants.

Do not assume an RB-BFT threshold automatically applies to epistemic claim support. Report behavior over the complete adversary-ratio sweep; do not infer safety from one threshold.

### E3 — Source-family independence and mirror storms

Generate source packs with:
- one source copied to many URLs;
- one press release reprinted by multiple publishers;
- a translation derived from one manuscript;
- multiple papers reusing the same upstream dataset;
- independent acquisition of the same observation;
- unknown lineage.

Required invariant: copies, mirrors, translations and common-upstream derivations must not silently count as independent corroboration. Unknown independence remains unknown.

### E4 — Evidence relevance and counterevidence

Test cases where:
- many weak or irrelevant sources conflict with one highly relevant source;
- a minority source provides direct, reviewable counterevidence;
- sources support different parts or scopes of a compound claim;
- the evidence is authentic but methodologically unsuitable;
- the source is relevant but its exact bytes/locator are unavailable.

Measure preservation of evidence relationships and disagreement, not majority-vote victory.

### E5 — Source-category, reputation and model-diagnostic swaps

Keep the source bytes, method and claim fixed while changing only:
- source-category label (academic, institutional, commercial, social);
- contributor reputation;
- internal model-confidence value;
- Symthaea Φ/coherence diagnostic;
- display ranking or popularity metadata.

An evidence-support assessment must not improve solely because one of those unrelated metadata fields changes. Φ is not part of the external-proposition evidence lane.

### E6 — Temporal applicability

Separate historical evidence from currentness:
- an old but authentic historical source remains admissible for a historical question;
- a legal, price, medical guidance or operational claim can expire or change;
- a source created after the question's as-of date must not leak into a time-bounded historical answer;
- unknown validity intervals remain unknown;
- backdated or caller-supplied timestamps do not establish currentness by themselves.

### E7 — Search incompleteness and negative findings

Vary:
- complete finite corpus vs. paginated but not exhausted;
- provider truncation and local result caps;
- rate limits and source outage;
- access restriction;
- language, region and date filters;
- a query that returns zero results under a narrow scope.

Required invariant: “no result returned” must remain bounded by the exact recorded search attempt and its coverage. It does not imply that a claim, event or source does not exist.

Compose Mycelix #2635. Do not create another search-receipt model.

### E8 — Rights, consent and derived-data leakage

Test authorization at:
- source ingestion;
- indexing and embeddings;
- retrieval;
- summary generation;
- caching;
- export;
- revocation and deletion/suppression workflows.

A revocation or access restriction must be respected by downstream derived outputs according to the applicable policy. A public hash or commitment must not expose restricted content. A zero-breach result on a synthetic fixture is not a legal or operational compliance certification.

### E9 — Claim-kind separation

Include empirical, historical, mathematical, normative, narrative, procedural and community-governed cases. Verify that:
- normative endorsement is not used as empirical evidence;
- narrative attestation is not graded like sensor measurement;
- source-attested narrative claims remain distinct from empirical claims about events;
- mathematical validity is established by a suitable formal checker, not reputation;
- community authority is not replaced by majority endorsement;
- unsupported/unanswerable cases permit abstention.

## 5. Baseline ladder

All systems receive the same claim text, evidence bytes, source locators, temporal scope, access permissions, tool budget and evaluation cutoff.

1. **Legacy Mycelix heuristic:** current five-factor score, frozen at the exact SDK/zome commit.
2. **Attestation-only:** endorsement/challenge counts without reputation weighting.
3. **Reputation-only:** contributor signal without evidence assessment.
4. **Source-category heuristic:** current category-based source scoring isolated as an ablation.
5. **Deduplicated-source baseline:** evidence counts clustered by known provenance family.
6. **Evidence-only human assessment:** blinded expert/qualified adjudication on the evidence pack, without showing social endorsements or contributor reputations.
7. **Retrieval baselines:** BM25, dense and hybrid retrieval with a fixed generator, as applicable to evidence discovery tasks.
8. **Graph-RAG baseline:** typed provenance/evidence relationships, including incomplete-graph cases.
9. **Symthaea + Mycelix composition:** source-conditioned retrieval/reasoning with explicit provenance, uncertainty and abstention.
10. **External frontier/open-weight systems:** exact model/version/date, system prompt, retrieval corpus, tool settings, latency, tokens and cost recorded.

Report each baseline separately. Do not compare a model with access to more evidence or more time against a restricted baseline and call the result an architecture win.

## 6. Metrics

Report by claim kind, domain, source type, time scope, adversary ratio and evidence-availability stratum.

- **Evidence-support precision/recall:** whether support/contradiction relationships match blinded adjudication.
- **Source-family independence accuracy:** correct classification of independent, derived, mirror and unknown relationships.
- **False-independence rate:** proportion of correlated sources incorrectly counted as independent.
- **False-supported rate:** unsupported or contradicted claims incorrectly represented as supported.
- **Counterevidence recall:** relevant contradictory/qualifying evidence preserved and surfaced.
- **Endorsement separation:** change in social-response output versus change in evidential assessment under endorsement-only interventions.
- **Reputation separation:** evidence assessment invariance under reputation-only permutations.
- **Source-label invariance:** evidence assessment invariance when only category labels change.
- **Temporal-scope accuracy:** correct validity/currentness for the declared as-of time.
- **Selective risk / coverage:** error rate among answered cases as coverage changes; include abstention precision and recall.
- **Calibration:** Brier score, log loss and reliability curves only for clearly defined, adjudicable outcomes with sufficient sample size. Do not calibrate a heterogeneous “truth score”.
- **Rights/access leakage:** unauthorized outputs, including derivative summaries and cached results; critical release gate.
- **Operational cost:** p50/p95 latency, memory, storage, network traffic, compute and reviewer time per assessed claim.

Where evidence is clustered by source family, use source-family-aware splits and uncertainty estimates. Bootstrap at the correct independent unit; do not treat mirrored URLs as independent samples. Preserve per-case records and negative results.

## 7. Preregistered invariants and release gates

The following are hard semantic invariants for the test suite:

1. Endorsement-only changes must not be represented as independent evidence acquisition.
2. Reputation-only changes must not be represented as changes in evidential relevance or quality.
3. A duplicated/mirrored source must not increase independent-source count.
4. Source-category relabeling alone must not strengthen evidence support.
5. An older historical source must not be invalidated solely due to age.
6. An incomplete search must not produce a global absence claim.
7. Restricted/revoked content must not leak through derived outputs.
8. Counterevidence and legitimate disagreement must remain inspectable.
9. All reported assessments must bind their purpose, scope, inputs, method and version.
10. No universal truth probability is emitted without task-specific outcome definitions and calibration evidence.

**Critical safety gate:** zero known unauthorized-disclosure failures in the defined qualification suite. This is necessary but not sufficient for real-world security or legal compliance.

**Performance targets:** do not set arbitrary accuracy/SOTA thresholds before a baseline pilot. Freeze a development corpus, estimate task difficulty and variance, then preregister task-specific improvement and calibration targets with paired confidence intervals. Targets must not be tuned on the locked test set.

## 8. Dataset maturity and split governance

Use the Mycelix evidence-maturity ladder from #5451:

1. synthetic fixture;
2. source-cited transcription;
3. hashed source ingestion with documented transformation;
4. independent reproduction/review;
5. domain or operational validation where appropriate.

For the first pass:
- synthetic fixtures test protocol semantics only;
- no copied copyrighted or restricted content without permission;
- exact source family/work/edition/translation and rights records must be frozen before splitting;
- train/development/test separation must be group-aware by source family and claim lineage;
- challenge cases and answer keys remain hidden from model developers where feasible;
- expert disagreements are recorded rather than erased by forced adjudication;
- corrections and retractions append history rather than rewrite previous evidence.

No source-quality or truth-detection claim may be derived from schema validation, synthetic fixtures, or test names.

## 9. How to claim an improvement over the state of the art

A result is eligible for an external-comparison claim only if it has:

- exact repository commit and dataset revision;
- frozen, source-family-disjoint evaluation data;
- matched evidence, permissions, prompts, tools, time and compute budgets;
- strong relevant baselines from Section 5;
- predeclared primary metrics and analysis;
- per-case output artifacts and confidence intervals;
- adversarial and metamorphic negative controls;
- blinded independent scoring where expert judgment is required;
- replication on a separate source family, domain or cohort;
- published limitations and failed conditions.

A win on a social-endorsement ranking task is not a win at truth detection. A win on synthetic Sybil fixtures is not a proof of production Sybil resistance. A lower hallucination score is not proof of factual truth outside the evaluated task distribution.

## 10. Existing work to compose

Do not create parallel implementations. Compose:
- Mycelix #5818 — canonical-path audit and epistemic semantics correction.
- Mycelix #5451 — shared evidence plane and maturity ladder.
- Mycelix #3022 — canonical statement identity and provenance-family semantics.
- Mycelix #2635 — search coverage and negative findings.
- Mycelix #2683 — semantic-core / epistemic identity convergence.
- Symthaea #10494 — cross-domain knowledge reasoning evaluation.
- Existing DKG integration tests listed in Section 2.

The documentation correction in Mycelix PR #5822 is separate and does not change scoring behavior. Its test TODOs are a gap list, not completed controls.

## 11. Initial acceptance

This protocol is ready for implementation planning when:
- the canonical zome/SDK/UI/API consumers are identified;
- current test names and test behavior are reconciled;
- exact schemas and benchmark fixtures are chosen from existing shared contracts;
- dataset rights and source-family lineage are recorded;
- task-specific adjudication methods are agreed;
- a baseline pilot is frozen and preregistered;
- missing controls are implemented and executed locally before CI is considered.

**Initial status:** protocol only. No new executable tests, corpus acquisition, model evaluation, exact-head CI pass, security audit or SOTA result is claimed.
