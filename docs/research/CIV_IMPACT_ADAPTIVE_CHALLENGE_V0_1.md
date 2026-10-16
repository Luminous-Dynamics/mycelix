# CIV-IMPACT Adaptive Challenge and Exposure Control v0.1

**Status:** protocol and contracts proposed; runtime selector and sealed-bank service not implemented.  
**Tracking:** [CIV-IMPACT-003 #5065](https://github.com/Luminous-Dynamics/mycelix/issues/5065)  
**Consumes:** [CIV-IMPACT-002 #5053](https://github.com/Luminous-Dynamics/mycelix/issues/5053), [PRAX-AI-001E #3568](https://github.com/Luminous-Dynamics/mycelix/issues/3568), evaluator identity #4569, transportability #4577, Symthaea red-team #2589.

## 1. What changes

A benchmark should be a measurement instrument, not merely a fixed collection of examples. Static public fixtures are essential regression tests, but repeated exposure can turn them into optimization targets. A sealed set can provide fresh challenges, but repeated peeking, evaluator adaptation, or selective reporting can still invalidate claims.

This profile combines:
- a permanent public regression lane for known failures;
- an evaluator-controlled sealed challenge bank;
- a preregistered selector (adaptive measurement-value or fixed random);
- an independent random audit tranche;
- frozen evaluator/scorer/candidate/comparator identities;
- an append-only exposure ledger and explicit post-reveal invalidation;
- claim ceilings bound to the actual challenge domain.

This profile does not implement a new generic evaluator identity or transportability model. It references those existing artifacts by digest and defines how CIV-IMPACT uses them.

## 2. Artifact separation

### Candidate-visible/public
The candidate may receive the frozen policy input and public regression corpus. It must not receive sealed case IDs, hidden truth, secret seeds, selector score internals, unselected-case identities, or per-item oracle loss before the relevant output is frozen.

### Evaluator-only
The evaluator keeps the sealed bank, seed material, truth, eligible candidate list, selection scores/probabilities, unselected-case reasons, and exposure ledger. These are stored separately and are not committed to the candidate-facing public corpus.

The public preregistration publishes a cryptographic commitment to the sealed bank—not the bank itself. To make that commitment meaningful, define a canonical serialization and a documented commitment construction before producing a bank; keep secret material out of public artifacts. This v0.1 design does not yet provide a production secret-management service.

## 3. Preregistered selection and stopping

Before running candidate outputs, freeze:
- one primary endpoint and the explicit direction of improvement;
- candidate, strongest fair comparator, evaluator, scorer, build/runtime environment and generator identities;
- public corpus digest and sealed-bank commitment;
- the adaptive selector version, objective, cost budget, maximum number of selected cases, minimum coverage by regime, and fixed random audit share;
- sampling/cluster unit, uncertainty method, treatment of timeouts/partial results, multiplicity and interim-peeking policy;
- exposure/reveal/rotation policy and claim ceiling.

The adaptive selector can prioritize challenge units with high expected value for distinguishing preregistered competing policy hypotheses per unit evaluation cost. It may **not** optimize for candidate success or change its objective after seeing results. Minimum coverage constraints force representation of safety-critical strata even if the selector estimates that another case has higher measurement value.

If adaptively selected items do not have defensible inclusion probabilities or an estimator matched to the design, results on that stream are descriptive only. The campaign can still be closed as `descriptive_complete`, but its receipt must carry the descriptive-only claim ceiling and `confirmatory_claim_valid=false`; it cannot be promoted to confirmatory completion. Always preserve a randomly sampled audit tranche that is selected independently of candidate outputs. Report the adaptive stream and random audit stream separately.

## 4. Freeze, reveal and rotate

Every campaign starts with a campaign_frozen ledger event that binds the exact preregistration, candidate/comparator artifacts, evaluator/scorer profiles and sealed-bank commitment.

For each selected challenge:
1. record the selection batch and all eligible alternatives considered;
2. record its commitment, eligibility, selection probability where defined, selector score and reason code in evaluator-only artifacts;
3. execute candidate and comparator on the same challenge input, preserving every disposition;
4. freeze the result receipt and evidence digests;
5. only then disclose enough to support audit and regression;
6. after disclosure, move the challenge to the public regression corpus and replace it only under a new version / bank commitment.

The exposure ledger is append-only. Each event contains the digest of the prior event and an event digest computed over canonical event-core bytes excluding the digest field itself. For this v0.1 Rust profile, canonical bytes mean compact serde_json serialization after parsing, with JSON object keys in lexicographic order, array order preserved, and the event_sha256 property removed before hashing. Validators must pin the serialization implementation/version and reject nonconforming digests rather than silently canonicalizing under a different convention. Verification must recompute those digests, enforce sequence continuity, ensure every selected commitment was in the eligible set, reconcile selected/executed/frozen/disclosed counts, and reject replayed event identities. A hash chain is not authentication: an actor who can rewrite the entire ledger can recompute every digest. Confirmatory use therefore also needs an independently retained, signed or transparency-log-anchored checkpoint outside the candidate/evaluator writer's control. The current v0.1 prototype checks the chain but does not issue or verify external signatures/anchors.

Any post-freeze material change to candidate, comparator, scorer, evaluator, generator, transform, cost model, metric, threshold, hidden-bank commitment or preprocessing invalidates the confirmatory claim for that version. Continue exploratory work under a new version rather than rewriting historical results.

## 5. Adaptive measurement is not candidate tuning

Keep distinct:
- selector objective and internal scores;
- candidate decisions and outputs;
- evaluator-only outcomes;
- analysis and claim decision.

The selector's objective is measurement efficiency—expected reduction in uncertainty about the preregistered candidate-vs-comparator contrast per unit test cost—subject to quotas and guardrails. It cannot adapt the candidate, rewrite the scoring rules, or choose to suppress inconvenient regimes.

When repeated interim looks are not covered by a frozen sequential method and multiplicity policy, the confirmatory claim is invalidated. A new analysis does not retroactively restore validity.

## 6. Schemas

- [CIV_IMPACT_ADAPTIVE_PREREGISTRATION_V0_1.schema.json](schema/CIV_IMPACT_ADAPTIVE_PREREGISTRATION_V0_1.schema.json): frozen hypothesis, artifact, selector, statistical, exposure and claim-ceiling profile.
- [CIV_IMPACT_ADAPTIVE_EXPOSURE_LEDGER_V0_1.schema.json](schema/CIV_IMPACT_ADAPTIVE_EXPOSURE_LEDGER_V0_1.schema.json): evaluator-only selection/execution/freeze/reveal/rotation chain with mutation-invalidates-claim flags.

Schema-valid JSON is not sufficient. The Rust verifier binds the exact preregistration file-byte digest and sealed-bank commitment, matches candidate/comparator/evaluator/scorer identities, checks cumulative evaluation cost and declared per-regime/random-audit quotas, requires a design-aware analysis for confirmatory completion, and semantically verifies digest chaining, selection counts, event transition order, disclosure after result freeze, probability completeness, mutation invalidation, and no-tune-after-reveal. A valid descriptive-only campaign may finish, but it cannot produce a confirmatory claim. The random-audit check verifies the ledger's declared labels/counts, not statistical randomness or independence from candidate outputs; a sealed RNG/seed-commitment receipt and independent seed custodian remain required. The verifier does not sign or externally anchor the log.

## 7. Initial acceptance gates

These are contract goals, not performance claims:
- zero candidate-facing leaks of sealed identities/truth before result freeze;
- 100% accounting of eligible, selected, executed, incomplete, frozen and disclosed cases;
- every material post-freeze mutation causes explicit campaign invalidation;
- public regression, adaptive-selected and independent random-audit results remain separate;
- all safety-critical regimes meet preregistered minimum coverage;
- unselected eligible challenge decisions and their reasons are included in the evaluator ledger;
- cost per completed evaluation and cost per uncertainty reduction are reported, without describing a selector-score gain as candidate capability;
- no new-domain performance claim without the existing transportability receipt.

## 8. Research anchors

- NIST [TEVV-Athlon Framework](https://www.nist.gov/artificial-intelligence/ai-research/tevv-athlon-framework-evaluating-ai-systems): context-specific evaluation of outcomes, capabilities and negative impacts.
- Nageshwaran et al., [Large language model agent evaluation and benchmarking: systematic survey and critical roadmap](https://link.springer.com/article/10.1007/s10462-026-11678-4), published 15 September 2026. The review frames contamination, nondeterminism and cost records as a joint validity problem in its reviewed sample; its sampling and screening limitations constrain how prevalence estimates should be generalized.
- Zhuang et al., [Position: AI Evaluation Should Learn from How We Test Humans](https://proceedings.mlr.press/v267/zhuang25e.html), ICML 2025: motivates adaptive testing as a complement to static gold-standard evaluation.
- Sun et al., [The Emperor's New Clothes in Benchmarking?](https://proceedings.mlr.press/v267/sun25t.html), ICML 2025: evaluates the fidelity and contamination resistance of benchmark-mitigation strategies.

## 9. Non-goals

No secret challenge material in the public repository; no second identity/provenance/authorization system; no external action or evidence acquisition; no score based on selector choice alone; and no economic-benefit or broad generalization claim from this protocol.
