# Praxis–Symthaea Advisor Integration v1

**Contract:** `PRAXIS-SYMTHAEA-ADVISOR-001`  
**Status:** Draft interface and conformance implementation; no live Symthaea adapter is qualified  
**Owner:** Praxis consumer contract; Symthaea is an optional advisory provider  
**Related:** [Mycelix #5472](https://github.com/Luminous-Dynamics/mycelix/issues/5472), [support-decision benchmark #5467](https://github.com/Luminous-Dynamics/mycelix/issues/5467), [planner and durable-learning experiment #5286](https://github.com/Luminous-Dynamics/mycelix/issues/5286), [evidence-bearing cyber-learning track #5124](https://github.com/Luminous-Dynamics/mycelix/issues/5124)

## Decision: yes, use Symthaea more—but in a narrow and measurable role

The first useful integration is an *optional reasoning adviser* for support selection and learning-path proposals. It should augment the existing deterministic Praxis planner, BKT and spaced repetition—not replace them.

Use Symthaea first for:
1. proposing the next learning action when evidence is incomplete or history is long;
2. comparing a small number of candidate learning routes against the learner's goal, prerequisites and constraints;
3. identifying which evidence gaps or counterfactual changes could alter a recommendation;
4. suggesting scaffold fading, retrieval practice, human/peer support, an offline path, clarification or abstention;
5. explaining a recommendation through references to the exact evidence items it used.

Do not use the first integration to generate a whole curriculum autonomously, make a mastery decision, write to the Holochain DHT, update canonical BKT state, change assessment policy, or issue credentials.

## Why the boundary matters

Symthaea's public repository describes a broad experimental Rust cognitive architecture with HDC/CfC, a causal/counterfactual reasoning surface, and a seven-step reasoning engine gated by the `reasoning_engine` feature. The repository's feature map declares that enabling the reasoning engine also enables `magi_loop` and `symthaea-causal-reasoning/counterfactual`. Its native `ReasoningContext` expects Symthaea-specific theory metrics, `phi`, planning actions, epistemic-quality inputs, and a time budget; this is not yet a stable, direct learner-advice DTO. See the [current repository](https://github.com/Luminous-Dynamics/symthaea), [feature map](https://github.com/Luminous-Dynamics/symthaea/blob/main/docs/FEATURE_FLAGS.md), and [reasoning-engine types](https://github.com/Luminous-Dynamics/symthaea/blob/main/src/consciousness/reasoning_engine/types.rs).

A source-level detail materially affects how we should integrate it: the current reasoning engine's `ReasoningContext` accepts a `phi` value, theory metrics, `epistemic_quality`, budgets and generic `PlannedAction` values; its MCTS path uses a `ForkedState` simulation state and returns an action index plus planner confidence. The reviewed path does not itself supply a validated learner-state transition model or educational reward function. Therefore the first production-bound adapter must **not** interpret learner proficiency as `phi`, and must not relabel MCTS plan confidence as calibrated pedagogical confidence. Until a domain-specific learning simulator/reward model is independently validated, treat direct educational action ranking as an experiment. Prefer evidence-conflict analysis, hypothesis/next-evidence suggestions, and counterfactual explanations where the request provides explicit evidence and the result can be checked.

There is also a reason to be ambitious *and* evidence-led. Symthaea's own public README discloses limits and retractions, including a corrected 56.2% overall Hendrycks Ethics result and an internal review in which only one of twelve consciousness-indicator probes has empirical support so far. Those claims are not tutoring benchmarks and should not be used as educational-efficacy evidence. They are examples of the research discipline Praxis should preserve: reproducible comparisons, visible negative results, and explicit claim limits.

Educational research supports a similarly bounded role. LongTutor reports that long-history evidence acquisition is easier than using that evidence well for diagnosis and adaptive teaching ([ACL 2026](https://aclanthology.org/2026.acl-long.1371/)). The [OECD Digital Education Outlook 2026](https://www.oecd.org/en/publications/oecd-digital-education-outlook-2026_062a7394-en.html) emphasizes distinguishing immediate output quality from durable learning and evaluating different deployment models. Field-trial evidence also warns that access or usage alone does not establish benefit: the [August 2026 NBER math experiment](https://www.nber.org/papers/w35621) found benefits in recovery after errors alongside slower progression/fewer attempts and mixed delayed outcomes.

## Architecture

```text
Praxis goal + local progress + purpose-limited evidence
  -> versioned request / exact evidence snapshot
  -> optional Symthaea advisory port
  -> proposal + cited evidence refs + uncertainty + alternatives
  -> strict Rust contract validator
  -> Praxis policy gate
       actual support availability
       course/assessment rules
       accessibility compatibility
       protected human-support obligations
       consent and evidence freshness
  -> learner / educator may accept, edit, reject, or ask
  -> Praxis records the actual attempt and its assistance
  -> later independent assessment of retention and transfer
```

Symthaea is not in the trusted execution path for learner data mutation or credentials. If it is unavailable, declined, malformed, stale, over budget or unable to cite input evidence, Praxis continues with its deterministic planner and ordinary BKT/SRS workflow. The advisory response never becomes canonical learner evidence merely because it is well-formed.

## Rust and wire-contract implementation

The initial consumer contract is in `praxis_core::symthaea_advisor`, with no new crate dependency and no direct dependency on the Symthaea workspace.

- `schemas/symthaea-advisor/v1/request.schema.json`: strict request shape.
- `schemas/symthaea-advisor/v1/advice.schema.json`: strict response shape.
- `examples/symthaea-advisor/v1/synthetic_request.json`: synthetic, non-identifying conformance request.
- `crates/praxis-core/src/symthaea_advisor.rs`: typed DTOs, strict deserialization, an optional-provider port trait, cross-field checks and exact-request BLAKE3 binding.

The response commits to the exact UTF-8 request bytes passed through the adapter. A mismatch prevents acceptance. Evidence references must exist in the request; options must be available and policy-permitted; a course policy that prohibits AI cannot be bypassed because a model is available. Stated accessibility requirements and protected human-support constraints are enforced by Praxis-side validation.

The request's evidence-snapshot digest is only syntax-checked by this boundary. The caller still needs to verify the source snapshot bytes and provenance. V1 does not infer whether each evidence item is recent enough for a decision; the caller must apply source-specific freshness/currentness policy before dispatch and mark insufficient or conflicting evidence honestly. Likewise, a build/feature digest labels a Symthaea configuration but does not independently authenticate the binary.

## A critical semantic separation

```text
Symthaea Phi / consciousness-related metric
    != learner proficiency
    != probability the learner knows a skill
    != calibrated confidence in a pedagogy
    != a credential or authorization

Symthaea proposal
    != accepted Praxis plan
    != learner action
    != independent performance evidence
    != mastery / credential eligibility
```

Do not map raw `phi`, theory conflict scores, model confidence, language fluency, time-on-platform or assistance use into a mastery score. Praxis may later use separately calibrated predictive models as advisory estimates only; they must be named/versioned and validated against independent assessments.

## Privacy and permission

Send a bounded context summary and opaque evidence IDs, not the learner's identity or an unfiltered transcript. Keep direct identifiers, private health/disability details, raw conversations, authentication tokens and sensitive records out of the v1 request. Where an accessibility accommodation is required, represent the functional requirement needed to choose an accessible route, not a diagnosis. Apply data minimization, purpose limitation, local consent, retention controls and a learner-visible explanation.

No response may write to the DHT, mutate learner history, alter course rules, publish evidence, invoke tools, issue credentials or authorize effects. Any future effect remains behind its own Praxis/Mycelix authority and reconciliation boundary.

## Benchmarks and comparison plan

The baseline is the existing deterministic planner/BKT/SRS path without Symthaea. The comparison treatment adds a pinned Symthaea build and named feature profile but leaves the hard gates and final policy validator unchanged. Use identical case bytes, learner-goal inputs, budgets and scoring rules.

Report metrics separately:

| Metric | Initial target / rule |
|---|---|
| Critical policy, accessibility, availability or protected-support violations | **Zero**; one critical violation fails the safety gate |
| Advice evidence-reference validity | 100% in deterministic contract tests |
| Unsupported evidence-reference rate | 0% in the frozen corpus |
| Exact request/build/profile binding | 100% valid; any mismatch makes the run unqualified |
| Acceptable support decision rate | Aspirational ≥90% after independent case adjudication and held-out testing |
| Counterfactual decision accuracy | Aspirational ≥95% on an independently labeled held-out paired set |
| Justified clarification/abstention | Report precision and recall separately; blanket abstention is not success |
| Latency, memory and compute budget | Measure p50/p95 first; set an SLO only after the baseline is observed |
| Independent attempts, delayed retention and transfer | Requires a controlled study; cannot be established by synthetic probes |

First expand the support-selection benchmark from 12 synthetic cases to at least 120 independently adjudicated cases, with held-out scenarios and multiple baselines. Measure the incremental value of Symthaea separately from its runtime/cost. Include a rule-based policy, existing deterministic planner, hint-first, direct-answer negative control, and human-supported policy when feasible. If Symthaea cannot beat the baseline with meaningful uncertainty intervals—or violates any hard gate—keep it opt-in or remove it from that path.

For educational efficacy, preregister an estimand and outcome plan, get appropriate ethics/privacy review, measure no-assistance performance and delayed transfer, and report attrition and subgroup uncertainty. Never use synthetic benchmark performance as evidence that learners learn more.

## Qualification stages

1. **Contract conformance (this tranche):** schemas, typed parser, evidence-reference checks, request-byte binding, and negative controls.
2. **Build an actual adapter:** pin an exact Symthaea commit and minimum feature profile; map only whitelisted request fields to native Symthaea types; produce an external JSON response conforming to this schema. Avoid enabling the full feature bundle by default.
3. **Offline integration test:** compare no-Symthaea baseline and Symthaea-assisted advice on frozen synthetic cases. Record latency, memory, fallback, digest and every violation.
4. **Held-out evaluation:** educator/accessibility review and sealed decision pairs; independently inspect failure cases before any score is published.
5. **Learner pilot:** only after model/API/runtime qualification, with opt-in consent, an approved plan and delayed independent outcome measures.

## Limits

This is a consumer contract and mockable port, not a live Symthaea call. It does not demonstrate compilation, API compatibility, tutor quality, learning gains, consciousness, general intelligence, mastery, or SOTA superiority. The precise Symthaea build and adapter compatibility must be pinned and tested before use. A valid JSON response is not proof that its rationale is true.
