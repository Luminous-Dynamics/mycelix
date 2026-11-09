# DelibBench v0.1 — research protocol and structural fixtures

**Status:** proposal / synthetic conformance only  
**Owning issue:** [Mycelix #5999](https://github.com/Luminous-Dynamics/mycelix/issues/5999)  
**Current protocol PR:** [Mycelix #6000](https://github.com/Luminous-Dynamics/mycelix/pull/6000)

DelibBench separates three evaluation questions that are often conflated:

1. **Opinion summarization:** does an AI summary represent each person's position, uncertainty and evidence—including minority positions?
2. **Group interaction:** how do different assistance conditions affect group discourse, information sharing, influence, trust, agency and actual decisions?
3. **Institutional follow-through:** do participant contributions and recommendations receive traceable responses, authorized decisions, implementation evidence, appeals and outcome observations?

A model that performs well on one layer has not thereby qualified the next.

## Repository artifacts

- `delib-bench-protocol-v0.1.json` — protocol, intervention arms, outcome families, claim ceilings and research basis.
- `delib-bench-protocol-v0.1.schema.json` — strict schema for the protocol manifest.
- `delib-bench-fixtures-v0.1.json` — 30 deterministic synthetic fixtures: 24 red-team cases across 12 attack families and 6 positive controls.
- `delib-bench-fixtures-v0.1.schema.json` — strict fixture-corpus schema.
- `tools/delib-bench-qualifier/` — Rust structural and fixture-oracle validator.
- `.github/workflows/delib-bench-qualifier.yml` — path-filtered hosted qualification workflow.

## Local command

From the repository root, with a stable Rust toolchain:

```bash
cargo test --manifest-path tools/delib-bench-qualifier/Cargo.toml

cargo run --manifest-path tools/delib-bench-qualifier/Cargo.toml -- \
  docs/research/deliberation/delib-bench-protocol-v0.1.json \
  docs/research/deliberation/delib-bench-protocol-v0.1.schema.json \
  docs/research/deliberation/delib-bench-fixtures-v0.1.json \
  docs/research/deliberation/delib-bench-fixtures-v0.1.schema.json
```

The qualifier implements a deliberately bounded JSON-Schema subset. It rejects unsupported schema keywords and unsupported patterns instead of silently claiming full JSON-Schema conformance. It checks required fields, types, constants/enums, the declared pattern and URI shape, minimums, unique arrays, nested properties, fixture taxonomy, expected evaluator violations and terminal dispositions.

The evaluator has deterministic oracles for:
- omission, duplication, misattribution or stance distortion of source claims;
- loss of uncertainty or source provenance;
- false consensus and unsupported authority claims;
- followed prompt injection and adopted leading framing;
- changes to a fixture's declared presentation order or reference allocation;
- a scenario-specific attention-share cap;
- duplicate principal counting.

The attention-share cap is a synthetic fixture oracle, **not** a universal fairness, participation or airtime standard. The fixture allocation comparison is also only a known-answer control; it does not assert that the reference allocation is normatively correct.

## Research grounding and external baselines

### Human-grounded opinion summarization

[DeliberationBank (COLM 2026)](https://arxiv.org/abs/2510.05154) reports 3,000 participant opinions over ten U.S. societal questions and 4,500 summary judgments from 7,500 participants, across representativeness, informativeness, neutrality and policy approval. Its key methodological advantage is that a generated summary is evaluated in relation to an individual participant's opinion, rather than only through a single global or LLM-judge score.

The [project publishes code and data](https://github.com/shuyhere/LLMs-for-Scalable-Deliberation) and the [dataset page currently shows an MIT license](https://huggingface.co/datasets/babytreecc/DeliberationBank). Before import, pin exact artifacts and review license, provenance, consent context, scope and data-use requirements. The corpus is a U.S.-topic opinion-summary benchmark; it is **not** a randomized group-interaction study or evidence of public-institution legitimacy. Policy approval must remain separate from neutrality and representativeness.

### Group-interaction experiments

- [Real-Time Group Dynamics with LLM Facilitation (FAccT 2026)](https://arxiv.org/abs/2605.14097) reports two experiments with 879 participants in groups of three. Participants preferred LLM facilitation, but the study did not find significant consensus or measured participation-equity improvements; some strategies shifted actual charity allocations.
- [Improving Group Deliberation through AI-Guided Self-Reflection (27 October 2026)](https://arxiv.org/abs/2610.37862) reports a randomized experiment with 1,173 U.S. participants across 391 groups. Non-directive pre-deliberation reflection improved measured Discourse Quality Index outcomes in that study; the reported effect sizes are study-specific and require replication.
- [Bringing Everyone to the Table / GRAIL (PACM HCI 2026)](https://arxiv.org/abs/2508.08242) reports a preregistered randomized study with 1,475 participants in 281 five-person groups. LLM facilitation increased information shared, while final decision quality did not significantly improve in the studied hidden-profile task. The authors released an open-source research platform and experimental data.

These results motivate **separate treatment arms**: unassisted discussion, human facilitation, platform-native workflow, non-directive pre-reflection, retrospective summaries, and in-conversation AI facilitation. Do not pool them into one "AI-assisted" condition.

### Process and impact evaluation

The [OECD evaluation guidelines](https://www.oecd.org/en/publications/evaluation-guidelines-for-representative-deliberative-processes_10ccbfcb-en.html) distinguish process-design integrity, deliberative experience and pathways to impact. DelibBench uses those as a lifecycle crosswalk, not as a single scalar score.

## Industry comparison

DelibBench does not aim to replace existing participation interfaces:

- [Decidim](https://decidim.org/) provides participatory-process lifecycle, proposals, assemblies and accountability/result tracking.
- [Loomio](https://www.loomio.com/) provides asynchronous group discussion, proposal templates, polls and decision records.
- [Polis](https://pol.is/) supports open-ended input, agree/disagree participation, opinion grouping and surfacing cross-group statements.
- [Stanford Online Deliberation Platform](https://stanforddeliberate.org/) supports structured synchronous small-group deliberation, speaker queues, agendas, moderation and analytics.
- [GRAIL](https://arxiv.org/abs/2508.08242) provides an experimental group-AI interaction platform and hidden-profile evaluation paradigm.

Comparisons must match intended process size, decision type, population, language, accessibility, platform configuration and outcome. The potential Mycelix differentiation is a portable evidence/authority/correction/decision-to-outcome layer—not a claim of a superior discussion UI.

## Claim ceiling

- Passing the Rust structural qualifier establishes only that the exact checked-in manifests and synthetic fixtures satisfy the encoded structural and fixture-oracle contracts.
- It does not measure model quality, actual human participation, legitimacy, fairness, inclusion, trust, welfare, or causal policy impact.
- Human evaluation requires applicable ethics review, consent, privacy safeguards, preregistration and independent human adjudication.
- The model or platform under evaluation must not be the sole evaluator of its own deliberative quality.
- No universal consensus, deliberation-quality, fairness, legitimacy or flourishing score is defined.
- No production voting rule, eligibility condition, authority grant or moderation policy is changed by this research.

## Validation state for this branch

The manifest and fixture JSON were fetched back from GitHub. An independent local structural replay checked both schemas and all 30 expected fixture dispositions: 24 red-team cases were detected, 6 positive controls passed, and the fixture families matched the protocol registry. Three protocol negative controls were also rejected.

The execution environment used for authoring does not have `cargo` or `rustc`; the Rust tests and hosted GitHub Actions workflow have **not** been run here. Do not treat authored tests or a workflow file as a CI pass.
