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
- `delib-bench-fixtures-v0.1.json` — 32 deterministic synthetic fixtures: 26 red-team cases across 13 attack families and 6 positive controls.
- `delib-bench-fixtures-v0.1.schema.json` — strict fixture-corpus schema.
- `delib-summary-baseline-intake-v0.1.json` / `.schema.json` — metadata-only inventory of DeliberationBank code, dataset and model artifacts, with explicit data-use and checkpoint-loading gates.
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

The qualifier implements a deliberately bounded JSON-Schema subset. It recursively audits the entire schema tree—including optional properties absent from the current instance—and rejects unsupported schema keywords, patterns and formats instead of silently claiming full JSON-Schema conformance. It checks required fields, types, constants/enums, the declared pattern and URI shape, minimums, unique arrays, nested properties, fixture taxonomy, expected evaluator violations and terminal dispositions.

The evaluator has deterministic oracles for:
- omission, duplication, misattribution or stance distortion of source claims;
- loss of uncertainty or source provenance;
- false consensus and unsupported authority claims;
- followed prompt injection and adopted leading framing;
- changes to a fixture's declared presentation order or reference allocation;
- a scenario-specific attention-share cap;
- duplicate principal counting and exact roster parity against speakers represented in the source claims;
- unsupported source citations and uncertainty labels that are not grounded in source claims.

The attention-share cap is a synthetic fixture oracle, **not** a universal fairness, participation or airtime standard. The fixture allocation comparison is also only a known-answer control; it does not assert that the reference allocation is normatively correct.

The principal-roster check is bounded to the principals represented by source claims in each fixture; it is not a complete participation, eligibility, abstention, or identity-proof theorem. Source citations and uncertainty labels are checked for both missing required entries and unsupported extras.

**Important limit of the synthetic claim fixtures:** the `representations` field is a controlled, typed claim-to-claim mapping, not free-form generated natural language. Exact field preservation is a deliberately narrow oracle for source identity, attribution, epistemic status and omission. It does not score acceptable paraphrase/compression, participant-conditioned representativeness, informativeness, neutrality or policy approval. Those require human-grounded summary evaluation (for example, DeliberationBank where data-use review permits) and independent human adjudication. The synthetic corpus is a structural/red-team control, not a language-summary benchmark.


## Research grounding and external baselines

### DeliberationBank intake and baseline-safety status

The current public artifact audit is recorded in [#6011](https://github.com/Luminous-Dynamics/mycelix/issues/6011), with a metadata-only machine-readable snapshot in `delib-summary-baseline-intake-v0.1.json`.

Important separations:
- The upstream code repository is Apache-2.0; the Hub dataset metadata declares MIT; the Hub model metadata declares Apache-2.0. These are three distinct artifacts and license scopes.
- The dataset Hub page currently exposes ten topic rows / a comments array; this does not establish that the paper's 4,500 summary-judgment annotations are available in that artifact.
- The dataset and model card README files are metadata-only in the current Hub rendering. The upstream dataset notes online-survey collection but does not disclose enough participant consent, ethics, recruitment, withdrawal or redistribution context for our data-use gate.
- The upstream DeBERTa evaluator can fall back to randomly initialized weights when checkpoint loading fails. This is a baseline-integrity blocker; see [#6014](https://github.com/Luminous-Dynamics/mycelix/issues/6014). The benchmark path must verify exact safetensors/tokenizer/config hashes, reject fallback and avoid loading pickle-bearing `training_args.bin`.

The current intake state is **metadata-only / review pending**. No participant-level data or model payloads have been downloaded into Mycelix; no full reproduction is claimed. The validator enforces that the rights gate stays blocked, the baseline remains unsupported, payload-download flags remain false, and the fail-closed model requirements remain present.

### Human-grounded opinion summarization

[DeliberationBank (COLM 2026)](https://arxiv.org/abs/2510.05154) reports 3,000 participant opinions over ten U.S. societal questions and 4,500 summary judgments from 7,500 participants, across representativeness, informativeness, neutrality and policy approval. Its key methodological advantage is that a generated summary is evaluated in relation to an individual participant's opinion, rather than only through a single global or LLM-judge score.

The [project publishes code and data](https://github.com/shuyhere/LLMs-for-Scalable-Deliberation) and the [dataset page currently shows an MIT license](https://huggingface.co/datasets/babytreecc/DeliberationBank). Before import, pin exact artifacts and review license, provenance, consent context, scope and data-use requirements. The corpus is a U.S.-topic opinion-summary benchmark; it is **not** a randomized group-interaction study or evidence of public-institution legitimacy. Policy approval must remain separate from neutrality and representativeness.

### Participant-conditioned opinion-summary evaluation

- [#6011 — participant-conditioned opinion summaries against human-grounded data](https://github.com/Luminous-Dynamics/mycelix/issues/6011) is a sibling child of #5999. It evaluates representativeness, informativeness, neutrality and policy approval separately against DeliberationBank, subject to exact artifact/license/consent review.
- Keep summary generation separate from summary evaluation. Use participant-conditioned judgments, topic-aware split controls, minority-position and uncertainty diagnostics, order sensitivity, and independent human adjudication.
- The 32 structured fixtures in this PR remain exact provenance/representation controls; they do not judge free-form paraphrase quality or lived participant experience.

### Group-interaction experiments

- [Real-Time Group Dynamics with LLM Facilitation (FAccT 2026)](https://arxiv.org/abs/2605.14097) reports two experiments with 879 participants in groups of three. Participants preferred LLM facilitation, but the study did not find significant consensus or measured participation-equity improvements; some strategies shifted actual charity allocations.
- [Improving Group Deliberation through AI-Guided Self-Reflection (27 October 2026)](https://arxiv.org/abs/2610.37862) reports a randomized experiment with 1,173 U.S. participants across 391 groups. Non-directive pre-deliberation reflection improved measured Discourse Quality Index outcomes in that study; the reported effect sizes are study-specific and require replication.
- [Bringing Everyone to the Table / GRAIL (PACM HCI 2026)](https://arxiv.org/abs/2508.08242) reports a preregistered randomized study with 1,475 participants in 281 five-person groups. LLM facilitation increased information shared, while final decision quality did not significantly improve in the studied hidden-profile task. The authors released an open-source research platform and experimental data.
- [Can LLMs Deliberate? / DelibSim (PoliSim @ CHI 2026)](https://openreview.net/forum?id=ggC651T5Vv) evaluates 1,980 five-agent deliberations across 12 policy topics and 11 model configurations. Its reported process-quality measures approach human reference levels, while LLM groups show substantially lower perspective diversity and different convergence dynamics. Treat this as a complementary agent-group benchmark, not a substitute for consented human-group evidence or a directly matched DelibBench baseline.

These results motivate **separate treatment arms**: unassisted discussion, human facilitation, platform-native workflow, non-directive pre-reflection, retrospective summaries, and in-conversation AI facilitation. Do not pool them into one "AI-assisted" condition.


### Perspective diversity is an outcome, not a cosmetic statistic

DelibSim makes a measurement gap explicit: a group may produce fluent, orderly, high-quality discourse while retaining substantially less viewpoint diversity than human reference groups. DelibBench should therefore report, as separate dimensions, (1) discourse/process quality, (2) perspective diversity and coverage, (3) warranted movement after evidence, (4) unsupported convergence or influence, and (5) task-specific decision outcome. Do not reward minimum stance variance or maximum consensus. Diversity should be interpreted relative to the task's evidence and legitimate normative pluralism, not as a universal preference for disagreement. Pair the aggregate group profile with participant-conditioned voice fidelity from #6011 and the private/public revision trajectory in Symthaea #11320.

### Process and impact evaluation

The [OECD evaluation guidelines](https://www.oecd.org/en/publications/evaluation-guidelines-for-representative-deliberative-processes_10ccbfcb-en.html) distinguish process-design integrity, deliberative experience and pathways to impact. DelibBench uses those as a lifecycle crosswalk, not as a single scalar score.

## Typed argument and open-world evidence semantics

DelibBench composes with the existing Mycelix domain owners rather than defining another argument graph:

- [#3116 — typed issue/claim/objection graph](https://github.com/Luminous-Dynamics/mycelix/issues/3116) owns issue, claim, evidence, support/challenge, objection, mitigation, and decision lineage. It requires unresolved objections and superseded claims to remain traceable.
- [#3126 — open-world uncertainty and contested evidence](https://github.com/Luminous-Dynamics/mycelix/issues/3126) owns simultaneous conflicting evidence, contextual verification, first-class unknown/indeterminate states, and decisions made under declared uncertainty.

DelibBench should use those semantics to test whether AI summaries and facilitated processes preserve supporting and challenging evidence, source attribution, unresolved objections and uncertainty. A summary that drops an unresolved challenge can appear more coherent while making the group less informed. A decision reached under uncertainty may be valid as a process outcome; it must not rewrite the evidence as settled.

Structural conformance of an argument graph and human-coded deliberative quality remain different evaluation dimensions.

## Industry comparison

DelibBench does not aim to replace existing participation interfaces:

- [Decidim](https://decidim.org/) provides participatory-process lifecycle, proposals, assemblies and accountability/result tracking.
- [Loomio](https://www.loomio.com/) provides asynchronous group discussion, proposal templates, polls and decision records.
- [Polis](https://pol.is/) supports open-ended input, agree/disagree participation, opinion grouping and surfacing cross-group statements.
- [Stanford Online Deliberation Platform](https://stanforddeliberate.org/) supports structured synchronous small-group deliberation, speaker queues, agendas, moderation and analytics.
- [GRAIL](https://arxiv.org/abs/2508.08242) provides an experimental group-AI interaction platform and hidden-profile evaluation paradigm.

Comparisons must match intended process size, decision type, population, language, accessibility, platform configuration and outcome. The potential Mycelix differentiation is a portable evidence/authority/correction/decision-to-outcome layer—not a claim of a superior discussion UI.

## South African multilingual extension

The protocol now records a **future** language-access extension rather than implying that U.S.-English research baselines transfer automatically to South Africa.

Initial proposed text profiles are South African English (`en-ZA`), Afrikaans (`af-ZA`), isiZulu (`zu-ZA`), isiXhosa (`xh-ZA`) and Sesotho (`st-ZA`). This is not a claim to cover all local languages, dialects, registers or communities. Each exact profile needs native-authored or independently native-adjudicated tasks, separate performance/uncertainty reporting, code-switching and orthographic policy, and a correction/appeal path in the language being evaluated. English-pivot evaluation is diagnostic only, not gold authority.

Compose the semantic/language layer with existing Symthaea work rather than duplicate it:
- [BROCA-V27 multilingual semantic fidelity #5347](https://github.com/Luminous-Dynamics/symthaea/issues/5347)
- [VOICE-SEM-001 African speech semantics #11057](https://github.com/Luminous-Dynamics/symthaea/issues/11057)
- [Broca online language acquisition #9607](https://github.com/Luminous-Dynamics/symthaea/issues/9607)
- [MORPH-NARRATIVE-001 South African language morphology #11232](https://github.com/Luminous-Dynamics/symthaea/issues/11232)

[UCT's 2026 MzansiBench project](https://showcase.sit.uct.ac.za/event/showcase-2026/mzansibench-36) describes a benchmark pipeline spanning 13 tasks, 16 datasets and South Africa's 11 official written languages. It is a useful task-coverage crosswalk, not a deliberation benchmark or a substitute for qualified local human references.

South Africa's constitutional text version dated 26 June 2026 includes South African Sign Language among the official languages. Signed-language deliberation therefore belongs on a distinct signed-video/multimodal path with qualified sign-language practitioners and community-led review—not a text-only translation checkbox. See the [2026 constitutional text](https://lawlibrary.org.za/akn/za/act/1996/constitution/eng%402026-06-26). This is design context, not a legal-compliance determination.

No local-language corpus ingestion, recruitment, model training, language-capability promotion or institutional deployment is authorized by the protocol. Exact source rights, consent, language variety, community governance and reviewer competence must be qualified before data collection or reuse.

## Claim ceiling

- Passing the Rust structural qualifier establishes only that the exact checked-in manifests and synthetic fixtures satisfy the encoded structural and fixture-oracle contracts.
- It does not measure model quality, actual human participation, legitimacy, fairness, inclusion, trust, welfare, or causal policy impact.
- Human evaluation requires applicable ethics review, consent, privacy safeguards, preregistration and independent human adjudication.
- The model or platform under evaluation must not be the sole evaluator of its own deliberative quality.
- No universal consensus, deliberation-quality, fairness, legitimacy or flourishing score is defined.
- No production voting rule, eligibility condition, authority grant or moderation policy is changed by this research.

## Validation state for this branch

The checked-in synthetic corpus contains 32 fixtures: 26 red-team cases across 13 registered attack families (two cases per family) and six positive controls. The protocol and fixture manifests were independently checked against their bounded schema contracts, and the fixture evaluator reproduced every expected result.

The Rust qualifier is deliberately bounded; it recursively audits the complete schema tree, rejects unsupported schema keywords/patterns/formats, checks structural and fixture-oracle invariants, and verifies the external baseline-intake gates. It does not claim full JSON Schema conformance.

**Exact-head hosted execution:** GitHub Actions [Deliberation Benchmark Qualifier run #42756023917](https://github.com/Luminous-Dynamics/mycelix/actions/runs/42756023917) completed successfully on exact PR head `44b0059286b3e991b096ea0f851452e36808fc6d`.

The job reported:
- **21 Rust tests passed, 0 failed**.
- Synthetic CLI: `fixtures=32 red_team=26 positive_controls=6 attack_families=13`.
- Metadata-only DeliberationBank intake CLI: PASS, with payload downloads disabled.
- Intake CLI claim ceiling: participant data-use approval, model checkpoint identity, baseline reproduction and evaluator performance remain unresolved.

The external data/model intake manifest deliberately remains `metadata-audit-only-review-pending`, with the data-governance gate `blocked-pending-review` and baseline reproduction `unsupported-pending-artifact-resolution`. No participant-level dataset or model payload was downloaded or added to the repository. The upstream evaluator's random-weight fallback and pickle-bearing training-args artifact are separately tracked in [#6014](https://github.com/Luminous-Dynamics/mycelix/issues/6014).

The generic Mycelix CI workflow was skipped and is not counted as a pass.

This qualifies only the checked-in bounded structural and synthetic fixture-oracle contract and the metadata-only intake guardrails. It does not measure natural-language semantic fidelity, human deliberation quality, AI efficacy, legitimacy, fairness, inclusion, trust, welfare, causal impact, participant consent, data-use rights, production UX or deployment readiness. Any human-subject work remains gated on applicable ethics review, informed consent, privacy safeguards, preregistration and independent human evaluation.
