# CrossProfile ownership and attribution extension — design v0.1

**Status:** preregistration design only; no tasks, models or participants executed  
**Parent:** Mycelix #6089 / DELIB-DIST-INFO-001

## Research trigger

Imel et al., *Rare, Relevant, and Still Wrong: How Memory Fails LLM Group Decisions* (arXiv:2610.37624, 27 October 2026) introduce CrossProfile. The preprint holds fact text and decision roles fixed while crossing whether supporting, decoy and irrelevant fact groups are shared or private. It specifies 60 main templates, four agents, three options, 12 facts (5 support, 3 decoy, 4 irrelevant), eight ownership variants and 480 template-variants. It also tests attribution tags and reports that source-holder metadata can act as social proof even when the underlying facts do not change.

Source: https://arxiv.org/abs/2610.37624

This is a preprint diagnostic, not a settled universal baseline. No underlying text, task payload or participant data is copied into this repository.

## Paradigm shift

Do not infer decision value from:
- rarity (how many agents hold or state a fact);
- relevance (whether the fact is on topic);
- repetition (how many times it appears);
- holder count or status attribution.

Instead test whether the memory preserves **the decision-relevant balance of verified support, counterevidence, scope, uncertainty and independent source origins**.

A fact can be:
- on-topic but support a decoy option;
- rare but irrelevant;
- repeated but derived from one origin;
- common but genuinely independently corroborated;
- decision-relevant but superseded or rights-restricted.

## Factorial design

Each independently authored template has:
- four synthetic agents;
- three candidate options;
- 12 fictional, evaluator-labelled facts;
- five support facts, three decoy facts and four irrelevant facts;
- stable fact text, claim identity, evidence origin, evidence version, scope and decision-role metadata;
- a deterministic answer/payoff oracle hidden from candidate agents.

For each template, cross three independent ownership factors:

| Factor | Shared level | Private level |
|---|---|---|
| Supporting facts | Every agent initially holds each support fact | Each support fact is initially held by one assigned agent |
| Decoy facts | Every agent initially holds each decoy fact | Each decoy fact is initially held by one assigned agent |
| Irrelevant facts | Every agent initially holds each irrelevant fact | Each irrelevant fact is initially held by one assigned agent |

This yields (2^3=8) ownership variants per template. Fact text and answer key remain identical across all eight variants. Only assignment/visibility changes. Candidate agents never receive the evaluator-only fact-role labels, complete holder map or answer key.

A prospective full programme uses 60 independently authored templates across 12 broad decision domains (five templates per domain), eight variants, and three fixed seeds: 1,440 task-seed executions per model/policy/communication condition. The count is a research target informed by the CrossProfile design, not existing evidence in this repository.

## Attribution intervention: orthogonal to ownership

Within a separate, explicitly matched control:
1. true source attribution tags;
2. attribution tags masked while fact text/order are unchanged;
3. attribution tags shuffled while fact text/order are unchanged.

The true owner map remains evaluator-side. A shuffled attribution condition must never overwrite the immutable evidence origin in the evaluator ledger. It changes only the information presented to the model. This permits measurement of social-proof displacement without corrupting evidence provenance.

## Memory and communication baselines

At minimum compare:
- empty memory;
- all-facts / unbudgeted upper-bound memory;
- full transcript;
- first-mention;
- random selection;
- rarest-first;
- relevance-first;
- evidence-preserving summary retaining conditions, uncertainty and disagreement;
- source-origin-aware evidence-balance summary.

The evidence-balance method must not see evaluator role labels or the correct answer. It may use only candidate-visible claim content, declared decision criteria, observed evidence and provenance that has actually been received. It must preserve opposing support, decoy evidence, uncertainty, scope and source lineage rather than silently resolving them.

Use both:
- fixed token budgets (for example 0.4 / 0.6 / 0.8 of the frozen full-evidence input under a pinned tokenizer); and
- fixed-item-count controls, because ownership tags and metadata can change token length.

Pin model/provider/revision, prompts, temperatures, decoding, tokenizer, agent count, communication rounds, task/seed, token budget, context length, tool access, memory policy and evaluator version. Keep line order fixed in attribution-only contrasts.

## Outcomes and primary comparisons

Report each axis separately:

1. **Decision outcome:** accuracy and regret against the independent answer/payoff oracle.
2. **Evidence retention:** fraction of each role/ownership stratum retained (support, decoy, irrelevant; shared, private).
3. **Evidence balance:** net retained support vs. competing/decoy evidence under the known synthetic scoring rule.
4. **Source integrity:** distinct source origins retained; duplicate copies not counted as independent corroboration.
5. **Attribution sensitivity:** decision and retention displacement from true vs. masked vs. shuffled tags.
6. **Provenance fidelity:** source, version, scope, time validity, uncertainty and contradiction preservation.
7. **Acquisition loss:** facts never surfaced before memory construction.
8. **Compression loss:** facts surfaced in discussion but dropped by memory.
9. **Reasoning loss:** facts retained but not correctly used by the final decision.
10. **Cost:** tokens, requests, latency and compute under matched budgets.

A correct final choice by luck is not sufficient. A faithful memory that drops all useful counterevidence is not sufficient. A high-accuracy system that invents independent support from duplicated sources is not sufficient.

## Preregistered contrasts

- Does the advantage of rarest-first depend on support ownership and decoy ownership?
- Does relevance-first admit true-but-misleading decoy facts because they are on-topic?
- Does a provenance-aware evidence-balance memory outperform first-mention, random, rarest-first and relevance-first under matched token and item budgets?
- How much decision displacement is caused by attribution tags alone when content and order remain fixed?
- Is failure upstream (facts not elicited), during compression (facts dropped), or downstream (retained facts misweighted)?
- Does performance remain stable in the worst ownership cell, not only on the eight-cell average?

Primary analysis should cluster by template and preserve all eight paired variants and seeds within each resample. A preregistered primary budget and outcome must be chosen before model runs; other budgets and model/policy slices remain secondary or exploratory. Use confidence intervals and appropriate multiple-comparison correction. Do not choose the best-performing budget after seeing results and label it primary.

## Qualification targets

Before claiming superiority:
- 100% exact known-answer reproduction by the independent synthetic oracle;
- zero candidate-visible evaluator labels, answer keys or hidden holder maps;
- zero unauthorized evidence admission;
- zero repeated-source inflation;
- no silent omission of an unresolved contradiction;
- matched token and item-count comparisons;
- improvement over baselines with uncertainty intervals, with no hidden worst-cell collapse;
- source- and domain-disjoint holdouts before any broad generalization claim.

No arbitrary universal accuracy percentage is asserted before feasibility and baseline runs. The objective is a robust Pareto improvement in decision quality, evidence balance, provenance fidelity and resource cost—not a single leaderboard score.

## Claim ceiling

This design is not a task corpus, model evaluation or human study. It does not reproduce CrossProfile, HiddenBench or human collective reasoning. A future evaluation may support claims only for its frozen synthetic task set, pinned models, prompts, budgets, ownership assignments, attribution conditions and evaluator. Human-subject work remains a separate ethics/consent/compensation/accessibility/withdrawal/retention gate.
