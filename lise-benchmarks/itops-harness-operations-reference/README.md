# ITOPS harness × operations reference v0.1

Public evaluator-side known-answer fixture for [ITOPS-HARNESS-OPS-001 #6877](https://github.com/Luminous-Dynamics/mycelix/issues/6877), under [ITOPS-CAP-001 #6846](https://github.com/Luminous-Dynamics/mycelix/issues/6846).

This first small contract tests a changing operational world and a changing agent harness together. It complements [ITOPS timeline PR #6868](https://github.com/Luminous-Dynamics/mycelix/pull/6868), [compound-fault PR #6872](https://github.com/Luminous-Dynamics/mycelix/pull/6872), [Symthaea harness evolution #11030](https://github.com/Luminous-Dynamics/symthaea/issues/11030), and [scenario curation #6873](https://github.com/Luminous-Dynamics/mycelix/issues/6873).

It is not an SRE runtime, agent benchmark execution, live Kubernetes deployment, hidden challenge set, tool registry, permission service or outcome ledger.

## Design

Each group contains a matched 2×2 factorial:

| Arm | Operational world | Agent harness |
|---|---|---|
| W0H0 | frozen | frozen |
| W1H0 | changing | frozen |
| W0H1 | frozen | changing |
| W1H1 | changing | changing |

For each outcome, the interaction contrast is the joint-arm outcome minus both single-drift outcomes plus the frozen baseline.

Two public known-answer groups are included:
1. **Synergistic interaction:** joint drift causes extra degraded time, prior-task forgetting, an unauthorized effect and silent inheritance of an invalidated qualification.
2. **Additive control:** the same dimensions have zero interaction, no authority violation and no inherited invalid qualification.

New-capability success is not applicable to frozen-harness arms; it is not coerced to zero. Prior-task retention and new-capability adaptation remain separate outcomes.

## Hidden-input contract

The public manifest exposes the evaluator schedule for maintainers. An evaluated agent must receive only time-gated observations and its permitted action surface—not world-event IDs, harness-transition assignments or terminal answers. A future private challenge adapter must use existing Mycelix challenge custody (#6031).

## Validation

- Draft 2020-12 JSON Schema, fully checked with pinned `jsonschema==4.26.0` in the exact-revision workflow.
- jq semantic gate for exact arm coverage, matched profiles, schedule visibility, transition coverage, known-answer interaction contrasts and hard-gate states.
- Sixteen adversarial mutation controls, including schema-level unknown-property, required-field and numeric-bound failures.
- Exact-revision workflow with pinned validation dependencies.

The jq gate remains a dependency-light semantic check; JSON Schema validation is a separate, explicit gate. The public groups are synthetic known answers, not empirical estimates.

## Reuse and claim boundaries

#6864 owns the continuous operations environment; #6877 owns the operations×harness cross-product; Symthaea #11030 owns generic harness-evolution semantics. #6851/#6853 own ITOPS action/outcome semantics. #6140/#6357/#4835/#5900/#6031 retain provenance, terminal verification, evaluator independence, run lineage and private challenge custody.

A fixture pass does not show that an agent can adapt to harness changes, recover a live system, preserve human capability or outperform any SRE benchmark.

## External comparators

- [EvoHarnessBench](https://arxiv.org/abs/2609.04280): staged evolution of tools, skills and specialist agents.
- [SRE-Marathon](https://arxiv.org/abs/2609.33023): persistent, continuous SRE operations.
- [SWEOps-Bench](https://arxiv.org/abs/2611.06099): verified source/configuration repair for cloud incidents.
- [Incident Arena](https://incident-arena.com/): incident diagnosis, repair and functional verification.

Headline scores are not comparable. Reproduce each native protocol before using it as a baseline; report this cross-product under its own population, effect surface and budget.
