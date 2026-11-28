# ITOPS compound fault interaction reference v0.1

Public known-answer fixture for [ITOPS-CONTINUOUS-001 #6864](https://github.com/Luminous-Dynamics/mycelix/issues/6864).

This is an evaluator/contract fixture. It is not a live Kubernetes environment, a fault injector, an agent task feed, or a hidden challenge set.

## What it tests

Each interaction group contains a matched 2x2 factorial:
- A0B0: neither causal factor present;
- A1B0: factor A only;
- A0B1: factor B only;
- A1B1: both factors present.

The primary interaction contrast is the difference-in-differences:

`Y11 - Y10 - Y01 + Y00`

where `Y` is the independently measured unrecovered degradation duration after the transient trigger has been removed. A synergistic interaction has a positive contrast; the additive control has contrast zero. The categorical terminal state (persistent metastable degradation) is reported separately and is never inferred from the scalar contrast alone.

Two groups are included:
1. **Metastable retry/capacity interaction:** retry amplification plus a binding CPU constraint creates persistent degradation that neither factor creates alone. The declared complete remedy requires correcting the retry policy, restoring adequate CPU quota and performing a controlled restart.
2. **Additive control:** DNS latency and cache capacity create independent recoverable delays; the joint outcome equals the sum of the individual outcomes, so the interaction contrast is zero.

The factors and oracle are public in this reference fixture. A future evaluated agent must receive only permitted observations/actions, not factor assignments or terminal answers. Private confirmatory interaction groups must use existing Mycelix #6031 challenge custody.

## Validation

- Draft 2020-12 JSON Schema.
- jq semantic gate for exact 2x2 arm coverage, matched run profile, counterfactual interaction contrast, terminal persistence and hidden oracle policy.
- Nine adversarial mutation controls.

The jq gate is a dependency-light semantic validator, not a full JSON Schema engine. Cross-arm equality and difference-in-differences are checked in jq because JSON Schema does not express those relationships conveniently.

## Reuse

- #6864 owns the continuous benchmark and scenario portfolio.
- #6851 owns ITOPS action/outcome mapping.
- #6140 owns observation provenance, dependency relationships and time semantics.
- #6357 owns independently verified terminal outcomes.
- #4835 owns evaluator/oracle separation.
- #5900 owns experiment identity and artifact lineage.
- #6031 owns private holdout custody.

This fixture adds no new runtime, ledger, authority model, experiment envelope, or challenge escrow.

## Claim ceiling

A passing fixture shows that the evaluator can distinguish a synthetic synergistic interaction from a matched additive control and rejects specified mutations. It does not demonstrate that an agent detects or repairs metastable failures, validate a physical Kubernetes environment, or establish superiority over SREGym or SRE-Marathon.

Prior art: [SREGym](https://arxiv.org/abs/2605.07161) documents compound metastable failures formed by an application-level trigger interacting with an infrastructure resource constraint. Its 90-problem portfolio includes metastable and correlated failures; this small public factorial fixture tests one synthetic known-answer pattern, not the full SREGym challenge distribution.
