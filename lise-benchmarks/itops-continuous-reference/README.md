# ITOPS continuous operations — public scenario reference v0.1

This directory is the first **scenario-contract fixture** for [ITOPS-CONTINUOUS-001 #6864](https://github.com/Luminous-Dynamics/mycelix/issues/6864), beneath [ITOPS-CAP-001 #6846](https://github.com/Luminous-Dynamics/mycelix/issues/6846).

The companion compound-cause known-answer fixture is [PR #6872](https://github.com/Luminous-Dynamics/mycelix/pull/6872). Scenario admission and postmortem curation are tracked separately in [ITOPS-SCENARIO-CURATE-001 #6873](https://github.com/Luminous-Dynamics/mycelix/issues/6873).

It is not an SRE runtime, environment provisioner, agent, outcome ledger, authorization service, simulator, or hidden challenge set. It is a public contract fixture for checking that a long-horizon scenario can represent persistent services, visible observations, evaluator-only fault schedules, overlap, and independent terminal propositions.

## Contents

- `itops-continuous-scenario-v0.1.schema.json`: Draft 2020-12 JSON Schema.
- `itops-continuous-scenario-v0.1.json`: one public, seeded, deterministic 2-hour reference timeline.
- `validate.sh`: jq structural/semantic contract gate.
- `mutation-tests.sh`: ten evaluator-contract negative transformations.
- `.github/workflows/itops-continuous-reference.yml`: small exact-revision workflow.

## Scenario semantics

The fixture includes:
- Three services across two zones, with explicit dependency edges.
- A deterministic 7,200-second horizon and 30-second simulation tick.
- Eight evaluator-only fault/change events across eight families.
- Overlapping faults, a benign false-positive signal, a planned deployment, stale authorization, a telemetry blind spot, delayed recurrence and a dependency partition.
- Eight operator observations delivered only after their observation time. Each scheduled event is linked only to observations occurring within that event's declared start/duration window; the semantic gate rejects impossible cause-after-observation timelines.
- Four terminal propositions requiring independent verification.

The agent receives only the time-filtered observation stream and its permitted action/tool surface. It must not receive the fault schedule, hidden causal labels, or terminal oracle. The schedule and terminal propositions are public in this *reference fixture* so maintainers can inspect validator semantics; they must be stripped from the evaluated agent's input by the future adapter.

## Reuse boundaries

- #6864 owns the continuous operations benchmark protocol and environment roadmap.
- #6851 owns ITOPS action/outcome semantics.
- #6357 owns the general VerifiedOutcomeLedger.
- #6140 owns observer provenance, shared-dependency and temporal evidence semantics.
- #4835 owns evaluator/oracle separation.
- #5900 owns run identity, artifact lineage and immutable experiment envelope.
- #6031 owns custody and purpose-bound access for future private confirmatory challenge sets.

This fixture must not create a parallel event store, ledger, authority profile, run manifest, or escrow mechanism. The scenario contract is narrower than the experiment/run envelope.

## Validation and claim ceiling

The jq validator is a dependency-light structural/semantic gate, **not a complete JSON Schema implementation**. In local authoring validation, the Draft 2020-12 schema passed `jsonschema` 4.26.0 meta-validation and the public scenario conformed; that schema-aware check is not yet wired into the hosted workflow. Referential integrity (such as fault-target service IDs and observation references) is enforced by the jq semantic gate rather than by JSON Schema.

A passing public fixture proves only that this manifest shape satisfies its declared structural/temporal controls. It does not prove that an environment can be provisioned, an agent can operate it, recovery occurred, or any system beats SRE-Marathon, Incident-Arena, SWEOps-Bench or ITBench-AA. Hosted exact-head CI is the qualification authority for this repository artifact.

External comparator contracts:
- [SREGym](https://github.com/SREGym/SREGym): 90 live high-fidelity SRE problems spanning cloud-native layers, ambient noise, metastable and correlated failures; its current SREGym-Lite-1004 leaderboard cohort is a separate 17-fault slice.
- [SRE-Marathon](https://arxiv.org/abs/2609.33023): continuous change-driven operations with overlapping incidents and deterministic sealed-bundle scoring.
- [Incident-Arena](https://arxiv.org/abs/2610.00648): production-grounded repairs with functional verification and sustained load.
- [SWEOps-Bench](https://arxiv.org/abs/2611.06099): source/configuration repair in live Kubernetes operator environments.
- [ITBench-AA](https://artificialanalysis.ai/evaluations/itbench-aa): root-cause diagnosis from Kubernetes incident snapshots; diagnosis is not recovery.

This v0.1 reference is a small contract fixture, not a reproduction of SREGym's 90-problem portfolio or the 17-fault SREGym-Lite cohort. Metastable, correlated, OS/hardware and postmortem-derived families belong in later held-out scenario waves after the first environment/replay path is qualified.
