# CIV-IMPACT causal field evaluation v0.1

- **Status:** research protocol and artifact verifier; no trial executed
- **Owner:** Mycelix CIV-IMPACT evaluation family
- **Depends on:** [CIV-IMPACT-001 cost/service census](https://github.com/Luminous-Dynamics/mycelix/pull/5050), [CIV-IMPACT-002 decision-value benchmark](https://github.com/Luminous-Dynamics/mycelix/pull/5054), [CIV-IMPACT-003 sealed challenge exposure control](https://github.com/Luminous-Dynamics/mycelix/pull/5067)
- **Claim ceiling:** specification + synthetic artifact-contract tests only until exact-head CI and independent review

## Thesis

A benchmark score estimates performance on a specified task distribution. It does not, by itself, identify the effect of deploying or using a system. CIV-IMPACT should distinguish three questions:

1. **Capability:** can the system perform a task under a controlled evaluation?
2. **Decision value:** does its evidence-seeking, deferral, and action policy reduce end-to-end loss against a fair baseline?
3. **Causal field impact:** what changes when an eligible person, team, site, or cluster is assigned to use the system instead of the comparator in the target setting?

This tranche addresses question 3 at the artifact-contract level. It does not run experiments, calculate effects, or assert that randomization occurred.

## Paradigm shifts

### 1. From a leaderboard to a target trial

Freeze population, eligibility, intervention, comparator, assignment unit, outcome definitions, observation window, estimand, cost boundary, missingness policy, stopping rules, and analysis before observing results. This makes the deployment decision answerable rather than allowing a generic score to stand in for impact.

### 2. Intention-to-treat as the default causal question

The primary randomized comparison should preserve each unit in its assigned arm, including non-compliance, crossover, failure, withdrawal, and missing outcomes. The intention-to-treat contrast estimates the effect of assignment under the trial's implementation conditions. Per-protocol or treatment-on-treated analyses may be secondary, with their extra assumptions stated, not substituted after seeing results.

### 3. Treat assignment evidence as a first-class artifact

The plan commits to the exact assignment-manifest bytes. The outcome ledger repeats hashes of the frozen plan and assignment manifest. The verifier reconciles all assigned units to exactly one outcome or explicit missing/withdrawal disposition, rejects duplicate or unassigned units, and checks cluster boundaries. It checks file consistency—not allocation authenticity. V0.1 has no signature verifier or independent transparency log, so an external allocator's attestation is a review requirement rather than a cryptographically proven fact.

### 4. Missingness and interference are part of the result

Do not drop unavailable, withdrawn, or non-compliant units from the denominator. Freeze a missing-data plan; report missingness by arm and all disposition counts. Identify the randomization unit and plausible spillovers. For cluster-randomized trials, forbid one cluster from appearing in both arms and require enough clusters per arm under the registered plan. An assumption field does not prove that spillovers are absent; measurement and sensitivity analyses are still required.

### 5. Value is a vector, not a single score

Report the primary outcome and its uncertainty alongside absolute rates/means, harms, subgroup/regime outcomes, full lifecycle costs, participant burden, and operational failure. Hard safety/rights constraints cannot be compensated away by a weighted average. Keep distributional and adverse outcomes visible even when the average improves.

## Data contract

Three closed Draft 2020-12 schemas define:
- **Trial plan:** design and claim tier, interventions, estimand/outcomes, assignment method and attestation, sample-size/precision rationale, analysis and uncertainty, interference, missingness, stopping, ethics/privacy, and cost accounting.
- **Assignment manifest:** one unit/cluster and assigned arm per enrolled unit, with a receipt identifier and timestamp.
- **Outcome ledger:** exactly one row per assigned unit, primary outcome or explicit non-observation, harm outcomes, non-compliance, and protocol-deviation codes.

The Rust CLI validates the schema and verifies semantic consistency. Its success string deliberately means internal artifact consistency, not causal proof or statistical significance.

## V0.1 confirmatory candidate gates

The verifier may label a design **eligible for independent confirmatory review** only if all of the following are satisfied:
- protocol status is frozen and the declared design is randomized individual- or cluster-level;
- assignment method agrees with design, the allocator is described as independent, and an attestation reference is present;
- the primary estimand is intention-to-treat, an uncertainty method is preregistered, and sample-size/precision justification is present;
- the interference assumption is not unknown; cluster assignment fields and cluster counts are consistent;
- all assigned units are represented once, including missing/withdrawn/not-measured cases;
- per-arm counts and missingness meet the frozen protocol limits;
- plan and assignment exact-byte SHA-256 references match the outcome ledger.

Observational and quasi-experimental plans can be described, but v0.1 will not label them confirmatory causal designs. This is a scope limit, not a claim that their evidence is useless.

## Benchmark matrix and proposed targets

These are initial qualification targets to test the verifier and define future study design. They are not achieved results or universal scientific standards.

| Dimension | Target | Qualification method |
|---|---|---|
| Denominator integrity | 100% assigned units reconciled to exactly one outcome/disposition | Missing, duplicate, unknown-ID and omitted-row negative controls |
| Assignment integrity | Zero duplicate units or cross-arm cluster assignments accepted | Duplicate IDs, cluster leakage, changed arm, unplanned unit fixtures |
| Frozen artifacts | Zero changed plan/assignment bytes accepted under the same trial identity | Exact-byte digest mutation tests |
| Missingness | 100% missing/withdrawn/unmeasured outcomes retained explicitly | Per-arm disposition census; exceed-ceiling control |
| Claim discipline | Zero observational/underspecified plans promoted to confirmatory in v0.1 | Descriptive and invalid-design negative controls |
| Outcome semantics | Observed outcome requires a numeric primary value; non-observed statuses require null | Cross-field semantic cases |
| Reproducibility | Identical bytes and inputs produce identical machine-readable verifier report | Exact artifact replay and digest comparison |
| Sample quality | Precision/power rationale tied to smallest decision-relevant effect, clustering, missingness, and multiplicity | Independent protocol review; no fixed-N shortcut |
| Real-world impact | No numeric benefit target until operational baseline, units, costs, harms, and decision threshold are declared | Target-trial review then prospective field evidence |

### Statistical targets for later field work

Before a real study, choose the smallest effect that would change a deployment decision; specify the estimand, uncertainty interval, Type-I error/coverage target, multiplicity/sequential plan, expected missingness, and cluster intraclass correlation if relevant. Sample size follows those assumptions and feasibility—not the fact that a benchmark happens to have thousands of rows. Publish effect estimates and uncertainty even when they are null or negative; avoid post-hoc subgroup promotion.

For multi-site work, report site/cluster-specific and distributional outcomes as well as the registered aggregate. Measure cross-arm spillover and contamination where plausible. Where randomization is infeasible, specify the target-trial emulation and identification assumptions separately; do not silently relabel observational association as an experiment.

## Adversarial contract corpus

Minimum cases:
1. plan byte mutation after manifest commitment;
2. assignment-manifest mutation or wrong digest;
3. outcome ledger bound to a different trial/plan;
4. duplicate unit IDs and an outcome for an unassigned unit;
5. a missing assignment row or outcome row;
6. primary outcome null for an observed row or non-null for a missing/withdrawn row;
7. cluster appears in both arms;
8. minimum per-arm units/clusters not met;
9. missingness exceeds the frozen per-arm maximum;
10. non-randomized design claims confirmatory causal tier;
11. frozen trial with no uncertainty method or unknown interference;
12. changed outcome definition or cost boundary presented under the same protocol version.

## Research basis

- Arbour, Bojinov, Feller & Ni, [“Toward Causal Field Evaluations of AI Systems”](https://hdsr.mitpress.mit.edu/pub/ak16gxi2/release/2), published 11 May 2026. Motivates evaluating assignment effects in real deployment contexts and distinguishing lab scores/preferences from outcome evidence.
- NIST, [TEVV-Athlon Framework for Evaluating AI Systems](https://www.nist.gov/artificial-intelligence/ai-research/tevv-athlon-framework-evaluating-ai-systems), draft announcement 7 August 2026. Motivates use-case-specific assessment of intended outcomes and negative effects.
- Chen et al., [CausalGame](https://proceedings.mlr.press/v306/chen26l.html), ICML 2026. Motivates evaluating experimental-design and causal reasoning under confounding, selection bias, and noisy measurements.
- Chen et al., [ACED-Bench](https://arxiv.org/abs/2610.14381), posted 9 October 2026. Motivates separating natural-language surfaces from hidden causal worlds and measuring evidence acquisition, retention, decision, and stopping behavior.

These publications motivate a direction. They do not validate this repository's implementation or imply a real field effect.

## Non-goals

- No field experiment, participant recruitment, live data ingestion, spending, dispatch, or other external intervention.
- No statistical effect estimator in this slice.
- No second evaluator identity or generalized transportability implementation.
- No assertion that file hashes authenticate authorship, timestamp, allocation randomness, ethics approval, or outcome truth.
- No score or synthetic test result is treated as real-world impact.
