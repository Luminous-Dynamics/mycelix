# PRAXIS-BENCHMARK-RUN-INTEGRITY-001

**Status:** contract, synthetic split fixture, and Rust verifier authored; executable validation and independent review are outstanding.

## Why this is the next paradigm shift

A good scorer is not enough. A benchmark result can be invalid even when the scoring code is correct if:

- turns from one learner trajectory are split across training and test;
- diagnostic families appear in both training and supposedly held-out test;
- the candidate receives the oracle, rubric, split labels, or evaluator metadata;
- two candidate configurations are compared on different trajectories, seeds, policy gates, scorers, or environments;
- model, prompt, dataset, verifier, and evaluator versions cannot be reconstructed later.

This work makes split integrity, artifact access declarations, and run identity executable contracts rather than prose-only promises.

## Split contract

`split_manifest.json` contains one assignment per full trajectory, a stable trajectory ID, a diagnostic-family ID, and one of train/validation/test. The verifier rejects duplicate trajectory assignments, empty required splits, family leakage across splits, and a held-out-family declaration that differs from the actual test families.

The synthetic demonstration maps the nine TE001–TE009 examples into three complete trajectories per split and withholds three complete diagnostic families from training and validation. This is a **contract fixture only**: nine examples are far too few for model selection or scientific claims. The real evaluation remains the 120+ expert-adjudicated trajectories across eight or more families proposed in issue #5481, with more coverage needed for reliable rare-failure estimates.

## Run identity contract

Every run records:

- exact benchmark, run, and split IDs;
- exact split-manifest BLAKE3 and the complete trajectory ID set for the selected role;
- immutable 40-hex system and evaluator revisions;
- model, prompt, policy-gate, candidate schema, observations, oracle, scorer, verifier, protocol, mutation-matrix, and execution-environment digests;
- explicit sampling configuration (temperature/top-p scaled integers, maximum output tokens, seed);
- BLAKE3 of the emitted prediction artifact;
- an artifact catalog with unique IDs, roles, digests, and declared access audience.

The exact split-manifest bytes are bound into the run. Any byte change makes the old run manifest invalid until a new manifest and digest are emitted. Other artifact digests are recorded here, but callers must separately verify those digests against the actual artifact bytes; this validator does not receive those bytes.

## Candidate/evaluator separation

The only candidate-visible artifact roles are observation manifest, observation schema, and candidate policy. Oracle, split assignments, scorer, verifier, protocol, and mutation matrix are evaluator-only. The validator requires the catalog and explicit audience lists to match, with each required role present exactly once.

This detects an inconsistent or self-contradictory run manifest; it does **not** prove that a remote model never received hidden content. Real evaluation must enforce separation in the harness and, where possible, produce independently verifiable access/ingestion receipts. A declaration in a JSON file is not a security boundary.

## Controlled B0/B1 comparison

`validate_comparable_pair` accepts only a baseline B0 and a Symthaea-assisted B1 pair with:

- the same held-out test split and exactly the same complete test trajectories;
- the same observation, candidate-schema, oracle, scorer, verifier, protocol, and mutation-matrix digests;
- identical policy-gate digests and evaluator revision;
- identical sampling settings and execution-environment digest;
- distinct run IDs, no advisor artifact for B0, and a pinned advisor artifact digest for B1.

System/model/prompt/advisor identities can differ as prescribed by the ablation; policy gates, data, evaluator, sample set, and sampling budget cannot. The pair validator does not itself execute a model, prove causation, or compute outcome statistics.

## Synthetic negative controls

The 11-row matrix exercises valid run identity, duplicate-trajectory rejection, diagnostic-family leakage, held-out-family declaration mismatch, exact-byte split binding, attempted oracle visibility, and B0/B1 drift in policy gates or test trajectories. It also self-audits mutation IDs against named Rust test definitions.

## Proposed qualification targets

1. **Zero split leakage:** no repeated trajectory assignment and no diagnostic-family overlap when family holdout is enabled.
2. **Zero evaluator-artifact exposure in candidate-visible manifests.** Enforce actual isolation in the harness, not merely the manifest.
3. **100% exact run reconstruction:** the recorded revisions and artifact digests resolve to retained artifacts.
4. **100% B0/B1 protocol equality** for split, examples, policy gates, evaluator, sampling, and execution environment.
5. Report per-family scores and confidence intervals; do not rely only on a pooled metric.
6. Before scientific claims, expand to the expert-adjudicated split plan and externally comparable datasets.

These are proposed gates, not measured production performance.

## Relationship to external benchmarks

[LongTutor (ACL 2026)](https://aclanthology.org/2026.acl-long.1371/) separates history acquisition, diagnosis, and adaptive teaching and explicitly motivates a generator–verifier pipeline. Praxis should follow the separation principle and qualify its own generator and verifier independently.

[Scale TutorBench](https://labs.scale.com/leaderboard/tutorbench) evaluates rubric-scored tutoring capabilities across 1,490 examples, six STEM areas, and multimodal inputs. The published rubric score is not a durable-learning outcome.

[StudentBench](https://arxiv.org/abs/2609.28470) reports real learner outcomes in a specific GRE tutoring study. Use it as a separate outcome comparator, with the original study's limitations, rather than blending its gains/costs with synthetic evidence-integrity scores.

## Limits

The module and test definitions have been authored but have **not** been compiled or executed in the current authoring environment. No Rust test, CI, benchmark model result, or learning efficacy is claimed. JSON source inspection is not a substitute for a full Draft 2020-12 validator. Keep the stacked PR in draft until exact-head Rust execution, full schema validation, run artifact byte verification, and independent review are complete.
