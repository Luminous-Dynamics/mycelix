# LISE-AUTH-MESH-48 v1 — public development seed

**Owner:** [#6216 — LISE-AUTH-MESH-001](https://github.com/Luminous-Dynamics/mycelix/issues/6216)
**Status:** public synthetic non-blinded development corpus.

## Files

- `scenarios.v1.json` — 48 synthetic histories in 24 minimal pairs.
- `oracle.v1.json` — separate per-property expected outcomes.
- `scenario.schema.json` — strict Draft 2020-12 scenario schema.
- `oracle.schema.json` — strict Draft 2020-12 oracle schema.

## Six families
1. Identity, key and registry lifecycle.
2. Control-plane, policy distribution and cache freshness.
3. PEP identity and complete mediation.
4. Prompt/data provenance and context integrity.
5. Delegation and workflow attestations.
6. Effect observation and outcome correspondence.

Each pair changes a material boundary fact. Outcomes are separate across authenticity, integrity, authorization, currentness, mediation, workflow binding and effect evidence. There is no composite trust score.

## Use and limitations

The corpus and oracle are both public. This is a development/calibration seed, not a blinded holdout, confirmatory evaluation, independent reproduction of any vendor claim or production security qualification.

Reviewers should first code the scenario file without consulting the oracle, lock their codes, then compare to the public oracle during calibration. For confirmatory evaluation, freeze a new held-out set after the protocol and codebook stabilize.

A valid signature does not create authority. A signed prompt can contain unauthorized instructions. A valid context hash chain does not prove complete event capture. A receiver-side PEP decision does not prove every route is mediated. A provider acknowledgement does not prove an independently observed effect.

## Claim ceiling

A future exact replay pass would qualify only the named synthetic boundary contract and finite fixtures. It would not establish system security, model robustness, product superiority, trust-root governance, legal validity or real-world effect observation.
