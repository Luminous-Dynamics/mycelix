# Mycelix Knowledge Graph

> Distributed knowledge graph for claims, evidence references, attestations, disputes and provenance.

**Maturity:** pre-alpha research software. The Knowledge hApp and its confidence-ranking logic are not an independently validated truth-detection or misinformation-resistance service. Verify the Maturity Matrix in the repository root before treating this cluster as deployable.

## Overview

The Knowledge hApp stores claims and their associated attestations and exposes query functions over the distributed graph. Its current DKG includes a legacy five-factor confidence heuristic. That score is a software-generated ranking signal; it is not a calibrated probability that a proposition is true.

The API name `get_truth` is retained for compatibility. It returns claims that pass a confidence threshold and orders them by the legacy score. It does **not** establish that the highest-ranked claim is true.

## Legacy confidence heuristic

The current SDK calculation combines five factors:

| Factor | Weight | What it currently represents | Important limitation |
|--------|--------|-------------------------------|----------------------|
| Attestation count | 25% | Number of endorsements, with diminishing returns | Coordinated or correlated attestations are not independent evidence |
| Reputation weighting | 30% | Reputation-weighted attester signal | Contributor reputation is not proposition truth or source reliability |
| Source quality | 20% | Existing source-category heuristic | A category such as academic, institutional, commercial or social is not a universal reliability ordering |
| Time decay | 10% | Domain-configured age adjustment | Score decay is not proof that older evidence is invalid; currentness must be assessed by claim type and temporal scope |
| Consistency | 15% | Penalty derived from recorded contradiction weights | Agreement/disagreement alone does not resolve factual validity or preserve all competing interpretations |

The factors and weights are legacy implementation choices, not empirically calibrated estimates. Their output should be named and interpreted as a **legacy confidence/ranking score**, not “truth”, “veracity probability” or a universal reliability measure.

A source's authenticity, its byte integrity, its relevance to a particular assertion, its independence from other sources, the authority of a knowledge holder, and social endorsement are distinct properties. No single scalar should silently collapse them.

## Genesis DHT demonstration — narrow scope

The genesis scenario creates two synthetic claims about sky colour and records one peer endorsement of the “Blue” claim. It checks that the endorsement changes the confidence ranking in the expected direction in that test configuration.

This demonstrates a narrow software behaviour across a multi-agent DHT scenario. It does **not** demonstrate:
- that the endorsed proposition is true;
- robust detection of misinformation;
- resistance to Sybil identities, brigading or correlated attestations;
- independent source verification;
- source-family or provenance independence;
- calibration against real-world outcomes;
- performance against professional fact-checkers or current frontier models.

The test's synthetic labels (“Blue” and “Green”) are fixture setup, not independent ground truth for a general truth-detection claim.

## Zome functions

### Claims
- `submit_claim` — create a claim.
- `get_claims` — retrieve claims about a subject.
- `get_truth` — legacy API name; retrieve claims filtered and ranked by the legacy confidence heuristic.

### Attestations
- `attest_claim` — record an endorsement, challenge or acknowledgement. An attestation records an agent's action; it is not, by itself, independent evidence.

### Reputation
- `get_agent_reputation` — query agent-reputation information. Reputation is a contributor signal and must not be interpreted as a proposition's evidential support.

### Discovery
- `list_subjects` — list subjects with claims.
- `ping` — health check.

## Research direction

The active audit is tracked in [KNOW-TRUTH-001 / Mycelix #5818](https://github.com/Luminous-Dynamics/mycelix/issues/5818). It will first identify the canonical claim/evidence path and all consumers before changing schemas or wire formats.

The intended direction is to keep the following separate and inspectable:

- claim kind, scope and intended use;
- evidence relevance and method-specific quality for the assertion;
- source authenticity, artifact integrity and provenance-family independence;
- temporal and geographic applicability;
- community authority, consent and access restrictions;
- endorsements, challenges, disputes and contributor reputation;
- assessment method, assessor, version, uncertainty and review history.

The implementation must compose the shared evidence plane and existing semantic/provenance work. Do not infer truth from consensus, trust scores, signatures, hashes, Holochain validation or model diagnostics.

## License

Apache-2.0
