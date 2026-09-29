# Integral D6Q — Evidence Bundles and Claim-Graph Closure

Status: **ReferenceModelOnly**

## Purpose

D6Q provides an explicit typed envelope for the evidence-to-conclusion chain:

`Source -> Evidence -> Statement -> Registration Receipt -> Validation -> Assessment -> Conclusion -> Human Disposition`.

The graph is an evidence-binding structure, not a truth oracle.

## Semantic boundaries

The model preserves these non-equivalences:

- source != evidence;
- evidence != statement;
- statement != truth;
- registration receipt != endorsement;
- validation != conclusion;
- conclusion != authorization;
- human disposition != evidence;
- provenance/custody != causal support;
- graph reachability != semantic validity;
- historical evidence != current authority.

Structural closure only proves that the graph is well-formed under the reference model. Evidentiary sufficiency and conclusion status remain separate questions.

## D6P composition boundary

D6Q is intentionally downstream of D6P. A later refinement should consume an exact D6P `EligibleCurrent` receipt rather than infer current eligibility from a node's frontier field alone.

The current D6Q implementation therefore treats frontier presence as a conservative reference-model guard, not as a substitute for D6P qualification.

## Cycles

The initial model rejects all graph cycles conservatively. A future refinement should distinguish semantic derivation/support cycles from benign provenance/custody cycles before relaxing this rule.

## Historical/current mixing

A bundle may contain historical evidence without that evidence becoming current. Future refinement should represent mixed historical/current bundles explicitly instead of letting one historical node collapse the entire bundle to a single disposition.

## Symthaea and Xenia

Mycelix remains the semantic root.

- Symthaea may analyze, search, propose, and identify missing or suspicious graph structure.
- Symthaea does not create authoritative conclusions, dispositions, eligibility, or authorization.
- Xenia supplies cryptographic mechanisms and verification; cryptographic verification does not itself establish semantic truth.

## Qualification

Adversarial tests cover dangling edges, incompatible endpoints, cycles, currentness absence, contradictory evidence, provenance/custody separation, human disposition, historical evidence, and non-authority boundaries.

No build/test/CI execution is claimed unless an execution receipt is available.

## Claim ceiling

**ReferenceModelOnly.** This document does not establish physical truth, causal validity, cryptographic authenticity, legal authority, production finality, economic settlement, or actuation safety.

## D6R non-amplification refinement

D6R adds a downstream semantic-conservation boundary. A D6Q graph assessment or D6P current-finality receipt may be used as an exact input, but wrapping, replaying, serializing, or traversing that artifact cannot silently increase its claim ceiling or currentness.

D6R also separates scope conservation from graph reachability: exact scope is preserved; narrowing requires an explicit witness; broadening and unknown scope are rejected. Missing evidence remains unresolved/insufficient rather than becoming a rejection.

See `docs/integral/semantic-conservation-non-amplification-v1.md`.

