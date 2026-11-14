# FSRE source-frame dependency audit benchmark v0.1

## Purpose

Provide a conservative structural preflight for the frame-of-frames qualification programme in #6190. It detects declared common collection processes, shared upstream ancestry, declared dependence and potential common ownership. It never proves statistical independence and never executes a coverage estimator.

## Adversarial corpus

- Two apparent registers copied from one upstream register.
- Three frames with a hidden common upstream ancestor at depth 2 or more.
- Shared collection process under different legal owners.
- Shared owner with separately documented field processes.
- Distinct URLs and custodians but shared source snapshot.
- Upstream source with incomplete horizon validity.
- Late dependency evidence that does not cover the full study horizon.
- Missing dependency endpoints, duplicate edges and cycles.
- Conflicting or unknown separation assessments.
- Source-use blocked or unknown.

## Outputs

Report pairwise structural dependence signals, shared ancestor IDs, common collection-process flags, common-owner review flags and exact graph/row counts. Every report must retain:
- independence conclusion: NOT_ESTABLISHED_BY_STRUCTURAL_AUDIT;
- dual-system estimator admissibility: NOT_ESTABLISHED;
- multiple-systems estimator admissibility: NOT_ESTABLISHED;
- source authentication: ASSERTED_NOT_AUTHENTICATED.

## Targets

- 100% detection of explicitly represented common ancestors and shared process IDs.
- 100% rejection of cyclic, dangling, duplicate or temporally incomplete dependency evidence.
- 0 independence claims based solely on distinct URLs, owners or filenames.
- 0 estimator-admissibility claims from this structural audit alone.
- 100% deterministic report under row permutation.
- p50/p95 runtime and peak memory at 10^2, 10^3, 10^4, 10^5 and the declared 500,000-row cap.

## Comparators

Static pairwise frame comparison, URL/owner-based heuristics, a source-dependency DAG without temporal qualification, and FSRE's bitemporal/evidence-qualified dependency graph. All receive the same frozen provenance edges and horizons.

## Next layer

A separate, independently reviewed design-evidence verifier may qualify whether a post-enumeration survey or multiple-systems design's independence assumptions are defensible. Even then, estimators must report sensitivity to unobserved interactions and heterogeneous capture probability. Structural absence of a declared dependency is not proof of statistical independence.
