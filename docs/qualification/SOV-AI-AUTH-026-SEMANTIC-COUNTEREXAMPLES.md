# SOV-AI-AUTH-026 — semantic completeness boundary and counterexamples

## Two relations, two failure meanings

This candidate deliberately keeps these predicates separate:

- Denotational containment: the child admits no request the parent rejects in the declared finite universe.
- Structural compound subsumption: every parent conjunction obligation has a distinct child witness, or every child disjunct is covered by a parent disjunct.

For an all expression, the reference denotation is the intersection of its atomic denotations. Injective clause matching is a conservative structural rule, not automatically a complete decision procedure for arbitrary request-set inclusion. A single child atom can denote a subset of two parent atoms while being unable to witness both parent obligations one-to-one.

The oracle distinguishes these outcomes:

- STRUCTURAL_SUBSUMPTION_PASS: bounded denotational containment and the declared structural relation both pass.
- AUTHORITY_EXPANSION: a concrete request exists in child-minus-parent (or child effective-policy minus parent effective-policy).
- STRUCTURAL_FALSE_NEGATIVE: bounded denotational containment holds but structural matching rejects; no authority-expansion request exists.
- POLICY_ATTENUATION_VIOLATION: effective request-set containment may hold, but at least one separate attenuation obligation fails—allow containment, deny preservation, or conflict-rule preservation.
- UNSUPPORTED_OR_UNDECIDABLE: an extension, cross-type pair, mode, or conflict rule has no admitted rule.
- EFFECTIVE_POLICY_CONTAINMENT_PASS: effective containment and the separate allow/deny/conflict-rule obligations all pass.

A structural false negative is not an authorization expansion. A finite-domain pass is not a claim about unbounded values, arbitrary predicates, or arbitrary policy languages.

## Counterexample oracle

The compound_subsumption_counterexamples.py tool accepts a JSON scenario and emits deterministic JSON. For an authority expansion it reports the smallest request under the order declared by the scenario universe, the first divergent parent constraint, all parent constraints rejecting that request, clauses supporting child admission, and a deterministic minimal clause core that preserves the request's admission/rejection outcome.

Effective-policy mode evaluates allow and deny denotations under an explicit conflict rule. It separates actual effective authority expansions from attenuation-component violations. Controls cover (a) deny deletion that really expands effective access, (b) deny deletion that is masked by an unchanged allow set and therefore does not change current effective access, and (c) an allow expansion masked by a deny. The latter two are still reported as POLICY_ATTENUATION_VIOLATION rather than passing solely because the effective request set happened not to change. Conflict-rule substitution is separately exercised.

Receipts include input and result SHA-256 digests. The companion control runner independently replays the reported request against the raw fixture data and enumerates the entire finite universe to confirm that a structural false negative has no child-minus-parent request.

## Boundedness and proof ceiling

The finite evaluator is complete only for the supplied universe, its all/any grammar, the four atomic dimensions in this candidate, and the explicitly supported conflict rules. Unknown extension identifiers fail closed. The implementation does not establish completeness for infinite numeric domains, arbitrary predicates, or all policy compositions.

Research/specification only. Hosted exact-head evidence and independent receipt review are required; no production authorization qualification is claimed.


## Differential corpus

The candidate now includes an exhaustive, deterministic differential corpus:

- 8 frozen atomic constraint templates;
- 2 compound operators: conjunction (all) and disjunction (any);
- 1- and 2-clause ordered expressions, with no repeated atomic template within a single expression;
- 128 syntactic expressions and 16,384 ordered parent/child pairs;
- a finite request domain of 32 tuples, giving 524,288 ordered-pair/request combinations.

For each supported same-operator pair, the production reference is compared with two separate implementations: a raw-JSON denotation evaluator that does not call the oracle's atom matcher, and a brute-force injective matching reference that enumerates candidate injections instead of using the oracle's augmenting-path algorithm. Cross-operator compound pairs are required to return unsupported/fail-closed.

The corpus checks:
1. the reported authority-expansion status against independently computed child-minus-parent denotation;
2. that any authority-expansion witness is the first request under the frozen total order;
3. that structural matching agrees with brute-force existence of the declared structural witness;
4. that clause-order permutations preserve status, witness and ID-mapped matching;
5. that a structural false negative has no request in child-minus-parent;
6. that a mismatch is reduced deterministically by deleting clauses and then reducing atom dimensions while preserving the mismatch category.

When a differential mismatch occurs, the job emits the original scenario, minimized scenario, observed output and minimized mismatch to the evidence artifact. The shrinker is a deterministic delta reducer; minimality is relative to its reduction operations, not a claim of globally minimum representation.

## Research basis

This uses the same broad verification-guided pattern described by the Cedar project: an executable reference/model checked against a separate implementation through differential testing, supplemented by property-based checks. Cedar's formalization and testing infrastructure is public at https://github.com/cedar-policy/cedar-spec and its verification-guided development paper is at https://arxiv.org/abs/2407.01688.

The IETF Attenuating Authorization Tokens Internet-Draft (June 2026, version -01) requires extension subsumption to be decidable, sound and deterministic, and permits conservative false negatives rather than unsound positives: https://datatracker.ietf.org/doc/draft-niyikiza-oauth-attenuating-agent-tokens/.

These are research references, not certification of this implementation. The differential corpus establishes evidence only for the frozen finite grammar and universe. It does not prove completeness for arbitrary constraints, infinite numeric domains, extension predicates, or production enforcement.
