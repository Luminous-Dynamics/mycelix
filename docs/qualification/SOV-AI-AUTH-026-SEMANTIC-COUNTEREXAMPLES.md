# SOV-AI-AUTH-026 — semantic completeness boundary and counterexamples

## Two relations, two failure meanings

This candidate deliberately keeps these predicates separate:

- Denotational containment: the child admits no request the parent rejects in the declared finite universe.
- Structural compound subsumption: every parent conjunction obligation has a distinct child witness, or every child disjunct is covered by a parent disjunct.

For an all expression, the reference denotation is the intersection of its atomic denotations. Injective clause matching is a conservative structural rule, not automatically a complete decision procedure for arbitrary request-set inclusion. A single child atom can denote a subset of two parent atoms while being unable to witness both parent obligations one-to-one.

The oracle must therefore distinguish:

- STRUCTURAL_SUBSUMPTION_PASS
- AUTHORITY_EXPANSION, with a concrete request in child-minus-parent
- STRUCTURAL_FALSE_NEGATIVE, where bounded denotational containment holds but structural matching rejects
- UNSUPPORTED_OR_UNDECIDABLE, when an extension, cross-type pair, or mode has no admitted rule
- EFFECTIVE_POLICY_CONTAINMENT_PASS for a policy comparison where the effective child request set is contained in the effective parent request set

A structural false negative is not an authorization expansion. A finite-domain pass is not a claim about unbounded values, arbitrary predicates, or arbitrary policy languages.

## Counterexample oracle

The compound_subsumption_counterexamples.py tool accepts a JSON scenario and emits deterministic JSON. For an authority expansion it reports the smallest request under the order declared by the scenario universe, the first divergent parent constraint, all parent constraints rejecting that request, clauses supporting child admission, and a deterministic minimal clause core that preserves the request's admission/rejection outcome.

Effective-policy mode evaluates allow and deny denotations under an explicit conflict rule. It detects deny deletion and conflict-rule substitution as effective authority expansions even when positive allow denotation is unchanged.

Receipts include input and result SHA-256 digests. The companion control runner independently replays the reported request against the raw fixture data and enumerates the entire finite universe to confirm that a structural false negative has no child-minus-parent request.

## Boundedness and proof ceiling

The finite evaluator is complete only for the supplied universe, its all/any grammar, the four atomic dimensions in this candidate, and the explicitly supported conflict rules. Unknown extension identifiers fail closed. The implementation does not establish completeness for infinite numeric domains, arbitrary predicates, or all policy compositions.

Research/specification only. Hosted exact-head evidence and independent receipt review are required; no production authorization qualification is claimed.
