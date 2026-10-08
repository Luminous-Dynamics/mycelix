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
