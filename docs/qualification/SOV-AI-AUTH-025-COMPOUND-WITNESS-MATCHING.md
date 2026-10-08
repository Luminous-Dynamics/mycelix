# SOV-AI-AUTH-025 — compound constraint witness matching

The supported fragment is a finite conjunction/disjunction language over atomic constraints.

For `all` / conjunction, structural subsumption requires a distinct derived witness for every parent clause. Additional child clauses are permitted only when they further restrict the denotation.

The matcher is deterministic and backtracking-based. This prevents a greedy early choice from producing a false negative when a different one-to-one assignment exists.

The implementation separately checks bounded denotational soundness for the supported fragment.

This deliberately does not claim completeness for arbitrary policy languages or arbitrary first-order predicates.

Research/specification only.
