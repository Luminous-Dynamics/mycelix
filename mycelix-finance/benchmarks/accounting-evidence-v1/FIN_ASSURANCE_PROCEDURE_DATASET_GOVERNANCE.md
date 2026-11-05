# FIN-ASSURE-PROCEDURE dataset governance

The v1.0.0 corpus has 18 synthetic cases and a public reference oracle. It is a regression/conformance corpus, not a blind holdout, professional audit dataset or benchmark leaderboard.

All entities, risks, documents, origin groups and periods are synthetic. No client data or audit workpapers are included. The public oracle must not be mounted in a candidate environment when measuring model performance. A separate access-controlled evaluator is required for blind candidate comparison.

The fixture set encodes machine-checkable evidence-purpose, period, population, denominator, origin-independence, contradiction, procedure-state, stale-workpaper and human-judgment boundaries. It does not encode the full ISA exposure drafts or claim compliance with current/proposed auditing standards.

Dataset extension rules:
1. Preserve v1.0.0 bytes and hashes once frozen; additions require a new version.
2. Add positive and negative controls for every new invariant.
3. Keep candidate inputs, public reference outcomes and any future blind evaluator physically separate.
4. Keep standards mapping and expert adjudication separate from deterministic fixture correctness.
5. Do not infer real-world audit sufficiency from these synthetic cases.
