# FIN-BANK-CHAIN dataset governance v1

**Dataset:** FIN-BANK-CHAIN v1.0.0  
**Status:** public synthetic semantic conformance fixtures  
**Not a hidden leaderboard holdout.**

## Files

- `bank_statement_chain_input_v1.0.1.schema.json`: candidate-visible structural schema.
- `bank_statement_chain_candidates_v1.0.1.json`: 26 synthetic candidate-visible chain tasks.
- `public-reference-oracle/bank_statement_chain_oracle_v1.0.1.json`: public reference outcomes for deterministic regression and independent implementation comparison.
- `public-reference-oracle/bank_statement_chain_oracle_v1.0.1.schema.json`: structural schema for those public outcomes.
- `BANK_STATEMENT_CHAIN_V1.md`: semantic contract and rationale.

## Important boundary

The public oracle is intentionally published beside the public conformance cases. This permits transparent independent reproduction and regression testing, but it is **not blind evaluation**. No model performance claim, benchmark leaderboard result, or candidate-isolation claim may be based on this public oracle.

A future blind evaluation must use a separate private evaluator service/repository or an access-controlled evaluation environment. Candidate-visible tasks must be materialized without evaluator labels, reason codes, oracle files, or answer-bearing metadata. Record the exact input bundle digest and evaluator release in the run receipt. Do not describe the public corpus as a hidden test set.

## Fixture digest semantics

The current semantic fixtures do not contain raw ISO 20022 XML. Their `fixture_digest_token` is a deterministic, format-valid token used to exercise identity, replay and changed-payload branches. It is **not** a SHA-256 hash of real XML bytes and must never be presented as a source artifact digest.

When raw CAMT XML fixtures are added, store the exact bytes separately, calculate SHA-256 over those bytes, and use a distinct field/contract such as `raw_payload_sha256`. Preserve the original source bytes and media type. Never infer source authenticity from a digest.

## Version history

- v1.0.0: 26 public semantic cases; preserved unchanged as the original baseline.
- v1.0.1: 28 public semantic cases. Adds exact statement-period boundary coverage and expected-sequence checks. The v1.0.0 files remain immutable.

## Corpus limits

- All account and entity references are synthetic tokens.
- The currency reference is explicitly a synthetic two-decimal ZAR-like profile and is not an admitted production currency profile.
- The corpus is semantic and parser-independent; it does not test XSD validity or bank-specific MUG conformance.
- The fixture suite does not demonstrate bank connectivity, source authenticity, account ownership, statement-population completeness, settlement finality, accounting-standard compliance or production readiness.

## Extension rules

1. Keep existing v1.0.0 cases immutable. Changes require a new dataset version.
2. Add both positive and negative controls for each new invariant.
3. Keep task inputs, expected outputs and mutation metadata separate.
4. Public and hidden splits must have distinct manifests and digests.
5. Expert-review ambiguous accounting assertions; preserve adjudication and disagreement.
6. Never put customer banking data, bank account numbers or credentials in public fixtures.
