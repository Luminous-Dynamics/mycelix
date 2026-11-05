# Jurisdiction Pack Qualification Dossier v1.0.0

Public schema and synthetic fixture for [Mycelix #5834](https://github.com/Luminous-Dynamics/mycelix/issues/5834), composed with global portability #5811.

## Files
- jurisdiction_pack_v1.0.0.schema.json: closed Draft 2020-12 dossier contract.
- fixtures/synthetic_jurisdiction_pack_v1.0.0.json: synthetic pack with country and province scopes, separate tax/e-invoice candidates, source metadata, temporal facts, review records and change control.
- ../../docs/JURISDICTION_PACK_QUALIFICATION_V1.md: lifecycle and qualification protocol.

## Important
This fixture is synthetic and intentionally unqualified. All source records are not_archived; every source digest and byte length is null. No source bytes or SHA-256 values are claimed. The example contains no real tax rate, legal threshold, filing deadline or executable rule.

A pack cannot be marked qualified_for_declared_scope while using synthetic-fixture scope. A real qualified pack must have archived exact source bytes, source-backed interpretation records, required local reviewers, test evidence and a resolver-compatible exact profile intersection. A country's name or an official-looking URL is not sufficient.

## Validation
Full Draft 2020-12 validation is still outstanding for this authored schema and fixture. No Rust execution or legal/accounting review is claimed.
