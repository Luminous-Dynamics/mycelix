# IF01 SMT witness

`integral_if01_authority_outcome.smt2` is a bounded SMT-LIB witness for IF01-FV-010..012.

## Expected result

The three contradiction checks must each return `unsat`.

The SMT-LIB standard defines `unsat` as establishing that the assertion set has no model, and supports requesting an unsatisfiability proof with `get-proof` when proof production is enabled.

Z3 exposes proof generation/checking options, including proof checking and proof-log support.

## Local verification

Run:

    z3 mycelix-manufacturing/crates/cos_conformance/proofs/integral_if01_authority_outcome.smt2

Expected output contains three `unsat` results.

This is a solver-executed witness, not by itself a production refinement proof or deployment authorization.

## Claim ceiling

Passing this file establishes only the encoded bounded semantic contradictions for:

- IF01-FV-010: certification failure is distinct from authorization failure.
- IF01-FV-011: authorization requires a matching explicit reference.
- IF01-FV-012: known recipient rejection is distinct from indeterminate delivery.

It does not establish cryptographic authentication, network delivery guarantees, production authorization, manufacturing qualification, security certification, or Integral ratification.
