# Integral D6W — Layered Derivation Commitments

Status: **ReferenceModelOnly**

## Purpose

D6W decomposes the D6S canonical derivation receipt into diagnostically useful commitment layers:

    C_input
      -> C_derivation
        -> C_result
          -> C_receipt

The purpose is mutation localization, not stronger semantic authority.

## Layer 1 — input identity

C_input commits to the exact qualified projection material:

- source DKG snapshot commitment;
- D6S projection commitment;
- selected node commitments;
- selected edge commitments;
- exact D6P current-receipt commitment set;
- D6N observer context;
- D6O lifecycle context;
- semantic-environment commitment.

Therefore an input-only mutation changes C_input and every downstream layer.

## Layer 2 — derivation identity

C_derivation commits to:

- C_input;
- exact derivation-profile commitment;
- exact execution/fixpoint trace commitment.

A rule/profile or execution-trace mutation therefore changes C_derivation and downstream commitments without pretending that the qualified evidence itself changed.

The trace is explicit even for a reference model. The caller may use a deterministic no-recursive-trace commitment when no recursive/fixpoint execution occurred, but it must never be silently omitted.

## Layer 3 — result identity

C_result commits to:

- C_derivation;
- result status;
- result commitment;
- preserved contradiction flag;
- preserved unresolved flag;
- D6S claim ceiling.

A result-only mutation therefore changes C_result and the final receipt while leaving C_input and C_derivation unchanged.

## Final receipt identity

C_receipt commits to:

- C_input;
- C_derivation;
- C_result;
- the exact D6S receipt commitment;
- the D6S claim ceiling.

The exact D6S receipt is verified before D6W decomposition. D6W therefore cannot decompose an invalid/tampered D6S receipt into a superficially valid layered receipt.

## Diagnostic law

For a fixed qualified environment:

    input mutation
      => C_input changes
      => C_derivation changes
      => C_result changes
      => C_receipt changes

    derivation/profile/trace mutation
      => C_input unchanged
      => C_derivation changes
      => C_result changes
      => C_receipt changes

    result mutation
      => C_input unchanged
      => C_derivation unchanged
      => C_result changes
      => C_receipt changes

This makes it possible for an independent verifier to say which commitment layer changed without treating that change as a semantic truth judgment.

## D6P/D6N/D6O boundary

D6W does not re-qualify:

- current finality;
- observer independence;
- observer lifecycle;
- external finality;
- authority;
- or actuation.

It inherits the exact D6S-qualified projection and its already committed D6P/D6N/D6O context.

A changed D6P/D6N/D6O context is an input-layer mutation and therefore propagates through the commitment chain.

## Symthaea boundary

Symthaea may:

- compare layer commitments;
- localize candidate mutations;
- identify whether an input, derivation, or result changed;
- propose a new derivation trace.

Symthaea may not:

- replace an input layer;
- rewrite a D6S receipt;
- manufacture a derivation trace;
- raise the claim ceiling;
- convert a result commitment into authority;
- or authorize actuation.

## Canonical encoding

D6W uses the frozen D6S-CANON-1 canonicalization profile through the existing D6S canonical hashing function. D6W adds domain-separated layer labels:

    d6w-input
    d6w-derivation
    d6w-result
    d6w-receipt

The labels are integrity-domain identifiers, not authority identifiers.

## Adversarial corpus

The reference module covers:

1. deterministic reconstruction;
2. input mutation propagation;
3. derivation-profile mutation isolation;
4. execution-trace mutation isolation;
5. result mutation isolation;
6. claim-ceiling mutation propagation;
7. tampered D6S receipt rejection;
8. D6P context binding;
9. exact three-layer chaining;
10. exact verifier reconstruction;
11. trace mismatch rejection;
12. authorization boundary.

## Claim ceiling

**ReferenceModelOnly.**

D6W does not establish source truth, cryptographic authenticity, legal authority, production finality, causal truth, actuation safety, or external-world outcomes. SHA-256 commitments identify the committed representation; they do not independently qualify the semantics represented by that commitment.
