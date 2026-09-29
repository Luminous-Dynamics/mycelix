# Integral D6X — Layered Verification Ceilings and Partial-Proof Integrity

Status: **ReferenceModelOnly**

## Purpose

D6X prevents partial commitment verification from silently becoming full receipt verification.

Verification scopes are:

    InputOnly
    InputAndDerivation
    Result
    FullReceipt

The verified scope is itself claim-bounded.

## Scope laws

1. InputOnly verifies only C_input integrity.
2. InputAndDerivation requires exact C_input and verifies C_derivation linkage.
3. Result requires exact C_input and C_derivation and verifies C_result linkage.
4. FullReceipt additionally verifies the exact D6S receipt through D6W.
5. Missing lower layers block higher scopes.
6. A higher-layer commitment cannot substitute for a missing lower-layer commitment.
7. A valid commitment proves integrity of its committed representation, not semantic truth, currentness, authority, or authorization.
8. Verification scope may be downgraded, but it may not be upgraded without the missing lower-layer evidence.

## Verification receipt

D6X emits a non-authoritative verification receipt containing the exact commitments that were actually verified.

The receipt commitment itself is canonical and domain-separated, but the verification receipt does not become semantic authority.

## D6W/D6S integration

FullReceipt verification calls the D6W verifier, which first verifies the complete D6S receipt.

Therefore:

    C_input verified != C_derivation verified
    C_derivation verified != C_result verified
    C_result verified != full D6S receipt verified

This keeps the commitment decomposition diagnostic rather than epistemically amplifying.

## Symthaea boundary

Symthaea may choose an efficient verification path and report missing layers.

Symthaea may not:

- claim FullReceipt from InputOnly;
- substitute C_result for C_input;
- substitute a D6W receipt for D6S verification;
- raise verification scope by inference;
- convert verification integrity into authorization.

## Adversarial corpus

The reference module covers:

1. input-only verification;
2. exact lower-layer binding for derivation verification;
3. successful derivation verification;
4. missing derivation blocking result verification;
5. successful result verification;
6. higher-layer substitution rejection;
7. full D6S-backed verification;
8. tampered D6S rejection;
9. deterministic verification receipts;
10. verification-scope ordering;
11. missing evidence as blocked scope rather than semantic rejection;
12. Symthaea claim boundary.

## Claim ceiling

**ReferenceModelOnly.**

D6X does not establish truth, trust, cryptographic authenticity, production finality, legal authority, economic settlement, physical causality, or actuation safety.
