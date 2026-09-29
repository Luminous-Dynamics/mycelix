# Integral D6W — Layered Derivation Commitments

Status: **ReferenceModelOnly**

## Purpose

D6W decomposes D6S's derivation identity into independently inspectable commitment layers. It is diagnostic structure, not a new source of semantic authority.

## Commitment pipeline

```
qualified projection + semantic environment
  -> input commitment
  -> derivation commitment (profile + optional deterministic trace)
  -> result commitment (status + payload + conflict/unresolved flags)
  -> layered receipt (all layer commitments + claim ceiling)
```

- **Input:** binds the source DKG snapshot, selected projection, semantic environment, exact node and edge commitments, D6P receipt commitments, and claim ceiling.
- **Derivation:** binds the input commitment to the exact derivation-profile commitment and optional deterministic execution-trace commitment. A D6U fixpoint trace belongs here, not in evidence identity.
- **Result:** binds the derivation commitment to the result status, result-payload commitment, contradiction state, unresolved state, and claim ceiling.
- **Receipt:** binds the three layer commitments and claim ceiling into one compact final identity.

Each layer uses a distinct D6S-CANON-1 domain label under the shared D6S hash-domain prefix. Each layer excludes its own commitment field from its preimage.

## Diagnostic invariants

- An input mutation changes the input commitment and therefore invalidates downstream layers.
- A rule/profile or execution-trace mutation changes the derivation commitment, without rewriting input identity.
- A result payload or status mutation changes the result commitment, without rewriting derivation identity.
- A claim-ceiling change is invalid unless represented by a separately qualified schema/profile; this reference model fixes the D6S ceiling.
- Layer links must be exact: derivation references the input commitment, result references the derivation commitment, and receipt references all three.
- Supported cannot silently carry contradiction or unresolved flags; disputed must preserve contradiction; unresolved/blocked statuses must preserve unresolved state.
- Hashes establish byte-level integrity only. They do not establish truth, causality, observer independence, currentness, authorization, or actuation safety.

## D6S integration

D6W is a diagnostic decomposition around the existing D6S receipt model. It does not replace D6P current-finality eligibility, D6N contestability, D6O lifecycle context, or D6T canonical encoding. A layered receipt is not interchangeable with a D6S receipt until a verifier explicitly reconstructs and cross-checks the shared projection, environment, profile, status, and result commitments.

## Verification and exit gate

The Rust reference model provides constructors, self-commitment checks, and mutation tests for input, derivation, result, and final receipt layers. These tests are authored but have not been reported as executed.

Before interoperability or production claims: independently implement the layer encodings, publish cross-language golden vectors, verify all layer links against D6S and D6U, and run the full Rust/Python/WASM/Holochain conformance corpus.

Claim ceiling: **ReferenceModelOnly**.
