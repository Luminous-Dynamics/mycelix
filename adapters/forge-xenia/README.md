# FORGE-005B1 — Xenia verifier receipt adapter

This crate defines the portable receipt boundary between Xenia operator authentication and Mycelix Forge.

It intentionally does **not** import the Xenia daemon, its HTTP/session-token machinery, or its RBAC policy as Forge authority.

## Trust boundary

Xenia currently reconstructs a typed challenge transcript, verifies **both** Ed25519 and ML-DSA-65 signatures, consumes the one-time challenge, and resolves the exact enrolled key pair to a stable logical `operator_id`. Xenia also supports replacing an operator's enrolled keys while preserving that logical id and role.

FORGE-005B1 captures only the evidence needed to map that result into Forge:

- exact Forge authentication request commitment;
- stable Xenia operator-id commitment;
- current hybrid key-lineage commitment;
- exact challenge commitment;
- explicit Ed25519 + ML-DSA-65 suite;
- cryptographic-verification evidence commitment;
- challenge-consumption evidence commitment;
- verifier/enrollment-state commitment;
- verification timestamp.

`observe_xenia_receipt(...)` checks those exact cross-links and emits the provider-neutral `AuthenticationObservation` from FORGE-005A.

A deserialized receipt is **not** trusted merely because it has the right shape. FORGE-005B2 must add the Xenia-side producer so the receipt can only be emitted after the real Xenia verification path succeeds.

## Stable identity and rotation

The bridge keeps these concepts separate:

```text
stable Xenia operator_id
        !=
current Ed25519 + ML-DSA key lineage
```

Replacing an operator's key material changes the lineage commitment while preserving the logical operator identity. Old receipt evidence therefore cannot be reused against a replacement binding.

## Xenia reference semantics

The adapter is grounded in `Luminous-Dynamics/xenia-peer` at verified main commit:

`af4fcefc6d4cc7c3f74a3ca48f26abcd97d1e930`

Relevant upstream behavior:

- `xenia-operator-proto::challenge_transcript` is crypto-free/shared between signer and verifier;
- daemon authentication requires both Ed25519 and ML-DSA-65 over the same transcript;
- the challenge is consumed before signature verification, making it single-use;
- `OperatorPolicy::lookup_verified` requires both presented public keys to match one enrollment record;
- `replace_operator_key` preserves `operator_id` and role while replacing key material.

Those facts are producer-side requirements. This Mycelix adapter does not reproduce or replace Xenia's verifier.
