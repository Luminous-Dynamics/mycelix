# SIF v0.1 Verification Boundary

The verifier boundary is intentionally commitment-only. Evidence providers must not need a subject identifier, actor name, case identifier, authority text, or underlying protected record in order to prove that their evidence belongs to the same operation.

`AttestationVerificationContext` contains only:

- frozen pre-attestation receipt statement;
- authenticated requester/source commitment;
- query commitment;
- purpose/scope commitment;
- policy commitment;
- optional minimum-necessary result commitment.

This allows Xenia to authenticate its signed execution binding and Symthaea to authenticate its computation proof against the same public context while Mycelix retains ownership of citizen-facing semantics and authorization policy.

For live cross-provider binding, Xenia additionally derives a privacy-safe operation nonce from its authenticated operation/session plus receipt/query commitments. Symthaea binds that nonce inside its computation proof. The nonce must not be interpreted as a citizen identifier or case identifier.

A production disclosure boundary should consume a `VerifiedReceipt` or stronger future capability, never a raw receipt whose evidence references have merely passed structural validation.
