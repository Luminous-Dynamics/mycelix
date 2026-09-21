# SIF v0.1 Cross-Stack Proof Composition

A high-assurance lookup should be understood as a composition of narrow claims rather than one omnibus "secure" proof.

1. **Mycelix semantic statement** commits the subject-routing handle, requester commitment, purpose/scope, authority, query, policy version, outcome, minimum disclosure, rights, notice mode, privacy budget, and inference metadata.
2. **Xenia execution binding** proves the authenticated session/operator context, signed ledger frontier, and exact receipt/query/purpose/policy/result commitments.
3. **Symthaea computation proof** proves a declared computation produced the committed minimum-necessary result while binding the same receipt/query/policy context and Xenia-derived live-operation nonce.
4. **Policy proof** may independently prove a policy program or decision where needed.
5. **External witness** strengthens commit-before-disclose ordering and independent oversight.

No one layer should claim more than it proves. In particular, a computation proof does not establish legal authority, an execution signature does not establish computation correctness, and a valid receipt structure does not establish cryptographic evidence authenticity.

The final disclosure boundary should require the configured composition to verify against one frozen statement before protected output can leave the holder.
