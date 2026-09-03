# Constitutional Transition Verifier Invariants

Status: **v0.1 draft / fail closed**

These invariants are normative for the `constitution_transition` runtime.

- **CTV-01 — Candidate != authority.** A stored `ConstitutionTransitionCandidate` is evidence-shaped input only. Every authoritative read re-verifies external tally and threshold evidence.
- **CTV-02 — DNA genesis.** Projection starts only from `constitution_authority.get_verified_constitution_genesis`; the legacy mutable constitution zome is never a parent source.
- **CTV-03 — Verified binding tally only.** The transition runtime resolves `binding_voting.get_verified_binding_tally` itself. A caller cannot supply `approved`, quorum, tally bytes, or tally verification state.
- **CTV-04 — Exact tally binding.** A separate rights verifier must bind the exact proposal, tally action, tally digest/profile, parent digest, child digest, authorization digest, and transition nonce.
- **CTV-05 — Exact threshold binding.** A separate threshold-authority verifier must bind the exact proposal, expected threshold digest/profile, parent digest, child digest, authorization digest, authorized time, and transition nonce.
- **CTV-06 — Missing verifier denies.** Missing zome/function, call error, decode error, malformed receipt, or inexact echoed binding aborts constitutional verification.
- **CTV-07 — No user verification evidence.** Submission accepts only child statement + transition authorization. `TransitionVerificationEvidence` is constructed inside the coordinator after verifier calls.
- **CTV-08 — Replay domain.** The same nonce may appear more than once only for a byte-identical authorization digest. Reuse across different verified authorizations freezes projection.
- **CTV-09 — Fork ambiguity.** Multiple distinct verified children of the current parent are never ordered by timestamp, author, DHT arrival, Phi, reputation, stake, or model output. The portable projector fails closed.
- **CTV-10 — DHT reorder independence.** Current constitutional state is a deterministic function of DNA genesis plus externally verified transition semantics, not link arrival order.
- **CTV-11 — Append-only candidates.** Candidate entries and discovery links cannot be updated or deleted.
- **CTV-12 — Reverification.** A prior verification receipt does not become a perpetual capability merely because it was stored. Authoritative projection re-calls the current verifier boundaries.
- **CTV-13 — Cross-network denial.** Parent, child, and transition identities must remain on the exact DNA-committed constitutional network/institution/constitution lineage.
- **CTV-14 — Parent rules govern change.** The binding-vote, threshold-authority, and amendment-policy profiles used for a transition are those authorized by the parent constitution, not the child.
- **CTV-15 — No legacy mutation authority.** Legacy charter/parameter/amendment writes cannot change the verified current constitution.

## Required adversarial tests before merge-ready

1. Directly committed forged candidate cannot become current.
2. Binding tally from another proposal is denied.
3. Binding tally with correct proposal but wrong profile/digest is denied.
4. Threshold authorization for another child/action/policy/epoch is denied.
5. Missing rights verifier fails closed.
6. Missing threshold verifier fails closed.
7. Undecodable verifier response fails closed.
8. Replayed nonce with different authorization fails closed.
9. Duplicate byte-identical candidate is harmless.
10. Two independently valid children of one parent produce an ambiguous fork, never an arbitrary winner.
11. Child version `parent + 2` is denied.
12. Stale/wrong parent digest is denied.
13. DHT link/record reordering produces the same projected constitution.
14. Mutation of the legacy constitution zome has no effect on verified projection.
15. Cross-DNA/genesis rebinding is denied.
