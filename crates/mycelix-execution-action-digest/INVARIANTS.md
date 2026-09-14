# Institutional Execution Action Digest Adapter Invariants

This crate is an institutional type adapter over `mycelix-execution-action-digest-core`.

## Registered profile

`mycelix-governance-execution-authority-v1-blake3-exact-json`

## Hard properties

1. The byte-framing algorithm is owned only by `mycelix-execution-action-digest-core`.
2. This crate MUST NOT instantiate a BLAKE3 hasher or duplicate the execution-authority domain/framing rule.
3. `execution_authority_digest(proposal_id, actions)` must equal `Digest32(execution_authority_digest_bytes(proposal_id, actions))` exactly.
4. The public profile and size constants are re-exported from the core rather than redefined.
5. Exact action bytes remain identity-bearing: no trim, parse, reorder, canonicalize or reserialize step exists here.
6. Proposal identity remains digest-significant.
7. This adapter grants no execution authority; it only projects content identity into the institutional `Digest32` type.
8. Any framing or interpretation change requires a new core profile rather than a wrapper-local fork.
