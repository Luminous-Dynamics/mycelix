# Exact Execution Action Digest Core Invariants

This crate is the substrate-neutral protocol waist for Mycelix's registered exact-byte execution-action identity.

## Registered profile

`mycelix-governance-execution-authority-v1-blake3-exact-json`

The digest input is exactly:

```text
"mycelix-governance-execution-authority-v1\0"
|| u64_le(len(proposal_id))
|| proposal_id UTF-8 bytes
|| u64_le(len(actions))
|| exact actions UTF-8 bytes
```

## Hard properties

1. No normalization: whitespace, ordering, escaping and every other action-byte difference remain identity differences.
2. Proposal identity is part of the digest domain.
3. Inputs are bounded to the historical 512-byte proposal / 4096-byte action ceilings.
4. Raw `[u8; 32]` and lowercase-hex projections are two representations of the same digest, not separate identities.
5. This crate owns no institutional, lifecycle, executor, freshness, deployment or effect semantics.
6. A digest proves content identity only; it grants no authority.
7. Any change to domain, framing, size rules or byte interpretation requires a new registered profile.

The institutional `mycelix-execution-action-digest` wrapper and recipe-free Commons evidence should delegate to this kernel rather than independently reimplementing the framing rule.
