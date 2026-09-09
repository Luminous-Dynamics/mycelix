# Exact Execution Action Digest Invariants

This crate owns one narrow compatibility contract: the exact-byte execution action digest shared by governance and response admission.

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

## Non-negotiable properties

1. **No normalization.** Whitespace, key order, escaping, action order and every other byte-level difference remain identity differences.
2. **Proposal binding.** Reusing identical action bytes under another proposal ID produces another digest.
3. **Bounded input.** The shared action payload ceiling is 4096 bytes, matching the historical execution-plan boundary.
4. **No authority.** A digest proves content identity only. It does not prove proposal legitimacy, threshold approval, executor designation, freshness, deployment state, effect safety or execution permission.
5. **JSON validity is external.** The digest function does not parse JSON because parsing must never alter the byte sequence being committed. Callers that require JSON validate the same exact bytes separately.
6. **Profile changes require versioning.** Any domain separator, framing, size rule or byte interpretation change requires a new profile rather than silently changing v1.

The implementation is extracted from the reviewed historical governance execution-plan verifier. New runtimes should depend on this crate instead of copying the hash rule.
