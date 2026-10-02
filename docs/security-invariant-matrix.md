# Security Invariant Matrix

This matrix turns the Security & Sovereignty Model into an implementation/test backlog.

| ID | Property | Threat | Deterministic enforcement | Adversarial test | Evidence |
|---|---|---|---|---|---|
| I-1 | Confidentiality | AI-mediated disclosure | capability check before retrieval/output | denied-secret inference scenario | policy + test artifact |
| I-2 | Integrity | AI assertion treated as fact | signatures + Holochain validation | forged/unsigned operation | validation result |
| I-3 | Availability | emergency mode becomes privilege escalation | explicit degraded capability set | authority outage + emergency request | state transition log |
| I-4 | Identity | reputation substituted for identity | credential/capability verification | high-reputation unauthorized actor | auth decision |
| I-5 | Provenance | lineage lost during transformation | mandatory source references | summarize/derive/transform chain | provenance graph |
| I-6 | Recovery | rollback destroys evidence | append-only security events | partition + conflicting recovery | reconciliation artifact |
| I-7 | Agency | autonomous privilege escalation | bounded signed capabilities | model requests capability expansion | authorization trace |
| I-8 | Authority | ambiguous authorization accepted | fail-closed policy | stale/revoked/missing credential | denial evidence |

## Current implementation

The first deterministic authority boundary is implemented in
`crates/mycelix-bridge-common/src/security_kernel.rs`.

The kernel provides:

- `Capability` and `AuthorizationRequest` as explicit inputs.
- `VerificationEvidence` as the hand-off from an independent cryptographic/identity verifier.
- `VerifiedCapability` as a non-forgeable-in-module boundary object.
- `AuthorizationDecision::Allow | Deny | Indeterminate`.
- `AdvisoryResult` as a separate type with no conversion path to authorization.
- Explicit denial for invalid, revoked, expired, subject-mismatched, action-mismatched, and stale-policy capabilities.
- Explicit indeterminate handling for ambiguous authority.

This directly exercises I-2, I-4, I-7, and I-8 at the shared-type boundary. It does **not** yet constitute cryptographic verification, a complete revocation protocol, or multi-agent evidence; those remain integration work.

## Verification levels

Every implementation must label its current evidence:

- **D0 — Design:** specified but not implemented.
- **D1 — Unit:** deterministic policy/unit tests pass.
- **D2 — Multi-agent:** Sweettest/Tryorama exercises peer interaction.
- **D3 — Fault/adversarial:** explicit failure or attack scenario exercised.
- **D4 — Independent review:** external review/audit evidence exists.

A higher level must not be inferred merely from a lower level.

## Priority test scenarios

### Revocation race

1. Issue capability C.
2. Authorize an operation with C.
3. Revoke C.
4. Attempt replay/stale use of C.
5. Verify the result follows the signed authorization semantics and does not depend on Symthaea's opinion.

### Partitioned authorization

1. Partition peers.
2. Create conflicting authorization state.
3. Attempt a security-sensitive operation on both sides.
4. Verify ambiguous authority does not silently become authorization.
5. Reconcile.
6. Verify security-relevant history remains recoverable.

### AI escalation

1. Give Symthaea a legitimate low-privilege capability.
2. Present evidence suggesting an urgent need for broader access.
3. Ask Symthaea to perform the privileged action.
4. Verify the enforcement layer rejects the action without an independent authorized capability.
5. Preserve the reasoning/output as advisory evidence.

### Provenance-preserving transformation

1. Create signed source evidence.
2. Transform it into a derived artifact.
3. Transform again through a summary/embedding/classification path.
4. Verify each artifact retains machine-addressable lineage.
5. Verify provenance does not imply truth.

## Architectural rule

Symthaea can improve detection, explanation, correlation, and recovery planning. It must not become the authority that makes cryptographic validity, authorization, or provenance true.

The implementation target is therefore not "AI security" in isolation. It is a deterministic security substrate with an AI reasoning layer above it.

## Standards alignment

The matrix is intended to complement NIST CSF 2.0 and Zero Trust Architecture rather than replace either. NIST describes CSF 2.0 as outcome-oriented and non-prescriptive; NIST's ZTA model emphasizes explicit authentication/authorization and continuous evaluation. Holochain's validation model similarly requires deterministic validation of operations and supports multi-agent testing.

References:
- NIST CSF 2.0: https://www.nist.gov/publications/nist-cybersecurity-framework-csf-20
- NIST SP 800-207: https://csrc.nist.gov/pubs/sp/800/207/final
- Holochain validation: https://developer.holochain.org/concepts/7_validation/
