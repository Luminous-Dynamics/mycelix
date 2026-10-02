- `VerifiedCapability` as a non-forgeable-in-module boundary object.
- `AuthorizationDecision::Allow | Deny | Indeterminate`.
- `AdvisoryResult` as a separate type with no conversion path to authorization.
- Explicit denial for invalid, revoked, expired, subject-mismatched, action-mismatched, stale-policy, and evidence-mismatched capabilities.
- Explicit indeterminate handling for ambiguous authority.

This directly exercises I-2, I-4, I-7, and I-8 at the shared-type boundary. It does **not** yet constitute cryptographic verification, a complete revocation protocol, or multi-agent evidence; those remain integration work.

## Current enforcement boundary

The kernel now has a second, narrower boundary after authorization:

1. `authorize_permit()` is the public authorization entry point: it evaluates the verified capability against the exact request and mints the bounded permit on success.
2. A successful decision yields an `AuthorizationPermit` that is not serializable and has no public constructor.
3. `EnforcementRequest::from_permit()` is the only public constructor for an enforcement request and revalidates the permit at the enforcement boundary.
4. Deny and Indeterminate outcomes produce no permit.
5. Post-issuance revocation or authority ambiguity blocks enforcement rather than relying on the earlier Allow.
6. Enforcement additionally rejects verification evidence whose capability commitment does not match the permit being exercised.
7. Permits also carry an opaque authority-freshness commitment supplied by the authority adapter; revalidation rejects evidence from a different authority generation.
8. `SecurityEvent` records Allow, Deny, and Indeterminate decisions and can carry explicit provenance references and recovery correlation, including the authority-freshness commitment for successful enforcement. Its general constructor cannot mint an Allow record; successful Allow events must originate from a revalidated `EnforcementRequest`, while Deny and Indeterminate records remain directly recordable.
Successful enforcement-event timestamps must equal the `EnforcementRequest` revalidation timestamp, preventing independent backdating or future-dating of the enforcement record.

This prevents a downstream enforcement adapter from accepting an arbitrary request as though it had already passed the policy decision point. It also makes the authorization decision reconstructable without making the event record itself authoritative.

The new implementation remains a policy/type boundary. `VerificationEvidence` is now intentionally opaque and can only be constructed inside the bridge crate; its capability binding is derived from the canonical capability semantics, so evidence cannot be substituted between capabilities. The actual identity/authority adapter still needs to supply trustworthy signature, revocation, and authority evidence. Its authority-freshness commitment is intentionally opaque in the bridge layer and must track the existing generation-bound authority semantics rather than create a parallel generation model. The intended adapter should reuse Mycelix's existing canonical institutional authority identity (PR #75) rather than duplicate grant identity semantics.

A zero authority-freshness commitment is treated as missing authority evidence and therefore yields `Indeterminate(AmbiguousAuthority)` at both verification and enforcement; it is never a valid “unknown” placeholder for a permit.

Enforcement security events additionally require `actor_id == request.subject`; a caller cannot use the authoritative enforcement-event constructor to attribute an authorized operation to another principal. The general event constructor also rejects policy-version mismatches, keeping newly constructed records internally coherent.

## Evidence durability invariant