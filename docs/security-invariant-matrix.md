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
| I-9 | Temporal integrity | unit confusion or lease extension | checked ms→µs conversion; exclusive freshness bound; permit expiry is the minimum of capability, evidence lease, and kernel maximum | conversion overflow, cross-unit comparison, lease refresh after permit, generation change after permit | qualification trace |

## Current implementation

The first deterministic authority boundary is implemented in
`crates/mycelix-bridge-common/src/security_kernel.rs`.

The kernel provides:

- `Capability` and `AuthorizationRequest` as explicit inputs; both wire-deserialization paths are constructor-gated so serialized input cannot bypass their constructor invariants. Their private wire schemas also reject unknown fields, preventing field-smuggling across the signed semantic boundary.
- Security-domain JSON inputs have a separate outer parser/input-envelope bound: `deserialize_bounded_security_json()` and `deserialize_bounded_security_json_reader()` reject the input budget at 512 KiB before untrusted security JSON crosses the parser boundary, closing the parser-level gap that field retention limits cannot address. The two helpers cover byte-slice and `io::Read` adapter paths and are the required boundaries for attacker-controlled JSON adapters.
- `ProvenanceRef` uses the same constructor-gated wire boundary, rejects unknown fields, and keeps its invariant-bearing fields private so callers cannot bypass `ProvenanceRef::new()` with a struct literal or mutation.
- Security-event recovery correlations are also constructor-gated during wire decoding, and untrusted provenance sequences are bounded before retention to keep audit input fail-closed and resource-bounded.
- `VerificationEvidence` as an opaque, non-serializable hand-off from an independent cryptographic/identity verifier; its trusted fields cannot be constructed or deserialized by downstream callers, and it intentionally does not implement `Debug`, `PartialEq`, or `Eq` so downstream code cannot turn trusted evidence into an observation/comparison surface.
- `VerifiedCapability` as a non-forgeable-in-module boundary object.
- `AuthorizationDecision` as failure-only authorization outcomes (`Deny | Indeterminate`); `SecurityEventDecision::Allow | Deny | Indeterminate` is reserved for audit records.
- `AdvisoryResult` as a separate type with no conversion path to authorization.
- Explicit denial for invalid, revoked, expired, subject-mismatched, action-mismatched, stale-policy, and evidence-mismatched capabilities.
- Explicit indeterminate handling for ambiguous authority.

This directly exercises I-2, I-4, I-7, and I-8 at the shared-type boundary. The capability commitment is additionally regression-tested against every authority-relevant capability field (subject, issuer, resource, action set, validity bounds, and policy version), so omission of a field from the signed/committed semantic representation is detected deterministically. The qualification lane also guards the negative API surface: trust-construction items must remain module-private under any Rust visibility form, the `Capability`/`AuthorizationRequest` wire constructor gates must remain present, the security wire identifiers must use bounded deserializers before retention, and the opaque evidence/linear enforcement tokens must not gain serialization implementations. These are regression guards, not provenance proof. It does **not** yet constitute cryptographic verification, a complete revocation protocol, or multi-agent evidence; those remain integration work.

## Current enforcement boundary

The kernel now has a second, narrower boundary after authorization:

1. `authorize_permit()` is the public authorization entry point: it evaluates the verified capability against the exact request and mints the bounded permit on success.
2. A successful decision yields an `AuthorizationPermit` that is not serializable, has no public constructor or public inspection methods, is marked `#[must_use]`, and is opaque until consumed by `EnforcementRequest::from_permit()`. `AuthorizationPermit` and `EnforcementRequest` intentionally do not implement `Debug`, `Serialize`, `Deserialize`, `Clone`, or `PartialEq/Eq`; their state therefore cannot be exposed through ordinary logging/serialization, duplicated through derived cloning, or observed through equality comparisons.
3. The resulting `EnforcementRequest` is also `#[must_use]`, making accidental dropping of the effect-bound hand-off visible to the compiler.
4. `EnforcementRequest::from_permit()` is the only public constructor for an enforcement request and revalidates the permit at the enforcement boundary. A permit is consumed by this operation and is intentionally not `Clone`, preventing safe in-memory duplication of the same authorization token. `SecurityEvent` likewise exposes no mutable serialized fields; callers receive read-only accessors and explicit consuming enrichment methods.
5. Deny and Indeterminate outcomes produce no permit.
6. Post-issuance revocation or authority ambiguity blocks enforcement rather than relying on the earlier Allow.
7. Enforcement additionally rejects verification evidence whose capability commitment does not match the permit being exercised.
8. Permits also carry an opaque authority-freshness commitment supplied by the authority adapter; revalidation rejects evidence from a different authority generation. Authorization permits and resulting enforcement requests are intentionally non-duplicable (not `Clone`), so a caller cannot create multiple enforcement hand-offs from the same in-memory permit. This does not define single-use semantics for the underlying effect; the enforcement adapter must decide whether its operation is idempotent or one-time.
9. `SecurityEvent` records Allow, Deny, and Indeterminate decisions and can carry explicit provenance references and recovery correlation, including the authority-freshness commitment for successful enforcement. Its entire serialized envelope is private, so external callers cannot fabricate or mutate an event by struct literal; callers receive read-only accessors and explicit consuming enrichment methods. Successful Allow events must originate from a revalidated `EnforcementRequest`. Deserialization is deliberately stricter: Deny/Indeterminate records may be re-ingested only without enforcement bindings, while `Allow` records are rejected entirely because serialized bindings alone cannot prove live enforcement provenance. Successful enforcement-event timestamps must equal the `EnforcementRequest` revalidation timestamp, preventing independent backdating or future-dating of the enforcement record.

This prevents a downstream enforcement adapter from accepting an arbitrary request as though it had already passed the policy decision point. It also makes the authorization decision reconstructable without making the event record itself authoritative.

The new implementation remains a policy/type boundary. `VerificationEvidence` is now intentionally opaque and its trusted constructors/proposition types are private to the `security_kernel` module subtree rather than exposed crate-wide. Qualification also treats any explicit `pub(...)` visibility as a regression, rejects reintroduction of serialization on trust tokens, and rejects `Debug`/`PartialEq`/`Eq` on `VerificationEvidence`, so containment and non-observability are checked mechanically rather than relying on convention. Its three boolean-like trust propositions are represented by distinct private enums rather than raw booleans, preventing accidental cross-proposition assignment at call sites; its capability binding is derived from the canonical capability semantics, so evidence cannot be substituted between capabilities. This visibility boundary is containment, not provenance proof: the kernel still must receive these propositions from an actual verifier/authority adapter, and no type-level claim of `Verified`, `Current`, or `Unambiguous` is by itself evidence that the underlying check was performed. The actual identity/authority adapter still needs to supply trustworthy signature, revocation, and authority evidence. Its authority-freshness commitment is intentionally opaque in the bridge layer and must track the existing generation-bound authority semantics rather than create a parallel generation model. The intended adapter should reuse Mycelix's existing canonical institutional authority identity (PR #75) rather than duplicate grant identity semantics.

A zero authority-freshness commitment is treated as missing authority evidence and therefore yields `Indeterminate(AmbiguousAuthority)` at both verification and enforcement; it is never a valid “unknown” placeholder for a permit.

Enforcement security events additionally require `actor_id == request.subject`; a caller cannot use the authoritative enforcement-event constructor to attribute an authorized operation to another principal. The general event constructor also rejects policy-version mismatches, keeping newly constructed records internally coherent.

## Evidence durability invariant

Build reproducibility is part of the security evidence boundary as well: the standalone bridge crate is committed with a `Cargo.lock`, and qualification invokes Cargo with `--locked`. A missing or divergent lockfile therefore fails qualification rather than silently changing the dependency graph.

The kernel enforces a stronger rule than a fixed permit maximum:

> **No authorization decision may become more durable than the evidence supporting it.**

A permit's effective validity is bounded by all of:

1. the capability's own validity window (exclusive at `expires_at_us`);
2. the verification evidence freshness lease (exclusive at `valid_until_us`);
3. the kernel's maximum authorization-permit lifetime.

The five-minute constant is therefore an upper bound, not an implied freshness guarantee. The kernel also refuses to mint a permit at the exact evidence-lease boundary, so freshness is enforced both at authorization issuance and at enforcement.

At enforcement time, the evidence lease is checked again alongside revocation and authority ambiguity. A permit cannot outlive the evidence that justified it merely because the permit's own timestamp has not expired. It also cannot be exercised before its issuance timestamp; the enforcement boundary rejects pre-issuance time travel. The evidence lease is an exclusive upper bound: `now == valid_until` is already stale, matching the existing generation-bound authority freshness semantics.

This is the bridge's local form of continual evaluation: the policy decision is not treated as permanently authoritative after issuance. NIST's Zero Trust Architecture similarly separates policy decision from enforcement and describes ongoing evaluation as supporting information changes over the course of a session.

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