# Security & Sovereignty Model

Status: design specification
Scope: Mycelix ecosystem and Symthaea integration
Version: 0.1

## Purpose

Mycelix should preserve not only the traditional CIA security properties, but the ability of legitimate participants to retain meaningful control over their identities, data, decisions, and capabilities under failure or adversarial conditions.

This specification extends the CIA triad with Identity, Provenance, Resilience, and Agency as explicit architectural concerns. It does not replace conventional security controls; it makes their relationships explicit.

## Model

### Confidentiality

Question: who is authorized to learn or derive this information?

Required properties:
- least-privilege access
- capability-scoped authorization
- explicit consent for cross-domain access
- private data remains local unless disclosure is authorized
- revocation is explicit and auditable
- derived information receives a sensitivity/provenance context

### Integrity

Question: can a state transition, record, authorization, or claim be trusted as unaltered and valid?

Required properties:
- authenticated authorship
- tamper-evident history
- deterministic validation for shared state
- explicit dependency references
- rejection or quarantine of invalid transitions
- no AI-generated assertion is treated as cryptographic proof

### Availability

Question: can an authorized capability continue operating despite disruption?

Required properties:
- local-first operation where feasible
- graceful degradation
- federation and redundant authorities
- bounded recovery paths
- replay/reconciliation after partition
- essential capabilities must not depend on a single reasoning service

### Identity

Question: who or what is exercising a capability?

Required properties:
- cryptographic identifiers
- explicit device/session binding where required
- delegation chains
- credential lifecycle and revocation
- separation of identity from reputation
- no trust score is itself an identity credential

### Provenance

Question: where did this assertion, state, or artifact come from?

A provenance record should make it possible to identify:
- author
- source artifact
- timestamp or signed temporal evidence
- transformations
- referenced evidence
- authorization context
- verification status
- contradictions or superseding records

Provenance is evidence about origin and transformation; it is not proof that the underlying claim is true.

### Resilience

Question: can the system continue, recover, and converge when components fail or disagree?

Required properties:
- partition-aware behavior
- bounded blast radius
- independent recovery paths
- deterministic reconciliation rules
- evidence-preserving recovery
- no silent downgrade of security policy

### Agency

Question: can the legitimate principal retain meaningful control?

Required properties:
- human-readable authorization boundaries
- explicit delegation
- reversible actions where technically possible
- visible security-relevant state changes
- no autonomous escalation of privileges
- AI recommendations cannot override cryptographic or policy constraints

## Signed capability boundary

The bridge kernel now has an optional Ed25519 path behind the existing identity feature.

- Capability signing bytes use explicit domain separation and length framing rather than JSON serialization.
- Capability actions are canonicalized as a set at construction; duplicate actions are rejected.
- Deserialized capabilities are structurally revalidated at the verification boundary, so serialization/deserialization cannot bypass constructor invariants.
- SignedCapability provides signing and signature verification when the identity feature is enabled.
- The verifier can additionally bind the signature to an expected issuer public key; this is key binding, not institutional authorization. The authority layer must still prove that the expected key is authorized for the issuer and that the authority grant remains current.
- Successful signature verification establishes integrity/authenticity of the signed capability bytes only. It does not establish issuer authorization, current revocation status, or policy authorization.
- The permit/enforcement boundary remains responsible for policy authorization, while the independent identity/revocation layer remains responsible for issuer authorization and current status.
- Verification evidence is bound to the exact canonical capability semantics by a stable capability commitment; the binding is checked both before `VerifiedCapability` creation and again before `EnforcementRequest` creation.
- Authorization permits are additionally capped by `MAX_AUTHORIZATION_PERMIT_LIFETIME_US` (currently five minutes), so a long-lived capability cannot create an equally long-lived enforcement credential. The permit is intentionally non-Clone and is consumed by `EnforcementRequest::from_permit()`, preventing safe in-memory duplication of the same authorization token.
- Permit lifetime is also bounded by the freshness lease of the verification evidence that produced it. A five-minute kernel cap is therefore a maximum, not a promise that five minutes of authority exists.
- Enforcement revalidates both current revocation/authority state and the evidence freshness lease before effect; stale evidence cannot authorize merely because the permit itself has not expired.
- Enforcement also revalidates an opaque authority-freshness commitment; evidence from a changed authority generation cannot silently extend an older permit. `EnforcementRequest` is likewise non-Clone, keeping the successful permit-to-enforcement hand-off linear in safe Rust.
- `VerificationEvidence` is now opaque outside the bridge crate; callers cannot manufacture trusted evidence by supplying boolean fields.
- The unbounded evidence-construction helper is test-only; production verification paths must supply an explicit bounded freshness lease.
- The next adapter should consume the existing Mycelix institutional authority identity rather than inventing a second grant-hashing or authority-identity scheme. The canonical profile developed in PR #75 (`mycelix-authority-grant-v1-blake3-framed-semantic`) binds authority-relevant grant semantics, including proof lineage and delegation parent. That work remains a separate draft integration dependency.

This separation prevents a valid signature from being treated as a synthetic proof of institutional authority, prevents arbitrary callers from manufacturing the verification hand-off itself, and prevents valid evidence for one capability from being replayed against another.

## Trust boundary

Symthaea must not be the root of trust.

The intended layering is:

1. Hardware / operating-system primitives
2. Cryptography and authenticated transport
3. Identity and capability enforcement
4. Holochain integrity validation / Mycelix policy
5. Provenance and evidence graph
6. Symthaea interpretation, anomaly detection, and decision support
7. Human or constitutionally authorized policy decision

Symthaea may identify anomalies, contradictions, missing evidence, or suspicious patterns. It must not manufacture authorization, signatures, provenance, or policy exceptions.
The bridge's public API preserves this ordering: verification yields a `VerifiedCapability`, `authorize_permit()` is the public authorization entry point and yields the bounded `AuthorizationPermit`, and `EnforcementRequest::from_permit()` is the only public path into enforcement. A free-standing public `Allow` helper is intentionally not exposed.

## Cross-property invariants

### I-1: No confidentiality bypass through reasoning

A Symthaea inference must not reveal data that the underlying authorization layer would deny.

### I-2: No integrity bypass through reasoning

A model assertion cannot substitute for a valid signature, source-chain transition, or deterministic validation result.

### I-3: No availability bypass through emergency mode

Degraded operation may reduce functionality, but must not silently broaden privileges.

### I-4: Identity is orthogonal to reputation

A participant's historical trust or contribution score must never become the sole proof of identity or authorization.

### I-5: Provenance survives transformation

When data is transformed, derived, summarized, embedded, or interpreted, the resulting artifact retains a machine-addressable relationship to its source evidence.

### I-6: Recovery preserves evidence

Recovery and reconciliation must not erase security-relevant history merely because a newer local state is selected.

### I-7: AI remains subordinate to enforceable policy

AI output is advisory unless an independently authorized policy explicitly grants a capability to an automated actor. Even then, the granted capability is bounded and revocable.

### I-8: Fail closed on authority ambiguity

When a security-critical authorization cannot be established deterministically, the operation must not be treated as authorized merely because an AI system considers it plausible.

## Security event envelope

Successful enforcement events produced from `EnforcementRequest` now carry the kernel-derived capability commitment in addition to any external capability reference. The event envelope is immutable outside the module: serialized fields are private, so a caller cannot mutate the actor, request, decision, policy version, or enforcement timestamp after construction. They also carry the opaque authority-freshness commitment supplied by the authority adapter. The event constructor additionally requires the recorded actor identity to equal the enforcement subject, preventing an authorized action from being attributed to a different principal. This makes the audit record able to distinguish the exact capability semantics, authority-generation evidence, and authorized actor that crossed the enforcement boundary from caller-supplied labels. `SecurityEvent::new()` cannot mint an `Allow` record directly; successful Allow records must originate from an enforcement request that already crossed permit revalidation. Deny and Indeterminate records may still be recorded directly, while serialized Allow records are accepted only when they carry both kernel-derived bindings and actor/request coherence; legacy Deny/Indeterminate records may omit bindings, while unbound historical Allow records are rejected on deserialization. Successful enforcement event timestamps are also required to equal the permit revalidation timestamp carried by `EnforcementRequest`, so the event cannot independently backdate or future-date the enforcement boundary.

Security-sensitive events should converge toward a common conceptual envelope:

- event identifier
- actor identity
- capability used
- action
- target/resource
- provenance references
- policy version
- validation result
- temporal evidence
- consequence/recovery metadata

This envelope can be implemented independently by each cluster while remaining compatible with shared bridge and evidence infrastructure.

## Symthaea integration

Symthaea should consume security state as structured evidence rather than opaque prompts.

Useful outputs include:
- anomaly hypotheses
- contradiction sets
- missing-evidence requests
- risk/context explanations
- recovery suggestions
- capability-impact analysis

These outputs should be explicitly typed as hypotheses or recommendations. The enforcement layer remains deterministic.

## Verification strategy

Every invariant should eventually have at least one executable test.

Preferred progression:

1. Rust unit/property tests for pure policy functions.
2. Multi-agent Sweettest/Tryorama scenarios for cross-peer validation.
3. Fault injection for partitions, stale credentials, replay, malformed records, and unavailable authorities.
4. Deterministic evidence capture with commit SHA and test artifacts.
5. User playtest only for workflows where human interpretation is itself part of the requirement.

Security claims should distinguish:
- mechanically verified
- integration verified
- simulated adversarial behavior
- externally audited
- proposed/design-only

## Relationship to existing standards

This model complements rather than replaces established frameworks such as NIST CSF 2.0 and Zero Trust Architecture.

NIST CSF 2.0 provides a broad taxonomy for managing cybersecurity risk. Zero Trust emphasizes explicit authentication and authorization rather than implicit trust based on network location. Mycelix can supply decentralized identity, provenance, validation, and resilient state mechanisms underneath those operational goals.

## Non-goals

This document does not claim:
- that Mycelix is externally audited;
- that Holochain validation alone solves confidentiality;
- that reputation is equivalent to trust;
- that Symthaea can reliably determine truth from arbitrary evidence;
- that decentralized infrastructure automatically provides availability;
- that the model replaces established security engineering practices.

## Initial implementation targets

1. Add a shared security-event/provenance vocabulary to the bridge layer.
2. Define typed capability and authorization outcomes.
3. Define machine-checkable cross-property invariants.
4. Add multi-agent tests for authorization, revocation, replay, and partition recovery.
5. Add Symthaea adapters that expose hypotheses without granting enforcement authority.
6. Produce an evidence capsule for each verified invariant.

## Design principle

> Preserve trustworthy human capability under adversarial conditions.

CIA remains the foundation. Identity, provenance, resilience, and agency make the foundation composable across a sovereign, federated system.