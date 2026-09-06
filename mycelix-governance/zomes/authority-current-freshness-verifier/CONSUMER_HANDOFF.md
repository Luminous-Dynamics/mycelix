# Current Operational Freshness — Consumer Handoff Contract v0.2

Status: **normative consumer boundary; current-freshness runtime remains unprovisioned and external effects remain disabled**

This contract defines how another local runtime component may consume `authority_current_freshness_verifier` output without turning a serializable receipt into an authority oracle.

## 1. The wire object is audit/transport data

The externally serializable response is `CurrentOperationalFreshnessAuditReceipt`.

It carries semantic authority identity, stable deployment identity, dynamic deployment evidence, currentness-composition provenance, final binding-constitution evidence and the final narrowed lease.

Deserializing, copying, replaying or receiving this object from a caller does **not** create positive current authority.

## 2. Positive currentness origin is a local verifier call

A live lifecycle, signing, executor, claim, effect-safety or external-effect boundary that needs current operational authority MUST obtain it by directly calling the designated local currentness verifier for the exact `AuthoritySubjectRef` being consumed.

Conceptually:

`target AuthoritySubjectRef`
→ local `authority_current_freshness_verifier::resolve_current_operational_freshness`
→ `CurrentOperationalFreshnessAuditReceipt`
→ exact consumer-side subject/domain/evidence/lease binding
→ later domain-specific authority theorem.

The trust boundary is the direct local verifier call plus the consumer's own exact cross-binding, never the serde shape of the returned object.

## 3. Caller-supplied positive currentness is forbidden

A caller-controlled live-admission request MUST NOT use any of these as positive authority input:

- `CurrentOperationalFreshnessAuditReceipt`;
- the superseded `VerifiedCurrentOperationalFreshnessReceipt`;
- `VerifiedAuthorityFreshness`;
- caller-asserted semantic/deployment authority identities;
- caller-asserted deployment-evidence identity;
- caller-asserted #192 provenance manifest identity;
- caller-asserted binding-constitution context identity/reference/window; or
- caller-asserted currentness/deployment lease.

Informational copies may be logged or correlated, but they are ignored for positive authority and independently re-resolved.

## 4. Consumer request surfaces are target/action oriented

A current execution consumer should receive only domain facts the caller may legitimately request: exact target subject, proposal/action/attempt identity, application payload and non-authoritative correlation metadata.

It must not ask the caller to prove current authority by submitting a positive receipt.

## 5. Exact subject and stable deployment binding is mandatory

After the direct local call, the consumer MUST require exact subject equality with the authority subject it requested.

Where the consumer has a proposal/action/executor/effect-safety domain, that domain must independently bind the expected semantic/stable deployment authority identity and require exact equality. Bare textual grant/policy/designation IDs are insufficient.

## 6. Two dynamic evidence phases are visible

The receipt exposes two related but causally distinct evidence phases.

### Currentness composition provenance

`composition_evidence_manifest_*` describes the canonical #192 evidence set that established #117 currentness.

### Final deployment-constitution evidence

`binding_constitution_*` describes the later final binding-constitution observation that fenced deployment after #117/#192 composition.

These fields are audit evidence. Neither phase alone is an execution grant.

A consumer MUST NOT treat the #192 manifest alone as sufficient deployment proof because the final constitutional re-observation happens after that manifest is qualified.

## 7. Final deployment evidence is the dynamic binding key

For live reuse/caching, the authoritative dynamic evidence key exported by this verifier is the final `deployment_evidence_digest/profile` plus its final verification reference/window.

That dynamic identity commits the canonical #192 provenance through #201 and the final binding-constitution context through the later deployment theorem.

A consumer may retain the component evidence fields for audit/cross-checking, but it MUST NOT reconstruct or substitute its own dynamic deployment-evidence identity from sidecar fields.

## 8. Lease is checked at the consuming boundary

The consumer MUST obtain its own current host time after the local verifier call and require the returned final deployment/currentness lease to remain live at admission time.

The final lease is already bounded by currentness evidence, final constitutional evidence, host DNA context and the deployment return cap. The consumer may shorten it further but never extend it.

## 9. Caching never changes origin semantics

A cache may avoid repeated work only under a separately qualified cache policy and while the exact final lease remains live.

At minimum a cache key must bind:

- exact target subject;
- exact stable deployment authority identity; and
- exact final dynamic deployment-evidence identity.

The final deployment-evidence identity already commits #192 composition provenance and final binding-constitution evidence. Storing those component identities separately is recommended for forensic audit but does not replace the final evidence key.

A cache must contain only receipts originally obtained from the designated local verifier and may never extend their lease.

## 10. Currentness is necessary but not effect authority

A positive local response does not itself satisfy threshold authorization, designate an executor, qualify effect safety, create lifecycle Ready/Claimed state, create a durable effect claim, authorize retry or permit an external effect.

Those remain later independent theorems.

## 11. Final pre-effect admission must re-resolve currentness

A durable Ready/Claim state or earlier positive receipt MUST NOT provide a lease across revocation, supersession, policy/constitution rotation, deployment change or evidence expiry.

Immediately before an effect-capable boundary is admitted, that boundary must directly re-resolve every current operational authority subject required by its exact domain unless a stronger same-invocation non-deserializable composition proves equivalent freshness.

## 12. Historical verification is separate

Stored receipts may support historical/as-of audit of what a verifier reported. Historical receipt validity cannot authorize a new live claim/effect.

## 13. Failure semantics

Any missing verifier, call/decode failure, subject/domain mismatch, malformed evidence identity/reference/profile, future verification time, expired final lease, wrong DNA/stable deployment identity, missing/contradictory currentness or final-constitution evidence, or inability to bind the response to the consumer domain means **DENY / unavailable**.

There is no reputation, Phi, model-score, Guardian, creator, role or best-effort fallback.

## 14. Packaging state

`authority_current_freshness_verifier` remains absent from binding `dna.yaml`. No effect-capable consumer is provisioned here and no external effects are enabled.

Before provisioning any effect consumer, adversarial tests must prove forged caller-supplied audit receipts are ignored, the consumer performs the direct local verifier call itself, the final lease is rechecked, and #192 sidecar provenance cannot substitute for final deployment evidence.
