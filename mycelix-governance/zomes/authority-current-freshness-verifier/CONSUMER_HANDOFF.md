# Current Operational Freshness — Consumer Handoff Contract v0.1

Status: **normative consumer boundary; current-freshness runtime remains unprovisioned and external effects remain disabled**

This contract defines how another local runtime component may consume the output of `authority_current_freshness_verifier` without turning a serializable receipt into an authority oracle.

## 1. The wire object is audit/transport data

The externally serializable response type is:

`CurrentOperationalFreshnessAuditReceipt`.

Its name is intentional. It carries the exact semantic, provenance, deployment and lease identities produced by the currentness runtime for audit, diagnostics, caching keys and direct local-provider consumption.

Deserializing, copying, replaying or receiving this object from a caller does **not** create positive current authority.

The old name `VerifiedCurrentOperationalFreshnessReceipt` is forbidden because it obscures this distinction.

## 2. Positive currentness origin is a local verifier call

A live lifecycle, signing, executor, claim, effect-safety or external-effect admission path that needs current operational authority MUST obtain it by directly calling the designated local currentness verifier for the exact `AuthoritySubjectRef` being consumed.

Conceptually:

`target AuthoritySubjectRef`
→ local `authority_current_freshness_verifier::resolve_current_operational_freshness`
→ `CurrentOperationalFreshnessAuditReceipt`
→ exact consumer-side field/lease/domain binding
→ later domain-specific authority theorem.

The trust boundary is the direct local verifier invocation plus the consumer's own exact cross-binding. It is not the Rust/serde shape of the returned object.

## 3. Caller-supplied positive currentness is forbidden

A public or caller-controlled request for lifecycle/effect admission MUST NOT contain any of the following as positive authority input:

- `CurrentOperationalFreshnessAuditReceipt`;
- the superseded `VerifiedCurrentOperationalFreshnessReceipt`;
- `VerifiedAuthorityFreshness`;
- a deployment-authority digest/profile asserted by the caller;
- a deployment-evidence digest/profile asserted by the caller;
- a provenance-manifest digest/profile/count asserted by the caller; or
- a claimed currentness lease.

Informational copies may be logged/correlated, but MUST be ignored for positive authority and recomputed/resolved through the local verifier.

## 4. Consumer request surfaces are target/action oriented

A current execution consumer should receive only the domain facts the caller is entitled to request, for example:

- exact target authority subject;
- exact proposal/action/attempt identity;
- non-authoritative correlation metadata; and
- application payload required by that domain.

It must not ask the caller to prove current authority by submitting a positive currentness receipt.

## 5. Exact subject and deployment binding is mandatory

After the direct local call, the consumer MUST require that the receipt's exact `subject` equals the authority subject it requested.

Where the consumer has a proposal/action/executor/effect-safety domain, that domain must independently bind the expected authority identity and require exact equality with the receipt's semantic/deployment identities. A bare textual grant/policy/designation ID is insufficient.

## 6. Lease is checked at the consuming boundary

The consumer MUST obtain its own current host time after the local currentness call and require the returned deployment/currentness lease to remain live at the moment of admission.

A caller timestamp, cached timestamp or earlier verifier timestamp cannot select the consuming verification time.

No consumer may lengthen the returned lease.

## 7. Caching never changes origin semantics

A cache may avoid repeated work only under a separately qualified cache policy and only while the exact receipt lease remains live.

At minimum a cache key must bind:

- exact target subject identity;
- exact stable deployment authority identity;
- exact dynamic deployment evidence identity; and
- exact canonical provenance-manifest identity or an equivalent commitment already included in deployment evidence.

A cache hit is not equivalent to caller-supplied authority: the cache must contain only receipts originally obtained from the designated local verifier and must never extend their lease.

## 8. Currentness is necessary but not effect authority

A positive local currentness response establishes only that the exact operational authority subject is currently qualified under the current constitution/policies/source state and local deployment context.

It does not by itself:

- satisfy a threshold signature;
- select/designate an executor;
- qualify an effect-safety mechanism;
- create an execution lifecycle `Ready` or `Claimed` event;
- create a durable effect claim;
- authorize a retry; or
- permit an external effect.

Those remain later independent theorems.

## 9. Final pre-effect admission must re-resolve currentness

A durable Ready/Claim state or earlier positive receipt MUST NOT provide a lease across revocation, supersession, policy/constitution rotation, deployment change or evidence expiry.

Immediately before an effect-capable boundary is admitted, that boundary must directly re-resolve every current operational authority subject required by its exact domain unless a stronger same-invocation non-deserializable composition proves equivalent freshness.

## 10. Historical verification is separate

Historical/as-of audit may consume stored receipts as evidence of what a verifier reported at a past time.

Historical receipt validity cannot authorize a new live claim/effect. Live execution always uses current local verification.

## 11. Failure semantics

Any of the following means DENY / unavailable for live admission:

- currentness verifier missing/unavailable;
- call/decode failure;
- subject mismatch;
- malformed profile/digest/reference;
- future verification time;
- expired lease;
- wrong DNA/deployment identity;
- missing or contradictory provenance/deployment evidence;
- inability to bind the response to the exact consumer domain; or
- temptation to fall back to caller-submitted/cached stale authority.

There is no reputation, Phi, model-score, Guardian, creator, role or best-effort fallback.

## 12. Packaging state

`authority_current_freshness_verifier` remains absent from binding `dna.yaml` in this tranche. No lifecycle/effect consumer is provisioned by this contract and no external effects are enabled.

Before provisioning any effect-capable consumer, CI/adversarial tests must prove that forged caller-supplied audit receipts are ignored/denied and that the consumer performs the direct local currentness call itself.
