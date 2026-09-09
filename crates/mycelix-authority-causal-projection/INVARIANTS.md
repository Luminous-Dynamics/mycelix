# Mycelix Authority Causal Projection v0.1 — Normative Invariants

Status: **pure historical causal selector over PR #91 covered authority state**

This crate answers one narrow question:

> Given one exact authority subject and one exact signed causal coordinate `(generation, transition digest)`, which historical authority state does that coordinate identify inside the fully and currently covered authoritative lineage?

It deliberately does not decide which causal coordinate a business/governance event should claim. The event protocol must sign that coordinate itself.

## 1. Full current source coverage comes first

The theorem always re-runs PR #91 `project_current_authority_state` before selecting any historical transition.

Therefore a valid old prefix is never enough. Later revocation/reactivation/supersession remains visible in the fully covered lineage even when the target is an older generation.

## 2. Causal coordinate, not wall clock

Historical selection is by exactly:

- authority subject;
- transition generation; and
- exact PR #91 transition identity digest.

There is no `as_of_ms`, event timestamp, DHT arrival time, author timestamp, or "newest" selector.

`effective_at_ms` remains part of PR #91 transition semantics and monotonic causal validation, but this crate never uses it to choose the target.

## 3. Generation alone is insufficient

A numeric generation without the exact transition digest is not a causal anchor.

The target digest protects the caller from silently rebinding an event to different same-generation semantics if corrupted or forked evidence is later encountered. PR #91 already rejects actual same-generation forks before this theorem can succeed.

## 4. Historical result cannot become live authority

`QualifiedCausalAuthorityStateProjection` has no conversion to `VerifiedAuthorityFreshness`.

A historical Active state may establish that an older event referenced an Active authority generation. It cannot reactivate that subject for current execution.

## 5. Later state changes do not rewrite the target

If generation 1 was Active and generation 2 later Revoked it, a signed anchor to generation 1 still resolves to generation 1 Active—provided the full currently covered lineage remains valid and contains that exact transition.

If later evidence proves the transition or source coverage invalid, qualification fails rather than preserving an invalid historical claim.

## 6. Input order is never authority

The underlying PR #91 projector reconstructs the causal generation chain independently of caller order. Reversing receipt order cannot change the causal projection identity.

## 7. Dynamic source evidence remains dynamic

Coverage verification time and lease are required to establish that the full lineage is currently and completely observed. They are carried as evidence bounds but do not choose the historical target.

## 8. Cross-lineage protocols must sign the anchor

This theorem does not infer which authority generation authorized an unrelated governance event.

A secure cross-lineage protocol must commit the exact authority subject + generation + transition digest inside the event's signed transcript. Choosing the anchor only after signature verification would allow retroactive authority-state substitution.

For the Identity historical-time-policy work, PR #424 provides the exact signer-authority subject. A later authority-anchored transition envelope must make the PR #91 causal coordinate part of what the policy-authority signer authenticates.
