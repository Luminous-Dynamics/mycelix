# Relational capability containment — Rust reference model v0.1

**Status:** bounded, dependency-free Rust reference candidate. Research-only; not a production authority service, credential, Holochain zome, or Xenia runtime adapter. Formatting, compilation, tests and Clippy must pass on the exact branch head before this model is treated as qualified.

## Question

Can a delegated capability be narrowed without accidentally manufacturing new combinations of otherwise valid fields?

A capability is a finite relation of complete effect tuples:

    (action, resource, direction, audience, purpose,
     data_class, mode, operation_binding, quantitative_limit)

A child is attenuated only when every child tuple has a complete parent-tuple witness and the child relation is semantically narrower:

    ChildEffects ⊊ ParentEffects

This is not the same as checking whether each column independently appears in the parent projection.

## Why marginal containment is insufficient

Suppose the parent grants exactly:

- screen.view → host-A → viewer-A → remote-support
- file.receive → host-B → viewer-B → file-delivery

A component-wise check sees both actions, both resources, both audiences and both purposes in the parent. It can therefore accidentally admit a cross-product such as screen.view → host-B → viewer-B → file-delivery.

That tuple was never authorized. The Rust model rejects it because no single parent tuple subsumes the complete effect tuple.

This is the remote-session instance of the relational-containment problem in [Mycelix #4852](https://github.com/Luminous-Dynamics/mycelix/issues/4852). Typed ROS-005 scope dimensions are tracked in [#6302](https://github.com/Luminous-Dynamics/mycelix/issues/6302).

## Implemented semantics

- Complete tuple membership over action, resource, direction, audience, purpose, data class, proposal/execute mode, operation binding and a numeric ceiling with an explicit unit.
- Parent-side wildcard (Any) subsumption. A wildcard child cannot be attenuated by an exact parent.
- Exact-match direction; proposal is contained by execute, but execute is not contained by propose.
- Quantitative ceiling attenuation only when the limit dimension/unit matches.
- Non-widening and semantic proper-subset attenuation, including wildcard/bound semantics.
- Distinct intersection for authority scopes that are all required and union for independent alternative grants.
- Deterministic, sorted, schema-versioned canonical bytes with a domain separator.
- Fail-closed rejection of unsupported schema versions, duplicate syntactic tuples, and malformed/oversized atoms.
- A bounded exhaustive 256 × 256 test over every pair of subsets of an eight-tuple universe.
- An adversarial control where every individual projection matches while the recombined tuple is unauthorized.

The subsumption rule requires a complete witness in a single parent tuple. This is intentionally conservative for this finite profile: it never combines separate parent tuple fields to create a new capability. It does not implement general semantic alias normalization.

## Relationship to existing SOV-AI-AUTH-v17 / PR #4867

Mycelix already has a formal relational-containment candidate in [PR #4867](https://github.com/Luminous-Dynamics/mycelix/pull/4867), including TLA+, Alloy, a Python reference explorer and ten named negative controls. That candidate models a payment-oriented tuple:

    (operation, target, audience, currency, argument_class, numeric_bound)

This Rust crate is a separate, dependency-free implementation aimed at Xenia-style remote effects. Its tuple includes direction, purpose, data class, mode and operation binding; it has a typed numeric dimension; and it currently does not normalize aliases such as acct-alice to alice. Therefore:

- It is a **companion implementation**, not a replacement for #4867's TLA+/Alloy model.
- It is **not yet a differential-equivalent implementation** of the SOV-AI-AUTH-v17 schema.
- Shared exact-tuple cases can be adapted for differential testing, but currency/alias/wildcard cases need an explicit schema mapping before comparison.
- No claim is made that its canonical bytes match the TLA+, Alloy or Python encodings.

The next useful validation is a frozen cross-language fixture subset with separately authored expected outcomes, followed by the independent TLA+/Alloy execution required by #4867. Do not infer formal qualification from a Rust unit-test pass.

## Verification

The dedicated workflow runs against this standalone manifest: pinned cargo fmt check, cargo test --all-targets, and strict cargo clippy --all-targets -- -D warnings.

The authoring container has no Rust toolchain. No local compilation, hosted pass, formal verification, or performance result is claimed yet.

## Explicit boundaries / non-claims

This model does not implement or qualify principal authentication, issuer signatures, parent record retrieval, ROS-005 authority/currentness, Holochain capability grants or DHT convergence; trusted time, authority epochs, revocation events or stale-parent propagation; durable/atomic budgets shared across sibling delegations, rolling-window rates, concurrency limits or restart-safe consumption; canonical operation digest cryptography or one-shot consume-once execution; exact OS/provider effect commits or independent outcome observation; complete route inventory or non-bypassability; alias equivalence, arbitrary hierarchical resource semantics or open-world schema negotiation.

The operation_binding field is an opaque, caller-supplied canonical identifier in this reference model. It is not authenticated, signed or independently recomputed. A numeric limit is a per-tuple ceiling, not a durable aggregate budget or proof that a sequence of operations respects a rolling quota.

For Xenia, the eight M1 permission/action/direction mappings in [draft PR #509](https://github.com/Luminous-Dynamics/xenia-peer/pull/509) are statically paired. This relational model is a prospective schema constraint for future purpose/audience/resource/budget extensions; Xenia does not currently consume ROS-005 or this crate.

## Owner composition

- ROS-005 semantic delegation: [Mycelix #3659](https://github.com/Luminous-Dynamics/mycelix/issues/3659)
- Addressable parent chains and content-bound identity: [#3677](https://github.com/Luminous-Dynamics/mycelix/issues/3677) / [PR #3679](https://github.com/Luminous-Dynamics/mycelix/pull/3679)
- Typed scope extensions: [#6302](https://github.com/Luminous-Dynamics/mycelix/issues/6302)
- Relational containment theorem: [#4852](https://github.com/Luminous-Dynamics/mycelix/issues/4852) / [PR #4867](https://github.com/Luminous-Dynamics/mycelix/pull/4867)
- Xenia ROS-005 ↔ M1 adapter contract: [Xenia #517](https://github.com/Luminous-Dynamics/xenia-peer/issues/517)
- One-shot effect-boundary enforcement and concurrency remain separate: Mycelix [#6189](https://github.com/Luminous-Dynamics/mycelix/issues/6189) / [#6193](https://github.com/Luminous-Dynamics/mycelix/issues/6193), Xenia [#504](https://github.com/Luminous-Dynamics/xenia-peer/issues/504).