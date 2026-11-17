# Relational capability containment — reference model v0.1

**Status:** bounded, dependency-free Rust reference model. Research-only; not a production authority service, credential, Holochain zome, or Xenia runtime adapter.

## Question

Can a delegated capability be narrowed without accidentally manufacturing new combinations of otherwise valid fields?

A capability is modeled as a finite relation of complete effect tuples:

    (action, resource, direction, audience, purpose, data_class, mode, operation_binding)

A child is attenuated only when its set of complete tuples is a proper subset of its parent's set:

    ChildEffects is a proper subset of ParentEffects

This is stronger than checking whether each child column independently appears in a parent projection.

## Why marginal containment is insufficient

Suppose the parent grants exactly:

- screen.view → host-A → viewer-A → remote-support
- file.receive → host-B → viewer-B → file-delivery

A component-wise check sees both actions, both resources, both audiences and both purposes in the parent. It can therefore accidentally admit a cross-product such as:

- screen.view → host-B → viewer-B → file-delivery

That tuple was never authorized. The reference model rejects it because the complete tuple is not a member of the parent relation.

This is the remote-session instance of the relational-containment problem already identified in [Mycelix #4852](https://github.com/Luminous-Dynamics/mycelix/issues/4852). Typed ROS-005 dimensions are tracked in [#6302](https://github.com/Luminous-Dynamics/mycelix/issues/6302); this crate supplies one independent finite oracle for the relation-inclusion part, not a replacement for ROS-005.

## Implemented semantics

- Exact tuple membership across action, resource, direction, audience, purpose, data class, proposal/execute mode and operation binding.
- Non-widening and proper-subset attenuation.
- Distinct intersection for authority bundles that are all required and union for independent alternative grants.
- Deterministic versioned canonical bytes with an explicit domain separator and sorted tuple ordering.
- Fail-closed rejection of unsupported schema versions, duplicate tuples and malformed/oversized atoms.
- A bounded exhaustive 256 × 256 test over all pairs of subsets of an eight-tuple universe.
- A constructive adversarial control where every individual projection matches but the recombined tuple is rejected.

## Verification

The dedicated workflow runs, against this standalone manifest:

1. pinned cargo fmt check;
2. cargo test --all-targets;
3. strict cargo clippy --all-targets -- -D warnings.

The authoring environment does not have a Rust toolchain. No local compilation or benchmark result is claimed.

## Explicit boundaries / non-claims

This model does not implement or qualify:

- principal authentication, issuer signatures, parent record retrieval, ROS-005 authority/currentness, Holochain capability grants or DHT convergence;
- trusted time, authority epochs, revocation events or stale-parent propagation;
- durable/atomic budgets shared across sibling delegations, including rolling-window rate limits;
- canonical operation digest cryptography or one-shot consume-once execution;
- exact operating-system/provider effect commits or independent outcome observation;
- a complete route inventory or non-bypassability proof.

The operation_binding field is an opaque, caller-supplied canonical identifier in this reference model. It is not authenticated, signed or independently recomputed here. Actual request binding and exact operation digest verification remain separate owner obligations.

For Xenia, the current eight M1 permission/action/direction mappings in [draft PR #509](https://github.com/Luminous-Dynamics/xenia-peer/pull/509) are statically paired. This relational model is a prospective schema constraint for future purpose/audience/resource/budget extensions; it does not mean Xenia currently consumes ROS-005 or this crate.

## Owner composition

- ROS-005 semantic delegation: [Mycelix #3659](https://github.com/Luminous-Dynamics/mycelix/issues/3659)
- Addressable parent chains and content-bound identity: [#3677](https://github.com/Luminous-Dynamics/mycelix/issues/3677) / [PR #3679](https://github.com/Luminous-Dynamics/mycelix/pull/3679)
- Typed scope extensions: [#6302](https://github.com/Luminous-Dynamics/mycelix/issues/6302)
- Relational containment theorem: [#4852](https://github.com/Luminous-Dynamics/mycelix/issues/4852)
- Xenia ROS-005 ↔ M1 adapter contract: [Xenia #517](https://github.com/Luminous-Dynamics/xenia-peer/issues/517)
- One-shot effect-boundary enforcement and concurrency remain separate: Mycelix [#6189](https://github.com/Luminous-Dynamics/mycelix/issues/6189) / [#6193](https://github.com/Luminous-Dynamics/mycelix/issues/6193), Xenia [#504](https://github.com/Luminous-Dynamics/xenia-peer/issues/504).
