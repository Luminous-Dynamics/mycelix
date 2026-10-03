# Mobility Qualification Adapter Boundary V1

## Purpose

This document defines the boundary between the runtime-neutral mobility qualification crate and a protocol-specific validation runtime.

The pure qualification layer reasons over explicit engineering identities and deterministic qualification state. It does not depend on Holochain types, hashes, host functions, timestamps, or DHT behavior.

## Three-outcome contract

The pure layer exposes `QualificationDecision<T>`:

- `Valid(T)`: qualification completed for the supplied dependency set.
- `Invalid { reason }`: the supplied records contain a definitive structural contradiction.
- `Unresolved { missing, partial }`: qualification cannot complete because one or more explicitly addressable logical dependencies are unavailable.

These are semantic outcomes, not transport errors.

The intended Holochain mapping is:

| Pure decision | Holochain validation result |
| --- | --- |
| `Valid(T)` | `ValidateCallbackResult::Valid` |
| `Invalid { reason }` | `ValidateCallbackResult::Invalid(reason)` |
| `Unresolved { missing, partial }` | `ValidateCallbackResult::UnresolvedDependencies(...)` |

A host/runtime failure remains an `ExternResult` error rather than a semantic `Invalid` result.

## Identity boundary

`IdentityRef` is an engineering identity consisting of namespace, identity kind, and identifier.

It is not a Holochain `ActionHash`, `EntryHash`, or other protocol address.

An adapter therefore MUST NOT:

- reinterpret an `IdentityRef.id` string as a Holochain hash;
- manufacture a protocol address from an engineering identity;
- use a mutable link collection as a substitute for an addressable dependency;
- treat an unavailable engineering identity as evidence that the underlying record is false or invalid.

An adapter MUST establish an explicit deterministic binding from each unresolved logical identity to exactly one protocol address used for dependency retrieval. The pure layer provides `QualificationDependencyBindingSet<A>` for this purpose without fixing the runtime address type.

The binding set is intentionally append-only: a logical identity cannot be rebound to a second address within the same binding set. Iteration is canonicalized by logical identity, so construction order cannot affect downstream dependency resolution. An unbound logical identity remains unresolved; it must not be converted into a negative semantic finding.

### Binding resolution contract

The pure layer exposes `QualificationDependencyBindingSet::resolve_required`, using the same three-outcome decision algebra as the adapter boundary:

- every requested identity is valid and bound: return `Valid` with bound logical-identity/address pairs in canonical identity order;
- a requested identity is structurally malformed: return definitive `Invalid`;
- one or more valid identities are unbound: return `Unresolved` with canonical missing logical identities and any already-bound partial bindings.

Duplicate requests for the same logical identity are deduplicated during resolution. The API does not synthesize, guess, hash, or otherwise derive protocol addresses. A runtime adapter must map the returned opaque addresses to the concrete dependency retrieval API and preserve the unresolved state when a required addressable dependency is unavailable.

## Dependency retrieval boundary

For Holochain, dependency retrieval SHOULD use the deterministic `must_get_*` host-function model.

The runtime adapter owns:

1. resolving each logical `IdentityRef` to its addressable protocol representation;
2. retrieving the referenced record/action through deterministic host functions;
3. converting a missing protocol dependency into Holochain's unresolved outcome;
4. feeding retrieved records back into the pure qualification layer.

The pure layer owns:

1. structural validation;
2. bounded graph traversal;
3. dependency identity calculation;
4. canonical missing-dependency ordering;
5. qualification-state composition;
6. structural-error precedence.

## Determinism requirements

For the same supplied logical records and dependency bindings, the adapter MUST produce the same semantic outcome regardless of:

- dependency retrieval order;
- serialization order;
- batching of independent dependency paths;
- peer identity;
- wall-clock time;
- mutable DHT link state.

The adapter MUST NOT turn a missing dependency into a negative semantic finding merely because that dependency was unavailable at validation time.

## Compatibility boundary

The current `mycelix-commons` workspace is intentionally Holochain-0.6-compatible. This pure adapter contract therefore does not pin `hdk`, `hdi`, or `holochain_zome_types` versions.

The runtime-specific implementation should be isolated so that a Holochain version migration changes only the adapter layer and its integration tests, rather than the deterministic mobility qualification algebra.

## Non-goals

This contract does not establish:

- real-world authority;
- legal or regulatory validity;
- cryptographic immutability;
- global DHT completeness;
- truth of an engineering claim;
- causal correctness.

It defines only the deterministic boundary between logical qualification semantics and a distributed validation runtime.
