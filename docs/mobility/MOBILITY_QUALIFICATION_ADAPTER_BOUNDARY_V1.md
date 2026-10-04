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

The binding set is intentionally append-only: a logical identity cannot be rebound to a second address within the same binding set. Every binding also carries an explicit runtime-neutral retrieval intent (`ValidRecord`, `Action`, or `Entry`). The runtime adapter must map that intent to a compatible address type and host function; the pure layer does not treat protocol address kinds as interchangeable. Iteration is canonicalized by logical identity, so construction order cannot affect downstream dependency resolution. An unbound logical identity remains unresolved; it must not be converted into a negative semantic finding.

Conflict precedence is deterministic as well as append-only: candidate-local address/retrieval typing is checked before registry state, and an existing logical-identity binding is the primary immutable conflict before secondary provenance-witness reuse. This mirrors the pure `QualificationDependencyBindingSet` semantics and prevents equivalent conflicts from being classified differently by runtime adapters.  When multiple immutable conflicts apply simultaneously, the adapter uses the primary-key conflict first: duplicate logical identity precedes duplicate provenance witness for runtime bindings, and duplicate authority precedes duplicate provenance witness for authority credentials. `EDT-194` and `EDT-195` pin these deterministic conflict partitions.

### Binding resolution contract

The pure layer exposes `QualificationDependencyBindingSet::resolve_required`, using the same three-outcome decision algebra as the adapter boundary:

- every requested identity is valid and bound: return `Valid` with bound logical-identity/address pairs in canonical identity order;
- a requested identity is structurally malformed: return definitive `Invalid`;
- one or more valid identities are unbound: return `Unresolved` with canonical missing logical identities and any already-bound partial bindings.

Duplicate requests for the same logical identity are deduplicated during resolution. The API does not synthesize, guess, hash, or otherwise derive protocol addresses. A runtime adapter must map the returned opaque addresses to the concrete dependency retrieval API and preserve the unresolved state when a required addressable dependency is unavailable.

Before host retrieval, `retrieve_resolved` canonicalizes any caller-supplied dependency vector by `ResolvedHolochainDependency.identity`. This preserves deterministic host-call ordering even when a caller does not pass the exact ordering produced by `resolve_required`. The canonicalization is mechanical and does not alter the logical bindings or provenance.

## Dependency retrieval boundary

For Holochain, dependency retrieval SHOULD use the deterministic `must_get_*` host-function model.

The runtime adapter owns:

1. resolving each logical `IdentityRef` to its addressable protocol representation;
2. preserving the protocol address *kind* required by the selected retrieval primitive;
3. retrieving the referenced record/action through deterministic host functions;
4. converting a missing protocol dependency into Holochain's unresolved outcome;
5. feeding retrieved records back into the pure qualification layer.

The binding MUST preserve address-kind compatibility. For example, Holochain's `must_get_valid_record` requires an `ActionHash`; an `EntryHash` or another address type must not be silently substituted. A binding that cannot satisfy the selected retrieval primitive is an adapter-boundary error, not an unresolved DHT result.

The concrete adapter tests also verify the exact protocol address handed to the selected `must_get_*` host function, rather than checking only the function name. The runtime retrieval path therefore has executable coverage for the full dispatch tuple: logical binding → concrete address kind → selected host primitive → exact bound address. Canonical retrieval ordering is likewise exercised at the host boundary, so a caller-provided reversed vector still reaches the host with the first logical dependency in canonical order before any host failure short-circuits retrieval.

`ValidRecord` has an intentionally stronger but narrower meaning than simple existence: it requests an action-addressed record that the visible validation authorities report as valid, enabling inductive validation. It must not be interpreted as proof that every later operation associated with that record is valid. `Action` and `Entry` retrieval likewise provide the referenced object without independently asserting its full validation history.

There is a second important unresolved case: `must_get_valid_record` can also terminate validation as unresolved when the referenced record is found but is reported invalid by upstream validators. Therefore protocol-level unresolved does not mean only transport absence; it means the adapter cannot yet establish the dependency as safely retrievable for the selected deterministic validation primitive. The pure semantic algebra remains separate from that host-level state.

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


## Binding provenance boundary

A protocol address must never become the sole justification for a logical qualification dependency. The concrete adapter therefore requires a separate `QualificationDependencyBindingProvenance` witness for every logical-to-protocol binding.

The pure witness contains:

- a distinct `ReconciliationWitness` identity;
- the exact `IdentityRef` being bound;
- a distinct authority `IdentityRef`;
- a unique basis set which includes that exact authority.

The witness does not contain, derive, parse, hash, or otherwise interpret the Holochain address. This keeps the provenance explanation in the pure qualification layer and the protocol address in the runtime adapter.

The adapter requires the witness to name the exact logical identity being bound. Malformed provenance is semantic invalidity; duplicate binding state or a protocol address-kind mismatch is an adapter boundary failure.

The witness is carried through resolved dependencies so downstream consumers retain the provenance context. This is intentionally a bounded structural provenance statement, not a cryptographic proof, legal authority determination, global DHT completeness claim, or assertion that the engineering claim itself is true.


## Optional cryptographic attestation

The runtime boundary may optionally carry a signed binding attestation whose canonical payload contains the schema identifier, pure binding-provenance witness, concrete protocol address, and retrieval intent.

The adapter verifies that payload with Holochain's deterministic Ed25519 `verify_signature` host function. This optional layer therefore provides a protocol-level authorship check over the exact binding statement while leaving semantic meaning in the pure layer.

The boundary is deliberately split:

- invalid attestation payload or non-matching signature → semantic `Invalid`;
- actual host failure while performing signature verification → `ExternResult::Err`;
- a valid signature does not implicitly resolve the relationship between the Holochain signer key and the domain authority identity named by the provenance witness.

The signed layer is optional because applications may have an independent authoritative mechanism for establishing the logical-to-protocol binding. It must never become an excuse to infer authority from a bare protocol signature.

The signed payload types also derive Holochain's `SerializedBytes` representation using `holochain_serialized_bytes = 0.0.57`. Their schema identifiers are owned strings rather than borrowed static references, allowing explicit `SerializedBytes` round-trip tests to define the wire representation. This does not introduce a second signing mechanism: `verify_signature` remains the single cryptographic verification path. Holochain's serialization project recommends `SerializedBytes` for canonical typed representations shared across systems.


## Signed wire-schema evolution boundary

The signed payloads are deliberately strict `v1` wire types. Their schema identifiers are part of the signed payload, and both payload types reject unknown fields during canonical deserialization.

This means a compatible change must preserve the exact `v1` schema identifier and field set. A breaking field addition, removal, rename, or type change requires a new schema identifier and a new payload contract rather than relying on silent forward/backward deserialization. The existing validators pin this policy, and negative `SerializedBytes` round-trip tests demonstrate rejection of unexpected fields for both signed payload types.

This is intentionally stricter than generic data-transfer compatibility: the payload participates in cryptographic attestation, so an unnoticed wire-schema reinterpretation must not preserve the appearance of a valid signature while changing the meaning of the signed statement.

## Authority-agent identity boundary

The protocol signer is not treated as the domain authority merely because a valid signature exists.

The adapter therefore uses an explicit `HolochainAuthorityAgentBindingSet`. Each accepted credential binds one domain `IdentityRef` to exactly one `AgentPubKey`. The credential:

- uses `QualificationAuthorityAgentBindingProvenance` to preserve the domain authority chain;
- is signed by the exact agent key it names;
- is admitted into an immutable authority→agent registry;
- cannot be duplicated for the same authority identity;
- cannot reuse a provenance witness already admitted for another authority-agent claim;
- a runtime binding cannot reuse a provenance witness already admitted by any authority-agent credential in the registry;
- must be present before a signed runtime dependency binding can be accepted.

A runtime dependency binding is accepted only when its signer matches the registered agent for the provenance authority. The registry retains the full verified authority-agent credential, not just the AgentPubKey, so downstream consumers can audit the exact signed provenance statement that established the mapping.

A missing authority→agent credential is a **preflight-unresolved** condition. It is not yet a Holochain `UnresolvedDependencies` result because no concrete protocol dependency address has been admitted. The adapter must stop before DHT retrieval in that state. Deterministic payload and signature checks may already have run; those checks are not DHT dependency retrieval. Only after an explicit address binding exists can Holochain's host-level unresolved-dependency machinery participate.

Structural validation of the runtime attestation payload takes precedence over that preflight lookup. A malformed schema, provenance witness, or address/retrieval pairing is a definitive adapter-side semantic invalidity even when the referenced authority has no registered agent credential. Deterministic signature verification also takes precedence over the same lookup: an invalid signature remains semantic `Invalid`, while a signature-verification host failure remains an `ExternResult::Err`. This preserves definitive-error precedence and prevents dependency absence from masking a contradiction already visible in the supplied attestation.

The runtime binding must also carry the exact same authority_scope and authority_delegation identities named by the retained authority-agent credential. Matching only the authority identity is insufficient because it would allow a valid agent key to be detached from the particular provenance scope and delegation chain that justified its admission. These continuity checks are structural; they do not independently prove legal authority, institutional entitlement, or real-world identity.

The retained credential's entire `basis` is also a minimum provenance set for the runtime binding: the runtime binding may add basis witnesses, but it may not silently drop one carried by the authority credential. The runtime binding uses a distinct provenance witness identity from the authority credential, preventing one witness record from being reused as evidence for two separate claims. `EDT-163`–`EDT-165` pin basis-loss rejection, complete-basis acceptance, and witness-role separation. The precedence invariants are pinned separately by `EDT-179` for payload structure, `EDT-183` for runtime signature validity, `EDT-184` for runtime signature-host failure, `EDT-185` for authority credential payload validation, and `EDT-186` for authority issuer/agent mismatch.

### Authority credential admission precedence

Credential admission is split into semantic verification and registry-state admission. The registry MUST evaluate the credential's own definitive validity before consulting duplicate-authority or duplicate-witness conflicts.

The order is:

1. signed payload structural validation;
2. issuer/agent identity match;
3. cryptographic signature verification;
4. immutable registry admission checks for duplicate authority and duplicate provenance witness;
5. state insertion.

A malformed credential or issuer mismatch therefore remains semantic invalidity even when it targets an authority that is already registered. A signature-invalid credential likewise remains semantic invalidity after deterministic signature verification. Only a fully verified credential can produce a duplicate-authority or duplicate-witness adapter rejection.

Rejected admissions are state-preserving: the existing verified credential remains the sole registry entry for the conflicting authority or witness. Corpus vectors EDT-187–EDT-191 pin both precedence and non-mutation.

A failure of the deterministic signature-verification host function is different again: it remains an `ExternResult::Err` and must occur before duplicate registry admission is considered. This prevents a runtime failure from being reclassified as a local duplicate conflict. `EDT-192` pins that precedence and state preservation.

The authority-agent credential itself follows the same local-definitive-before-crypto rule used by runtime attestations: validate the signed payload first, then require the credential issuer to equal the exact agent key named by the payload, and only then call `verify_signature`. A malformed credential payload or issuer mismatch therefore cannot be masked by, or cause, signature-verification host activity. This ordering is a semantic safety boundary as well as an efficiency boundary: Holochain permits deterministic Ed25519 verification during validation, but definitive invalidity should remain distinct from runtime failures and unavailable DHT dependencies.

This establishes a protocol-level identity binding, not a claim about a real-world person's or institution's legal identity. The semantic authority determination remains in the pure provenance graph.
