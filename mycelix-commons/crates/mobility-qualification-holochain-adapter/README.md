# Mobility Qualification Holochain Adapter

This crate is the concrete runtime boundary for Mobility qualification under the currently recommended Holochain 0.7 line.

It pins hdi 0.8.0 and remains an isolated Cargo workspace so the existing Holochain-0.6-compatible mycelix-commons workspace is not silently upgraded.

The adapter adds no semantic qualification rules. It only:

1. binds a validated logical IdentityRef to an explicit Holochain ActionHash or EntryHash;
2. rejects an address-kind mismatch before any host call;
3. preserves the pure QualificationDecision algebra;
4. dispatches ValidRecord to must_get_valid_record(ActionHash);
5. dispatches Action to must_get_action(ActionHash);
6. dispatches Entry to must_get_entry(EntryHash);
7. propagates host retrieval failures unchanged so a real validate callback can retain Holochain UnresolvedDependencies behavior.

The two-phase rule is intentional.

Preflight: logical identities must first become explicit runtime bindings. An unbound logical identity is still the pure layer's Unresolved state and has no protocol address to place into a Holochain unresolved-dependency set.

Validation: once bindings are concrete, the adapter invokes only the matching deterministic must_get_* primitive. A missing addressable dependency is then handled by Holochain itself as UnresolvedDependencies.

The crate contains no identity-string parsing, hashing, address derivation, link enumeration, wall-clock use, or peer-specific behavior.

## Intended validation integration

An integrity-zome validate callback should:

- perform the pure qualification first;
- obtain the exact logical dependency set produced by that qualification;
- build or look up the immutable binding set for the supplied records;
- require the preflight decision to be QualificationDecision::Valid(...);
- call retrieve_resolved(...);
- feed retrieved records back into the pure qualification layer;
- return the pure result as Valid or Invalid;
- allow must_get_* failures to propagate without translating them into semantic Invalid.

The adapter deliberately does not turn an unbound IdentityRef into ValidateCallbackResult::UnresolvedDependencies. No correct Holochain unresolved-dependency value can be built until the logical identity has an addressable protocol binding.

This separation keeps the semantic algebra in one place and the protocol mechanics in one place, rather than creating a second semantic interpretation layer.


## Callback semantic boundary

The adapter exposes one callback-facing mapping seam. Pure `Valid` becomes `ValidateCallbackResult::Valid`; pure structural `Invalid` becomes `ValidateCallbackResult::Invalid`. A malformed logical identity discovered defensively during binding is the same semantic invalidity and follows the same `Invalid` path. A pure `Unresolved` cannot be converted yet because its missing values are logical identities rather than Holochain hashes. True adapter defects such as duplicate bindings or address-kind mismatches remain boundary errors rather than becoming facts about the underlying record.

This keeps semantic interpretation single-sourced: the pure qualification layer decides the meaning, while this crate only performs typed protocol binding, deterministic retrieval, and the final protocol-result mapping.


## Binding provenance witness

Every concrete runtime binding requires a `QualificationDependencyBindingProvenance` witness.

The witness names:

- the exact logical dependency being bound;
- a distinct provenance-witness identity;
- the exact authority identity supporting the binding;
- a unique basis set containing that authority.

The witness intentionally contains no Holochain address. It explains the provenance of the logical binding while the adapter separately carries the concrete `ActionHash` or `EntryHash`.

This is a bounded structural provenance statement. It does not claim cryptographic proof, legal authority, global DHT completeness, or correctness of the underlying engineering claim.

A binding is therefore accepted only after both halves are independently valid:

**provenance witness** + **typed protocol address**

The resolved dependency carries the same provenance witness forward, so downstream validation does not lose the explanation for why the logical dependency was bound.


## Optional signed attestation

The adapter also supports an optional signed binding attestation. The attestation payload canonically serializes:

- the schema identifier;
- the complete pure binding-provenance witness;
- the concrete Holochain address;
- the retrieval kind.

Holochain 0.8 provides deterministic `verify_signature` over serializable values, so the adapter can verify this payload before accepting an attested binding. A failed signature is semantic invalidity; an actual host failure from signature verification remains an `ExternResult::Err`.

The signature establishes authorship of the exact binding statement by the supplied Holochain agent key. It does not by itself establish that the signer corresponds to the domain authority identity named inside the provenance witness. That relationship remains a separate provenance/authority question and must not be inferred by the adapter.
