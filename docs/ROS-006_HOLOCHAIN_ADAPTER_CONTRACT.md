# ROS-006 Holochain adapter contract

Status: design/hardening artifact for PR #3703  
Scope: Holochain-host integration for explicit qualification dependencies  
Authority boundary: the adapter retrieves and validates dependencies; core qualification remains Holochain-version-neutral.

## 1. Required dependency flow

For every declared QualificationDependency:

1. The dependency contains a stable logical ID and an exact opaque record address.
2. The adapter converts that address into the host-specific Holochain hash type.
3. Validation retrieves only addressable content through a deterministic must_get_* operation.
4. Missing/unavailable content maps to Unresolved, never Invalid.
5. Retrieved content is decoded into the expected domain type.
6. The adapter verifies that the retrieved content's canonical identity equals the declared logical ID and that the fetched address is the declared address.
7. Malformed content or identity/address mismatch maps to Invalid.
8. Only an independently verified dependency produces an Attested observation.
9. The resulting dependency manifest remains ordered and is bound into the qualification certificate and Relationship 360 projection.

The adapter must not consult wall-clock time, mutable metadata, caller identity, link state, current application state, or network-dependent queries whose answer can vary between validators.

## 2. Holochain 0.7 host mapping

The production target is the Holochain 0.7 compatibility generation: Holochain 0.7.0, HDK 0.7.0, and HDI 0.8.0. The adapter must isolate host-version details from core-types.

Holochain validation provides deterministic must_get_* host functions, including must_get_action, must_get_entry, and must_get_valid_record. A dependency that cannot be retrieved must remain an unresolved dependency rather than being silently converted into a validation failure.

For dependencies where the action/record itself must already be validated, prefer must_get_valid_record. The adapter must still perform application-level type, logical-identity, lifecycle, and exact-address checks after retrieval.

Important limitation: must_get_valid_record establishes validity for the CreateRecord operation; it is not a substitute for ROS domain identity or lifecycle validation of later Update/Delete history.

The 0.6 → 0.7 migration changes the Action representation: Action now contains shared header data plus ActionData, and TypedAction is used by several operation types. This difference must remain entirely inside the concrete adapter/host layer; core qualification must not depend on Holochain action enums or version-specific re-exports.

## 3. Repository migration boundary

The repository currently contains active 0.6-era Holochain dependency declarations. The target is to normalize active Holochain workspaces onto one coherent 0.7 compatibility generation rather than introducing a special 0.7 dependency island for ROS-006.

Do not mechanically edit version strings without regenerating lockfiles and migrating the associated 0.6 API surface. Holochain's official upgrade guidance identifies action-model, transport, conductor, and test-harness changes, and integrity-zome dependency changes alter DNA compatibility.

ROS-006 therefore remains Holochain-version-neutral until the host workspace itself has been migrated and verified.

## 4. Adapter result contract

The semantic mapping is intentionally strict:

| Host observation | Core result |
| --- | --- |
| exact address retrieved, expected type decoded, logical identity verified | Attested observation |
| address unavailable / dependency not yet retrievable | Unresolved |
| malformed bytes, wrong entry/action type, logical ID mismatch, address mismatch, or explicit validation failure | Invalid |

`QualificationDependencyResolution::Valid` is retained only as a legacy compatibility enum variant and is deliberately rejected by core qualification. A host adapter must return `Attested { observed: ... }`, not bare `Valid`.

Never map an identity mismatch to Unresolved: the dependency was retrieved, so the failure is definitive.

Never map an unavailable dependency to Invalid: that would make temporary DHT incompleteness indistinguishable from corrupt or malicious data.

## 5. Stronger attestation requirement

The resolver seam is intentionally narrow, but a bare success result cannot by itself prove that an adapter actually validated the exact requested identity and address.

The production adapter contract therefore returns an observation containing:

- dependency kind;
- observed logical ID;
- exact retrieved record address.

Core ROS compares all three fields with the requested dependency before accepting the dependency as qualified.

This makes the security property enforceable at the version-neutral boundary rather than relying entirely on adapter discipline.

The Holochain adapter must construct the observation from the record/hash it actually retrieved. It must not construct it by copying the requested dependency before retrieval.

## 6. Determinism requirements

For a fixed validation context, adapter resolution must be observationally pure:

- same dependency address -> same host retrieval target;
- same retrieved bytes -> same decoded type;
- same decoded content -> same logical identity;
- no current-time input;
- no caller/agent identity input;
- no mutable metadata lookup;
- no link traversal as an authority-bearing dependency;
- no local cache whose freshness changes the semantic result.

Caching is allowed only when it is semantically transparent: a cache hit and a fresh retrieval must produce the same result.

## 7. Required adapter test matrix

The eventual concrete adapter should have host-level tests for at least:

- valid dependency resolves repeatedly to the same result;
- missing dependency produces unresolved semantics;
- malformed record produces invalid;
- wrong record/entry type produces invalid;
- logical ID mismatch produces invalid;
- exact record-address mismatch produces invalid;
- dependency-kind mismatch produces invalid;
- same logical ID with different dependency kind is rejected by core;
- same logical ID with different record address is rejected by core;
- reordered dependency declarations produce the same canonical manifest;
- resolver output cannot depend on wall-clock time;
- resolver output cannot depend on caller identity;
- resolver output cannot depend on mutable link/current-state metadata;
- a retrieved dependency that is valid as a CreateRecord but fails ROS domain identity/lifecycle checks is still invalid for ROS-006;
- an adapter cannot return Attested while changing kind, logical ID, or exact record address in its observed result;
- a bare Valid result is rejected as unattested;
- repeated qualification with the same pinned inputs produces the same certificate digest and projection bytes.

## 8. Integration placement

Do not attach this adapter to an unrelated existing DNA merely because that DNA already has Holochain infrastructure.

The current public Mycelix topology contains many zomes, including commons and civic zomes, but no Relationship-360-specific integrity/coordinator zome. The adapter should therefore land beside the eventual Relationship 360 DNA/integrity zome once that host boundary exists.

The core-types crate should remain Holochain-version-neutral. The adapter should be a thin translation layer:

Holochain hash/record -> deterministic typed dependency observation -> ROS-006 resolver result -> core qualification.

## 9. Security invariant

A successful qualification certificate proves only that the declared dependency set was deterministically resolved and that the core qualification rules accepted the supplied records.

It is not a bearer capability, does not grant authority, and must not be interpreted as current authorization outside the explicitly validated input set.

## 10. Recommended sequencing

1. Normalize active Holochain workspaces to the official 0.7 compatibility generation.
2. Regenerate and commit workspace lockfiles from the 0.7 environment.
3. Migrate 0.6 API usage using compiler diagnostics and Holochain's upgrade guide.
4. Define the Relationship 360 integrity/coordinator boundary.
5. Implement the thin adapter with observed identity/address binding; the core attestation check is already present.
6. Run the adversarial matrix at the adapter boundary.
7. Only then allow Relationship 360 projection to consume qualified inputs.

## References

- Holochain 0.7 compatibility: https://developer.holochain.org/resources/compatibility/holochain-0.7/
- Holochain 0.6 -> 0.7 upgrade notes: https://developer.holochain.org/resources/upgrade/upgrade-holochain-0.7/
- Holochain validation: https://developer.holochain.org/build/validation/
- Holochain must_get host functions: https://developer.holochain.org/build/must-get-host-functions/
- Holochain validate callback: https://developer.holochain.org/build/validate-callback/
