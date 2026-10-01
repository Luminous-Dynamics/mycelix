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
8. Only an independently verified dependency maps to Valid.
9. The resulting dependency manifest remains ordered and is bound into the qualification certificate and Relationship 360 projection.

The adapter must not consult wall-clock time, mutable metadata, caller identity, link state, current application state, or network-dependent queries whose answer can vary between validators.

## 2. Holochain host mapping

Holochain validation explicitly requires deterministic validation for a given operation. The supported deterministic dependency retrieval functions include must_get_action, must_get_entry, and must_get_valid_record. A dependency that cannot be retrieved causes validation to terminate with an unresolved-dependency result rather than becoming a validation failure.

For dependencies where the action/record itself must already be validated, prefer must_get_valid_record. The adapter must still perform application-level type and identity checks after retrieval.

Important limitation: Holochain documents that must_get_valid_record establishes validity for the CreateRecord operation, not necessarily every later Update/Delete operation associated with the same underlying data. ROS-006 therefore must not treat "valid record" as a substitute for domain identity or lifecycle validation.

Holochain 0.6 is maintenance-mode. Its current compatibility table specifies HDK 0.6.3 and HDI 0.7.3. Holochain 0.7 changes the action model and uses TypedAction<...>; the adapter must isolate these differences from core-types.

Repository pin audit: mycelix-core-types currently declares optional HDI 0.7.1, holo_hash 0.6.1, holochain_integrity_types 0.6.1, and hdk_derive 0.6.1. These pins do not exactly match the current Holochain 0.6.3 compatibility table. Do not describe the repository as being on the current 0.6 compatibility set until the workspace pins are deliberately reconciled. This mismatch is an integration risk worth resolving before the concrete Relationship 360 adapter is implemented.

## 3. Adapter result contract

The semantic mapping is intentionally strict:

| Host observation | Core result |
| --- | --- |
| exact address retrieved, expected type decoded, logical identity verified | Valid |
| address unavailable / dependency not yet retrievable | Unresolved |
| malformed bytes, wrong entry/action type, logical ID mismatch, address mismatch, or explicit validation failure | Invalid |

Never map an identity mismatch to Unresolved: the dependency was retrieved, so the failure is definitive.

Never map an unavailable dependency to Invalid: that would make temporary DHT incompleteness indistinguishable from corrupt or malicious data.

## 4. Stronger attestation requirement

The three-state resolver is intentionally a narrow seam, but a bare Valid result cannot by itself prove that an adapter actually validated the exact requested identity and address.

The preferred production adapter contract should therefore return an observation containing:

- dependency kind;
- observed logical ID;
- exact retrieved record address.

Core ROS should compare all three fields with the requested dependency before accepting Valid.

This makes the security property enforceable at the version-neutral boundary rather than relying entirely on adapter discipline.

The Holochain adapter should construct the observation from the record/hash it actually retrieved. It must not construct it by copying the requested dependency before retrieval.

## 5. Determinism requirements

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

## 6. Required adapter test matrix

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
- a retrieved dependency that is valid as a CreateRecord but fails the ROS domain identity/lifecycle checks is still invalid for ROS-006;
- an adapter cannot return Attested while changing kind, logical ID, or exact record address in its observed result;
- a bare Valid result is rejected as unattested;
- repeated qualification with the same pinned inputs produces the same certificate digest and projection bytes.

## 7. Integration placement

Do not attach this adapter to an unrelated existing DNA merely because that DNA already has Holochain infrastructure.

The current public Mycelix topology contains many zomes, including commons and civic zomes, but no Relationship-360-specific integrity/coordinator zome. The adapter should therefore land beside the eventual Relationship 360 DNA/integrity zome once that host boundary exists.

The core-types crate should remain Holochain-version-neutral. The adapter should be a thin translation layer:

Holochain hash/record -> deterministic typed dependency observation -> ROS-006 resolver result -> core qualification.

## 8. Security invariant

A successful qualification certificate proves only that the declared dependency set was deterministically resolved and that the core qualification rules accepted the supplied records.

It is not a bearer capability, does not grant authority, and must not be interpreted as current authorization outside the explicitly validated input set.

## 9. Recommended sequencing

1. Reconcile and explicitly document the Holochain dependency matrix used by the workspace.
2. Define the Relationship 360 integrity/coordinator boundary.
3. Implement the thin adapter with observed identity/address binding; the core attestation check is already present.
4. Run the adversarial matrix at the adapter boundary.
5. Only then allow Relationship 360 projection to consume qualified inputs.

## References

- Holochain validation: https://developer.holochain.org/build/validation/
- Holochain must_get host functions: https://developer.holochain.org/build/must-get-host-functions/
- Holochain validate callback: https://developer.holochain.org/build/validate-callback/
- Holochain 0.6 compatibility: https://developer.holochain.org/resources/compatibility/holochain-0.6/
- Holochain 0.6 -> 0.7 upgrade notes: https://developer.holochain.org/resources/upgrade/upgrade-holochain-0.7/
