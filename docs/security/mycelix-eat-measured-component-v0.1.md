# Mycelix EAT Measured Component Profile v0.1

**Status:** executable reference-model representation  
**Claim ceiling:** `ReferenceModelOnly`

This artifact defines the semantic mapping boundary between the TPM/platform measurement adapter and a future EAT Evidence representation.

RFC 9711 defines EAT as a RATS-aligned evidence format. RFC 10013 defines the measured-component data model for sampled component state, including firmware, early-boot software, filesystem objects, and other measurable state.

## Scope

This profile is **not** a complete EAT or COSE implementation.

It qualifies only the semantic shape required for a measured component before an enclosing EAT is constructed.

## Profile decisions

The v0.1 profile uses:

- JSON fixture representation;
- SHA-256 measured-component digests;
- component identity as required;
- measured digest as required;
- `authorities` disabled;
- `flags` disabled.

Those fields remain disabled until an exact Mycelix EAT profile defines their semantics.

## Authority separation

The component authority, if one is ever used, remains distinct from the signer of the enclosing EAT Evidence.

```
component authority
!= enclosing EAT signer
```

and:

```
component digest
!= policy approval
!= workload authorization
```

## TPM binding

For the #3983 vTPM fixture, the measured component is linked to:

```
measurement_profile_id
PCR bank = SHA-256
PCR index = 16
binding type = quoted-pcr-reference
```

That reference is evidence lineage, not proof that PCR 16 has PC-client measured-boot meaning.

## Qualification corpus

Sixteen deterministic vectors cover:

- component-name substitution;
- digest substitution;
- digest-algorithm substitution;
- malformed base64url;
- wrong digest length;
- unknown component profile;
- serialization-profile substitution;
- unauthorized authorities/flags;
- authority/signer confusion;
- PCR binding substitution;
- security-domain/profile substitution;
- duplicate identity conflict;
- raw measurement without profile;
- JSON key-order invariance.

## Boundary

The mapping remains:

```
TPM/platform Evidence
  -> measured-component representation
  -> future EAT Evidence
  -> RATS Verifier
```

No step above is itself authorization.

## Qualification ceiling

A green semantic run establishes only the selected measured-component representation.

It does not establish EAT/COSE cryptographic verification, TPM correctness, measured boot, firmware trust, RATS verifier trust, or local authorization.
