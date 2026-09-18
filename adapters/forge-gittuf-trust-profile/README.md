# mycelix-forge-gittuf-trust-profile

FORGE-004D2C1 defines the first hermetic verifier trust profile for Mycelix Forge.

The profile is intentionally narrow: it permits only gittuf v0.16.0 verification paths whose public-key material is carried directly in repository policy metadata and verified in-process.

## Pinned gittuf semantics

- version: `v0.16.0`
- annotated tag object: `5d4d7652bf84e347eefbad1a7e07fc88dede9b92`
- release commit: `fa3c295e1e46c1cff10aec1194edc29f19677723`

The v0.16.0 tag is a GitHub-verified PGP-signed annotated tag.

## Allowed methods

`LocalEmbeddedKeysV1` permits only:

- SSH
- GPG

Both v0.16.0 verifier paths parse public-key material from gittuf metadata and verify signatures in-process. The profile excludes Sigstore because its trust-root / transparency / TUF dependencies are not yet fully injectable as explicit local inputs through gittuf's current verifier API.

## Complete policy inventory

`PolicyTrustInventory` binds an exact repository-policy-state digest to the complete set of verification methods found in that policy lineage. Qualification succeeds only when the inventory is non-empty and every method is permitted by the local-key profile.

A single Sigstore method invalidates this profile. Unknown serialized methods fail deserialization rather than being treated as local keys.

## Runtime requirements

The profile commitment additionally requires:

- network denied by the execution/isolation layer;
- no ambient user keyring as a verification authority;
- repository policy metadata as the public-key authority;
- exact gittuf v0.16.0 semantics.

Those requirements are composition constraints: this crate does not itself scan policy metadata or enforce a network namespace.

## Claim boundary

`QualifiedLocalKeyTrustProfile` proves that one complete reported policy-method inventory is compatible with the pinned local-key verifier profile. FORGE-004D2C2 must implement the policy inventory collector. Final repository `OfflineEvidence` still requires the exact execution spec, runtime isolation evidence, runtime NAR closure evidence, and verifier result.
