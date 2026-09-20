# PSI-002B2B r2Q — runner-neutral exact-source qualifier

Status: **PREPARED / NOT EXECUTED / NOT A PASS**

Exact product subject:

```text
3193bcf7b92feed1f0599dca47ecafbfa178404f
```

This qualifier replaces superseded #2353 and binds the corrected provider-provenance-only theorem.

## Offline gate

```text
cargo fmt --check --all
cargo test --offline --all-targets
cargo clippy --offline --all-targets --all-features -- -D warnings
```

## Static authority checks

The qualifier requires:

- exact registry provider-attestation and verifier-identity domains;
- exact authenticity-only claim marker;
- exact B2A structural rebind;
- independently trusted Ed25519 + ML-DSA-65 verification;
- `PolicyTrustedXeniaRegistryProviderProvenanceV1`;
- `provider_signatures_verified() == true`;
- `provider_identity_trusted_under_policy() == true`;
- `producer_contract_qualified() == false`;
- `registry_authenticated() == false`;
- `registry_current() == false`;
- no currentness field in the portable provider envelope;
- exact committed test count.

## Success ceiling

A real PASS establishes only exact consumer-side provider-provenance execution under the bound source.

It does not qualify the Xenia producer contract, registry authentication, currentness, PSI security, full composition, production admission or application authority.
