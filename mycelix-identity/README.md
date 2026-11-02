# Mycelix Identity

**Decentralized identity, credential, recovery, and trust infrastructure for Mycelix applications.**

Mycelix Identity is a Holochain-based identity subsystem. It includes a custom `did:mycelix` identifier format, credential schemas, verifiable-credential-shaped records, credential lifecycle and revocation functions, social-recovery workflows, MFA, and cross-hApp identity interfaces.

> **Interoperability and security boundary:** This repository contains W3C VC Data Model 2.0-shaped data structures and Mycelix-specific signing, proof, DID, and status mechanisms. That is not, by itself, proof of W3C VC Data Model / Data Integrity cryptosuite conformance, DID Core resolution interoperability, OpenID4VCI/VP conformance, eIDAS 2.0 or EUDI Wallet compliance, or production security. The custom `did:mycelix` method and the project cryptosuite identifiers require their own published specifications, resolver/verifier implementations, test vectors, and independent interoperability testing. See the [Identity interoperability and assurance roadmap](../docs/identity/INTEROPERABILITY_AND_ASSURANCE.md) before relying on standards or security claims.

## Capabilities and implementation boundaries

| Area | Implemented project surface | Boundary to observe |
|---|---|---|
| DID registry | Create, update, deactivate, key-continuity, service-endpoint, and cross-hApp identity operations | `did:mycelix` is a custom method format. Do not assume arbitrary external DID resolvers can resolve it. |
| Credential schemas | Registry entries, versions, JSON-schema text, required/optional field metadata, and active-state checks | Runtime field checks are not automatically full JSON Schema validation or proof of external schema interoperability. |
| Verifiable credential records | VC 2.0-shaped context, type, issuer, subject, validity, schema, status, and proof fields | The current signature input uses a Mycelix-specific content hash. It is not W3C Data Integrity canonicalization. Check the roadmap's field-binding audit. |
| Ed25519 signing | HDK-backed signing and verification for the native Ed25519 path | The signed byte representation and key relationship must be aligned with a published interoperable cryptosuite before claiming W3C Data Integrity conformance. |
| Hybrid / post-quantum signatures | Algorithm-tagged signature envelopes and an off-chain signing path are present | The current WASM verifier checks the Ed25519 component for the hybrid path; the PQC component is not verified there. |
| Pure ML-DSA / SLH-DSA credential proofs | Tagged signature types and off-chain signing path are represented | **The current WASM verifier structurally accepts recognized pure-PQC signature lengths; it does not cryptographically verify those signatures.** Do not treat its `valid` result as proof that a pure-PQC signature verified. |
| Selective disclosure | Mycelix-specific derived-credential/Merkle proof structures and a separate eIDAS-ZKP data model | JSON filtering, proof metadata, or a non-empty proof string is not proof verification. The `eidas-zkp` crate's current tests include placeholder proof strings and shape assertions; they do not establish cryptographic or EUDI conformance. |
| Revocation | Mycelix revocation registry queries and fail-closed handling of unknown/unavailable status in the credential verifier | The current VC status fields resemble Bitstring Status List fields, but a populated status-list credential, interoperable publication, and external conformance are not established by field names alone. |
| Social recovery / MFA | Trustee, threshold, time-lock, and authentication-related zomes | Treat policy parameters and tests as implementation-specific; recovery safety needs adversarial multi-agent and independent security review. |

## Architecture

```text
mycelix-identity/
├── zomes/
│   ├── did_registry/          # did:mycelix registry and key continuity
│   ├── credential_schema/     # credential schema metadata and validation
│   ├── verifiable_credential/ # issuance, presentations, verification, derived claims
│   ├── revocation/            # revocation and suspension status
│   ├── recovery/              # social recovery workflows
│   ├── mfa/                   # multi-factor authentication
│   ├── trust_credential/      # Mycelix trust credentials
│   ├── bridge/                # cross-cluster integration
│   └── web-of-trust/          # social trust relationships
├── crates/
│   ├── mycelix-crypto/        # algorithm-tagged signature abstractions
│   └── eidas-zkp/             # experimental credential / disclosure data model
├── cli/                       # issuer / credential tooling
└── tests/                     # unit, integration, and conductor-oriented tests
```

The exact workspace membership and build commands are declared in [Cargo.toml](Cargo.toml), [flake.nix](flake.nix), and [tests/Cargo.toml](tests/Cargo.toml). Use the pinned Nix environment and the current repository instructions rather than old absolute paths copied from historical developer machines.

## DID format

The project currently represents local agent identities in this form:

```text
did:mycelix:<Holochain AgentPubKey>
```

Example values in tests and documentation are illustrative only.

This is a project-specific DID method format, not a claim that the method has a complete public DID Method specification, a general-purpose resolver, or cross-implementation DID Core interoperability. The roadmap defines the work required to publish and test those pieces.

## Credential lifecycle

The current architecture separates several concerns:

1. **Identity / issuer binding:** derive the local author from Holochain agent context; do not trust a client-supplied issuer identity.
2. **Schema handling:** optionally validate claim fields against a registered schema. Strict-schema issuance should be used when validation is required; non-strict mode can continue when a schema is missing or unavailable.
3. **Issuance:** create a credential record and proof value using the implementation-specific signing path.
4. **Presentation:** bind a holder proof to a challenge and domain when supplied.
5. **Verification:** check the supported signature path, proof purpose, validity period, and revocation status.
6. **Selective disclosure:** use the separate derived-credential and ZKP-related code paths only within their documented verification guarantees.

A successful local Holochain call or a schema-valid JSON object does not establish external VC interoperability. For the exact checks currently performed—including the pure-PQC structural-accept path—see [the interoperability and assurance roadmap](../docs/identity/INTEROPERABILITY_AND_ASSURANCE.md).

## Development

From a clone of this repository:

```bash
nix develop
cd mycelix-identity
cargo build --release --target wasm32-unknown-unknown
hc dna pack dna/
hc app pack .
```

Integration tests are defined under `tests/`. Conductor-based tests require the configured Holochain conductor / app environment and must be run explicitly. A test file that is not enabled or executed is not a passing test.

## Interoperability roadmap

The next step is to preserve the Holochain-native identity and trust model while adding explicit interoperability adapters:

- Publish a DID method specification and a resolver / deactivation / key-rotation test suite.
- Separate the internal Holochain action identity from externally resolvable DID documents.
- Map credential and presentation objects to W3C VC Data Model 2.0 using a supported, independently implemented Data Integrity cryptosuite or another explicitly selected proof format.
- Define canonicalization and complete signature coverage; bind every security-relevant field.
- Replace structural acceptance of pure-PQC signatures with actual verification in an appropriate trusted verifier; the caller must receive an explicit verification result and assurance level.
- Implement externally resolvable revocation/status resources and validate them with independent verifiers.
- Add OpenID4VCI issuance and OpenID4VP presentation adapters, then run the relevant conformance suites.
- Keep Mycelix-specific trust, recovery, privacy, and governance semantics as explicit extensions instead of silently redefining standard claims.

See [INTEROPERABILITY_AND_ASSURANCE.md](../docs/identity/INTEROPERABILITY_AND_ASSURANCE.md) for the staged plan and measurable acceptance criteria.

## Security reporting

The repository's [SECURITY_STATUS.md](../SECURITY_STATUS.md) records an internal author-binding remediation review, not a third-party audit. It was reviewed in July 2026 and explicitly documents public-export sync lag, open issues, and components not yet assessed. Do not read a "fixed internally" status as evidence that the public HEAD or a deployed bundle contains the fix. Verify the exact source revision and tests for the component under review.

Report security-sensitive issues privately according to the organization [security policy](https://github.com/Luminous-Dynamics/.github/blob/main/SECURITY.md); do not publish an exploitable vulnerability in a public issue.

## Licensing

The identity workspace is licensed AGPL-3.0-or-later. Review the applicable crate and dependency license files before redistribution or commercial integration.
