# Mycelix Identity

**Universal Self-Sovereign Identity for the Mycelix Civilizational OS**

## Overview

Mycelix Identity provides decentralized identity management for the entire Mycelix ecosystem. It enables:

- **DID:mycelix** - Decentralized identifiers for all agents
- **Verifiable Credentials** - W3C-compliant credentials with Mycelix extensions
- **Social Recovery** - No seed phrase loss via trusted contacts
- **Privacy** - Selective disclosure and zero-knowledge proofs

## Architecture

```
mycelix-identity/
├── dna/
│   └── dna.yaml              # DNA manifest
├── zomes/
│   ├── did_registry/         # DID:mycelix format, key management
│   │   ├── integrity/        # Entry validation
│   │   └── coordinator/      # Business logic
│   ├── credential_schema/    # Schema definitions
│   │   ├── integrity/
│   │   └── coordinator/
│   ├── revocation/           # Credential revocation registry
│   │   ├── integrity/
│   │   └── coordinator/
│   └── recovery/             # Social recovery
│       ├── integrity/
│       └── coordinator/
├── client/                   # TypeScript client
└── tests/                    # Integration tests
```

## Zomes

### did_registry

Manages DID:mycelix identifiers:

- Create/update/deactivate DIDs
- Key rotation with continuity
- Service endpoint management
- Cross-hApp identity queries

### credential_schema

Defines credential types for the ecosystem:

- Standard schemas (education, employment, identity)
- Custom schema registration
- Schema versioning
- Validation rules

### revocation

Handles credential lifecycle:

- Revocation list management
- Suspension (temporary revocation)
- Status queries for verifiers
- Batch operations

### recovery

Enables social recovery:

- Trustee designation (3-7 contacts)
- Threshold-based recovery (60% default)
- Time-locked recovery process
- Emergency recovery protocols

## DID Format

```
did:mycelix:<agent_pub_key>

Example:
did:mycelix:uhCAkX1234567890abcdef...
```

## Integration

All Mycelix hApps use Identity for:

```rust
// Query identity from any hApp
use mycelix_identity::did_registry::get_did_document;

let did_doc = get_did_document(agent_pub_key)?;
```

## Portable sovereign interoperability across regions

The Africa work is one application of a broader reusable model. See [portable sovereign interoperability framework](../docs/portable-sovereign-interoperability-framework.md) for the proposed core/region/national/service profile split and a comparison of EU EUDI Wallet, ASEAN digital-ID interoperability priorities and Latin American/Caribbean IdLAC. It argues for adapting to published regional frameworks, not launching a competing identity scheme or assuming cross-border legal equivalence.

The shared principle is **standards and conformance are reusable; trust, legal permission, assurance mapping and service purpose are not transferable by default**. Each deployment needs its own authority, privacy and security review, operational agreement, fallback and release approval tied to the exact build.

A reusable, jurisdiction-neutral starting point is now available as a deliberately unqualified template:
- [Portable adapter profile template](PORTABLE_ADAPTER_PROFILE_TEMPLATE_V1.json)
- [Closed-schema and release-gate validator](validate_portable_adapter_profile.py)
- [Adversarial contract tests](tests/test_portable_adapter_profile.py)

The template starts with no authority, protocol endpoint, issuer, trust metadata, or authorization, and keeps all 12 release gates closed. From this directory, run `python3 validate_portable_adapter_profile.py` and `python3 -m unittest discover -s tests -p 'test_portable_adapter_profile.py' -v`. The validator is a structural guard; it does not authenticate written approvals or independently verify credentials.

### Synthetic end-to-end service journey

The [synthetic service journey corpus](SYNTHETIC_SERVICE_JOURNEY_V1.json) and [deterministic evaluator](synthetic_service_journey.py) model a proof-only appointment-intake flow, not a government integration. Eighteen frozen scenarios exercise minimum disclosure, exact purpose-grant binding, invalid/replayed/unknown proof states, verifier outages, assisted/non-digital fallback, correction/appeal routing, and rejection of identity-as-entitlement decisions.

From `mycelix-identity/`, execute the frozen vector corpus with an exact source commit:

```sh
python3 synthetic_service_journey.py --source-commit <40-character-commit-sha> --report /tmp/synthetic-service-journey-report.json
python3 -m unittest discover -s tests -p 'test_synthetic_service_journey.py' -v
```

The report is bound to the source SHA, evaluator hash and canonical corpus hash. Receipts are allow-listed, contain only outcome metadata and claim names, and are emitted as ephemeral outputs only. The harness does not store durable appeal cases, validate actual credentials/signatures, contact an external endpoint, or make eligibility decisions. Synthetic test approval is not real legal authorization.

### Correction and independent appeal case contract

The [case contract](APPEAL_CASE_CONTRACT_V1.json) and [in-memory state machine](appeal_case_engine.py) define owner/steward/reviewer roles, explicit state transitions, independent appeal review, retention-policy gates, and a SHA-256 hash-chained event sequence. Audit events use allow-listed codes and opaque case-scoped actor references; raw credentials, identity values, biometrics, free-text notes and verifier responses are prohibited.

Run the contract check and frozen demo from this directory:

```sh
python3 appeal_case_engine.py
python3 appeal_case_engine.py --source-commit <40-character-commit-sha> --report /tmp/appeal-case-report.json
python3 -m unittest discover -s tests -p 'test_appeal_case_engine.py' -v
```

The report binds the source commit, engine digest, contract digest and 12-event chain summary. The example keeps durable storage, external notifications and public-DHT publication disabled. Retention defaults to `not_assessed`; a reviewed policy must specify the same jurisdiction and record class as the case's controlled issue-code mapping, along with lawful-basis, responsible-party, maximum-retention, deletion, backup-deletion and review references. An unassigned jurisdiction or class mismatch fails closed. Even a complete policy does not turn on persistence or perform deletion. An expired state is not proof that database replicas/backups have been erased.

The audit chain is tamper-evident metadata, not an authenticated signature, trusted timestamp, authorization service, or production audit log. Opaque handles may still be personal information. **Only service-record correction and appeal are supported in v1.** Issuer-source correction, access/deletion requests, processing objections, regulator complaints and accessibility referrals are explicitly marked unimplemented; this harness does not send external referrals or resolve those rights. Map them to the official controller/regulator route for the applicable jurisdiction before processing real data.


## African Digital Public Infrastructure interoperability

A continent-level contract defines the governance boundaries for interoperating independently governed national systems:
- Design and country capability strategy: [Africa DPI interoperability strategy](../docs/africa-digital-public-infrastructure-interoperability.md)
- Machine-readable invariant contract: [AFRICA_DPI_INTEROP_CONTRACT_V1.json](AFRICA_DPI_INTEROP_CONTRACT_V1.json)
- Contract validator: [validate_africa_dpi_contract.py](validate_africa_dpi_contract.py)
- Adversarial tests: [test_africa_dpi_contract.py](tests/test_africa_dpi_contract.py)
- Source-backed country research matrix (South Africa OVS + smartphone Digital ID, Ghana NIA, Rwanda e-ID, Nigeria NIMC/NIN): [AFRICA_COUNTRY_RESEARCH_MATRIX_V1.json](AFRICA_COUNTRY_RESEARCH_MATRIX_V1.json)
- Country-matrix validator: [validate_africa_country_research_matrix.py](validate_africa_country_research_matrix.py)
- Country-matrix adversarial tests: [test_africa_country_research_matrix.py](tests/test_africa_country_research_matrix.py)

The contract deliberately contains no qualified national profiles and keeps every release gate closed. It prohibits a continental identity database/root of trust, storing raw government identity material in a public DHT, using identity as authorization, undocumented API access, and mandatory smartphone-only access. It requires a legal/purpose review, minimum disclosure, assisted/non-digital alternatives, correction/grievance paths, and country-specific release approval. The AU framework is a coordination reference, not a claim that a country implements a particular protocol or that Mycelix is a continental trust authority.

### Validate the contracts

From `mycelix-identity/`, run `python3 validate_africa_dpi_contract.py` and `python3 -m unittest discover -s tests -p 'test_africa_dpi_contract.py' -v`. The focused SA-DID workflow runs these checks alongside the South African research profile. Also run `python3 validate_africa_country_research_matrix.py` and `python3 -m unittest discover -s tests -p 'test_africa_country_research_matrix.py' -v` to confirm every research route remains unauthorized and production-disabled.

## South African DHA Digital ID (research-only)

A separate, fail-closed research profile tracks possible interoperability with South Africa's Department of Home Affairs (DHA):
- Design and release gates: [SA-DID-001 interoperability profile](../docs/sa-digital-id-interoperability.md)
- Machine-readable policy and synthetic expected-outcome corpus: [SA_DID_PROFILE_V1.json](SA_DID_PROFILE_V1.json)
- Structural contract validator: [validate_sa_did_profile.py](validate_sa_did_profile.py)
- Synthetic-only policy oracle: [sa_did_policy_engine.py](sa_did_policy_engine.py)
- Contract regression tests: [test_sa_did_profile_contract.py](tests/test_sa_did_profile_contract.py)
- Executable policy tests: [test_sa_did_policy_engine.py](tests/test_sa_did_policy_engine.py)
- OVS onboarding request draft (not sent): [DHA_OVS_ONBOARDING_REQUEST.md](DHA_OVS_ONBOARDING_REQUEST.md)

**This is not a production integration.** The existing server-side OVS route and the future holder-presented smartphone Digital ID route are distinct evidence methods. The profile keeps production disabled, configures no official DHA trust anchor, and does not claim DHA access, approval, certification, or POPIA compliance. Tests use only synthetic policy scenarios; no real identity data or DHA endpoint is used.

### Run the synthetic contract checks

From `mycelix-identity/`, run `python3 validate_sa_did_profile.py`, then `python3 sa_did_policy_engine.py --source-commit <40-character-commit-sha> --report /tmp/sa-did-synthetic-policy-report.json`, then `python3 -m unittest discover -s tests -p 'test_sa_did_*.py' -v`. The report contains expected and actual dispositions for all 20 vectors and explicitly records that it is synthetic-only, not a cryptographic verification result. It records the source commit SHA and hashes the profile file's raw bytes, normalized profile, and engine/validator source files. These fingerprints aid reproducibility; they are not a signature or proof that a trusted CI runner executed the code.

The focused [`sa-did-contract.yml`](../.github/workflows/sa-did-contract.yml) workflow runs this Python-only contract independently of the monolithic CI change-detection job, checks out the exact commit it records, and uploads the report as a 14-day artifact. The main CI identity job also explicitly checks out the same reported head revision.

## Development

```bash
# Enter Nix environment
cd /srv/luminous-dynamics/mycelix-workspace
nix develop

# Build zomes
cd ../mycelix-identity
cargo build --release --target wasm32-unknown-unknown

# Package DNA
hc dna pack dna/

# Package hApp
hc app pack .

# Run tests
cd tests && npm test
```

## Part of the Civilizational OS

Identity is the foundation of the Three-Pillar architecture:

- **Governance Pillar**: Voting eligibility, proposal authorship
- **Economic Pillar**: Credit scoring, property ownership
- **Knowledge Pillar**: Author attribution, fact-checker credentials

## License

Apache 2.0
