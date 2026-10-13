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

From `mycelix-identity/`, run `python3 validate_sa_did_profile.py`, then `python3 sa_did_policy_engine.py --report /tmp/sa-did-synthetic-policy-report.json`, then `python3 -m unittest discover -s tests -p 'test_sa_did_*.py' -v`. The report contains the exact expected and actual dispositions for all 20 vectors and explicitly records that it is synthetic-only, not a cryptographic verification result. It binds the result to the profile file's raw-byte SHA-256 (when run through the CLI), the normalized profile SHA-256, and the exact engine and validator source SHA-256 values. These fingerprints make artifacts comparable to the inputs/code that generated them; they are not a digital signature or proof that a trusted CI runner executed the code. CI uploads the report as the `sa-did-synthetic-policy-report` artifact when it is produced.

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
