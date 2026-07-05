# Release Notes — Net Steward v0.1.0-beta

Net Steward v0.1.0-beta is a read-only federated infrastructure witness and security posture console.

## Supported Capabilities

- **Localhost Read-Only Witness Daemon**: Bound strictly to localhost for default control plane isolation.
- **Unified Rest APIs**: Exposes endpoints for active observed topology, config drift tracking, evidence logs, incident capsule verification, and peer posture summaries.
- **Incident Capsule Mechanics**: Supports generation of tamper-resistant capsule artifacts logging EDR event states with cryptographic validation checks.
- **Federated Claim Envelopes**: Integrates a decoupled claim envelope structure storing signed peer assertions with signature scheme headers.
- **Mycelix/Holochain Transport Scaffold**: Implements `FederationTransport` bindings using mock and sandboxed websocket channels.
- **Conflict Reconciliation Engine**: Real-time evaluation weighting peer freshness, expiration, and identifying conflicting assertions without pretending certainty.
- **DID Capability-Scope Validation**: Restricts peers to authorized metadata fields (e.g. NixOS topology assertions) based on registered DID capabilities.
- **Safety Doctrine Verification**: Automated validation ensuring no false EDR remediation powers are simulated or claimed.
- **Zero-Panic Infrastructure Audit**: Validated to process empty, malformed, or extreme inputs safely.

## Explicit Non-Capabilities (Out of Scope)

- **Autonomous Remediation**: Net Steward is strictly observation-only.
- **Malware Removal**: No active execution agent is present.
- **Rollback Application / Remediation Execution**: No active modifications of nftables, systemd units, or host state are permitted.
- **Host Isolation & Process Termination**: Out of scope for this milestone.
- **Production-Grade ZK Verifier**: The ZK verifier operates in a simulated verification mode.
- **Production-Grade Identity Binding**: DID-to-agent bindings are scaffolded and will be hardened in future releases.
