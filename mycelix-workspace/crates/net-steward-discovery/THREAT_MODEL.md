# Threat Model - Net Steward (Read-Only Witness Version)

This threat model outlines safety considerations and design gates for the Net Steward discovery and witness daemon. 

## 1. Trust Boundaries
Net Steward runs as a localhost-bound service daemon. It exposes REST API endpoints on `127.0.0.1:3030`. The primary user interface runs in a sandboxed client browser environment (Leptos).

## 2. Threat Vector Matrix

### T1: Localhost API Misuse
* **Threat**: Non-authorized local processes or users querying daemon endpoints to glean network topology details.
* **Impact**: Information disclosure of local adapters, routing tables, and peer configurations.
* **Mitigation**: Bind daemon strictly to loopback (`127.0.0.1`). In future versions, require a session handshake token or unix socket permissions to restrict access to the logged-in system operator.

### T2: DNS Rebinding & Cross-Origin Misuse
* **Threat**: Malicious websites in the operator's browser running scripts that query `http://localhost:3030`.
* **Impact**: Cross-origin read of local topology, drift, and evidence hashes.
* **Mitigation**: Tight CORS headers restricting responses, strict host header validation, and token-based origin assertions on all non-safe HTTP methods.

### T3: Forged Adapter Data
* **Threat**: Compromised local interfaces or modified system binaries feeding fake state (ARP, routing table, bridging specs) to the discovery engine.
* **Impact**: Poisoned network topology display or missed configuration drift detection.
* **Mitigation**: Use independent, multi-source topology merge logic. Cross-verify ARP entries with active DHCP lease fixtures and signed identity metadata.

### T4: Operator Overtrust / False ZK Badging
* **Threat**: Operator assumes safety properties are fully verified mathematically without an active cryptographic verifier running.
* **Impact**: Dangerous policy overrides based on false certainty.
* **Mitigation**: Strict, claim-disciplined UI labels (e.g. `Verifier: Not Active`, `Proof Envelope: Simulated`). Never claim proof validation unless a zero-knowledge proof verifier is running and fails closed on failure.

### T5: Rollback Plan Misuse
* **Threat**: Automation scripts or operators applying rollback plans blindly without human preview.
* **Impact**: Service outages, routing loops, or system bricking on incorrect configuration rollback.
* **Mitigation**: Rollback API remains dry-run only. It generates descriptive operator steps but never runs actual mutating commands. Setting `apply_supported: false` and `requires_approval: true` is hardcoded.

### T6: Privilege Escalation
* **Threat**: Hijacking Net Steward to execute commands as `root` (e.g. `nixos-rebuild switch --rollback`).
* **Impact**: Complete compromise of system integrity.
* **Mitigation**: The daemon process runs with low/unprivileged permissions and does not execute system modification shell commands.

### T7: Supply-Chain Risk
* **Threat**: Malicious dependencies in the build graph compromising proof generation libraries or serialization paths.
* **Impact**: Hidden backdoors or payload leakage.
* **Mitigation**: Explicit dependency pinning, Cargo lock verification, and minimized dependency footprints for core schema crates.

---

## 3. Federated Posture Threat Vectors (Added in alpha.4)

### T8: Malicious Peer Claims & Spoofing
* **Threat**: A compromised peer on the local subnet broadcasting false posture statuses or forging identity signatures to trick the coordinator.
* **Impact**: Quarantined node false positives, or failure to spot real drift on compromised devices.
* **Mitigation**: Explicit `PeerTrustStatus` tracking (Signed vs Unsigned vs Stale). Receipts must be cryptographically signed by known peer identity DIDs (`claimed_by` assertions).

### T9: Replayed or Forged Incident Capsules
* **Threat**: Intercepting and replaying outdated incident capsule snapshots to confuse operators or hide current posture changes.
* **Impact**: Stale baseline evaluations and operator distraction.
* **Mitigation**: Capsules contain Unix timestamps and unique `capsule_id` identifiers. The verification library checks timestamps, hashes_valid, and fails on duplicate or expired receipts.

### T10: Live vs Simulated Mode Confusion
* **Threat**: Operators mistaking simulated threat scenarios for live system alerts or vice-versa.
* **Impact**: Unnecessary system lockdowns or ignored real posture alerts.
* **Mitigation**: Clear global visual banners (`⚠️ Scenario Mode: Simulated`) rendered whenever a mock switcher option is selected. Simulated modes never interact with the live REST endpoints.
