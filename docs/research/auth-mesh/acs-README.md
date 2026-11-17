# ACS Comparator — public development seed

**Corpus:** `ACS-COMP-18-v1`  **Owner:** [#6216 — LISE-AUTH-MESH-001](https://github.com/Luminous-Dynamics/mycelix/issues/6216)

- `acs-scenarios.v1.json`: 18 synthetic scenario packets in nine minimal pairs.
- `acs-oracle.v1.json`: separate per-property oracle.
- `acs-scenario.schema.json` and `acs-oracle.schema.json`: strict Draft 2020-12 input/output schemas.

Pairs exercise hook completeness, deny/ask/defer honoring, liveness, context-chain integrity vs event completeness, Core versus Crypto/Audit claim limits, dynamic AgBOM inventory, provenance trust monotonicity, and tool-result versus independent effect evidence.

The oracle distinguishes protocol conformance from broader claim disposition. An ACS-Core pass does not automatically establish complete mediation, secure policy, resistance to compromised Guardian, source truth or independent effect observation.

All scenarios and oracle answers are public. This is a development/calibration set, not official OWASP ACS conformance, certification or a blind holdout. Pin the exact ACS release/schema/profile for any real conformance adapter.

## Normative conformance distinctions

- ACS-Core mandatory hook minimum and additional hooks that SHOULD be emitted when the host can observe them are distinct categories.
- The six optional profiles are declared during handshake; that declaration is self-asserted and is not proof of implementation. Profile-specific behavior must be tested.
- MODIFY is SHOULD-support. A MODIFY-capable client must apply the modified operation. A client that cannot modify must follow the Guardian's DENY substitution with `modify_unsupported`, not execute the original request.
- A2A wrapping is deferred beyond ACS v0.1.4; the namespace is reserved, but normative A2A method mapping is a v0.2.0 item. MCP wrapping is in v0.1.

See `acs-extension-README.md` and extension v1.1 for the corresponding known-answer pairs.