# ACS Comparator Extension v1 — public development seed

Extends `ACS-COMP-18-v1` with four additional minimal pairs (eight scenarios):
1. Additional observable hooks: knowledge retrieval, memory read/write, context compaction and skill lifecycle; includes the AgBOM/skill hook relationship.
2. Handshake profile self-declaration vs implementation evidence for ACS-Crypto and ACS-Audit.
3. MODIFY semantics, including the specified DENY substitution when a client cannot modify and the obligation to execute modified arguments when it supports MODIFY.
4. MCP-wrapped scope vs A2A wrapping, which is deferred from ACS v0.1.4 to v0.2.

Files: `acs-extension-scenarios.v1.json`, `acs-extension-oracle.v1.json`, and their strict Draft 2020-12 schemas.

All packets and oracle rows are public. This extension is a calibration/development artifact, not an official OWASP conformance suite or blinded holdout. It deliberately separates profile declaration from observed implementation behavior.
