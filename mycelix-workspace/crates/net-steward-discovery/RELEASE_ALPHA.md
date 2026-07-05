# Net Steward v0.1-alpha — Witness Daemon Release

Status: v0.1-alpha read-only witness daemon.

### What it does:
- Discovers local network/topology facts (ARP, Routing table, Wireguard status, virtual interfaces).
- Exposes a localhost-only, read-only REST API endpoint structure.
- Reports NixOS system drift configurations against previous profiles.
- Generates safety commitments and simulated ZK-STARK proof envelopes.
- Generates dry-run, preview-only rollback plans.
- Serves live metrics and state dynamically to the Leptos admin dashboard.

### What it does not do:
- Does not apply configuration mutations.
- Does not execute active rollback.
- Does not run privileged commands.
- Does not verify real cryptographic ZK proofs yet.
- Does not provide production-grade authentication/authorization yet.
- Does not claim complete or comprehensive topology truth.
