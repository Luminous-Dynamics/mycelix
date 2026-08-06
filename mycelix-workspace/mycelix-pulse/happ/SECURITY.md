# Security Policy

## Supported Versions

**Note (2026-07-25): the "Phase 1 (Core DNA development)" status below is stale** — this
file predates the working-alpha contract. See `docs/adr/ADR-002-working-alpha-contracts.md`
for the current canonical security/lifecycle status and `PULSE_READINESS_PLAN.md` /
`PULSE_NEXT_SESSION_PLAN_2026-07-18.md` for the actual roadmap. Security updates are
provided for the main branch only.

| Version | Supported          |
| ------- | ------------------ |
| main    | :white_check_mark: |

## Reporting a Vulnerability

**Please do not file public issues for security vulnerabilities.**

If you discover a security vulnerability in Mycelix Mail, please report it by emailing:

**tristan.stoltz@evolvingresonantcocreationism.com**

Please include:

1. **Description**: A clear description of the vulnerability
2. **Impact**: How the vulnerability could be exploited
3. **Steps to Reproduce**: Detailed steps to reproduce the issue
4. **Suggested Fix**: If you have ideas on how to fix it (optional)
5. **Your Contact Info**: So we can follow up with you

### What to expect:

- **Initial Response**: Within 48 hours
- **Status Update**: Within 1 week
- **Fix Timeline**: Varies by severity
  - Critical: 7 days
  - High: 14 days
  - Medium: 30 days
  - Low: 90 days

### Security Best Practices

When using Mycelix Mail:

1. **Keep Dependencies Updated**: Regularly update Holochain and Rust
2. **Use Strong DIDs**: Follow DID best practices for key generation
3. **Trust Score Thresholds**: Set appropriate minimum trust scores for your use case
4. **Monitor Logs**: Watch for unusual activity in Holochain logs
5. **Backup Keys**: Securely backup your agent keys

### Security Features

Mycelix Mail implements multiple security layers:

- **E2E Encryption**: All message bodies encrypted
- **Trust-Based Filtering**: Prevents spam at protocol level
- **Holochain Validation**: Distributed validation of all messages
- **MATL Byzantine Tolerance**: 45% is the theoretical design-ceiling of the composite trust
  formula under worst-case adversary assumptions, not an empirical result — the empirically
  validated figure (mycelix-core federated-learning benchmarks) is 34%. Cite 34% for tested
  behavior, 45% only for the formula's design ceiling (see `mycelix-workspace/CLAUDE.md`).
- **Open Source**: Auditable by anyone

### Known Limitations (current, 2026-07-25 — supersedes the stale Phase-1 list below)

- The SMTP gateway (Phase 5A) is implemented and live (`crates/pulse-smtp-gateway/`);
  external interop (Phase 5B — real MX/DKIM, non-Holochain accounts) is deliberately
  descoped, not merely unbuilt — see `PULSE_READINESS_PLAN.md`.
- A Leptos browser UI exists and is the supported alpha surface, not CLI-only — see ADR-002.
- The `V2HybridPqc` hybrid-PQC envelope is implemented and live-verified (default for new
  messages), but per `docs/PULSE_V2_CRYPTO_SPEC.md` and `README.md` it remains **behind its
  complete release-evidence gate** — no production-security claim is made. No forward secrecy
  or post-compromise recovery against static-key compromise (no ratchet exists).
- No professional/independent third-party crypto audit has occurred. The project's current
  gating mechanism is ADR-002's internal *release-evidence gate* (Sweettest proof +
  live-verification + documented exclusions), not an external audit — treat these as distinct
  claims and don't conflate "evidence-gated" with "audited."

<details>
<summary>Original Phase-1 limitations list (stale, kept for history)</summary>

- MATL integration is simulated (trust scores not yet real)
- SMTP bridge not yet implemented (no email interop)
- UI not yet built (CLI only)
- No formal security audit yet (planned for Phase 3)

</details>

### Future Security Work

- [ ] Independent/professional crypto audit (no longer gated to a specific phase number;
      distinct from, and in addition to, the release-evidence gate above)
- [ ] Penetration testing
- [ ] Formal verification of validation rules
- [ ] Bug bounty program
- [ ] Security documentation

---

Thank you for helping keep Mycelix Mail secure! 🍄
