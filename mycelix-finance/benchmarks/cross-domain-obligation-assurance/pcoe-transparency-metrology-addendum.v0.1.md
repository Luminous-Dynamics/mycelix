# PCOE Transparency, Metrology and Market-Adapter Addendum v0.1

**Status:** proposed interoperability and qualification profile; not a regulatory approval, conformance certificate, or production authorization.  
**Date:** 22 November 2026.

## 1. Architecture decision: compose existing standards, do not reinvent them

PCOE should not become a new transparency-log, agent-identity, credential, metrology or electricity-market standard. It should define the cross-domain assurance semantics and reuse specialist infrastructure with explicit boundaries.

### 1.1 SCITT — transparency of signed statements

IETF RFC 9943 (June 2026, Standards Track Proposed Standard) defines an architecture for single-issuer signed statement transparency. A Transparency Service verifies the Issuer's signature and registration policy, records the signed statement in a verifiable data structure, and returns a receipt proving inclusion. Receipt issuance follows registration; the process may be asynchronous. A statement may be registered with multiple transparency services, producing independent receipts.

**PCOE mapping**

- Put a signed, immutable statement envelope around each accountable claim, meter export, verifier attestation, contract acceptance and rail receipt reference.
- Bind the statement to schema/profile version, issuer key identifier, event time, record time, subject/obligation, exact evidence digests and supersession reference.
- Store SCITT receipt(s), transparency-service identity, consistency/checkpoint evidence and receipt verification outcome as a provenance layer.
- Use multiple transparency services only where their independence and governance are actually qualified; different endpoints do not automatically imply independent failure domains.
- An unregistered or pending statement remains pending. No transparency receipt means no inclusion claim.
- A receipt is not a physical measurement, verifier accreditation, legal mandate, contract acceptance or payment-finality proof.

**Critical boundary:** RFC 9943's transparency architecture does not make a dishonest or compromised Issuer truthful. It provides traceability and auditability of the Issuer's signed statements. A false statement can be correctly signed and transparently registered. PCOE therefore requires a separate truth-scope decision: which independent method, device, inspection, contractual authority or rail record supports the assertion?

A SCITT receipt should attach to the statement node in the obligation graph, never replace the observation/verification/acceptance/payment nodes.

### 1.2 OWASP AISVS — agent-specific implementation controls

As of this research snapshot, OWASP AISVS 1.01 is the latest stable version found; version 1.02 is in development. AISVS is a vendor-neutral catalogue of testable controls for AI-enabled systems, including agent orchestration and MCP security. It is intended to complement, not replace, broader application and infrastructure controls.

The PCOE agent-authority profile should explicitly map to these requirements (pinning the version is mandatory because IDs can change):

| AISVS 1.01 requirement | PCOE mapping |
|---|---|
| C9.2.1–9.2.5 | trusted approvals, canonical action presentation, reversibility class, and policy-bound self-modification |
| C9.2.8 | cryptographically bind approvals to parameters, requester, context and single-use nonce |
| C9.2.9 | isolate approval credentials/keys from the agent runtime |
| C9.2.10–9.2.11 | highest-impact action-chain approval and reapproval when the chain expands |
| C9.3.1–9.3.9 | sandbox/isolate tools, validate outputs, separate untrusted data from tool execution, contain policy violations |
| C9.4.1–9.4.4 | distinct cryptographic agent identity, execution-chain attribution, credential rotation, integrity-protected state |
| C9.5.1–9.5.7 | fine-grained runtime authorization, scoped integrity-protected tokens, deterministic policy, delegation rules, per-action policy refresh and attenuation |
| C9.6.1–9.6.3 | human-controlled shutdown, approval timeout blocks, out-of-band kill switch |
| C10.2.4–C10.2.5 | expose only authorized MCP tools and enforce per-invocation tool and argument authorization |

The current Rust kernel implements only a subset of these as deterministic reference rules. Cryptographic credentials, nonce verification, key isolation, sandboxing, tool-discovery filtering, actual runtime revocation, kill switches and independent event logging remain adapter/runtime work. Do not claim AISVS conformance from passing this kernel.

### 1.3 OIML R 46 — electricity-meter legal metrology

OIML Recommendation R 46 (2012 edition, shown by OIML as current) specifies metrological and technical requirements for active electrical energy meters subject to legal metrological controls, including type approval, verification, re-verification and modifications to approved devices.

PCOE should distinguish:

1. **Software plausibility:** integer arithmetic, interval ordering, register monotonicity, capacity/time bounds and sequence/replay checks.
2. **Device and installation qualification:** meter model/type approval, installation conditions, wiring, transformer ratios, firmware/configuration, sealing and tamper controls where applicable.
3. **Legal metrology:** valid verification/re-verification status and certificates under the applicable jurisdiction.
4. **Independent measurement assurance:** calibration/verification provenance, uncertainty and the exact metering boundary.
5. **Commercial settlement:** the market/contract rules that determine which boundary is billable and how losses, imports, exports and storage are accounted for.

The pure Rust meter function only addresses a portion of (1). It cannot make a meter legally approved or physically accurate. A production evidence adapter must ingest the applicable metrology records and route expired, revoked, mismatched or unverifiable device status to an indeterminate state.

### 1.4 South African electricity-market adapter

The Electricity Regulation Amendment Act 38 of 2024 commenced on 1 January 2025. NERSA's 17 July 2026 Version 03 consultation paper explicitly presents proposed Rules for Electricity Trading for public consultation. NERSA's 15 September 2026 Electricity Regulation Projects Dashboard still listed Trading Rules as CONSULTATION, with public consultation extended to 28 September and a member workshop in October.

The latest official status evidence located for this research does not establish the final adopted Trading Rules or prove all market-access procedures currently in force as at 22 November 2026. The October target completion date on the dashboard is a target, not evidence of adoption.

Therefore:

- treat NERSA Version 03 as a consultation draft in the current evidence package;
- do not encode draft provisions as authoritative law or production permissions;
- version and hash every legal/regulatory instrument and implementation notice consumed by an adapter;
- gate live market participation on fresh NERSA/legal confirmation, applicable licensing/registration, market-code and contracting rules, metering requirements, wheeling arrangements, tariff treatment and settlement rules;
- build the first demonstration as a sandbox procurement/settlement workflow, not as an unlicensed market-trading platform.

The first community installation can still validate project financing, procurement, commissioning, calibrated measurement, contract acceptance and payment reconciliation under a legally reviewed contract without assuming that the emerging wholesale/retail trading rules are final.

## 2. New adversarial scenarios

The companion `pcoe-transparency-metrology-scenarios.v0.1.json` adds ten cases. They are currently adapter-level qualification targets and are not implemented by the in-memory Rust kernel.

The tests must distinguish statement integrity, transparency inclusion, factual correctness, instrument status and jurisdictional authority. A successful signature check or receipt verification is not enough.

## 3. Metrics to add

| Metric | Target / reporting rule |
|---|---|
| Signed-statement receipt coverage | 100% of statements required by the pinned profile have verifiable receipts before the corresponding gate passes |
| Pending-receipt promotion | 0 pending receipts accepted as anchored |
| Transparency-service diversity | Report independent operator, legal, key-custody, infrastructure and governance failure domains; do not count endpoints |
| Statement-to-evidence binding | 100% of accepted statements bind the exact content/evidence digest and profile version |
| Meter legal-status coverage | 100% of revenue-settlement intervals linked to a legally valid meter/device status where applicable |
| Meter boundary mismatch | 0 unapproved boundary, unit, CT/PT ratio or device-configuration substitutions |
| Regulatory instrument freshness | 100% of live-market decisions cite the currently effective, jurisdiction-specific instrument and effective date |
| Adapter fail-closed behavior | 100% of missing, stale, conflicting or draft-only regulatory inputs block live execution and preserve a reasoned disposition |

Measure proof/receipt verification latency and storage cost, but do not trade away the safety conditions to improve throughput.

## 4. Cross-system reference profile

The final evidence graph should be reconstructable without trusting the producer's database:

`Mandate → Task Capability → Signed Statement → Transparency Receipt → Meter Observation → Independent Verification → Contract Acceptance → Payment Intent → Rail Receipt → Reconciliation`

Each edge must bind an exact parent digest and declare the scope of the relation. The order is logical rather than a mandatory wall-clock sequence: for example, mandates and a contract can be established before metering begins. Later corrections are new records that supersede prior records without deleting history.

## 5. References

- IETF RFC 9943, *An Architecture for Trustworthy and Transparent Digital Supply Chains* (June 2026): https://datatracker.ietf.org/doc/rfc9943/
- OWASP Artificial Intelligence Security Verification Standard, current repository and release policy: https://github.com/OWASP/AISVS
- OWASP AISVS 1.01, C9 Orchestration and Agentic Security: https://github.com/OWASP/AISVS/blob/main/1.01/en/0x10-C09-Orchestration-and-Agentic-Action.md
- OWASP AISVS 1.01, C10 MCP Security: https://github.com/OWASP/AISVS/tree/main/1.01/en
- OIML Recommendation R 46, *Active electrical energy meters* (2012, current edition listed by OIML): https://www.oiml.org/en/publications/recommendations/46/
- NERSA, Consultation Paper on Version 03 of the Rules for Electricity Trading, 17 July 2026: https://www.nersa.org.za/files/files/2026/07/ConsultationPaperonDraftNERSARulesforElectricityTrading17July2026.pdf
- NERSA, Electricity Regulation Projects Dashboard, September 2026 (last updated 15 September 2026): https://www.nersa.org.za/files/files/2026/09/ELRStakeholderDashboard-September2026-Rev1-HighRes.pdf

---

**Interpretation rule:** transparency proves registration under a transparency service's rules; it does not prove the issuer's statement is true. Software plausibility checks do not establish legal metrology conformity. A draft market rule is not an adopted legal authority. These evidence classes must remain separate in every PCOE decision.
