# Paradigm Shifts and Qualification Plan for Mycelix Regional Federation

**Status:** Research and engineering plan; proposals, not verified product claims.
**Date:** 2026-10-16
**Related work:** Regional federation RFC; regional-policy kernel draft PR #5098.

## Thesis

The goal is not to ship six forks. The goal is a federation of locally governed civic systems that can cooperate through narrow, explicit and verifiable agreements. Sovereignty must be an observable system property, not a label in configuration or a marketing promise.

The first policy kernel is a foundation, not the end state. It evaluates bilateral rules, but production assurance requires trusted profile and consent verifiers, placement/replication enforcement, operational governance, and evidence from real deployments.

## Eight paradigm shifts

### 1. From region flags to policy-carrying data paths

A region field on a record cannot constrain where that record travels. Every read, write, replication, export, backup, telemetry emission, search index, cache fill, and adapter call must pass through a boundary that knows the data class, action, source and destination scopes, purpose, recipient, profile digests, evidence freshness, and decision result.

Use hard policy before performance or cost optimization. An optimizer must not turn a denied transfer into an allowed one. A path that cannot enforce the relevant policy must be disabled for that data class.

### 2. From identity federation to claim federation

Most cross-community workflows do not need a full identity record. Where appropriate, exchange a minimal, audience-bound, expiring claim such as a qualification or membership fact rather than copying transaction histories or raw identity data.

Selective disclosure or zero-knowledge proofs may reduce disclosure, but do not prove that an issuer is trustworthy, that the claim is legally valid, or that physical data residency is enforced. Proof systems require soundness review; unqualified proofs must never be treated as authority.

### 3. From declared sovereignty to independently evidenced sovereignty

A peer's country string, IP geolocation, certificate, or self-attested cloud region does not prove where all copies are stored or processed. Keep placement claims separate from hosting/processing evidence. The verifier must identify exactly what was attested, by whom, for which time interval, and what the evidence does not establish.

Design the network topology around sensitivity. Public records may be globally replicated; community records need community-scoped replication; restricted personal records may need local storage or explicitly controlled private networks. Do not put sensitive data into a global DHT and attempt to retrofit residency through metadata.

### 4. From always-online services to local-first continuity

Local essential workflows should continue when global control planes, identity providers, payment providers, or international links are unavailable, when this is consistent with the specific workflow's safety and legal limits. Offline actions must be signed, idempotent, bounded, and auditable.

Crucial rule: a queued cross-region action is not pre-approved forever. Before it executes after reconnect, re-evaluate the current profile version, recipient, purpose, consent/revocation state, and validity window. If evidence is stale, the action waits for renewed authorization or is denied.

### 5. From one global consensus to scoped governance

Local communities should not have to run all local decisions through one global voting or consensus process. Keep local decisions local, then federate only agreed events and claims. Agreements should bind both parties' profile digests, the permitted data classes/actions, purpose, validity window, and a real termination and dispute process.

Profile updates need their own trust model: authorized maintainers, multi-party approval for high-impact changes, key rotation/revocation, replay protection, rollback controls, and a documented appeal path. A global platform administrator must not be able to silently rewrite every community's rules.

### 6. From opaque civic scores to bounded, contestable decisions

Do not make a composite reputation, participation, trust, or so-called consciousness score a prerequisite for access to essential services, due process, or basic civic participation. Use the minimum credential genuinely required for a specific action. Any consequential eligibility decision needs a human-understandable reason, a way to correct bad data, an appeal, and an audit trail.

Where communities choose reputation or contribution mechanisms for non-essential programs, scope them to that voluntary purpose, publish the policy, test for capture and exclusion, and prevent those scores from silently crossing into unrelated profiles or regions.

### 7. From one global currency to accounting and settlement interoperability

Do not force regions into one token or one economic policy. Provide common, auditable accounting interfaces and adapters to locally authorized currencies, cooperatives, regulated payment rails, and settlement systems. Exchange rates, fees, tax, custody, redemption, and licensed payment obligations stay explicit. Mycelix SAP, TEND, and MYCEL names do not by themselves make a product legal tender or exempt it from financial regulation.

### 8. From static compliance documents to versioned evidence

A profile should pin its schema, canonical digest, source instruments, version/effective dates, reviewers, trust roots, test corpus and last qualification evidence. A source update should trigger a diff, review, re-qualification, and—when materially incompatible—an explicit hold or migration plan. An AI summary can help triage changes but cannot decide legal meaning or sign the profile.

## Newest research signals to incorporate

These sources point toward adapters and evidence inventories, not blanket continental profiles:

- **Africa:** the AfCFTA Digital Trade Protocol explicitly promotes interoperable open standards and cross-border digital trade. The African Union's May 2025 Guidelines for Integrating Data Provisions in Protocols on Digital Trade are a practical addition to the research register. Country-level law and local authority still have to be established for each actual pilot.
  - https://www.au.int/en/treaties/protocol-agreement-establishing-african-continental-free-trade-area-digital-trade-0
  - https://au.int/en/documents/20250522/guidelines-integrating-data-provisions-protocols-digital-trade
- **South Africa as a possible Africa-track integration, not an assumed pilot:** the South African Reserve Bank's current Payments Ecosystem Modernisation programme describes work toward more inclusive, interoperable payment infrastructure and a national payments utility; SARB published QR+ Standard 1.2 on 7 July 2026 and describes a Payment Credential Ecosystem initiative. These are signals to investigate with authorized partners, not permission to connect to an API or execute payments.
  - https://www.resbank.co.za/en/home/what-we-do/payments-and-settlements/pem
  - https://www.resbank.co.za/en/home/publications/publication-detail-pages/other-publications/2026/qr-1-2-overview
- **European Union:** Regulation (EU) 2024/1183 is the foundational European Digital Identity Framework, but regional profile maintenance must also watch the implementing acts. Commission Implementing Regulation (EU) 2026/1731, dated 15 July 2026, amends technical standards/specifications affecting European Digital Identity Wallets; Commission Implementing Regulation (EU) 2026/798 addresses remote onboarding. Pin the applicable acts and national deployment/conformance evidence, rather than treating the 2024 regulation as the complete frozen specification.
  - https://eur-lex.europa.eu/eli/reg/2024/1183/oj
  - https://eur-lex.europa.eu/eli/reg_impl/2026/1731/oj
  - https://eur-lex.europa.eu/eli/reg_impl/2026/798/oj
- **India:** the Ministry of Electronics and IT notified the Digital Personal Data Protection Rules, 2025 on 13 November 2025. Official government communications describe phased implementation over an eighteen-month transition period. Therefore each profile needs the provision-specific commencement schedule, not one universal effective date copied from a summary.
  - https://www.pib.gov.in/PressReleasePage.aspx?PRID=2190014&lang=2&reg=3
  - https://www.pib.gov.in/PressReleasePage.aspx?PRID=2294910&lang=1&reg=3
- **China:** the Cyberspace Administration of China published its Provisions on Facilitating and Regulating Cross-Border Data Flows on 22 March 2024. Those rules specify conditions and thresholds for certain transfer mechanisms and exemptions; neither blanket localization of every data class nor unrestricted cross-border transfer is a sound default. The actual data category, parties, activity, and applicable route need review.
  - https://www.cac.gov.cn/2024-03/22/c_1712776611775634.htm
- **United States:** NIST's Privacy Framework remains a useful voluntary risk-management reference, but the NIST page now also identifies a Privacy Framework 1.1 Initial Public Draft. Track the status of the draft accurately and do not represent a voluntary framework as law.
  - https://www.nist.gov/privacy-framework
- **Latin America and the Caribbean:** BIS Paper 153 (7 March 2025) finds that fast-payment systems are expanding, while interoperability, fraud/cybersecurity, end-user and participant fees, and universal access remain important design challenges. Measure those outcomes instead of assuming the availability of a payment API proves inclusion.
  - https://www.bis.org/publications/paper-153-fast-payments-and-financial-inclusion-latin-america-and-caribbean

## Benchmarks: hard safety gates first

All targets below are proposed. Report the exact commit, environment, corpus digest and result. Unit test success must never be presented as deployment security or legal compliance.

| Gate | Proposed acceptance criterion | Required evidence |
|---|---|---|
| Unauthorized restricted-data egress | Zero observed in the complete declared adversarial corpus | Instrument storage, Holochain DHT/gossip, transport, caches, indexes, backups, logs, telemetry and adapters |
| Fail-closed correctness | 100% of seeded unknown, expired, malformed, conflicting and untrusted cases denied for the expected reason | Positive controls run alongside negative controls |
| Bilateral agreement | 100% of source-only or destination-only permission cases denied | Exact profile digests and route matrix recorded in each fixture |
| Profile tamper/replay | 100% of seeded digest/signature, signer rotation/revocation, rollback and replay cases rejected by the verifier + kernel | Production verifier, not the unit-test mock |
| Consent and purpose | 100% of mismatched recipient/purpose, revoked, expired and stale-evidence cases denied | Authoritative consent/revocation adapter and freshness deadline evidence |
| Profile change control | No single untrusted operator can rewrite a high-impact profile or bypass threshold policy | Key compromise, unauthorized signer and rollback tests |
| Local policy latency | Initial p95 < 5 ms for the pure already-verified decision path | Fixed corpus, hardware/toolchain manifest, p50/p95/p99; separate verifier and I/O costs |
| Offline continuity | Demonstrate 72 hours of declared local pilot workflows where safe/appropriate | Reconnect, idempotency, conflict resolution and proof that stale queued cross-boundary actions are not automatically released |
| Recovery | Measure loss, replay, repair and recovery time under process/device failure | Fault injection and repeatable recovery corpus |
| User outcomes | Baseline with pilot partners before setting numeric targets | Time/cost to complete key tasks, access gaps, appeal outcomes, user comprehension, privacy/security incidents |

Never compress these into one overall score that lets a security or rights failure be compensated by better latency or adoption. A single confirmed unauthorized disclosure is a release blocker for the affected data class.

## Qualification ladder: what must happen next

1. **Kernel review and compile:** run formatting, unit tests, and warnings-denied Clippy on the exact PR head. Current code in this branch has not been locally compiled; queued CI is not a pass.
2. **Production profile verifier:** choose and pin canonical digest and signature rules; verify exact bytes and key IDs; define signer trust roots, validity periods, threshold approval, rotation, revocation, recovery, and audit evidence. Algorithm agility must not permit an unreviewed downgrade.
3. **Production consent verifier:** authenticate the precise grant and issuer; query authoritative revocation state or use a bounded evidence freshness window; bind recipient, purpose, validity and action.
4. **Conformance fixtures:** publish valid and invalid profile/consent fixtures with expected stable denial codes, test corpus version, and fixture digests. Include malformed serialization and canonicalization cases.
5. **One real boundary integration:** place the kernel before one concrete persistence/replication/export boundary; instrument the attempted egress paths and prove denial is enforced below the UI.
6. **Adversarial integration:** test DHT/gossip, backups, queues, telemetry, crash reports, cache/index rebuilds, time rollback, offline replay and recovery. If any path bypasses policy, disable the affected data class or transport until repaired.
7. **One co-designed pilot:** choose the actual country, community, data classes, services, legal sources and operators with local partners. Record consent, governance, threat model, accessibility, and outcome baselines before choosing a second region.
8. **Extend through evidence:** add EU, India, China, US-state/sector, and South American country profiles only when source review, profile ownership, trust configuration and repeatable qualification criteria exist.

## Explicit release blockers

- An assertion of physical data residency based only on configuration or self-declared peer location.
- A production verifier that can be replaced by an untrusted implementation or caller-supplied boolean.
- Unknown or expired profile versions silently mapped to a permissive default.
- Essential service or core civic rights gated by an opaque composite reputation/consciousness score.
- A queued cross-border action automatically executing after offline time without revalidating current policy and consent.
- Claims of legal compliance or production readiness based only on compiling code or passing local unit tests.