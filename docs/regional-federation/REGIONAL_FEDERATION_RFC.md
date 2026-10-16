# Regional Sovereign Federation — Architecture RFC

**Status:** Proposed design; research-stage, not an implementation or compliance claim  
**Date:** 2026-10-16  
**Repository:** Public Luminous-Dynamics/mycelix  
**Decision sought:** Adopt one shared federation architecture with jurisdiction/community profiles instead of creating region-specific forks.

## Executive decision

Continue the Africa initiative without resetting or moving its existing work. In parallel, define how Mycelix can support Europe, India, China, the United States, and South America through the same interoperable core.

The intended shape is:

- **One protocol foundation** for identity and credentials, consent, governance, commons coordination, provenance, dispute handling, and safe exchange.
- **Versioned local profiles** for law, governance, languages, accessibility, infrastructure, institutions, and integration adapters.
- **Explicit federation agreements** between participating communities. Cross-border exchange is opt-in and policy-bound, not an automatic consequence of being on the same network.
- **Local control by default:** a community can operate locally, keep sensitive data within its permitted boundary, and decline external dependencies without losing its local records or governance.
- **No compliance theatre:** a profile, test pass, cryptographic proof, or software build does not itself establish legal compliance or official recognition.

The paradigm shift is from a single global service to a **federation of sovereign civic systems**. Interoperability should not require identical institutions, identity systems, currencies, or governance structures.

## Why this needs an architecture change

The public Mycelix tree already includes jurisdiction records, sovereign-civic gating, privacy-preserving eligibility proof work, and bioregional coordination. Those are useful foundations, but they solve different problems:

- A jurisdiction record describes a policy boundary; it does not prove that a specific host or peer actually stores data in that jurisdiction.
- A zero-knowledge proof can hide a claim's underlying value; it does not independently establish that the claim is legally valid, current, or correctly issued.
- A governance gate can determine whether an action is permitted; it cannot prevent a lower layer from gossiping or copying data unless storage, replication, transport, logs, and backups enforce the same boundary.
- A region or language setting is not a legal jurisdiction, and a country code is not enough to model an Indigenous nation, city, bioregion, treaty area, or supranational framework.

**Critical invariant:** restricted data must never enter a replication or telemetry path that cannot enforce its placement and disclosure policy. A shared DHT is not an appropriate home for a record that must remain confined to a smaller trust or jurisdiction boundary. Do not claim residency based only on a peer's self-declared location, IP address, or a profile label.

## Architecture

### Layer A — Common protocol core

Keep the reusable pieces neutral with respect to any single nation's institutions:

1. Agent-controlled identity and credential wallet.
2. Selective-disclosure, audience-bound, expiry-bound claims.
3. Explicit consent and purpose records, including withdrawal and downstream-recipient handling.
4. Governance proposals, auditable decision provenance, appeals, and dispute resolution.
5. Commons workflows for mutual aid, housing, food, water, energy, education, health coordination, and cooperative activity.
6. A content/transport policy interface that can reject disallowed reads, writes, sync, export, backup, and telemetry.
7. Versioned schemas and stable protocol capabilities for cross-region interoperability.
8. A documented trust model for issuers, operators, profile maintainers, and governance authorities.

The core should not encode any region's legal conclusions as universal truths. It should offer explicit policy inputs and enforceable boundaries.

### Layer B — Versioned regional and community profiles

Profiles should be composable, not a single hard-coded region-equals-country switch. Examples include a continental framework, a country, a supranational bloc, a state or province, a recognized Indigenous nation, a municipality, and a bioregional community. Which profile controls which decision must be explicit.

A profile should carry at least:

| Field | Required meaning |
|---|---|
| profile_id, schema_version | Stable namespaced identity and schema compatibility |
| profile_version, content_digest | Immutable version and digest of canonical profile content |
| effective_at, reviewed_at, expires_at | Validity window and review freshness |
| legal_sources | Source URL, issuing authority, title, retrieval date, document/version reference, and content digest where available |
| review_attestations | Which authorized reviewers approved which digest and when; this is not a claim of government certification |
| jurisdiction_scope | Legal scope distinct from user residence, peer location, hosting region, or language |
| data_classification_rules | Data classes, permitted purposes, retention/deletion actions, and disclosure restrictions |
| placement_and_transfer_rules | Allowed storage and processing scopes, export rules, required recipient assurances, and unknown-policy behaviour |
| credential_trust_roots | Accepted issuer identifiers and revocation/freshness rules, without making one identity system mandatory for everyone |
| governance_rules | Local decision authority, quorum/threshold configuration, appeal route, emergency limits, and change-control policy |
| integration_capabilities | Optional, permissioned identity, payment, education, public-service, cooperative, and messaging adapters |
| languages_accessibility_connectivity | Supported languages, accessible journeys, low-bandwidth/offline modes, and device constraints |
| test_manifest | Required positive and negative test cases, fixture digests, test-run evidence, and known gaps |
| maintainer_keys | Authorized profile-update keys, threshold/rotation policy, recovery, and revocation |

This table is a proposed contract, not a claim that those fields already exist in one Rust type. The implementation should use typed Rust data, canonical serialization, signed updates, version pinning, and deterministic validation.

### Layer C — Federation agreements

A federation agreement should identify the two communities, their approved profile digests, the exact data or claim classes that can cross the boundary, purposes, recipient roles, validity window, and dispute/termination process.

- Both sender and recipient must authorize the intended exchange.
- A recipient may reject a claim even when it is cryptographically valid.
- Unknown, unsupported, conflicting, tampered, or expired profile state means **deny the cross-boundary operation**; it must not silently downgrade to a weaker profile.
- When rules conflict, stop and require an explicit, reviewed agreement rather than guessing that one jurisdiction's permissive rule overrides another's.
- Revoking consent can prevent future disclosures and trigger recipient obligations; it cannot magically recall information already observed or copied. The UI must state this honestly.

### Data-placement and replication boundary

Before deployment, map every route by which records can travel: source storage, Holochain DHT/gossip, peer-to-peer messages, caches, indexes, analytics, crash reports, logs, backups, support tooling, and third-party adapters.

For every data class, choose and test one explicit strategy:

1. Public and intentionally global records may use a shared federation layer.
2. Community records use a community-scoped network and membership policy.
3. Personal or sensitive records stay local or in an explicitly authorized private network; share only the minimum necessary claim where possible.
4. Data subject to strict placement or transfer conditions uses a deployment boundary and transport rules capable of enforcing those conditions. If that boundary cannot be demonstrated, disable the affected cross-boundary feature.
5. Encryption keys and access grants must not be replicated more broadly than the data's authorized audience.

Do not place sensitive personal data in a globally replicated DHT and then claim that encryption or a jurisdiction field alone enforces data residency. Minimize sensitive metadata too. Retention and deletion must cover indexes, local caches, backups, and derived artifacts; crypto-erasure is not a universal substitute for legally required deletion.

## Regional design envelopes

These are engineering hypotheses to validate with local partners and qualified reviewers, not blanket descriptions of whole continents or legal advice.

### Africa — continue the existing initiative

**Do not treat Africa as one legal jurisdiction.** Preserve a continental interoperability layer where useful, then use country-specific profiles and community-level governance. The African Union's Digital Trade Protocol sets out goals including interoperable open standards and a secure digital-trade ecosystem; the AU Convention on Cyber Security and Personal Data Protection is another important reference, but neither eliminates the need to check each country's applicable law, implementation, and relevant regional commitments.

Possible capability priorities, to be chosen with communities rather than imposed from outside:

- Offline-first mutual aid and commons workflows for food, water, housing, energy, care, and cooperative production.
- Low-bandwidth operation, safe synchronization, and optional integration with feature-phone/SMS/USSD gateways where local operators and threat models support them.
- Portable, selective-disclosure credentials that do not require one national identity system as a condition of participation.
- Integration adapters for locally used payment/mobile-money systems only through authorized interfaces and with the right financial/legal boundaries.
- Multilingual journeys, community-level consent, transparent governance, and controls against elite capture.
- Country-specific rules for data protection, cross-border transfer, identity, payments, consumer protection, and dispute handling.

**Acceptance criterion:** the Africa work should identify its actual pilot countries and communities, their legal sources and co-design partners, the data that must not leave each boundary, and the outcomes it intends to improve. Avoid claiming a pan-African deployment based on one country profile.

### European Union — rights-aware interoperability

Design around GDPR obligations where applicable and the EU digital rule set without assuming that every country has identical public-service systems. The European Digital Identity Framework (Regulation (EU) 2024/1183) supports wallets, selective disclosure, pseudonyms, transaction transparency, and data portability. The Commission states that member states are to make wallets available by the end of 2026; on this RFC date, verify actual national availability and conformance instead of assuming universal rollout. The Data Act applies from 12 September 2025, with specific provisions and timelines that need to be mapped to each use case.

Engineering priorities:

- Implement credential-wallet adapters to published EU specifications as available, without replacing Mycelix's agent-controlled wallet or making one wallet provider mandatory.
- Purpose limitation, data minimization, access/export/deletion request workflows, consent/permission records where legally appropriate, and traceable disclosure logs that avoid storing excess personal data.
- Multilingual and accessible experiences, cross-border credential verification, and precise separation of EU, EEA, UK, and other European legal profiles.
- Machine-readable processing and data-sharing policies; explicit recipient identity and purpose before release.
- A verified inventory of which features are covered by which obligations. Never label the entire platform GDPR-compliant because one feature passes a test.

### India — consent, language, DPI integration, and phased legal dates

Model the Digital Personal Data Protection Act, 2023 together with the Digital Personal Data Protection Rules, 2025 and the notified commencement schedule. Do not assume every provision took effect on the same date. The profile must pin the current official instruments and review dates.

Engineering priorities:

- Plain-language consent and notices, clear purpose binding, data minimization, accountable processors, and auditable rights workflows mapped to provisions actually in force for the use case.
- Multilingual experiences and low-bandwidth access across a diverse set of regions and languages.
- Optional, permissioned adapters for systems such as UPI, DigiLocker, or other public/private infrastructure, subject to approved integration methods and local review.
- Never require Aadhaar merely because a user is in India; identity should be purpose-specific, proportional, and support alternatives where lawful.
- Support cooperative finance, informal work, learning/credential portability, and local commons with community-led design and clear protections against exclusion.

### China — explicit local operation and carefully scoped exchange

Treat mainland China as a distinct deployment and legal-review profile, not a toggle on a global server. The design must account for the Personal Information Protection Law, Data Security Law, Cybersecurity Law, and current rules/guidance for cross-border data flows. China's 2024 cross-border-data rules adjusted when particular transfer procedures apply; do not simplify them into either "all data must stay in China" or "cross-border transfer is unrestricted."

Engineering priorities:

- Support locally operated deployments with locally administered keys and policy updates, subject to the actual deployment requirements.
- Classify personal information, sensitive personal information, important data, and applicable operator categories before defining transfer behaviour.
- Keep restricted records out of global DHTs and third-party telemetry; use explicit local-to-foreign export gates and recipient/purpose records.
- Permit cross-border exchange only after the applicable transfer route and review are established for the precise data and parties; do not provide features intended to evade lawful controls.
- Do not assume availability of APIs, identity credentials, payment interfaces, or independent review mechanisms. Make each an explicit capability with a documented basis.

### United States — state, federal-sector, and tribal-aware profiles

Avoid presenting the United States as one comprehensive privacy regime. Product obligations may depend on federal sector laws, state privacy laws, state breach/security rules, contracts, and the particular service. Use a current state-law inventory such as NCSL's consumer-privacy database, dated and verified when each profile is released. NIST's voluntary Privacy Framework can support risk-management mapping but is not itself law.

Engineering priorities:

- Keep state-specific policies versioned rather than flattening them into an inaccurate national default.
- Add sector profiles only where a product truly handles that data or service (for example, education, health, children, employment, or financial data).
- Represent tribal nations and other legitimate community authorities as first-class governance scopes where appropriate; do not reduce all community rules to a state selector.
- Build transparent access, correction, export, deletion, consent, data-broker/third-party sharing, and appeal workflows as required by the applicable service rules.
- Integrate banking and payment services through authorized providers; do not turn SAP, TEND, or MYCEL into claims of legal tender or regulated payment functionality by naming them in a profile.

### South America — country profiles within a Latin America and Caribbean federation

South America should not receive one continental legal profile. A practical discovery track can begin with Brazil, if local partners identify a compelling pilot, while keeping the architecture capable of adding other South American countries and Latin America and the Caribbean separately.

Brazil is a useful integration example because its central bank documents Pix and Open Finance, including payment initiation through participating institutions. That does not make Pix a regional payment rail or imply that every institution/API is open to Mycelix. The Inter-American Development Bank and BIS research on regional instant payments emphasizes differing national experiences and recurring issues such as interoperability, fees, fraud, cybersecurity, and universal access.

Engineering priorities:

- Country-specific legal/data-protection profiles (starting with Brazil's LGPD where applicable) and explicit cross-border data-flow rules.
- Portuguese and Spanish as practical initial language tracks, with Indigenous and local languages included through partner-led priorities.
- Local integration adapters for regulated payment rails and cooperatives; no platform-controlled custody or payment execution by default.
- Offline/low-bandwidth operation, migrant/remittance and small-enterprise workflows, cooperative supply chains, and cross-border credential verification.
- Measure real costs, access, fraud, and inclusion rather than assuming that a successful payments API equals a successful civic system.

## Cross-region comparison

| Dimension | What stays common | What changes by profile |
|---|---|---|
| Identity | Agent-controlled credentials; selective disclosure; key recovery and revocation | Accepted issuers, assurance levels, local wallet/DPI integrations |
| Data privacy | Data minimization, purpose-bound access, transparent logs, safe defaults | Legal bases, deadlines, rights workflows, retention and transfer restrictions |
| Hosting and sync | Typed placement policy enforced at real I/O boundaries | Approved hosting scopes, replication topology, backup and transfer rules |
| Governance | Provenance, explicit authority, appeals, bounded emergency powers | Recognized decision-makers, quorum, local institutions, dispute venues |
| Economics | Common accounting interfaces and auditable records | Legal currency, taxes, licensed payment providers, local accounting and consumer rules |
| Commons | Shared workflows and interoperable records | Local priorities, land/water rules, cooperative forms, service providers |
| Language/access | Accessible APIs and internationalization | Local languages, literacy, disability needs, device/connectivity assumptions |
| External exchange | Explicit bilateral agreement and minimal claims | Permitted data classes, recognized issuers, local legal review, connector capability |

## Fail-closed implementation contract

An eventual Rust crate (candidate name: mycelix-region-policy; name subject to repository conventions) should expose a pure, deterministic policy decision layer. It should not perform network I/O or invent jurisdiction facts.

Required decision inputs:

- Exact profile ID, version and digest.
- Requested action (read, write, replicate, export, backup, telemetry, disclosure, or delete).
- Data class and content/record identity.
- Purpose and recipient/operator identity.
- Hosting, storage, processing, and destination scopes with their evidence and assurance levels.
- Consent/permission state if relevant.
- Explicit evaluation time and evidence validity windows.
- Federation agreement and its exact profile digests for cross-boundary exchange.

Required behaviour:

1. Reject malformed profiles, unrecognized policy versions, invalid signatures, invalid threshold policy, conflicting constraints, stale evidence, expired profiles, and unresolved boundaries.
2. Enforce hard eligibility and data-placement constraints before any preference or optimization ranking. A latency/cost/reliability score must never override a policy denial.
3. Treat self-asserted peer location or self-labeled assurance as unverified. The crate must not mint independent assurance or prove a peer's physical location merely by accepting a string field.
4. Bind any proof or attestation to the exact subject, action, destination, policy digest, and validity window; prevent replay in the consuming layer.
5. Return a structured decision: allow/deny/needs-review, stable reason codes, policy/profile digests, and minimal audit evidence. Avoid leaking sensitive data in the denial result.
6. Keep Symthaea or any other optimizer advisory. A deterministic gate and the authorized governance process retain the final authority; optimization cannot waive hard policy.
7. Re-evaluate at every trust boundary. A UI check is not an authorization boundary.
8. Make the profile-update path threshold-authorized, replay-resistant, auditable, and recoverable, without one global administrator being able to rewrite every region's rules.

## Benchmark and acceptance plan

These are proposed acceptance targets for the first measurable implementation, not claims about current code or product performance.

### Must-pass safety and correctness gates

- **Zero unauthorized restricted-data egress** in a deterministic adversarial test suite covering local storage, DHT/gossip, peer messaging, backup, logs, analytics, and adapter calls.
- **Zero fail-open policy cases** for unknown profiles, stale profiles, invalid signatures, missing destination evidence, malformed validity windows, conflicting jurisdiction rules, or unsupported schema versions.
- **No plaintext personal/sensitive records** in globally shared DHT fixtures, public telemetry, or diagnostic logs.
- **100% rejection of seeded tampering/replay cases** for profile digest, profile signature, consent state, recipient, action, destination, and expiry binding.
- **No single operator key** can make a high-impact profile change or silently disable a hard policy without the profile's explicit, reviewed threshold process.
- **Recipient-scoped consent:** withdrawing a grant blocks new authorized reads/disclosures under that grant and records downstream notification obligations. Tests must not claim to recall copies already legitimately disclosed.
- **Negative controls stay negative:** a test must fail for the expected reason when a restricted destination, expired authority, or untrusted issuer is deliberately supplied.
- **Exact-head evidence:** every qualified run records commit SHA, profile digest, test corpus/version, toolchain, and result. Queued or skipped jobs are not passes.

### Performance and resilience measurements

- Benchmark policy evaluation separately from cryptographic verification, disk operations, and network calls. Initial target: **p95 under 5 ms** for local pure policy evaluation over a fixed, published corpus on the reference host; revise only with measured evidence.
- Exercise reconnect and synchronization after **72 hours offline** for a pilot's declared local workflows; verify idempotency, bounded queues, conflict handling, and preservation of local authority.
- Keep profile bundles small enough for low-bandwidth distribution; measure actual cold-cache download size and upgrade/recovery time rather than guessing.
- Measure end-to-end credential verification, safe sync, local record creation, and policy-update propagation at defined dataset sizes and device classes. Publish median/p95/p99, peak memory, CPU, energy where practical, failure counts, and environment manifest.
- Conduct fault injection for dropped connections, stale clocks, key rotation, unavailable authority services, damaged caches, and partial profile rollout.
- Test migration from an older profile without silently changing historical decision evidence; decisions must be reproducible against their pinned profile digest.

### User and community outcome metrics

Technical safety is necessary but not sufficient. With local partners, establish baseline and target measurements for:

- Time and out-of-pocket cost to join a cooperative or access a mutual-aid service.
- Credential acceptance across participating services without unnecessary personal-data disclosure.
- Time to complete an appeal, dispute, data export, correction, or applicable deletion request.
- Successful completion of essential tasks under intermittent connectivity and on low-cost devices.
- Access and completion gaps by language, disability/accessibility needs, and connectivity—measured with consent, aggregation, and privacy safeguards.
- Local operator independence, successful key/profile recovery, and ability to continue core activities when global infrastructure is unavailable.

Do not publish an aggregate "regional success score" that hides safety failures, erases local variation, or treats privacy and rights as a weighted trade-off.

## Rollout order

1. **Inventory and preserve the Africa work.** Record its actual target countries/communities, current code/doc locations, open design decisions, partners, and evidence. Do not overwrite, relocate, or fork it as part of this RFC.
2. **Define and review the profile contract.** Produce typed Rust models and a schema/version test suite; separate legal jurisdiction, governance scope, hosting region, and user-selected locale.
3. **Build the storage/replication boundary first.** Demonstrate that the transport and persistence topology can enforce the policy. Do not start cross-region personal-data federation until negative tests prove the boundary.
4. **Complete one partner-backed Africa country pilot.** Validate usability, policy sources, local governance, offline workflows, and threat model. Share only the claims needed by an expressly agreed second community.
5. **Develop EU and India profiles as parallel design tracks** once the generic contract is stable; each needs independent review of exact scope and effective dates.
6. **Develop South American country profiles** with Brazil as a possible candidate when partners and integration permission exist; then extend by demand.
7. **Develop state/sector/community profiles for the United States** rather than shipping one blanket national profile.
8. **Develop the China deployment profile** with appropriate local legal and deployment expertise. Keep cross-border exchange disabled until the specific lawful route, data class, parties, and safeguards are reviewed.
9. **Add further jurisdictions only when a partner, profile owner, source set, and repeatable qualification suite exist.** The goal is not the largest number of region labels; it is trustworthy local operation plus proven, limited interoperability.

## Open questions that require local ownership

- Who is authorized to maintain, review, sign, rotate, and revoke a profile?
- Which laws and institutional rules apply to this specific service, data, operator, recipient, and deployment?
- Which records can be global, community-scoped, local-only, or exported only as minimal claims?
- What independent evidence demonstrates hosting/processing boundaries, as distinct from a peer's self-assertion?
- What are the local threat model, accessibility requirements, languages, connectivity limits, dispute routes, and community-defined measures of success?
- Which integrations are legally and technically authorized, and what happens when their provider changes terms, API access, or availability?

## Primary research references

The following official sources anchor the first research pass. They are starting points for profile review, not exhaustive legal inventories. Review each source and its amendments before implementing a jurisdiction's rules.

- African Union, Protocol to the AfCFTA Agreement on Digital Trade: https://www.au.int/en/treaties/protocol-agreement-establishing-african-continental-free-trade-area-digital-trade-0
- African Union, Convention on Cyber Security and Personal Data Protection: https://www.au.int/en/treaties/african-union-convention-cyber-security-and-personal-data-protection
- European Commission, Principles of the GDPR: https://commission.europa.eu/law/law-topic/data-protection/information-business-and-organisations/principles-gdpr_en
- EU Regulation 2024/1183, European Digital Identity Framework: https://eur-lex.europa.eu/eli/reg/2024/1183/oj
- EU Regulation 2023/2854, Data Act: https://eur-lex.europa.eu/eli/reg/2023/2854/oj
- European Commission, European Digital Identity: https://commission.europa.eu/topics/digital-economy-and-society/european-digital-identity_en
- Ministry of Electronics and Information Technology, India, DPDP Rules 2025 and related notices: https://www.meity.gov.in/documents/act-and-policies/digital-personal-data-protection-rules-2025-gDOxUjMtQWa?pageTitle=Digit
- Cyberspace Administration of China, Provisions on Facilitating and Regulating Cross-Border Data Flows (22 March 2024): https://www.cac.gov.cn/2024-03/22/c_1712776611775634.htm
- Supreme People's Procuratorate of China, Personal Information Protection Law (English text): https://en.spp.gov.cn/2021-12/29/c_948419_2.htm
- National Conference of State Legislatures, Consumer Privacy Legislation Database (updated 15 September 2026): https://www.ncsl.org/technology-and-communication/consumer-privacy-legislation-database
- NIST, Privacy Framework and profile approach: https://www.nist.gov/privacy-framework
- Banco Central do Brasil, Pix information: https://www.bcb.gov.br/meubc/faqs/s/pix
- Banco Central do Brasil, Open Finance and Pix payment initiation: https://www.bcb.gov.br/detalhenoticia/20214/noticia
- Inter-American Development Bank, Beyond Cash: The Digital Payments Revolution in Latin America and the Caribbean (September 2025): https://publications.iadb.org/en/beyond-cash-digital-payments-revolution-latin-america-and-caribbean
- Bank for International Settlements, Fast payments and financial inclusion in Latin America and the Caribbean (7 March 2025): https://www.bis.org/publications/paper-153-fast-payments-and-financial-inclusion-latin-america-and-caribbean

**Release rule:** no regional profile may be described as legally compliant, production-ready, or fully qualified until its scope, sources, reviewers, deployment boundary, negative controls, and exact-head evidence have all been recorded and checked.