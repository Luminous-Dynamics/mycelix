# Portable sovereign interoperability: lessons for regions and nations

**Research snapshot:** 2026-10-14  
**Status:** Strategy proposal. This document does not claim endorsement, procurement interest, or production interoperability from any government or regional organization.

## Conclusion

**Yes, other regions may find this model useful—but the offer should be a portable, open interoperability toolkit, not a competing identity scheme.** Different regions already recognize the problem of cross-system interoperability. Their legal instruments, architectures and deployments differ, so the correct opportunity is to reuse the method and conformance tooling while adopting the relevant region's own framework and each country's official contract.

Three useful reference cases show different stages of the problem:

- **European Union:** the European Digital Identity Wallet programme has a common Architecture and Reference Framework (ARF), implementing specifications and an open-source reference implementation. The Commission continues publishing implementing acts and specifications in 2026. The lesson is disciplined common protocol and conformance work across national implementations—not that Mycelix should replace the EU's ARF or become an EU trust authority. [European Commission: EUDI Wallet](https://commission.europa.eu/topics/digital-economy-and-society/european-digital-identity_en) · [EUDI documents and implementing rules](https://commission.europa.eu/publications/european-digital-identity-documents_en) · [2026 implementing acts index](https://interoperable-europe.ec.europa.eu/collection/assessments/report/2175)
- **ASEAN:** its Digital Master Plan 2030 explicitly identifies cross-border digital ID interoperability, open data standards and secure cross-border digital services as objectives. That is evidence of a strategic priority, not proof that every member has a production-compatible verifier. Potential contribution: country-specific conformance, fallback and trust-lifecycle tooling aligned with ASEAN's own work. [ASEAN Digital Master Plan 2030](https://asean.org/wp-content/uploads/2026/01/ASEAN-Digital-Master-Plan-2030-final-2026.pdf)
- **Latin America and the Caribbean:** the Inter-American Development Bank reports that IdLAC, a regional public-good identity broker, demonstrated a cross-border migration transaction involving national identity systems in Uruguay, Colombia and Brazil and a Chilean service; additional countries have joined its development. The lesson is to investigate collaboration, reuse and test interoperation with existing regional public goods before creating a parallel broker. IdLAC is a potential model or partner, not a target to displace. [IDB: IdLAC and digital integration in Latin America and the Caribbean](https://www.iadb.org/en/news/idb-drives-digital-integration-latin-america-and-caribbean)

The OECD's 2026 Digital Government Outlook makes a related point: technical standards are necessary but cross-border exchange also needs institutional trust, mutual recognition, liability arrangements, privacy and security safeguards, and adoption incentives. Its 2024 DPI paper likewise treats governance, inclusion, public-private collaboration, privacy and security as core design issues. [OECD Digital Government Outlook 2026](https://www.oecd.org/en/publications/digital-government-outlook_0496b2bc-en/full-report/strengthening-digital-public-infrastructure-and-data-governance_2c7323c7.html) · [OECD DPI policy paper](https://www.oecd.org/en/publications/digital-public-infrastructure-for-digital-governments_ff525dc8-en.html)

## What should be portable—and what must not be

Build a small, jurisdiction-neutral core that implements reusable safety and evidence rules. Add region profiles and national profiles separately.

### Portable core

- Typed, versioned evidence outcomes: verified within the permitted scope, rejected, or indeterminate.
- Exact issuer/trust-metadata provenance, key lifecycle and protocol version pinning.
- Holder binding, audience, nonce/replay, expiry, status and revocation checks when the official profile defines them.
- Explicit service purpose and lawful-basis fields, data minimization, retention/deletion and allowed output claims.
- Separate identity proof from entitlement, eligibility, organizational authority, membership or execution permission.
- Short-lived, purpose-bound receipts; no raw credential, biometrics, national ID number or replayable presentation in a public distributed ledger.
- Offline, assisted and non-digital fallback; accessible and localized services.
- Open conformance corpus, positive and negative fixtures, exact source/release SHA, independent review and reproducible evidence.
- Vendor-neutral adapters and a clear exit/migration path.

### Regional and national profiles

A **region profile** records the regional framework's semantics, trust model, common protocol specifications, mutual-recognition arrangements and dispute/incident responsibilities. It does not itself authorize national data processing.

A **national profile** pins domestic law and authority, official issuer and trust metadata, supported protocol versions, relying-party rules, permitted purposes and claims, status/key lifecycle, privacy controller, correction/appeal routes and operational terms.

A **service profile** states why a relying party needs a particular claim, which legal basis applies, which disclosure is necessary, who can access it, how long outputs are retained, and how people contest errors.

Do not infer that a credential accepted in one country is legally equivalent to a credential in another. Legal equivalence and mutual recognition require decisions by competent authorities and the relevant legal instruments.

## A stricter readiness model: gates, not a single score

A weighted score is useful for prioritizing research but dangerous as a release decision: strong documentation must not compensate for missing legal authority or unresolved key-compromise handling. Separate **readiness scoring** from **non-compensatory release gates**.

Use statuses:

- **not_assessed:** evidence collection has not begun.
- **research_only:** official sources are being studied; no integration permission is implied.
- **partner_review:** a named authority/partner is reviewing a concrete scope; still no live access unless expressly granted.
- **sandbox_authorized:** written authorization and sandbox scope exist; only the named conformance activities are allowed.
- **production_authorized:** all required legal, technical, privacy, security, operational, accessibility and human release gates pass for the exact version/commit and written scope.

For a production release, require every gate to pass independently: competent legal authority and signed permission; official protocol and trust metadata; key rotation/status/revocation; holder binding and anti-replay; purpose and minimum disclosure; data-protection impact and retention/erasure; offline/assisted access; localization/accessibility; independent threat review; positive and negative conformance; incident/correction/grievance process; service-level objectives and operator; dependency/supply-chain review; approved rollback/kill switch; and human sign-off over the exact release artifact.

If any gate is unknown, unreviewed or failed, production remains disabled. Never convert unknown to false, a warning to a pass, a queued CI run to success, or a report hash into a trusted signature.

## Standard adapter interfaces

Use explicit artifacts rather than one oversized "universal identity" schema:

1. **AuthorityProfile:** legal authority, jurisdiction, source evidence, accountable controller, contact and governance.
2. **ProtocolProfile:** exact protocol/version, issuer identifiers, keys/trust roots, status semantics and conformance suite.
3. **PurposeGrant:** relying party, lawful purpose, permitted claims, audience, expiry, allowed onward processing and revocation.
4. **AssuranceMapping:** how a source assurance level maps to a specific service requirement, what cannot be inferred, and which authorized human approved the mapping.
5. **FallbackProfile:** offline/assisted/non-digital path, service parity, accessible languages and outages.
6. **OperationalAgreement:** authorization scope, service level, liability, incident escalation, audit access, retention, costs and termination.
7. **ConformanceBundle:** test corpus, positive/negative results, exact commits, source hashes, signed release attestation and test runner identity where available.

The region and nation choose whether to implement these artifacts; the core only validates that required information and gates are present. Protocol translation must not silently increase assurance or broaden purpose.

## How to position the model for adoption

### Offer interoperability as public infrastructure

Publish the schema, validators, test vectors, local deployment architecture, threat model and governance templates under a permissive open-source licence. Avoid a vendor-controlled central register, proprietary trust root or mandatory cloud service. Provide Rust-first reference libraries and auditable CLI tooling, plus documented bindings where public partners require them.

### Build alongside—not over—existing regional efforts

- **EU:** map conformance requirements to the published EUDI ARF and implementing rules; participate in standards/conformance work when eligible; never present a Mycelix profile as equivalent to an EUDI wallet.
- **ASEAN:** align to its relevant regional objectives and each member's actual national rules; start with one willing service owner and an authorized test environment.
- **Latin America/Caribbean:** study IdLAC's protocol and governance model; ask whether an independent adapter or evaluation tool would help. Do not build a parallel identity broker before understanding the existing regional public-good work.
- **Africa:** align terminology and future pilots with AU digital ID interoperability, AU data policy, AfCFTA digital trade and Smart Africa work. Keep national profiles sovereign and route-specific.
- **Other federations or bilateral groups:** reuse the same core only after mapping their legal instruments, trust model, liability and recognition arrangements.

### Make a partner package, not just code

For each prospective partner, provide a bounded discovery package: system/authority map, published-source register, legal/privacy questions for local counsel, threat model, non-functional requirements, synthetic conformance harness, offline/fallback design, cost model, and a go/no-go memo. This is a proposed engagement shape, not a promise of duration or a claim any partner has accepted it.

Make every proposal answer five questions:

1. Which real public or private service problem is being fixed?
2. Which existing regional/national standard does it align to, and which gaps remain?
3. What data is deliberately **not** collected or exchanged?
4. Who owns operation, liability, funding and user support locally?
5. How can the partner reject, fork, leave or replace the software without losing records or trust?

## Improve the model beyond identity

Digital public infrastructure is more than identity. A portable trust/contract layer can also support consentful data exchange, business/cooperative credentials, licensing evidence, permits, verified service-provider claims, digital signatures, public procurement, payments and portable professional credentials—each with its own authority and release gates.

This makes the toolkit useful even when no government identity endpoint is available. Start with public-service navigation, community maintenance, public asset/resilience mapping, open climate/agriculture knowledge and local planning. These can deliver value from public or consented data without creating population-level identity dependencies.

For Symthaea, Mycelix, Sol-Atlas and Symtropy:
- Symthaea: source-linked multilingual, uncertainty-calibrated decision support, not final legal or benefits authority.
- Mycelix: consent, permissions, adapter evidence and governance, not a continental identity database.
- Sol-Atlas: local-first infrastructure/resource geospatial layers with provenance, sensitivity labels and correction paths.
- Symtropy: transparent scenario comparisons and participatory coordination with explicit assumptions and distributional impacts.

## Measuring whether it is genuinely adoptable

Measure:
- time and cost saved in a full service journey, not just successful authentication;
- adoption and conformance effort for an independently implemented adapter;
- data minimization, retention and deletion compliance;
- false accept/reject/indeterminate outcomes and subgroup/accessibility disparities;
- offline and assisted-route completion parity;
- correction/appeal success and incident response time;
- uptime, recovery, local operability and migration cost;
- local staff trained and local procurement/control retained;
- ease of forking/replacing the software and lack of lock-in.

Do not use "countries onboarded" as the primary success metric. A single safe, authorized service that reduces cost and exclusion may be more valuable than ten logo partnerships without production use.

## Immediate implementation priorities in Mycelix

1. Keep the African DPI invariant contract and country research matrix; they are a regional example, not the global schema.
2. **Implemented in this PR, awaiting exact-head CI:** a jurisdiction-neutral [portable adapter profile template](../mycelix-identity/PORTABLE_ADAPTER_PROFILE_TEMPLATE_V1.json), [closed-schema validator](../mycelix-identity/validate_portable_adapter_profile.py), and [21 adversarial tests](../mycelix-identity/tests/test_portable_adapter_profile.py). This is a research-only template, not an authorized adapter.
3. Add cross-region reference mappings as research notes only. Do not hard-code EU/ASEAN/IdLAC status into production trust settings.
4. Preserve exact-head evidence: contract tests must pass on the exact commit before claiming implementation validation.
5. **Implemented in this PR, awaiting exact-head CI:** run the [18-case synthetic service journey](../mycelix-identity/SYNTHETIC_SERVICE_JOURNEY_V1.json) through the [deterministic evaluator](../mycelix-identity/synthetic_service_journey.py) and its [20-method adversarial suite](../mycelix-identity/tests/test_synthetic_service_journey.py). It covers purpose-grant binding, minimum claim disclosure, verifier outage/unknown-state fallback, replay rejection, identity-versus-entitlement boundaries, and ephemeral appeal receipts. It is not a credential verifier or durable case-management service.
6. **Correction/appeal lifecycle contract added:** the [machine-readable case contract](../mycelix-identity/APPEAL_CASE_CONTRACT_V1.json), [in-memory state machine](../mycelix-identity/appeal_case_engine.py), and [29 adversarial tests](../mycelix-identity/tests/test_appeal_case_engine.py) model requester ownership, assigned steward, independent appeal reviewer, scoped audit access, retention-policy review, and tamper-evident event chaining. Durable storage and public-DHT publication remain disabled. Expiration is only a lifecycle state: actual deletion and backup-deletion must be performed and verified by a separately reviewed storage adapter.
6. Treat regional organizations as potential partners/standard-setters, not customers who have already expressed demand. Ask for review, publish gaps and listen.

## Sources

- European Commission, [European Digital Identity Wallet](https://commission.europa.eu/topics/digital-economy-and-society/european-digital-identity_en), [implementing documents](https://commission.europa.eu/publications/european-digital-identity-documents_en), and [2026 implementing acts index](https://interoperable-europe.ec.europa.eu/collection/assessments/report/2175).
- ASEAN, [Digital Master Plan 2030](https://asean.org/wp-content/uploads/2026/01/ASEAN-Digital-Master-Plan-2030-final-2026.pdf).
- Inter-American Development Bank, [IdLAC regional digital identity integration](https://www.iadb.org/en/news/idb-drives-digital-integration-latin-america-and-caribbean).
- OECD, [Digital Government Outlook 2026: strengthening DPI and data governance](https://www.oecd.org/en/publications/digital-government-outlook_0496b2bc-en/full-report/strengthening-digital-public-infrastructure-and-data-governance_2c7323c7.html) and [Digital public infrastructure for digital governments](https://www.oecd.org/en/publications/digital-public-infrastructure-for-digital-governments_ff525dc8-en.html).
- World Bank ID4D, [Principles on Identification for Sustainable Development](https://id4d.worldbank.org/guide/1-principles).
- African Union, [Digital ID Interoperability Framework](https://au.int/en/documents/20231211/au-interoperability-framework-digital-id), [Data Policy Framework](https://www.au.int/en/documents/20220728/au-data-policy-framework), and [AfCFTA Digital Trade Protocol](https://www.au.int/en/treaties/protocol-agreement-establishing-african-continental-free-trade-area-digital-trade).
