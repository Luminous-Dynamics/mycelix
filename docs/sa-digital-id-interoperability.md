# South African Digital ID interoperability profile for Mycelix

**Profile ID:** SA-DID-001  
**Research snapshot:** 2026-10-13  
**Status:** Design proposal; no DHA integration or certification is claimed  
**Tracked by:** [Mycelix issue #4990](https://github.com/Luminous-Dynamics/mycelix/issues/4990)

## Executive decision

Mycelix should prepare an adapter for South Africa's Department of Home Affairs (DHA) smartphone-based Digital ID, but must not claim direct integration until DHA publishes or authorizes the relying-party protocol, trust metadata, credential status method, and onboarding process.

This is timely. In an address dated 1 October 2026, the Minister of Home Affairs announced a working Digital ID prototype, demonstrating a real credential issued from the Population Register and bound to a smartphone chip. The address also said DHA was processing comments on draft Digital ID regulations before finalising the rules. The system is intended to be voluntary and coexist with the physical Smart ID card.

Official sources:
- DHA / South African Government, 1 October 2026, *Home Affairs Quantum Leap: A Vision for the Future of Government*: https://www.gov.za/news/speeches/minister-leon-schreiber-home-affairs-quantum-leap-vision-future-government-01-oct
- DHA / South African Government, 5 May 2026, draft Digital Identity regulations for public comment: https://www.gov.za/news/media-statements/minister-leon-schreiber-invites-public-comment-draft-digital-identity

The public materials reviewed for this profile do not disclose a third-party developer API, supported credential wire format, verifier certification process, key/trust-metadata endpoint, status/revocation protocol, or relying-party onboarding procedure. This is a finding about the reviewed public materials, not proof that no private technical documentation exists.

**Recommendation:** implement a protocol-neutral trust-evidence boundary and deterministic synthetic tests now. Add a real protocol adapter only after DHA's official profile and terms are reviewed. Do not use scraping, undocumented endpoints, reverse-engineered app internals, screen scraping, or unofficial biometric checks as substitutes.


## Additional official roadmap evidence: prototype is not the production verifier contract

The DHA's *Annual Performance Plan 2026/27* gives a useful implementation-phase distinction:

- The 2026/27 annual target for “Digital identity operational” is specifically **completion of hosting infrastructure for digital identity and PKI**, with a Q4 sign-off milestone.
- Its medium-term table separately places “Digital identity operational (verifiable credentials issued via digital wallet)” in 2027/28, followed by additional agreed credentials in 2028/29.
- The plan describes the PKI, certificate-authority, credential, wallet and service-consumption components at a programme level. It does not publish a Mycelix-ready relying-party protocol, a verifier SDK, issuer trust metadata endpoint or a concrete credential/status wire contract in the reviewed material.

Official source: [DHA Annual Performance Plan 2026/27, Digital Identity section, printed pp. 62–63 and 122](https://www.dha.gov.za/images/AnnualReports/DHA-APP-2026-27_260326.pdf).

This makes the safest interpretation more precise: the minister's October 2026 prototype announcement is evidence of prototype progress, but it is not evidence that the production wallet/verifier ecosystem or third-party access contract is ready. Programme targets are plans, not proof of completion. Continue to track official releases and written DHA partner guidance; do not infer interoperability from the existence of PKI or verifiable-credential terminology.

### Historical implementation risk: hosting and PKI governance

A useful historical qualification appears in Parliament's *Announcements, Tablings and Committee Reports*, published 8 May 2026. Its Home Affairs committee assessment of the 2025/26 third-quarter reporting period says the Digital Identity and PKI programme target was not achieved in that period: procurement had been paused while the Department clarified the revised hosting/governance model, including the proposed SARS-based hosting approach and Certificate Authority responsibilities. The assessment says PKI procurement was intended to resume and be recovered in 2026/27.

Source: [Parliament committee report, printed p. 121](https://www.parliament.gov.za/storage/app/media/Docs/atc/01ls62wghipk7yjq7tmng272dk6bqkozmm.pdf).

This is a **historical delivery-risk signal for the 2025/26 reporting period**, not evidence that the October 2026 working prototype did not exist and not proof of the present completion status. It reinforces why Mycelix must not guess at the trust anchor, hosting authority, certificate authority, issuer metadata source, or status endpoints. An eventual partner specification must make the operational owner and trust governance explicit, in addition to describing the credential format.

## Executable synthetic policy oracle

The machine-readable profile now includes **20** explicit synthetic policy scenarios, including invalid proof, stale credential status, unknown validity time, and unsupported algorithm/profile downgrade, in addition to issuer spoofing, replay, wrong audience, holder binding, expiry/revocation, privacy leakage, and identity-to-authority escalation. `mycelix-identity/sa_did_policy_engine.py` evaluates the inputs with deterministic ordering across consent, alternate access, source availability, profile support, issuer trust, proof, schema, audience, nonce, holder binding, validity, status, receipt privacy and authorization scope.

This oracle is deliberately **not a credential verifier**. It performs no parsing, signature checks, network calls, trust-key retrieval or DHA interaction. The only positive result is a synthetic test-corpus disposition, and the engine refuses vectors not explicitly marked synthetic and ineligible for production. Its value is that CI executes the policy logic and compares exact expected disposition/reason pairs, rather than merely checking that a JSON file contains plausible labels.

The focused `.github/workflows/sa-did-contract.yml` workflow runs the Python-only contract and policy checks independently of the monolithic CI change-detection dependency. Both this focused workflow and the identity job in the root CI workflow explicitly check out `github.event.pull_request.head.sha` (or `github.sha` outside a pull request), matching the commit SHA written into the report. This makes the declared tested revision unambiguous at the workflow configuration level; confirm the actual workflow run and artifact before treating the report as execution evidence.

### Evidence fingerprints and report limitations

Report schema v3 records the exact source commit SHA supplied by CI, plus SHA-256 fingerprints for the exact profile bytes read by the CLI, normalized profile representation, and source files of both the policy engine and contract validator. Repeated runs over the same files and source commit should produce byte-identical JSON reports. These identifiers let a reviewer compare a report to its declared input/code, but they are **not** a digital signature and do not independently prove that a trusted CI runner generated it. CI artifact retention is time-limited; preserve the report together with the workflow run when it is needed as evidence.

## Near-term official path: the existing Online Verification Service

The smartphone-based Digital ID is not the same thing as DHA's existing Online Verification Service (OVS). OVS is a separate, existing server-to-server service that registered third parties can use to verify client identity information against the National Population Register.

The Department's public notice of 23 June 2025 says organisations wishing to connect should email **Verifications@dha.gov.za**. The 14 April 2026 government statement says the fee correction took effect on 1 July 2025: R10 per real-time private-sector verification and R1 per verification field request for off-peak batch verification. The same statement reports improved uptime and response time after system upgrades. These are official published figures and access instructions, not a guarantee that Mycelix is eligible or will be approved.

Sources:
- DHA / South African Government, 23 June 2025, OVS access process and fees: https://www.gov.za/news/media-statements/home-affairs-corrects-unsustainable-under-pricing-national-population
- DHA / South African Government, 14 April 2026, OVS operating update: https://www.gov.za/news/media-statements/minister-leon-schreiber-welcomes-withdrawal-act-court-application-against

### Recommended two-path strategy

1. **Pursue OVS onboarding now, conditionally.** Contact DHA through the published channel to establish whether the legal entity/operator, intended use, and proposed data processing are eligible. Request the official integration specification, supported fields and result codes, authentication and network security requirements, test environment, rate limits, fee/billing rules, terms, audit requirements, allowed purposes and retention constraints. Do not assume access just because the public notice invites organisations to enquire.
2. **Prepare the smartphone Digital ID adapter independently.** The wallet-based flow should be user-mediated and claim-minimising; it must be selected according to the official profile that DHA ultimately publishes or authorizes.
3. **Normalize outcomes without erasing assurance differences.** A server-side OVS result and a user-presented wallet credential must carry different method/profile identifiers and assurance semantics. They may enter a common higher-level policy interface, but they are not interchangeable evidence.
4. **Never cache more than the approved contract allows.** For OVS, minimise request fields, restrict operator access, avoid logging raw identifiers, and retain only the smallest receipt allowed by the DHA agreement and lawful purpose. For wallet presentations, avoid storing the full credential or reusable presentation token.
5. **Keep optionality and alternatives.** Do not make official identity verification mandatory for low-risk community participation or for reporting outages. Never make digital proof the only path to essential service access.

OVS is an immediate potential path for approved institutional verification. It is not a public drop-in API, not a credential held by the user, not a substitute for the Digital ID wallet, and not evidence that Mycelix has been authorized by DHA.


## The right identity boundaries

Three entities must remain distinct:

1. **DHA Digital ID** — a national credential and its government-issued claims, with DHA as the authority for claims it issues.
2. **Mycelix identity** — a user-controlled application/community identity, using the existing did:mycelix system.
3. **Relying-party authorization** — a separate local policy decision about what the person may access or do in one particular service.

A successful verification of a DHA-issued claim is not a Mycelix DID, proof of Mycelix membership, a reputation score, a voting entitlement, a financial eligibility determination, or authority to execute an action. Those decisions must remain separate and be justified by the policy of the relevant service.

A Mycelix DID should never be derived from a South African identity number. A public, permanent mapping between a government identifier and a Mycelix DID would create a powerful cross-service correlation handle and is not needed for most use cases.

## High-value use cases

Start with low-risk, low-data use cases that need only a narrow eligibility or identity assertion. Examples to investigate with willing service providers:

- prove that a user is an adult, where age is genuinely relevant and the DHA profile supports an authenticated age predicate;
- prove that a person has completed an identity-verification step for one named service, without sharing a complete identity document;
- prove a limited claim needed to open a user-controlled account or receive a specific service, while retaining an accessible non-digital alternative;
- let a user link a national identity proof to a Mycelix account by explicit consent without publishing the link on the DHT.

These are candidate uses, not claims that the current DHA prototype already exposes these particular predicates to third-party verifiers. Do not start with welfare scoring, person-level reputation, biometric identification, automated eligibility denial, high-impact financial decisions, or public service voting.

## Target architecture

### User-mediated trust flow

1. The relying service declares the exact claim it needs, purpose, policy, recipient, and retention period.
2. The user is shown what will be shared and can approve or decline. The interface must not silently turn a service interaction into a broad, reusable identity disclosure.
3. If DHA publishes/authorizes an interoperable protocol, the official Digital ID holder flow presents the supported credential or proof. Mycelix does not collect or match biometric templates.
4. A dedicated adapter validates the exact approved profile: issuer trust, signature/proof, format/schema, holder binding, verifier/audience and transaction nonce, validity period, status/currentness, and claim scope.
5. The verifier returns VERIFIED, REJECTED, or INDETERMINATE with typed reason codes. Missing trust metadata, unsupported format, unavailable status service, stale status, or a malformed response never defaults to VERIFIED.
6. A short-lived, purpose-scoped receipt records the result needed by the relying service. It must not contain the full credential, national identity number, biometric data, reusable presentation token, or full claim payload unless a specific, reviewed lawful need requires it.
7. Mycelix's application policy separately decides whether that evidence satisfies a given requirement. It must record its policy version and not infer powers the proof does not establish.

### Suggested component boundary

DHA official holder / trust interface → approved protocol adapter → profile validator → minimum-disclosure verification receipt → application-specific authorization policy

The adapter should be replaceable. The canonical Mycelix identity, credential schema, revocation, recovery and identity-resolution mechanisms remain authoritative for their own semantics. Do not fork the identity system to accommodate a single country-specific credential.

### Profile registry

Create a versioned issuer/profile registry with production profiles disabled by default. Each profile must state:

- exact issuer identity and authoritative trust-anchor provenance;
- exact specification/profile version and permitted credential formats;
- exact proof/signature algorithms and cryptographic parameters;
- credential schema/claim definitions and allowed predicate requests;
- holder/device-binding semantics and what is actually proven by them;
- audience / verifier identifier and nonce / transaction binding requirements;
- issue/expiry semantics and clock-skew policy;
- status, revocation, suspension, freshness and outage behavior;
- key rotation, issuer metadata update and trust-anchor rollover;
- verifier certification or onboarding requirements and terms of use;
- output claim ceiling: facts the result does establish and facts it does not establish.

No profile should be called DHA production based solely on a synthetic test, a successful generic cryptographic check, a familiar issuer display name, or an unofficial implementation guide.

## Candidate standards are conditional, not assumptions

Relevant published standards offer an interoperability starting point, but the DHA profile must determine the actual adapter:

- [W3C Verifiable Credentials Data Model 2.0](https://www.w3.org/TR/2025/REC-vc-data-model-2.0-20250515/) is a W3C Recommendation for issuer/holder/verifier credential exchange.
- [OpenID for Verifiable Presentations 1.0](https://openid.net/specs/openid-4-verifiable-presentations-1_0.html) is a final protocol for requesting/presenting credentials across supported formats.
- [OpenID for Verifiable Credential Issuance 1.0](https://openid.net/specs/openid-4-verifiable-credential-issuance-1_0.html) is relevant only if the approved DHA integration involves credential issuance.

These standards do not prove that DHA uses any one of them, make Mycelix a trusted relying party, or supply DHA's issuer keys/status endpoints. If DHA documents a different profile, implement that profile after independent review rather than forcing the national system into a preconceived format.

## Privacy and key-management rules

### Do not import unnecessary identity data into Mycelix

- Never place a national identity number, biometric template, identity-document image, complete DHA credential, or reusable presentation token in a public Holochain DHT.
- Do not put such values in telemetry, crash reporting, logs, URLs, analytics, or test output.
- Prefer local user-controlled association and service-scoped/pairwise references over a public national-ID-to-DID map.
- Store only the minimum short-lived verification receipt that an application genuinely needs; define retention and deletion before pilot deployment.
- Do not preserve a credential or presentation merely because it makes later debugging easier.
- Keep the user's identity claim separate from their membership, trust, reputation, eligibility and authority records.

### Biometric boundary

DHA's official workflow may use biometric verification or device-bound credentials, but this does not grant Mycelix authority to acquire, compare, store or infer biometrics. The app must rely only on the exact holder-authentication result provided by the approved protocol and explain its scope accurately.

A Smart ID card image, OCR result, manual photograph match, device face unlock, and a DHA-verified credential are different evidentiary events. Never label them all DHA verified.

### POPIA and legal review

Before processing real people's data, obtain South African privacy/legal review for purpose limitation, minimality, lawful basis, retention/deletion, data-subject rights, security controls, operator responsibilities, special personal information, biometric processing, unique-identifier linkage and any prior-authorisation obligations. The Information Regulator notes that certain uses of unique identifiers across purposes/sources and certain special-information processing can trigger authorisation requirements.

Sources:
- POPIA, Act 4 of 2013: https://www.justice.gov.za/legislation/acts/2013-004.pdf
- Information Regulator prior authorisation portal: https://eservices.inforegulator.org.za/priorauthorisation/default.aspx
- Information Regulator special-personal-information guidance: https://inforegulator.org.za/knowledge-base/category/popia/chapter-3-conditions-for-lawful-processing/part-b-processing-of-special-personal-information/

This document is an engineering profile, not legal advice or a claim of POPIA compliance.

## Deterministic implementation plan

### Phase 0 — protocol discovery and governance

- Track the final DHA regulations and official technical documentation.
- Identify the DHA team or officially designated partner responsible for digital-credential verifier integration.
- Ask for the exact credential format, profile/version, issuer-trust mechanism, status protocol, holder binding, audience/nonce requirements, testing environment, certification terms, permitted use cases, and relying-party contract.
- Keep production mode unavailable until every mandatory item is reviewed.
- Record terms, allowed processing, retention, and non-digital alternatives for every proposed use case.

**Exit gate:** written official technical profile and permitted relying-party pathway, plus privacy/legal review. Until then, only synthetic tests are permitted.

### Phase 1 — source-neutral verifier contract

Create Rust-facing types around the existing Mycelix Identity system rather than a second identity store. The result should be typed and explicit:

- IssuerProfileRef
- CredentialProfileRef
- VerificationRequest (purpose, required predicate, verifier, nonce, expiry)
- VerificationEvidence (opaque references to validated inputs; no secret payload copied by default)
- VerificationDisposition (Verified, Rejected, Indeterminate)
- ReasonCode (unsupported profile, issuer not trusted, invalid proof, holder-binding failure, wrong audience, nonce replay, expired, revoked, status stale/unknown, schema mismatch, scope mismatch)
- ScopedVerificationReceipt (profile/version, policy/version, time, scoped subject reference, disposition, reason, expiry)

These are proposed semantics, not existing public types. Choose names based on the canonical Rust API after reviewing the current identity crates. Do not duplicate an existing qualified contract if one already provides the same semantics.

### Phase 2 — synthetic issuer and adversarial corpus

Build a local-only test issuer with clearly fake keys and subjects. Mark every fixture SYNTHETIC_TEST_CREDENTIAL so no UI, log, test artifact, or evidence summary can be mistaken for a real South African government-issued credential.

Required cases:
- valid synthetic credential under an explicitly loaded test trust anchor;
- self-signed credential whose display name is Department of Home Affairs;
- unknown, substituted, or rolled-over issuer trust anchor;
- tampered claim, malformed credential, unsupported algorithm/cryptosuite and profile downgrade;
- wrong audience, transaction nonce, and replay of the same proof;
- holder/device-binding mismatch;
- expired, revoked, suspended and status-stale credentials;
- status endpoint unavailable, timeout, malformed status, unknown status, and valid-empty status response;
- schema/purpose mismatch and asking for more claims than the user approved;
- attempted leakage of national ID, full credential, biometrics, nonce, or presentation token into receipt/log/DHT payload;
- one identity proof linked across unrelated relying parties (correlation test);
- valid identity claim wrongly promoted to membership, voting, finance, role, governance or execution authority;
- only a physical card image/OCR result supplied but treated as cryptographic validation;
- user declines proof, has no smartphone, loses their device, or cannot reach the verifier—service must preserve a reasonable alternative route.

Tests must assert exact outcomes and reason codes, not merely that the function returned an error.

### Phase 3 — standards adapter in a separate module

Only after Phase 0, implement the exact official profile with pinned dependencies and conformance vectors. Validate that issuer-trust material comes from the recognized DHA channel. Bind every presentation to the actual verifier and transaction; implement current status and key rotation semantics. Keep adapter errors distinguishable from trust-policy outcomes.

Run deterministic offline tests from frozen vectors, followed by an authorized integration sandbox. A valid signature alone is insufficient if the issuer is untrusted, holder binding is missing, the wrong relying party requested it, or current status cannot be established under the approved profile.

### Phase 4 — scoped application pilot

Start with a low-risk use case and consenting participants. Keep the credential proof flow user-mediated; publish no national ID linkage; never make Digital ID the sole way to access an essential service. Record service outcomes with privacy-safe aggregates. Higher-impact uses need a separate threat model, legal review, and explicit approval.

## Cross-project integration

### Mycelix

Own the issuer/profile registry, verification disposition, scoped receipt, consent boundary, local identity association and authorization separation. Reuse the existing Identity DNA and deterministic DSID lifecycle laboratory. Do not create a parallel DID namespace or store government identity as a public application profile.

Related work:
- [Identity DSID lifecycle lab #3857](https://github.com/Luminous-Dynamics/mycelix/issues/3857)
- [did:mycelix method contract #3863](https://github.com/Luminous-Dynamics/mycelix/issues/3863)
- [Cross-border identity and trust-service recognition #1047](https://github.com/Luminous-Dynamics/mycelix/issues/1047)
- [SA-DID-001 tracking issue #4990](https://github.com/Luminous-Dynamics/mycelix/issues/4990)

### Symthaea

May later help explain a verification result or choose among user-approved workflows, but must not be the root verifier for cryptographic issuer trust or decide that an unsupported credential is valid. Reasoning models cannot invent missing trust metadata, status evidence or legal authority.

### Sol Atlas

Can show the public onboarding/accessibility status of service points or help users find authorized identity-service facilities. It must not put private identity assertions on a geographic map or imply that the presence of a Smart ID service point makes a user verified.

### Symtropy

Not needed in the trust path. It may simulate queue/branch access or service resilience if a clearly scoped scenario and valid inputs exist. Simulation results must never influence the cryptographic validity of a credential.

## Acceptance criteria

- [ ] Production issuer profiles remain disabled by default.
- [ ] Every profile pins issuer, format, schema, proof algorithms, holder binding, audience/nonce, validity and status semantics.
- [ ] Current official DHA protocol and authorized relying-party pathway are documented before live integration.
- [ ] Synthetic issuer and fixture lineage are unmistakable.
- [ ] All 20 synthetic policy inputs execute through the test-only decision oracle and match exact expected outcomes/reasons.
- [ ] CI produces a machine-readable policy report that explicitly disclaims cryptographic verification and DHA endpoint contact.
- [ ] Exact adversarial corpus produces deterministic outcomes and reason codes.
- [ ] Unknown/unavailable current-status evidence cannot become verified-current.
- [ ] Receipt/log/DHT schema leaks no national ID, biometric template, complete credential or replayable presentation.
- [ ] User consent, scope, expiry, unbinding and retention semantics are explicit.
- [ ] Government identity proof cannot implicitly grant Mycelix roles or authority.
- [ ] A non-digital/alternate path remains available.
- [ ] POPIA/privacy review and DHA's final regulatory profile are considered before using real credentials.
- [ ] Native, WASM and multi-agent checks have successful exact-head results before qualification; queued checks are not passes.

## Explicit nonclaims

This profile does not claim:
- that DHA currently publishes a public relying-party API;
- that the DHA prototype uses W3C VC, OpenID4VP, OpenID4VCI, mdoc or SD-JWT VC;
- that Mycelix is recognized or certified by DHA;
- that a locally valid Mycelix DID proves government identity;
- that a cryptographic identity result grants citizenship, residency, service entitlement, legal status or authorization;
- that this design is a POPIA legal determination or compliance certification;
- that any real Digital ID credential has been accessed, parsed, verified, stored or tested by Mycelix.

## Research note

This profile was written on 13 October 2026 from public DHA/Government announcements, the Mycelix public Identity README and current identity issues, W3C/OpenID specifications, and South African privacy guidance. No DHA SDK, credential, test vector, public endpoint or private partner documentation was available in the reviewed material. No real person or government credential was tested.
