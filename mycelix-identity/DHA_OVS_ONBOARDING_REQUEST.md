# Draft DHA OVS onboarding enquiry

**Status:** Draft only. Not sent.  
**Recipient published by DHA:** Verifications@dha.gov.za  
**Purpose:** Establish whether a proposed application is eligible for approved OVS access and obtain the official integration contract/specification before any implementation.

## Suggested email

**Subject:** Enquiry: OVS eligibility and approved integration requirements for a user-consented service

Dear Department of Home Affairs OVS team,

We are researching whether an application operated by Luminous Dynamics could use the Department's Online Verification Service (OVS) for a narrowly scoped, user-consented identity verification workflow.

Before building or requesting any production access, we would like to establish whether our organization and intended use case are eligible under DHA's current policy and onboarding requirements. We understand that OVS access is registered/authorized, that the service has separate real-time and off-peak batch pricing, and that permitted usage and data handling are controlled by DHA's terms. We do not assume that public access to this contact means that access has been approved.

Could you please advise on the following, or point us to the appropriate onboarding team and current documentation?

1. What legal-entity, sector, use-case, and registration criteria apply to a new applicant? Is a small public-interest/open-source application eligible, and is a specific regulated or accredited status required?
2. What application documents, contracts, lawful-purpose declarations, privacy/security assessments, or data-protection officer details are required before onboarding?
3. What official technical specification is available for authorized clients, including transport, authentication, request signing, response schema, error codes, certificate/key management, network allow-lists, sandbox/test access, and versioning?
4. Which request fields and verification purposes are permitted, and what evidence may be returned? Are there field-level minimization controls?
5. What are the current per-transaction and batch charges, batching limits, rate limits, minimum commitments, service windows, and billing arrangements?
6. What are the required retention/deletion periods, audit-log obligations, restrictions on delegation/subcontracting, and prohibitions on onward disclosure, caching, resale, or use for unrelated purposes?
7. What currentness, correction, disputed-record, timeout, service-outage, and retry rules must a relying application follow?
8. Is there a separate official engagement route for the smartphone-based Digital Identity credential system? If so, what public roadmap, partner criteria, relying-party profile, or technical consultation process is available?

Our intended design principle is to minimize the data received and retained, keep any verification outcome tied to its permitted purpose, and avoid publishing national identity identifiers, document images, biometric information, or raw verification results in a public distributed ledger. We would not use OVS results to infer membership, reputation, voting eligibility, financial authority, or access rights beyond a separately reviewed service policy.

We will not submit real-person records or connect to an endpoint until DHA confirms eligibility, permitted use, technical access, fees, and contractual requirements. We would appreciate any current onboarding checklist or application form.

Kind regards,

Luminous Dynamics  
[Insert the appropriate legal entity name, registration details, and contact person before sending]

## Internal send checklist

- [ ] Confirm the exact legal contracting entity and who is authorized to correspond with DHA.
- [ ] State one concrete use case and why verification is necessary; avoid a general-purpose identity broker pitch.
- [ ] Confirm the intended purpose and lawful basis with South African privacy counsel.
- [ ] Do not attach real identity data, example National ID numbers, credentials, photographs, biometrics, or access secrets.
- [ ] Do not imply the project is endorsed by DHA, certified, or already integrated.
- [ ] Request the official spec through the authorized channel; do not use reverse-engineered endpoints or scrape the official app.
- [ ] Record DHA's response, applicable version, terms, allowed use, and approval evidence before opening production configuration.
- [ ] Re-evaluate whether OVS is necessary at all for the first pilot; low-risk community services should not require government-ID verification without a compelling purpose.

## Authoritative public access notice

DHA's 23 June 2025 statement says organizations seeking to connect to OVS should email `Verifications@dha.gov.za`. It describes registered third parties checking identity and biographical information against the National Population Register and lists R10 real-time and R1-per-field off-peak batch fees for private-sector use. The 14 April 2026 statement says the OVS fee change remains in effect after the legal challenge was withdrawn.

- https://www.gov.za/news/media-statements/home-affairs-corrects-unsustainable-under-pricing-national-population
- https://www.gov.za/news/media-statements/minister-leon-schreiber-welcomes-withdrawal-act-court-application-against

This is a draft enquiry, not a request already sent, an assurance of eligibility, legal advice, or authorization to process personal information.
