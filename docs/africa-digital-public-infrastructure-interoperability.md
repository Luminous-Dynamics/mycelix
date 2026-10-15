# African digital public infrastructure: sovereignty-first interoperability strategy

**Research snapshot:** 2026-10-14  
**Status:** Research and implementation plan. No cross-border production adapter is authorized or implemented.  
**Portable architecture:** [Portable sovereign interoperability framework](portable-sovereign-interoperability-framework.md) compares EU, ASEAN and Latin American/Caribbean approaches without treating them as interchangeable.  
**Related work:** [South African DHA Digital ID profile](sa-digital-id-interoperability.md), [Mycelix identity module](../mycelix-identity/README.md), [machine-readable contract](../mycelix-identity/AFRICA_DPI_INTEROP_CONTRACT_V1.json).

## Executive recommendation

Build a **federation of independently governed national adapters**, not a pan-African identity database. Mycelix should contribute an open, testable compatibility layer: document what an authority actually publishes or authorizes, verify only the evidence allowed by that profile, disclose the minimum claims needed, and refuse to convert identity proof into entitlement or authority.

This follows the direction of the African Union (AU) Interoperability Framework for Digital ID. The framework explicitly does **not** call for one unified continental ID system; it aims to interoperate existing national systems while respecting national sovereignty, differences in infrastructure, regulation and inclusion needs. It envisions credentials that complement national systems and can work online or offline. See the [AU framework](https://au.int/en/documents/20231211/au-interoperability-framework-digital-id) and [English framework PDF](https://au.int/sites/default/files/documents/43393-doc-AU_Interoperability_framework_for_D_ID_English.pdf).

The wider policy backdrop includes the [AU Data Policy Framework](https://www.au.int/en/documents/20220728/au-data-policy-framework), the [AfCFTA Digital Trade Protocol](https://www.au.int/en/treaties/protocol-agreement-establishing-african-continental-free-trade-area-digital-trade), and [Smart Africa's Digital ID / Smart Africa Trust Alliance work](https://smartafrica.org/service-post/digital-identity-for-africa/). These are alignment anchors, **not proof** that a particular country has deployed a compatible verifier API or implemented each proposed rule in domestic law. The parallel **infrastructure track** should also align with the AU's [Continental Policy on Climate Resilient and Smart Infrastructure in Africa](https://au.int/sw/node/46676), which covers energy, transport, water and ICT across planning, delivery, operation and maintenance, and with [STISA 2034](https://au.int/en/documents/20250718/science-technology-and-innovation-strategy-africa-stisa-2034).

## Four boundaries that should not collapse

1. **National authority** issues legal identity, licenses, registrations and public decisions under its laws. Mycelix does not become an issuer or substitute government.
2. **Evidence adapter** checks a specific official protocol, credential, response or verification service. It records the exact route and assurance used. A server-side register check and a holder-presented credential never silently substitute for each other.
3. **Service policy** determines what the relying service may do under its lawful purpose and specific rules. Identity proof alone never grants citizenship, residency, work permission, a benefit, a bank account, membership, a vote, credit or execution authority.
4. **Receipt and audit** retain only the minimum evidence needed to show which rule was applied and why. Do not put national identity numbers, raw credentials, biometrics, full document images, replayable presentations or cross-service identity graphs into a public DHT.

A shared data model can standardize concepts such as issuer, subject binding, purpose, assurance, validity, status, audience, disclosure and decision reason. It must **not** erase differences in national law, legal meanings, documentary evidence or permitted uses.

The machine-readable design contract is AFRICA_DPI_INTEROP_CONTRACT_V1.json. It deliberately contains no qualified country-profile entries; it is an invariant contract, not a claim of country coverage.

## South Africa: immediate improvements

Keep SA-DID-001 as the first national research profile, but split integration into separate service tracks.

### DHA Online Verification Service (OVS)

This is an authorized, server-side register-verification route. Prepare the onboarding dossier, use DHA's published channel, establish the legal basis and purpose for each query, agree field minimization and retention, and do not implement until access and terms are granted. OVS is not proof that the smartphone Digital ID verifier protocol is open to third parties.

### Smartphone Digital ID

The 1 October 2026 ministerial address announced a working prototype but, in the public material reviewed, did not establish a Mycelix-facing protocol, trust-metadata endpoint, status method, verifier conformance programme or onboarding contract. DHA's Annual Performance Plan separates its 2026/27 hosting/PKI target from medium-term wallet-issued verifiable-credential plans. Treat milestones as targets, not proof of completion. Sources: [1 October address](https://www.gov.za/news/speeches/minister-leon-schreiber-home-affairs-quantum-leap-vision-future-government-01-oct), [draft regulations announcement](https://www.gov.za/news/media-statements/minister-leon-schreiber-invites-public-comment-draft-digital-identity), and [DHA Annual Performance Plan 2026/27](https://www.dha.gov.za/images/AnnualReports/DHA-APP-2026-27_260326.pdf).

**Next step:** request a written relying-party specification covering credential/protocol versions, issuer trust and key rotation, test vectors, expiry/status/revocation, holder binding, verifier authorization, data minimization, security reporting, offline alternatives, fees, retention, incident escalation and a supported conformance sandbox. Never reverse-engineer app internals or scrape undocumented endpoints.

## Other African countries: research, don't guess

National systems, legal powers and access methods differ. Build an evidence-backed **country capability matrix**, not one assumed "African ID API". The following are research leads, not production-readiness rankings or endorsements.

| Research lead | What official sources indicate | What must be verified before any integration |
|---|---|---|
| **Ghana — Ghana Card / NIA** | The National Identification Authority describes an identity-verification service with a formal institutional onboarding process. Its checklist includes the institution's legal authority to collect the data, data-protection certification and requested datasets. A 2026 notice says new regulations require biometric verification where available and restrict copying/retaining card copies except where legally permitted. [NIA verification services](https://nia.gov.gh/service/verification-services/) · [2026 notice](https://nia.gov.gh/government-makes-biometric-verification-mandatory-for-ghana-card-verification-under-new-l-i-2523/) | Confirm current law and regulated scope with Ghana counsel; determine eligibility for onboarding; obtain written API/protocol and test access. Do not infer API access from a service page or collect biometrics as a shortcut. |
| **Rwanda — e-ID / Single Digital ID** | Official NIDA/Irembo materials describe a national e-ID programme. Official 2026 reporting describes nationwide identity capture/verification activity; the Irembo FAQ describes digital and physical forms and expanded biometric collection. This does not establish an external verifier contract. [NIDA](https://www.nida.gov.rw/) · [Irembo e-ID FAQ](https://support.new.irembo.gov.rw/en/support/solutions/articles/47001285585-faqs-about-new-digital-id-e-id-or-e-ndangamuntu-and-digital-id-pre-enrollment) | Identify the statutory data controller, lawful relying-party route, trust/status APIs, cross-border transfer rules, correction/appeal process and non-biometric alternative. Never export biometric data to a federation or Mycelix DHT. |
| **Nigeria — NIN / NIMC** | NIMC's June 2026 announcement says the NIMC Act 2026 assigns NIMC a Root Certification Authority role for national PKI/DPI and roles relating to interoperable exchange. [NIMC announcement](https://nimc.gov.ng/press-releases/president-tinubu-signs-nimc-act-2026-into-law) | Have Nigerian counsel review the enacted text and implementing instruments; verify standards, trust metadata, delegated verifiers, purpose limits and integration procedures. A legal mandate does not itself provide an API, sandbox or project permission. |
| **South Africa — DHA** | Two separate routes are under research: OVS and the announced phone-bound Digital ID prototype; production wallet-verifier terms remain unestablished in the reviewed material. | Keep OVS and smartphone credential routes separate; request written guidance and retain SA-DID-001 as the baseline. |

Select a first bilateral pilot based on **a consenting public authority, a real service need, an accessible test environment, reviewed laws, an accountable privacy authority, an offline fallback and a measurable benefit**. If a condition is missing, keep that route in research-only status.

## Interoperability layers to implement once

### Country adapter manifest and trust lifecycle

Each country profile should pin:
- statutory authority and source date, including the exact law/regulation reviewed;
- official issuer identifiers and trust-metadata/key retrieval, rotation, revocation and audit process;
- supported credential/response protocols and exact versions;
- holder binding, audience, nonce/replay, expiry, status and revocation semantics;
- authorized relying parties, onboarding, contract, scopes and fees;
- data-controller roles, purpose/legal basis, allowed claims, retention and erasure;
- offline access, languages, accessibility, support, correction and appeal pathways;
- positive **and negative** conformance vectors, source commit, hashes and human release approval.

Unknown issuer, unknown key status, unsupported versions, stale status, clock uncertainty or unavailable status must never produce a successful outcome. Offer a safe route to a person or an alternative channel; do not downgrade silently.

### Minimal claims, not shared population databases

Prefer a narrow proof such as "over the required age" or "business registration is valid" only when an official source supplies that claim, it is lawfully usable for the stated purpose, and the profile technically proves it. Avoid passing full identity details when a minimal claim suffices. Do not create a pan-African person lookup or permanent identifier graph as a prerequisite for interoperability.

Consent must be meaningful and revocable where applicable, but consent alone is not automatically a sufficient legal basis. Every relying party still needs a lawful purpose, proportionality, security, retention limits and a way to contest mistakes.

### Consentful data exchange and service discovery

A future public-sector exchange layer should expose human- and machine-readable service metadata: responsible agency, eligible caller, lawful purpose, permitted fields, fee/rate terms, availability, response expectations, data locality, retention, incident contact and supported version. Use authenticated official APIs when provided; do not scrape public portals or call endpoints without authorization.

### Offline, assisted and low-connectivity access

The AU framework contemplates online/offline verification and different credential forms. Design for intermittent power/data, low-cost Android phones, local languages, disabilities, people without smartphones and people whose records need correction. Use QR/NFC/paper/smartcard options only when the national issuer defines and authorizes them. Lack of a phone or a verifier outage must not be treated as fraud or as proof of ineligibility.

### Digital trade and organisational identity

Keep company/cooperative/farm/vendor credentials separate from natural-person identity. Future proofs could cover official business registration, tax/VAT status where legally available, permitted signatures, licences, shipment evidence, invoice status and procurement documents. A person's identity alone does not prove company ownership or authority to sign.

Map this to the AfCFTA Digital Trade Protocol and regional mechanisms, while verifying domestic implementation and each participating country's rules. Make translation and multilingual forms part of conformance; English-only integration is not continent-ready.

## Start with projects that do not require identity APIs

The best first Africa-facing pilot should not be blocked on a government identity API. Start with services that can deliver value using public data, clearly licensed datasets, or community-provided information with consent:

1. **Official service navigator:** a multilingual, source-linked guide to municipal and national service requirements, application steps, opening/access routes, accessibility and appeals. Show source dates and flag stale pages; never infer an individual's legal eligibility or ask for identity documents.
2. **Local resilience atlas:** show public infrastructure, water points, clinics, public transport, power assets, flood and heat exposure, and published maintenance notices. Attach provenance, license, update date and correction channel to every layer. Aggregate or suppress sensitive locations and personal details.
3. **Community maintenance workflow:** let residents and field workers submit consentful water-leak, streetlight, road, sanitation or clinic-stock reports; route them to a participating operator; publish aggregate status, time-to-repair and reopening evidence. Do not publish reporters' identities or treat an unconfirmed report as fact.
4. **Local planning and learning:** model a school, clinic, market or neighbourhood's water, power, heat and transport alternatives from explicit assumptions and locally validated measurements. Compare costs, resilience and distributional impacts before introducing any high-impact automation.

These pilots create evidence of public benefit and learn how local institutions operate without building a person-surveillance layer. Then add identity-dependent functions only where a specific public authority has approved a concrete use case and verification route.

## What Symthaea, Mycelix, Sol-Atlas and Symtropy can contribute

| Project | Best role | Concrete Africa-facing deliverable |
|---|---|---|
| **Symthaea** | Auditable reasoning, uncertainty handling, local scientific assistant and decision support; never a legal identity issuer or final authority for high-impact decisions. | Evidence-grounded multilingual service assistant; source-linked summaries of regulations; missing-document triage; climate/agronomy/maintenance copilots for constrained hardware and offline degradation. Require calibrated abstention and human review. |
| **Mycelix** | Consent, identity-adapter boundaries, credentials/claims, organizational trust, governance and privacy-preserving evidence. | Open national-adapter toolkit, governance charter, conformance vectors, data-exchange agreements, business/cooperative connectors and low-disclosure audit receipts. Citizen records stay with national authorities. |
| **Sol-Atlas** | Geospatial evidence and public asset/infrastructure mapping. | Local-first atlas for water points, clinics, roads, transit, power, farms, flood/fire risk, public works, maintenance and resilience. Every layer needs provenance, timestamp, resolution, license, sensitivity class and correction route. Do not expose protected people or sensitive sites. |
| **Symtropy** | Participatory simulation and coordination across households, cooperatives, municipalities and supply chains. | Compare water/energy/transport/housing options; model cooperative logistics and local manufacturing; expose trade-offs and who bears costs; preserve reproducible assumptions and community review instead of automatically optimizing people. |

Integrate through stable, permissioned interfaces rather than a new monolith. Mycelix records who authorized what; Symthaea summarizes evidence and uncertainty; Sol-Atlas anchors the model to spatial assets; Symtropy compares choices and feedback. Keep real-world actuation and government decisions outside model authority.

## Shared high-leverage use cases

1. **Water and drought resilience:** map boreholes, water-quality checks, supply reliability, catchments, rainfall and maintenance teams; predict outage risk and prioritize repairs with local review.
2. **Smallholder agriculture and food security:** climate/soil/crop calendars, locally validated seed/input advice, pest alerts, shared equipment, aggregation, cold-chain and market logistics; capture farmer corrections and outcomes.
3. **Distributed energy:** identify clinics, schools, pumps, cold storage and micro-enterprises needing reliable power; compare solar/storage/microgrid options and maintenance; coordinate spare parts and service technicians.
4. **Clinics and medicine supply:** licensed-provider verification, stock/outage alerts, cold-chain monitoring, pharmacovigilance and referral coordination. Manufacturing/prescribing needs national health authorization, qualified humans, safety standards and traceability; an identity or AI result is not medical authorization.
5. **Public-service navigation:** explain requirements, application steps, accessible routes and appeal channels using current official sources; cite sources and abstain rather than invent eligibility rules.
6. **Regional small-business trade:** legal company credentials, multilingual documentation, transport/consignment evidence, cooperative aggregation and dispute resolution under explicit transfer and payment agreements.
7. **Skills and local manufacturing:** map repair/fabrication capability, vocational curricula, parts and tool libraries; publish open designs with safety-critical certification and local material constraints.

These are infrastructure priorities, not claims that an AI system can solve them alone. Reliable power/connectivity, trusted institutions, trained operators, affordable devices, community governance and funding are prerequisites.

## Phased execution plan

### Phase 0 — Contract and evidence

- Keep SA-DID-001 fail-closed and separate OVS from holder-presented Digital ID.
- Validate the continent-level invariant contract; never fill country profiles from guesses.
- Maintain a source register with retrieval date, country, authority, exact claim supported, uncertainty, reviewer and next review date.
- Preserve synthetic negative controls, exact-head CI evidence and source hashes. Do not claim production while any release gate remains unmet.

### Phase 1 — National capability matrix

For each country, score **evidence readiness**, not national worth: primary sources; legal authority/purpose; official verifier procedure; trust/status/key-rotation spec; privacy and correction/appeal routes; offline/assisted access; sandbox/conformance support; and local operator/funding. Unknown means unknown, never an assumed pass.

### Phase 2 — Authority-approved single-service pilots

Choose one low-risk service with a real service owner—for example, an approved institutional verification route, business-licence verification or public-service navigation that collects no identity data. Require written scope, privacy review, baseline metrics, outage fallback and exit/deletion plan.

### Phase 3 — Bilateral federation

Only after each national adapter independently passes its gates, test one consented cross-border use case under written agreements between authorities. Keep trust roots and records sovereign; do not mint a regional "verified African identity" result without a recognized governance mechanism.

### Phase 4 — Regional reference implementation

Contribute open specifications, test vectors and implementation lessons to AU/REC/Smart Africa discussions. Align vocabulary and assurance, but keep admission, revocation and legal interpretation under the relevant public authorities. The model is federated agreement, not a Mycelix-controlled continental root of trust.

## Outcomes to measure

Measure public value and failure modes, not credentials issued alone:
- end-to-end service completion time/cost;
- completion rates without smartphones, high bandwidth or biometric handoff;
- unnecessary fields requested/retained (target zero for prohibited fields);
- false accept/reject and indeterminate rates on synthetic and authorized conformance vectors;
- successful correction/appeal and time to resolution;
- service availability and safe fallback rates;
- incident notification/containment time;
- retention/erasure compliance;
- reach by language, disability, rurality and income band where lawful and privacy-preserving;
- share of hosting, support, training, procurement and operating ownership retained by local institutions and businesses.

Avoid person-level surveillance dashboards and national rankings that reward exclusion of people who are harder to serve.

Before claiming the design travels across regions, map each target framework's published architecture and conformance process. In particular, learn from the EU's published wallet reference framework, ASEAN's stated cross-border interoperability objective, and IdLAC's regional public-good identity broker in Latin America and the Caribbean. The appropriate action may be mapping, integration, or contribution to an existing public good—not creating a parallel regional broker. See the [portable framework](portable-sovereign-interoperability-framework.md).

## Immediate next actions

1. Finish the Africa DPI contract validator and wire positive/negative tests into the focused evidence workflow.
2. Build a source-backed research matrix for South Africa, Ghana, Rwanda and Nigeria, using statuses such as not assessed, research only, partner review, authorized sandbox and production authorized only with explicit evidence.
3. Extend documentation and synthetic contracts until a public authority provides an authorized protocol/sandbox. Do not add a network client with guessed government endpoints.
4. Separately fix the D6S qualification workflow's stale Cargo test-target names; do not change it into an unrelated green check.

## Primary sources

- African Union, [Interoperability Framework for Digital ID](https://au.int/en/documents/20231211/au-interoperability-framework-digital-id) and [full English PDF](https://au.int/sites/default/files/documents/43393-doc-AU_Interoperability_framework_for_D_ID_English.pdf).
- African Union, [Data Policy Framework](https://www.au.int/en/documents/20220728/au-data-policy-framework) and [AfCFTA Digital Trade Protocol](https://www.au.int/en/treaties/protocol-agreement-establishing-african-continental-free-trade-area-digital-trade).
- African Union, [Science, Technology and Innovation Strategy for Africa (STISA) 2034](https://au.int/en/documents/20250718/science-technology-and-innovation-strategy-africa-stisa-2034).
- African Union, [Continental Policy on Climate Resilient and Smart Infrastructure in Africa](https://au.int/sw/node/46676).
- Smart Africa, [Digital Identity / Smart Africa Trust Alliance](https://smartafrica.org/service-post/digital-identity-for-africa/).
- South Africa DHA, [1 October 2026 ministerial address](https://www.gov.za/news/speeches/minister-leon-schreiber-home-affairs-quantum-leap-vision-future-government-01-oct), [draft regulation announcement](https://www.gov.za/news/media-statements/minister-leon-schreiber-invites-public-comment-draft-digital-identity), and [Annual Performance Plan 2026/27](https://www.dha.gov.za/images/AnnualReports/DHA-APP-2026-27_260326.pdf).
- Ghana NIA, [identity verification services](https://nia.gov.gh/service/verification-services/) and [2026 verification notice](https://nia.gov.gh/government-makes-biometric-verification-mandatory-for-ghana-card-verification-under-new-l-i-2523/).
- Rwanda, [NIDA](https://www.nida.gov.rw/) and [Irembo e-ID FAQ](https://support.new.irembo.gov.rw/en/support/solutions/articles/47001285585-faqs-about-new-digital-id-e-id-or-e-ndangamuntu-and-digital-id-pre-enrollment).
- Nigeria NIMC, [June 2026 NIMC Act announcement](https://nimc.gov.ng/press-releases/president-tinubu-signs-nimc-act-2026-into-law).
- World Bank, [Cotonou Declaration on regional digital transformation, November 2025](https://www.worldbank.org/en/news/statement/2025/11/18/regional-summit-on-digital-transformation-in-western-and-central-africa-cotonou-declaration).
