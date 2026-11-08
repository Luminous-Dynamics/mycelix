# Maritime Commons Global Interoperability Profile v0.1

**Status:** research profile; not normative protocol and not a country/legal compliance package  
**Tracking:** Mycelix #5945  
**Architecture:** global common semantics + industry standards profiles + independently governed jurisdiction/locality overlays

## 1. Design objective

Maritime Commons must interoperate globally without requiring uniform software, law, language, identifiers, infrastructure or connectivity.

The core invariant is:

Global interoperability does not confer global authority.

A global protocol can standardize the meaning of provider offers, commitments, evidence, time, quantities and references. It cannot make one jurisdiction's law, one issuer's credentials, one payment rail, one port's operating procedures or one community's access rules universal.

The target is a small portable semantic core, explicit standards adapters and reviewable local overlays. Providers and authorities retain their own source-of-truth systems.

This profile complements the Maritime Stay Protocol v0.1 and Maritime Network #5920. It does not modify the v0.1 stay schema. Monetary, time and identifier compatibility questions are proposals requiring a separately reviewed schema decision.

## 2. The three-layer architecture

### Layer A — global common semantics

The common core owns only portable concepts:
- provider, operator, community, vessel, resource, location and stay references;
- offers, holds, commitments, receipts and derived views;
- event, delivery and operation identity distinctions;
- typed physical quantities and monetary obligations;
- source/effective/expiry/observation times and currentness;
- profile/version, provenance, uncertainty, conflicts and nonclaims.

Core fields must not smuggle in a national procedure, tax interpretation, legal conclusion or external authority.

### Layer B — standards and industry profiles

A standards profile binds:
- issuing body and exact standard/profile identifier;
- version/edition/circular/release and maturity status;
- implementation role/direction;
- source URI and retrieved-at time;
- immutable source artifact digest once captured;
- tested requirements and coverage/correctness results;
- explicit unsupported fields and transformation loss;
- claim ceiling and nonclaims.

The first profile register references:
- IMO Compendium on Facilitation and Electronic Business, FAL.5/Circ.56 (approved at FAL 50, March 2026);
- IMO Maritime Single Window framework and relevant national MSW/PCS interfaces;
- DCSA Port Call 2.0 and relevant JIT/PCO semantics;
- IHO S-100 framework and exact product specifications/editions;
- UN/CEFACT Recommendation 16 / UN/LOCODE;
- W3C Verifiable Credentials Data Model 2.0 and separately pinned securing/status mechanisms;
- ISO 4217 currency codes and minor-unit metadata;
- UCUM 2.2 or another pinned physical unit vocabulary;
- IETF BCP 47 language tags;
- Unicode CLDR 49 for locale presentation;
- IANA Time Zone Database 2026f for civil-time rules.

Do not assume a standard is universally applicable because it exists. A private recreational stay manifest is not a government Maritime Single Window submission, DCSA port-call event or IHO navigational product.

### Layer C — jurisdiction and locality overlays

A jurisdiction pack is data and reviewed policy references, not an executable universal law engine. It must identify:
- geographic/jurisdictional coverage and exclusions;
- responsible issuer and competent local authority references;
- local authority, MSW, PCS and reporting workflow references where relevant;
- applicable legal instrument references, exact version/effective period and local-review record;
- privacy controller/processor and data-residency/transfer policy references;
- local languages, scripts, accessibility and legal-translation requirements;
- accepted identity/vessel/provider schemes and issuer scope;
- currency, tax, payment-provider and price-disclosure profile references;
- time zone, civil-time and local-calendar rules;
- local physical-unit conventions and required measurement precision;
- environmental, port, coastal, public-access and community-policy references;
- operational contacts and escalation route;
- status: research, reviewed, approved by the stated authority, superseded, withdrawn or unknown.

National law, port-authority procedures and community rules are separate layers. A community policy cannot override statutory safety, border, port-entry or navigation authority. A protocol cannot decide whether foreign law applies or whether a legal transfer mechanism is enforceable.

## 3. Global identifier discipline

Every external identifier is a tuple, not a bare string:

- scheme, namespace and scheme version;
- issuer/maintaining authority;
- subject type;
- exact identifier value;
- jurisdiction and purpose scope;
- effective and expiry interval if supplied;
- currentness/verification evidence;
- source provenance and claim ceiling.

### 3.1 Locations

UN/LOCODE identifies trade and transport locations. It is not a marina, pier, terminal, berth or slip identifier. Model a provider/authority-scoped facility hierarchy below the location code.

Distinguish official production publication from mutable pre-release data. Do not silently promote pre-release codes to production. Preserve publication identity, revision, retrieval time and artifact digest. An unresolved location remains unresolved; do not fuzzy-match two similarly named ports into one identity.

### 3.2 Vessels

Use an IMO ship identification number only when the vessel falls within the scheme's eligibility and the issuer evidence is verified. Pleasure yachts and small craft may not have an IMO number.

Support scheme-qualified national registry identifiers and other relevant maritime identifiers. Do not treat vessel name, flag, MMSI or call sign as a universal immutable vessel key. Preserve the identifier's subject type, issuer, lifecycle and reassignment rules. IMO company/registered-owner identifiers are not interchangeable with ship identifiers and are not available for every small provider or local service business.

### 3.3 Organizations, facilities and resources

Reuse Mycelix organization identity and existing external-registry references. IMO company identifiers, legal entity identifiers and national business registrations are scheme-specific evidence; none is required universally.

Provider IDs for marinas, berths, rooms, service appointments and utilities are provider-scoped. A resource identifier does not prove current availability, ownership or authority.

## 4. Monetary semantics — v0.1 compatibility review required

The existing stay contract represents money using an integer amount_minor and a three-letter currency value. This avoids binary floating-point arithmetic, but an integer minor-unit amount is not self-describing without the relevant currency exponent/metadata.

ISO 4217 includes currencies with different minor-unit conventions. Currency presentation metadata from CLDR is not an accounting rule.

Before changing the wire schema, review a compatible monetary profile that binds:
- ISO 4217 alphabetic and numeric currency identifiers;
- exact currency metadata artifact/release and digest;
- minor-unit exponent effective for the transaction date;
- integer representation, bounds and overflow behavior;
- quote currency and settlement currency as separate values;
- any FX conversion as a separate sourced event (rate source, timestamp, direction, spread/fee and rounding);
- line-item arithmetic, tax/levy inclusion, mandatory fees and rounding mode;
- external payment provider/rail reference and independently recognized finality evidence.

A signed quote, invoice, webhook or receipt does not prove payment settlement. No payment rail is created by this profile.

## 5. Time and calendar semantics

Use UTC instants for cross-provider event ordering. Preserve the original timestamp lexeme and offset as source evidence.

When local civil time matters, bind:
- IANA time-zone identifier;
- tzdb version used for historical interpretation;
- source local date/time and calendar where supplied;
- resolved UTC instant or explicit ambiguous/nonexistent disposition;
- policy for daylight-saving gaps and folds.

Keep date-only values distinct from instants. Do not convert local midnight to UTC without a declared zone. Do not assume whole-hour UTC offsets. A tzdb update must not rewrite historical source evidence; future schedules may require reevaluation under the new zone rules.

Keep provider event time, provider revision time, transport delivery/attempt time, and local receipt time separate. Do not use webhook arrival order as source truth.

## 6. Language, script, and translation semantics

Use BCP 47 language tags and a pinned CLDR release for display conventions. Preserve original signed bytes and source-language text. Store translations as separate derived artifacts bound to:
- source content digest;
- source and target language tags;
- translation tool/provider and version;
- generation time;
- human/legal review state;
- known omissions and uncertainties.

Machine translation is not an authoritative translation of law, contract terms, safety instructions or provider acceptance. No silent fallback language may change a user's understanding or consent.

Support right-to-left layout, writing-system-specific forms, diacritics, non-Latin identifiers and Unicode normalization in search/index projections. Never normalize or rewrite signed source bytes or identifiers in the name of display convenience.

## 7. Units and geospatial semantics

Every physical measurement binds a numeric value, dimension, unit code, precision, uncertainty and source. UCUM 2.2 may be used for a machine-readable unit vocabulary where its semantics fit the domain. Nautical miles, knots, fathoms, vessel length/beam/draft and local utility measures remain explicit.

Unit conversion is a versioned derivation containing input value/unit, output value/unit, conversion implementation/version, rounding and uncertainty. Incompatible physical dimensions must fail closed.

For geospatial evidence preserve horizontal CRS, vertical datum, coordinate epoch where applicable, axis order, units, precision, geographic coverage, cell/feature identity, quality/uncertainty and transformation lineage. IHO product edition, producer data revision and operational suitability are separate facts.

## 8. Maritime Single Window and public-sector boundaries

The IMO Compendium is a global semantic reference for maritime facilitation/electronic business. The IMO FAL Convention's Maritime Single Window requirement concerns official information exchange with competent public authorities.

Maritime Commons must not:
- represent a private StayManifest as a national MSW submission;
- claim that a provider webhook is official filing or clearance;
- infer government acceptance from transport delivery;
- bypass national MSW/PCS authority or duplicate a government reporting database;
- mint customs, immigration, port-entry, pilotage, navigation or safety permission.

Where authorized, an adapter may map the minimum necessary provider facts to a government-defined reporting schema. Preserve the government's submission identifier, version, acknowledgement type, status and legal meaning separately.

UNECE Recommendation 33 is a useful Single Window design reference. Maritime Commons is a provider/community coordination layer that may interoperate with public infrastructure; it is not a government-led Single Window.

## 9. Connectivity, inclusion and resilience

Support low-bandwidth and intermittent-connectivity providers without weakening authority:
- compact profile metadata and content-addressed references;
- asynchronous event ingestion, durable inboxes and bounded retries;
- local provider operation during federation outages;
- manual/provider-issued receipt path with explicit evidence grade and source;
- printable/accessible fallback receipts;
- explicit pending, stale, conflict and NeedsReconciliation states;
- no offline reservation confirmation without a separately qualified allocation lease/idempotency contract.

A missing public API does not mean a provider cannot participate. A provider may participate through documented manual attestations or an operator-controlled export, subject to provenance and permission. Manual evidence must not be disguised as an automated integration.

## 10. Global security and data governance

Use the current IMO maritime cyber-risk guidance and applicable local requirements alongside existing Mycelix security, provenance and capability systems.

A jurisdiction pack may reference:
- data controller/processor roles and local privacy instruments;
- data residency and cross-border transfer requirements;
- retention and deletion schedules;
- incident notification routes and contacts;
- key issuance, rotation and revocation authorities;
- approved identity/credential schemes;
- permitted purpose and field disclosure profiles;
- audit/assurance evidence and third-party dependencies.

These references are not legal advice or automatic compliance. Local counsel and competent authorities must validate each jurisdiction pack before it is marked approved.

Sensitive crew identity, home addresses, exact private itineraries, payment credentials, private inventory and secrets must not be globally published.

## 11. Starter global validation regions

Use these as research coverage groupings, not legal jurisdictions or pre-approved country packs:
- Southern Africa and wider African coastal/Island contexts;
- EU/EEA and nearby European port systems;
- North America, with national and subnational differences preserved;
- Gulf and wider Middle East;
- South, East and Southeast Asia;
- Latin America;
- Caribbean and small-island developing states;
- Pacific/Oceania and remote island/coastal contexts.

Each country/port pack remains research-only until its exact authority sources, effective dates, official schemas, privacy roles, translations and local review are captured. Do not treat a regional grouping as one legal system.

## 12. Acceptance principles

- Common semantics, industry profiles, national overlays and local community rules are separate.
- Every external scheme has issuer, version/maturity, retrieval time and exact artifact identity.
- No global identifier is mandatory unless its real scheme scope covers the subject.
- UN/LOCODE never becomes a berth identifier.
- Currency amount interpretation includes exact exponent metadata.
- UTC event time and local civil-time interpretation remain distinct.
- Signed source text is preserved; translations are derived and reviewable.
- Unit and coordinate transformations are explicit and reproducible.
- Public-sector reporting and clearance remain with competent authorities.
- Local privacy/data-transfer policy is not bypassed by federation.
- Unverified jurisdiction packs remain explicitly unapproved.
- Global tests measure correctness and coverage separately; NotTested is not PASS.

Passing a synthetic global profile corpus establishes only the declared technical behavior for exact pinned inputs and versions. It does not establish local legal compliance, regulatory acceptance, field interoperability, government adoption, physical truth, navigation safety or global production readiness.


## 13. Compose with existing maritime federation infrastructure

Maritime Commons must not recreate a global maritime identity system, service directory, secure messaging layer, or trusted port-platform network.

### IALA Maritime Connectivity Platform

IALA's Maritime Connectivity Platform (MCP) is an existing decentralized maritime information-exchange architecture. Its documented components include the Maritime Identity Registry (MIR), Maritime Service Registry (MSR), and Maritime Messaging Service (MMS). Its identity architecture uses Maritime Resource Names (MRNs), certificates/PKI, and web-service authentication and authorization mechanisms. MCP is provided collectively by multiple organizations rather than by one mandatory central operator.

The integration rule is:

- use MRNs and MIR/MSR/MMS when the relevant participant already operates within MCP and the exchange is authorized;
- preserve MRN as one scheme-qualified identifier, never a required universal vessel, marina, company, or provider identifier;
- treat identity authentication, endpoint authentication, local issuer recognition, capability authorization, endpoint currentness, and provider commitment as separate claims;
- never infer live inventory, reservation rights, statutory authority, local legal recognition, or a physical service effect from MCP membership, a valid certificate, or an MSR listing;
- use the existing Mycelix identity/recognition mechanisms for local policy and cross-sovereign recognition rather than making a parallel MIR-like trust root.

IALA Guideline G1157 describes the MCP architecture. The Maritime Resource Name portal documents its resource-naming scope. The G1191 Maritime Service Registry Technical Specification is versioned independently; the observed v1.2 record was revised 11 June 2026. Every adapter must bind the exact guideline/specification edition and its artifact digest.

### IPCSA Network of Trusted Networks

IPCSA's documented Network of Trusted Networks (NoTN) is an adjacent platform-to-platform interoperability effort for Port Community Systems, Cargo Community Systems, and Single Window operators. Its published description concerns trusted cross-border platform exchange, not an individual-vessel identity system or a multi-provider leisure-stay booking contract.

Maritime Commons should therefore:

- integrate, where authorized, at a PCS/platform boundary rather than bypassing that boundary;
- recognize platform membership as a transport/governance relationship, not blanket permission to disclose every user's or provider's data;
- apply purpose, field-level disclosure, jurisdiction, retention, and user/contract permissions before any cross-platform transfer;
- keep private itineraries, crew/household details, provider inventory, and stay commitments out of cargo/PCS flows unless a lawful, purpose-specific exchange requires them;
- avoid creating a competing trusted-platform network.

The NoTN material describes a trial/proof-of-concept scope. Do not describe it as universal deployment or assume that every port, PCS, country, or marina participates.

### Preserve each standard's claim boundary

DCSA Port Call 2.0 defines port-call event exchange, but its published API specification leaves subscriber registration and bilateral authentication/authorization out of scope. A DCSA-conforming event adapter therefore does not, by itself, supply a complete federation trust or authorization model.

IHO S-131 Edition 2.0.0 is recorded as the first operational edition of Marine Harbour Infrastructure. It describes harbour infrastructure and port-service information for planning. That information can improve discovery and planning, but it is not a provider's live reservation inventory, a capacity hold, or a contractual commitment.

IHO S-127 Edition 2.0.0 is recorded as the first operational edition of Marine Traffic Management. It describes information such as vessel traffic services, pilotage, routing measures, and ship reporting systems. It is not a berth booking, a vessel clearance, or permission to enter a port.

### Standards are a dependency lock graph, not a flat list

A standards profile must pin the dependency closure needed for the actual implementation:
- root product specification and edition;
- the exact framework version it declares or requires;
- feature and portrayal catalogues;
- schema packages, code lists, and validation checks;
- referenced code sets or external registries;
- maturity, source URI, retrieval time, and immutable digest for every artifact.

Do not assume that the newest framework edition is automatically compatible with every product edition. For example, the IHO registry record for S-131 Edition 2.0.0 identifies S-100 version 5.2.0 and separately published Feature Catalogue and Portrayal Catalogue artifacts. A family-level statement that the implementation uses "latest S-100" is not an adequate dependency lock.

The proposed machine-readable lock contract is \`standards-dependency-lock-v0.1.schema.json\`. It requires semantic verification of dangling dependencies, cycles, digest presence, maturity transitions, and compatibility constraints; JSON Schema alone cannot prove those graph properties.

A follow-on, not-yet-executed integration corpus is \`existing-federation-cases-v0.1.json\`. It is separate from the original 71-scenario comparative benchmark; it does not retroactively change that benchmark's scenario count or execution status.

### Source references

- IALA MCP: https://maritimeconnectivity.net/
- IALA G1157: https://www.iala.int/product/g1157/
- IALA MRN portal: https://www.iala.int/technical/data-modelling-mrn/
- IALA G1191: https://www.iala.int/product/g1191/
- IPCSA NoTN: https://tenant.ipcsa.international/initiatives/network-of-trusted-networks/
- DCSA Port Call 2.0 API: https://reference.dcsa.org/content/standards/releases/port-call/v2/pc-v2-openapi-specification
- IHO S-131: https://registry.iho.int/productspec/view.do?category=product_ID&domainS=ALL&idx=233&product_ID=S-131&searchValue=&statusS=5
- IHO S-127: https://registry.iho.int/productspec/view.do?category=product_ID&domainS=ALL&idx=232&product_ID=S-127&searchValue=&statusS=5


### Adapter semantics and field-loss accounting

An adapter connection is not yet semantic interoperability. Every adapter profile should publish a mapping ledger using the adapter-mapping-ledger-v0.1.schema.json contract.

For every source field in the adapter's declared coverage:
- identify its exact source path and source meaning;
- identify the target path and target meaning, or explicitly state that it is omitted/unsupported;
- classify the transformation as preserved, normalized-preserving, approximated, omitted, unsupported, unknown-source-value, or invented-value;
- report loss severity and whether the source can be reconstructed exactly, from retained source evidence, partially, or not at all;
- state the evidence grade used to evaluate the mapping.

The source value itself should not be copied into the mapping ledger by default; reference the protected source evidence instead. This keeps the ledger useful for semantic review without creating a second store of customer/provider payloads.

Required semantic rules:
- A required target field cannot be silently omitted or marked unsupported.
- An invented value is a hard rejection for normative export; a target schema's requirement does not authorize fabrication.
- Unknown source values must be preserved opaquely, quarantined, or rejected under the exact adapter profile; they must not silently collapse to a default enum.
- "Preserved" means exact round-trip preservation, not merely that a value was populated.
- Normalization is preservation only where the profile declares and tests semantic equivalence.
- A structurally valid ledger does not prove a mapping correct. The independent semantic verifier must compare frozen source/target vectors and verify required-field coverage.
- Calculate coverage and loss rates from the records; do not trust caller-supplied summary percentages.

Benchmark declared-lossless fields at 100% preservation and zero silent loss. Report all approximation, omission, unsupported values, and unknown fields separately; do not let a high adapter count compensate for a broken required field.
