# Global Accounting Portability and Cross-Border Conformance v1

**Status:** Research architecture and synthetic conformance proposal; not global accounting software, tax advice, an audit standard, or a qualified jurisdiction pack.  
**Tracking:** [FIN-GLOBAL-001 / #5811](https://github.com/Luminous-Dynamics/mycelix/issues/5811)  
**Composes:** #5782, #5785, #5790, #5792, #5793 and #5798.

## 1. Thesis: portable core, not universal law

Global applicability must not mean embedding every jurisdiction's law, taxes, chart of accounts, filing deadlines and e-invoice schema into one universal ruleset. It means making the economic/evidence core portable while making every legal, accounting, tax, reporting and interoperability context explicit, composable, versioned and independently qualified.

The system must identify portable facts, required profile dimensions, exact qualified revisions, missing/conflicting/stale/unsupported dimensions, and what conclusions must be withheld.

**No global compliance claim is implied.** Global capability is a tested profile-resolution and interoperability architecture. Jurisdictional authority is local and must remain locally qualified.

## 2. Why country is not a sufficient key

An international transaction can involve distinct legal entity domicile and establishment; seller, buyer, ship-from, ship-to and place-of-supply jurisdictions; tax registrations; accounting framework; functional, transaction and presentation currencies; e-invoice syntax, network, CIUS and validator versions; reporting taxonomy; source language, locale, calendar, time zone and identifier namespace.

One jurisdiction does not imply all the other dimensions. IFRS does not make local tax law uniform. A country code does not establish tax registration, permanent establishment, place of supply or framework eligibility.

The IFRS Foundation maintains jurisdiction adoption profiles and states each jurisdiction determines which entities should use IFRS for SMEs. The 2025 third edition is effective for periods beginning on or after 1 January 2027, with early application permitted. Preserve framework edition, effective time, local adoption and entity eligibility as separate facts. [IFRS profiles](https://www.ifrs.org/use-around-the-world/use-of-ifrs-standards-by-jurisdiction/) · [IFRS for SMEs](https://www.ifrs.org/issued-standards/ifrs-for-smes/)

## 3. Global accounting context graph

| Axis | Minimum context | Must not imply |
|---|---|---|
| Legal entity | entity ID, registration namespace, domicile, status | tax nexus or accounting framework |
| Establishment | establishment/location, period, authority evidence | permanent establishment conclusion without rules |
| Transaction | counterparties, roles, supply/ship facts, event/valid times | place-of-supply or tax treatment from country alone |
| Accounting framework | framework, edition, local adoption, entity eligibility, election | tax rules or universal GAAP conversion |
| Tax | jurisdiction, tax type, registration, rule-pack revision, valid/known-at frontiers | accounting recognition or reporting framework |
| Currency | transaction, functional, presentation currency; precision; FX source, purpose, rate and time | one base currency for every purpose |
| E-invoice | syntax, semantic standard, network, CIUS, validator/codelists | authenticity, delivery, tax correctness or settlement |
| Digital reporting | taxonomy, extension, filing schema, regulator and release | underlying ledger or accounting policy |
| Locale/time | language/script, locale, calendar, timezone, DST resolution | source rewriting or event-time truth |
| Evidence | raw bytes, namespace, source/authority, parser/validator, custody, assertion-purpose | universal trust or truth |

Policy resolution is deterministic over the full required context. Missing or conflicting coordinates return typed INDETERMINATE, NEEDS_REVIEW or UNSUPPORTED_PROFILE results, not guessed defaults.

## 4. Architecture layers

### 4.1 Portable economic and evidence core

Own jurisdiction-neutral concepts: exact monetary quantity and unit/currency identity; parties and issuer-namespaced identifiers; economic events, obligations, settlement observations, delivery/acceptance and correction; evidence objects, byte digests, source identity, custody/capture and transformations; event, valid, capture, knowledge and approval times; append-only revisions, reversals, disputes and reviewer decisions.

Do not hardcode local tax rates, thresholds, filing calendars, statutory retention periods, account codes, company types or mandatory invoice fields into the core.

### 4.2 Independent profile registry

Use separate versioned packs for legal jurisdiction and entity/establishment facts; financial-reporting framework and edition; local adoption/endorsement and entity eligibility; tax type and tax jurisdiction; reporting taxonomy/filing schema; e-invoice syntax/standard/network/CIUS and validators; currency precision and FX policy/provider/quotation direction; identity namespace, language/locale, calendar and time zone.

Each pack binds source authority, publisher, document/release version, source URL and captured bytes/digest, applicable entities/transactions, valid-time and knowledge-time, review/approval, dependencies, exclusions, tests and supersession lineage.

A package's existence is not a capability claim. Only qualified_for_declared_scope may be called qualified, and it must identify the exact tested intersection.

### 4.3 Policy compiler and resolver

Treat policy sources and executable rules as separate. LLMs can find, summarize, translate and compare proposed interpretations but cannot promote them into executable rules. Only constrained typed representations, statically validated and independently reviewed, may be approved. Never execute model-generated arbitrary code from a profile.

A profile resolver returns chosen profile IDs/revisions, evidence frontier/context fields, supported and unsupported dimensions, ambiguity/conflicts, reviewer/missing-evidence requirements, evaluator identity and limited claim scope.

### 4.4 Interoperability adapters

OpenPeppol BIS Billing 3.0 is a CIUS of EN 16931. Its validation stages include syntax, EN 16931, general CIUS rules and country-qualified rules. Do not collapse XML parsing or network acceptance into local invoice compliance. [OpenPeppol BIS Billing 3.0](https://docs.peppol.eu/poacc/billing/3.0/bis/)

Adapters preserve source bytes and emit original syntax/profile, exact validator release, raw and normalized fields, transformation graph/parser version, field-level preservation/loss/ambiguity report, and precisely scoped signature/certificate/transport facts.

Initial formats to research: UBL/Peppol, EN 16931, UN/CEFACT, EDIFACT, ANSI X12 and jurisdiction-specific SAF-T/e-report exports. Never silently flatten extension fields.

### 4.5 Reporting and taxonomy

XBRL describes digital expression of reporting concepts; it does not prescribe the underlying accounting requirements. Reporting taxonomies vary across IFRS, local jurisdictions, sectors and standard setters. Taxonomy mapping remains a distinct versioned adapter. Preserve unmapped concepts and extension loss. [XBRL standard](https://www.xbrl.org/the-standard/what/how-the-xbrl-standard-works/) · [XBRL taxonomies](https://www.xbrl.org/the-standard/what/key-concepts-in-xbrl/taxonomies/)

### 4.6 Global tax and digital reporting

OECD's 2026 DCTR report notes national regimes are expanding but remain heterogeneous and often domestically designed. It discusses e-invoicing convergence and interoperability. Use adapters and country profiles rather than one global tax-reporting schema. OECD reports are design references, not local legal authority. [OECD DCTR](https://www.oecd.org/en/publications/digital-continuous-transactional-reporting-for-value-added-tax_34c88c39-en.html) · [OECD Tax Policy Reforms 2026](https://www.oecd.org/en/publications/tax-policy-reforms-2026_43d18a55-en/full-report/tax-policy-reforms_82075677.html)

## 5. Localisation is a correctness property

Preserve source-language content and bytes. Normalize numeric/date representations only when locale/calendar/timezone evidence is explicit and unambiguous. Never mutate original strings.

An identifier is a tuple of issuer namespace, scheme, value, validity and provenance—not just a string. Distinguish registered entity, VAT/GST account, branch, establishment and local tax registration.

Time handling preserves source local timestamp/offset, normalized instant where unambiguous, IANA timezone, capture time, reporting calendar/cutoff and ambiguity where DST/offset prevents unique resolution.

Currency preserves transaction, functional and presentation roles. Use exact decimal/rational arithmetic, explicit precision and rounding, and FX purpose, source, quotation direction, timestamp and policy. Never coerce currency precision from display locale.

Locale and timezone behavior also depends on versioned data. Unicode CLDR 49 (published 21 October 2026) supplies locale, numbering-system, calendar and formatting data; IANA tzdb 2026f (released 27 October 2026) changed Cuba's 1 November 2026 time-zone behavior. A replay must pin those releases and the runtime implementation rather than silently inheriting the host OS/browser defaults. [Unicode UTS #35 / CLDR 49](https://unicode.org/reports/tr35/) · [IANA time zones / 2026f release](https://www.iana.org/time-zones).

## 6. Public development corpus v1.0.0

The candidate input and evaluator oracle are separate. Twenty-two synthetic cases cover multi-axis jurisdiction context, missing context, framework/tax separation, IFRS for SMEs edition/eligibility, currency/FX, timezone/DST, locale parsing, UBL/EN 16931 versus CIUS, transport versus authenticity/delivery/settlement, identity namespaces, XBRL extension loss, bitemporal revisions, partial qualification, framework conversion, missing source bytes, Arabic-Indic numbering, Unicode-normalized names without shared identifiers, unpinned non-Gregorian calendar data and time-zone database version drift.

All synthetic jurisdiction, currency, tax and framework aliases are non-legal. No tax rate, filing rule, national accounting treatment or legal conclusion is encoded. The IFRS for SMEs date is a sourced reference fact, not an executable accounting rule.

## 7. Benchmark design and target gates

Do not publish one aggregate global accounting accuracy score. Report by task family, profile intersection, source format, language/locale, currency configuration and materiality.

Hard gates:
- zero execution of unapproved, stale or scope-mismatched profiles;
- zero silent framework conversion without approved mapping;
- zero currency-scale or direction coercion;
- zero locale/DST ambiguity silently resolved by guessing;
- zero adapter field/source-byte loss without explicit loss record;
- zero transport/syntax receipt treated as proof of authenticity, delivery, tax or settlement;
- zero unsupported cross-border result labelled fully qualified;
- deterministic replay for identical context, source frontier, profile set and evaluator build.

Quality metrics: context-axis exact match and missing-axis detection; profile selection and correct abstention; cross-border exception recall; exact rational FX/minor-unit accuracy; adapter field preservation and loss accounting; locale/timezone parsing and referral; taxonomy mapping and unmapped-extension recall; bitemporal replay parity; material error and reviewer agreement; reviewer minutes, fully loaded cost, p50/p95/p99 latency and repeatability.

Initial synthetic target: 100% expected behavior on this 22-case public development set. This is conformance, not field readiness.

Research target after expansion to 100+ expert-reviewed cases, at least 5 language/locale families, 3 framework families and 3 currency precision classes:
- at least 95% profile-context axis recall;
- zero critical unsupported-profile false qualifications;
- zero seeded data-loss, currency-scale, locale-ambiguity and historical-replay hard-gate failures;
- at least 15 percentage-point paired improvement over the strongest matched baseline on the preregistered hard subset, with confidence intervals and per-class results;
- at least 30% lower reviewer effort in a bounded supported-profile workflow, without increased false assurance or exception leakage.

These are goals to test, not results.

## 8. Industry comparison: compete at the seams

NetSuite OneWorld describes multi-subsidiary, tax-jurisdiction and currency handling/consolidation. SAP S/4HANA localization covers local statutory/tax/payment reporting and parallel accounting. Xero markets multi-currency transaction capture, FX tracking and reporting. These are established ERP capabilities. [NetSuite OneWorld](https://docs.oracle.com/en/cloud/saas/netsuite/ns-online-help/section_N266701.html) · [SAP localization](https://learning.sap.com/courses/configuring-sap-s4hana-cloud-public-edition-with-sap-central-business-configuration/defining-localizations-and-scope_b21b44bb-a7eb-418e-8c64-da171b2a4f18) · [Xero multi-currency](https://www.xero.com/za/accounting-software/use-multiple-currencies/)

Do not rebuild an ERP. Integrate with accounting/tax systems. Test whether Mycelix adds expected-evidence completeness, source-specific provenance, independent corroboration, authority-aware abstention, bitemporal reproducibility, loss-aware interoperability and portable evidence across vendors. Compare by capability, profile intersection, cost, reviewer effort and error class. Vendor feature pages are vendor claims, not independent performance measurements.

## 9. Qualification and release model

Profile states: not_in_scope, candidate, research_only, review_required, qualified_for_declared_scope, suspended, retired.

Every qualification identifies exact jurisdiction/entity/framework/tax/e-invoice/reporting/validator/locale/time/currency intersections, authoritative snapshots/digests, reviewer approval, corpus version, exact code/build and limitations. Material dependency change invalidates or narrows qualification pending review.

The initial reference catalog labels real frameworks and standards research/reference only. It contains no real qualified country profile. South Africa remains review-required; its policy source registry remains separate.

## 10. Ownership and claim ceiling

- Mycelix: profile registry, source evidence, deterministic authority and receipts.
- Symthaea: read-only context discovery, exception analysis and candidate mapping.
- Symtropy: scenarios on named approved profile snapshots; never legal/accounting authority.
- Existing Mycelix issues retain their own contract ownership; this package composes them.

Passing this synthetic corpus does not establish international accounting compliance, IFRS conformity, tax compliance, e-invoice legal compliance, audit assurance or production readiness. Every real profile intersection requires exact primary-source snapshots and local professional review.
